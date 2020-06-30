/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>

LOG_MODULE_REGISTER(modem_simcom, CONFIG_MODEM_LOG_LEVEL
);

#include <kernel.h>
#include <device.h>
#include <sys/ring_buffer.h>
#include <sys/util.h>
#include <net/ppp.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>
#include <soc.h>
#include <stdlib.h>

#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_cmd_handler.h"
#include "../console/gsm_mux.h"

#define GSM_CMD_READ_BUF       128
#define GSM_CMD_AT_TIMEOUT     K_SECONDS(2)
#define GSM_CMD_SETUP_TIMEOUT  K_SECONDS(6)
#define GSM_RX_STACK_SIZE      CONFIG_MODEM_GSM_RX_STACK_SIZE
#define GSM_WORK_STACK_SIZE    CONFIG_MODEM_GSM_RX_STACK_SIZE
#define GSM_RECV_MAX_BUF       30
#define GSM_RECV_BUF_SIZE      128
#define GSM_BUF_ALLOC_TIMEOUT  K_SECONDS(1)
#define GSM_RSSI_TIMEOUT       K_SECONDS(30)
#define GSM_CREG_TIMEOUT       K_SECONDS(1)

/* During the modem setup, we first create DLCI control channel and then
 * PPP and AT channels. Currently the modem does not create possible GNSS
 * channel.
 */
enum setup_state {
	STATE_INIT = 0,
	STATE_CONTROL_CHANNEL = 0,
	STATE_PPP_CHANNEL,
	STATE_AT_CHANNEL,
	STATE_DONE
};

static struct gsm_modem {
	struct modem_context context;

	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_read_buf[GSM_CMD_READ_BUF];
	uint8_t cmd_match_buf[GSM_CMD_READ_BUF];
	struct k_sem sem_response;

	struct modem_iface_uart_data gsm_data;
	struct k_delayed_work gsm_configure_work;
	char gsm_isr_buf[PPP_MRU];
	char gsm_rx_rb_buf[PPP_MRU * 3];

	uint8_t *ppp_recv_buf;
	size_t ppp_recv_buf_len;

	enum setup_state state;
	struct device *ppp_dev;
	struct device *at_dev;
	struct device *control_dev;

	struct modem_iface ppp_iface;

	uint8_t creg_status;
	uint8_t network_connected;
	uint8_t operator_connected;

	struct {
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
		struct device *power;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), reset_gpios)
		struct device *reset;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
		struct device *status;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), enable_gpios)
		struct device *enable;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), sim_select_gpios)
		struct device *sim_select;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), spk_enable_gpios)
		struct device *spk_enable;
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), ring_gpios)
		struct device *ring;
#endif
	} pins;
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
	struct gpio_callback gsm_status_cb;
#endif
	struct {
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
		bool status;
#endif
	} input_pins;

	struct k_delayed_work modem_power_on_work;
	struct k_delayed_work modem_power_off_work;
	struct k_delayed_work modem_reset_work;
	struct k_delayed_work modem_creg_change_status;
	struct k_delayed_work modem_rssi_query_work;
	struct k_delayed_work modem_no_carrier;
	struct k_delayed_work modem_connect_network_query_work;
	struct k_delayed_work modem_start_ppp_work;

	bool mux_enabled: 1;
	bool mux_setup_done: 1;
	bool setup_done: 1;

	bool power_ok: 1;
	bool enable: 1;
} gsm;

static struct net_mgmt_event_callback modem_mgmt_cb_connect;
static struct net_mgmt_event_callback modem_mgmt_cb_ppp;
static struct net_mgmt_event_callback modem_mgmt_cb_if;

NET_BUF_POOL_DEFINE(gsm_recv_pool,
					GSM_RECV_MAX_BUF, GSM_RECV_BUF_SIZE,
					0, NULL);
K_THREAD_STACK_DEFINE(gsm_rx_stack, GSM_RX_STACK_SIZE
);

K_THREAD_STACK_DEFINE(modem_work_q_stack, GSM_WORK_STACK_SIZE
);

struct k_thread gsm_rx_thread;

static struct k_work_q modem_work_q;

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
#define GSM_POWER_HW_PORT         DT_GPIO_LABEL(DT_INST(0, simcom_sim800), power_gpios)
#define GSM_POWER_HW_PIN          DT_GPIO_PIN(DT_INST(0, simcom_sim800), power_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), reset_gpios)
#define GSM_RESET_HW_PORT         DT_GPIO_LABEL(DT_INST(0, simcom_sim800), reset_gpios)
#define GSM_RESET_HW_PIN          DT_GPIO_PIN(DT_INST(0, simcom_sim800), reset_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
#define GSM_STATUS_HW_PORT        DT_GPIO_LABEL(DT_INST(0, simcom_sim800), status_gpios)
#define GSM_STATUS_HW_PIN         DT_GPIO_PIN(DT_INST(0, simcom_sim800), status_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), enable_gpios)
#define GSM_ENABLE_HW_PORT        DT_GPIO_LABEL(DT_INST(0, simcom_sim800), enable_gpios)
#define GSM_ENABLE_HW_PIN         DT_GPIO_PIN(DT_INST(0, simcom_sim800), enable_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), sim_select_gpios)
#define GSM_SIM_SELECT_HW_PORT    DT_GPIO_LABEL(DT_INST(0, simcom_sim800), sim_select_gpios)
#define GSM_SIM_SELECT_HW_PIN     DT_GPIO_PIN(DT_INST(0, simcom_sim800), sim_select_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), spk_enable_gpios)
#define GSM_SPK_ENABLE_HW_PORT    DT_GPIO_LABEL(DT_INST(0, simcom_sim800), spk_enable_gpios)
#define GSM_SPK_ENABLE_HW_PIN     DT_GPIO_PIN(DT_INST(0, simcom_sim800), spk_enable_gpios)
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), ring_gpios)
#define GSM_RING_HW_PORT          DT_GPIO_LABEL(DT_INST(0, simcom_sim800), ring_gpios)
#define GSM_RING_HW_PIN           DT_GPIO_PIN(DT_INST(0, simcom_sim800), ring_gpios)
#endif

/* helper macro to keep readability */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

/**
 * @brief  Convert string to long integer, but handle errors
 *
 * @param  s: string with representation of integer number
 * @param  err_value: on error return this value instead
 * @param  desc: name the string being converted
 * @param  func: function where this is called (typically __func__)
 *
 * @retval return integer conversion on success, or err_value on error
 */
static int modem_atoi(const char *s, const int err_value,
					  const char *desc, const char *func) {
	int ret;
	char *endptr;

	ret = (int) strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", log_strdup(s), log_strdup(desc),
				log_strdup(func));
		return err_value;
	}

	return ret;
}

extern char *net_sprint_ll_addr_buf(const uint8_t *ll, uint8_t ll_len,
									char *buf, int buflen);

static inline char *net_sprint_ll_addr(const uint8_t *ll, uint8_t ll_len) {
	static char buf[sizeof("xx:xx:xx:xx:xx:xx:xx:xx")];

	return net_sprint_ll_addr_buf(ll, ll_len, (char *) buf, sizeof(buf));
}

static void set_ppp_carrier_on(struct gsm_modem *gsm_p) {
	struct device *ppp_dev = device_get_binding(CONFIG_NET_PPP_DRV_NAME);
	const struct ppp_api *api;
	LOG_DBG("PPP carrier on");

	if (!ppp_dev) {
		LOG_ERR("Cannot find PPP %s!", "device");
		return;
	}

	api = (const struct ppp_api *) ppp_dev->driver_api;
	api->start(ppp_dev);
}

static void gsm_rx(struct gsm_modem *gsm_p) {
	LOG_DBG("starting");

	while (true) {
		k_sem_take(&gsm_p->gsm_data.rx_sem, K_FOREVER);

		/* The handler will listen AT channel */
		gsm_p->context.cmd_handler.process(&gsm_p->context.cmd_handler,
										   &gsm_p->context.iface);
	}
}

MODEM_CMD_DEFINE(gsm_cmd_ok) {
	modem_cmd_handler_set_error(data, 0);
	LOG_DBG("ok");
	k_sem_give(&gsm.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(gsm_cmd_connect) {
	modem_cmd_handler_set_error(data, 0);
	LOG_DBG("connect ok");
	k_sem_give(&gsm.sem_response);
	k_delayed_work_submit_to_queue(&modem_work_q,
								   &gsm.modem_start_ppp_work,
								   K_MSEC(10));
	return 0;
}

MODEM_CMD_DEFINE(gsm_cmd_no_carrier) {
//    modem_cmd_handler_set_error(data, 0);
	LOG_DBG("NO CARRIER");
	k_delayed_work_submit_to_queue(&modem_work_q,
								   &gsm.modem_no_carrier,
								   K_MSEC(1));
	return 0;
}

MODEM_CMD_DEFINE(gsm_cmd_creg) {
	int stat;
	stat = ATOI(argv[0], 0, "stat");
	LOG_DBG("CREG: %d", stat);
	gsm.creg_status = stat;
	k_delayed_work_submit_to_queue(&modem_work_q,
								   &gsm.modem_creg_change_status,
								   K_MSEC(1));
	return 0;
}

MODEM_CMD_DEFINE(gsm_cmd_error) {
	modem_cmd_handler_set_error(data, -EINVAL);
	LOG_DBG("error %d", ATOI(argv[0], 0, "e"));
	k_sem_give(&gsm.sem_response);
	return 0;
}

///* Handler: +CSQ: <signal_power>[0],<qual>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_csq) {
	int rssi;
	struct modem_context *mctx;
	mctx = modem_context_from_id(0);

	rssi = ATOI(argv[0], 0, "signal_power");
	if (rssi == 31) {
		mctx->data_rssi = -46;
	} else if (rssi >= 0 && rssi <= 31) {
		/* FIXME: This value depends on the RAT */
		mctx->data_rssi = -110 + ((rssi * 2) + 1);
	} else {
		mctx->data_rssi = -1000;
	}

	LOG_INF("RSSI: %d", mctx->data_rssi);
	return 0;
}

static struct modem_cmd response_cmds[] = {
		MODEM_CMD("OK", gsm_cmd_ok, 0U, ""),
		MODEM_CMD("+CME ERROR: ", gsm_cmd_error, 1U, ""),
		MODEM_CMD("CONNECT", gsm_cmd_connect, 0U, ""),
		/* Register in network (automatic message) */
		MODEM_CMD("+CREG: ", gsm_cmd_creg, 1U, ""),
		/* No carrier */
		MODEM_CMD("NO CARRIER", gsm_cmd_no_carrier, 0U, ""),
		/* RSSI */
		//	MODEM_CMD("+CSQ: ", on_cmd_atcmdinfo_rssi_csq, 2U, ","),
};


#define MDM_MANUFACTURER_LENGTH  10
#define MDM_MODEL_LENGTH         16
#define MDM_REVISION_LENGTH      64
#define MDM_IMEI_LENGTH          16

struct modem_info {
//    char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
//    char mdm_model[MDM_MODEL_LENGTH];
//    char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
};

static struct modem_info minfo;

#if defined(CONFIG_MODEM_SHELL)
/*
 * Provide modem info if modem shell is enabled. This can be shown with
 * "modem list" shell command.
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_manufacturer,
					sizeof(minfo.mdm_manufacturer) - 1,
					data->rx_buf, 0, len);
	minfo.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(minfo.mdm_manufacturer));

	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_model,
					sizeof(minfo.mdm_model) - 1,
					data->rx_buf, 0, len);
	minfo.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", log_strdup(minfo.mdm_model));

	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_revision,
					sizeof(minfo.mdm_revision) - 1,
					data->rx_buf, 0, len);
	minfo.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", log_strdup(minfo.mdm_revision));

	return 0;
}


#endif /* CONFIG_MODEM_SHELL */

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei) {
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_imei, sizeof(minfo.mdm_imei) - 1,
								data->rx_buf, 0, len);
	minfo.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", log_strdup(minfo.mdm_imei));

	return 0;
}


static struct setup_cmd reset_cmds[] = {
		/* no echo */
		SETUP_CMD_NOHANDLE("AT+CFUN=1,1"),
		/* hang up */
		SETUP_CMD_NOHANDLE("AT"),
};

static struct setup_cmd setup_cmds[] = {
		/* no echo */
		SETUP_CMD_NOHANDLE("ATE0"),
		/* hang up */
		SETUP_CMD_NOHANDLE("ATH"),
		/* extender errors in numeric form */
		SETUP_CMD_NOHANDLE("AT+CMEE=1"),
		/* Enable detailed call info */
		SETUP_CMD_NOHANDLE("AT+CLCC=1"),
#if defined(CONFIG_MODEM_SHELL)
		/* query modem info */
		SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
		SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
		SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
#endif
		/* Get serial number */
		SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),

		/* disable unsolicited network registration codes */
		//SETUP_CMD_NOHANDLE("AT+CREG=0"),
		/* enable unsolicited network registration codes */
		SETUP_CMD_NOHANDLE("AT+CREG=1"),
//		SETUP_CMD_NOHANDLE("AT+CREG?"),

		/* create PDP context */
		SETUP_CMD_NOHANDLE("AT+CGDCONT=1,\"IP\",\"" CONFIG_MODEM_GSM_APN "\""),
};

static struct setup_cmd connect_cmds[] = {
		/* connect to network */
//		SETUP_CMD_NOHANDLE("ATD*99#"),
		/* hang up */
		SETUP_CMD_NOHANDLE("ATH"),
		SETUP_CMD_NOHANDLE("ATD*99***1#"),
};

static int gsm_setup_mccmno(struct gsm_modem *gsm_p) {
	int ret;

	if (CONFIG_MODEM_GSM_MANUAL_MCCMNO[0]) {
		/* use manual MCC/MNO entry */
		ret = modem_cmd_send(&gsm_p->context.iface,
							 &gsm_p->context.cmd_handler,
							 NULL, 0,
							 "AT+COPS=1,2,\""
		CONFIG_MODEM_GSM_MANUAL_MCCMNO
		"\"",
				&gsm_p->sem_response,
				GSM_CMD_AT_TIMEOUT);
	} else {
		/* register operator automatically */
		ret = modem_cmd_send(&gsm_p->context.iface,
							 &gsm_p->context.cmd_handler,
							 NULL, 0, "AT+COPS=0,0",
							 &gsm_p->sem_response,
							 GSM_CMD_AT_TIMEOUT);
	}

	if (ret < 0) {
		LOG_ERR("AT+COPS ret:%d", ret);
	}

	return ret;
}


static void gsm_finalize_connection(struct gsm_modem *gsm_p) {
	int ret;

	if (IS_ENABLED(CONFIG_GSM_MUX) && gsm_p->mux_enabled) {
		ret = modem_cmd_send(&gsm_p->context.iface,
							 &gsm_p->context.cmd_handler,
							 &response_cmds[0],
							 ARRAY_SIZE(response_cmds),
							 "AT", &gsm_p->sem_response,
							 GSM_CMD_AT_TIMEOUT);
		if (ret < 0) {
			LOG_DBG("modem setup returned %d, %s",
					ret, "retrying...");
			(void) k_delayed_work_submit_to_queue(&modem_work_q,
												  &gsm_p->gsm_configure_work,
												  K_SECONDS(1));
			return;
		}
	}

	(void) gsm_setup_mccmno(gsm_p);

	ret = modem_cmd_handler_setup_cmds(&gsm_p->context.iface,
									   &gsm_p->context.cmd_handler,
									   setup_cmds,
									   ARRAY_SIZE(setup_cmds),
									   &gsm_p->sem_response,
									   GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("modem setup returned %d, %s",
				ret, "retrying...");
		(void) k_delayed_work_submit_to_queue(&modem_work_q,
											  &gsm_p->gsm_configure_work,
											  K_SECONDS(1));
		return;
	}

	LOG_DBG("modem setup returned %d, %s", ret, "enable PPP");

//	k_sleep(K_SECONDS(15));
//	ret = modem_cmd_handler_setup_cmds(&gsm_p->context.iface,
//									   &gsm_p->context.cmd_handler,
//									   connect_cmds,
//									   ARRAY_SIZE(connect_cmds),
//									   &gsm_p->sem_response,
//									   GSM_CMD_SETUP_TIMEOUT);
//	if (ret < 0) {
//		LOG_DBG("modem setup returned %d, %s",
//				ret, "retrying...");
//		(void) k_delayed_work_submit_to_queue(&modem_work_q,
//											  &gsm_p->gsm_configure_work,
//											  K_SECONDS(1));
//		return;
//	}

	gsm_p->setup_done = true;

	/* If we are not muxing, the modem interface and gsm_rx() thread is not
	 * needed as PPP will handle the incoming traffic internally.
	 */
#ifndef CONFIG_GSM_MUX
	if (!IS_ENABLED(CONFIG_GSM_MUX)) {
		k_thread_abort(&gsm_rx_thread);
	}
#endif

//	set_ppp_carrier_on(gsm_p);

	if (IS_ENABLED(CONFIG_GSM_MUX) && gsm_p->mux_enabled) {
		/* Re-use the original iface for AT channel */
		ret = modem_iface_uart_init_dev(&gsm_p->context.iface,
										gsm_p->at_dev->name);
		if (ret < 0) {
			LOG_DBG("iface %suart error %d", "AT ", ret);
		} else {
			/* Do a test and try to send AT command to modem */
			ret = modem_cmd_send(&gsm_p->context.iface,
								 &gsm_p->context.cmd_handler,
								 &response_cmds[0],
								 ARRAY_SIZE(response_cmds),
								 "AT", &gsm_p->sem_response,
								 GSM_CMD_AT_TIMEOUT);
			if (ret < 0) {
				LOG_DBG("modem setup returned %d, %s",
						ret, "AT cmds failed");
			} else {
				LOG_INF("AT channel %d connected to %s",
						DLCI_AT, gsm_p->at_dev->name);
			}
		}
	}
}

static int mux_enable(struct gsm_modem *gsm_p) {
	int ret;

	/* Turn on muxing */
	ret = modem_cmd_send(
			&gsm_p->context.iface,
			&gsm_p->context.cmd_handler,
			&response_cmds[0],
			ARRAY_SIZE(response_cmds),
#if defined(SIMCOM_LTE)
			/* FIXME */
			/* Some SIMCOM modems can set the channels */
			/* Control channel always at DLCI 0 */
			"AT+CMUXSRVPORT=0,0;"
			/* PPP should be at DLCI 1 */
			"+CMUXSRVPORT=" STRINGIFY(DLCI_PPP) ",1;"
			/* AT should be at DLCI 2 */
			"+CMUXSRVPORT=" STRINGIFY(DLCI_AT) ",1;"
#else
			"AT"
			#endif
			"+CMUX=0,0,5,"
			STRINGIFY(CONFIG_GSM_MUX_MRU_DEFAULT_LEN),
			&gsm_p->sem_response,
			GSM_CMD_AT_TIMEOUT);

	if (ret < 0) {
		LOG_ERR("AT+CMUX ret:%d", ret);
	}

	return ret;
}

static void mux_setup_next(struct gsm_modem *gsm_p) {
	(void) k_delayed_work_submit_to_queue(&modem_work_q,
										  &gsm_p->gsm_configure_work,
										  K_MSEC(1));
}

static void mux_attach_cb(struct device *mux, int dlci_address,
						  bool connected, void *user_data) {
	LOG_DBG("DLCI %d to %s %s", dlci_address, mux->name,
			connected ? "connected" : "disconnected");

	if (connected) {
		uart_irq_rx_enable(mux);
		uart_irq_tx_enable(mux);
	}

	mux_setup_next(user_data);
}

static int mux_attach(struct device *mux, struct device *uart,
					  int dlci_address, void *user_data) {
	int ret = uart_mux_attach(mux, uart, dlci_address, mux_attach_cb,
							  user_data);
	if (ret < 0) {
		LOG_ERR("Cannot attach DLCI %d (%s) to %s (%d)", dlci_address,
				mux->name, uart->name, ret);
		return ret;
	}

	return 0;
}

static void mux_setup(struct k_work *work) {
	struct gsm_modem *gsm_p = CONTAINER_OF(work,
										   struct gsm_modem,
										   gsm_configure_work);
	struct device *uart = device_get_binding(CONFIG_MODEM_GSM_UART_NAME);
	int ret;

	switch (gsm_p->state) {
		case STATE_CONTROL_CHANNEL:
			/* Get UART device. There is one dev / DLCI */
			if (gsm_p->control_dev == NULL) {
				gsm_p->control_dev = uart_mux_alloc();
			}
			if (gsm_p->control_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel",
						"control");
				goto fail;
			}

			gsm_p->state = STATE_PPP_CHANNEL;

			ret = mux_attach(gsm_p->control_dev, uart, DLCI_CONTROL, gsm_p);
			if (ret < 0) {
				goto fail;
			}

			break;

		case STATE_PPP_CHANNEL:
			if (gsm_p->ppp_dev == NULL) {
				gsm_p->ppp_dev = uart_mux_alloc();
			}
			if (gsm_p->ppp_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel", "PPP");
				goto fail;
			}

			gsm_p->state = STATE_AT_CHANNEL;

			ret = mux_attach(gsm_p->ppp_dev, uart, DLCI_PPP, gsm_p);
			if (ret < 0) {
				goto fail;
			}

			break;

		case STATE_AT_CHANNEL:
			if (gsm_p->at_dev == NULL) {
				gsm_p->at_dev = uart_mux_alloc();
			}
			if (gsm_p->at_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel", "AT");
				goto fail;
			}

			gsm_p->state = STATE_DONE;

			ret = mux_attach(gsm_p->at_dev, uart, DLCI_AT, gsm_p);
			if (ret < 0) {
				goto fail;
			}

			break;

		case STATE_DONE:
			/* At least the SIMCOM modem expects that the Internet
			 * connection is created in PPP channel. We will need
			 * to attach the AT channel to context iface after the
			 * PPP connection is established in order to give AT commands
			 * to the modem.
			 */
			ret = modem_iface_uart_init_dev(&gsm_p->context.iface,
											gsm_p->ppp_dev->name);

			if (ret < 0) {
				LOG_DBG("iface %s uart error %d", "PPP ", ret);
				gsm_p->mux_enabled = false;
				goto fail;
			}

			memcpy(&gsm_p->ppp_iface, &gsm_p->context.iface,
				   sizeof(struct modem_iface));

			LOG_INF("PPP channel %d connected to %s", DLCI_PPP,
					gsm_p->ppp_dev->name);

			gsm_finalize_connection(gsm_p);
			break;
	}

	return;

	fail:
	gsm_p->state = STATE_INIT;
	gsm_p->mux_enabled = false;
}

static void gsm_configure(struct k_work *work) {
	struct gsm_modem *gsm_p = CONTAINER_OF(work,
										   struct gsm_modem,
										   gsm_configure_work);
	int ret = -1;

	LOG_DBG("Starting modem %p configuration", gsm_p);

	ret = modem_cmd_send(&gsm_p->context.iface,
						 &gsm_p->context.cmd_handler,
						 &response_cmds[0],
						 ARRAY_SIZE(response_cmds),
						 "AT", &gsm_p->sem_response,
						 GSM_CMD_AT_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("modem not ready %d", ret);

		(void) k_delayed_work_submit_to_queue(&modem_work_q,
											  &gsm_p->gsm_configure_work,
											  K_NO_WAIT);
		return;
	}

	if (IS_ENABLED(CONFIG_GSM_MUX) && ret == 0 &&
		gsm_p->mux_enabled == false) {
		gsm_p->mux_setup_done = false;

		ret = mux_enable(gsm_p);
		if (ret == 0) {
			gsm_p->mux_enabled = true;
		} else {
			gsm_p->mux_enabled = false;
		}

		LOG_DBG("GSM muxing %s", gsm_p->mux_enabled ? "enabled" : "disabled");

		if (gsm_p->mux_enabled) {
			gsm_p->state = STATE_INIT;

			k_delayed_work_init(&gsm_p->gsm_configure_work, mux_setup);

			(void) k_delayed_work_submit_to_queue(&modem_work_q,
												  &gsm_p->gsm_configure_work,
												  K_NO_WAIT);
			return;
		}
	}

	gsm_finalize_connection(gsm_p);
}

static void modem_disable_ppp() {
	struct device *ppp_dev = device_get_binding(CONFIG_NET_PPP_DRV_NAME);
	const struct ppp_api *api;

	LOG_DBG("PPP disable");

	if (!ppp_dev) {
		LOG_ERR("Cannot find PPP %s!", "device");
		return;
	}

	api = (const struct ppp_api *) ppp_dev->driver_api;
	api->stop(ppp_dev);
	k_sleep(K_SECONDS(5));
}

static void modem_start_ppp_work(struct k_work *work) {
	struct gsm_modem *gsm_p = CONTAINER_OF(work,
										   struct gsm_modem,
										   modem_start_ppp_work);
	set_ppp_carrier_on(gsm_p);
}

static void modem_connect_network_query_work(struct k_work *work) {
//	int ret;
//    if (work && gsm.operator_connected) {
	int ret = 0;
	if (gsm.operator_connected) {
		LOG_INF("Connect network...");

		struct gsm_modem *gsm_p = CONTAINER_OF(work,
											   struct gsm_modem,
											   modem_connect_network_query_work);

		(void) k_delayed_work_cancel(&gsm_p->modem_rssi_query_work);

//		ret = modem_iface_uart_init_dev(&gsm_p->context.iface,
//										gsm_p->ppp_dev->name);

		ret = modem_cmd_handler_setup_cmds(&gsm_p->ppp_iface,
										   &gsm_p->context.cmd_handler,
										   connect_cmds,
										   ARRAY_SIZE(connect_cmds),
										   &gsm_p->sem_response,
										   GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_DBG("modem setup returned %d, %s", ret, "retrying...");
			(void) k_delayed_work_submit_to_queue(&modem_work_q,
												  &gsm_p->modem_rssi_query_work,
												  K_MSEC(1));
			(void) k_delayed_work_submit_to_queue(&modem_work_q,
												  &gsm_p->modem_connect_network_query_work,
												  K_SECONDS(5));
			return;
		}

#ifdef CONFIG_GSM_MUX
		//		if (gsm_p->mux_enabled) {
		//			/* Re-use the original iface for AT channel */
		//			ret = modem_iface_uart_init_dev(&gsm_p->context.iface,
		//											gsm_p->at_dev->name);
		//			if (ret < 0) {
		//				LOG_DBG("iface %suart error %d", "AT ", ret);
		//			} else {
		//				/* Do a test and try to send AT command to modem */
		//				ret = modem_cmd_send(&gsm_p->context.iface,
		//									 &gsm_p->context.cmd_handler,
		//									 &response_cmds[0],
		//									 ARRAY_SIZE(response_cmds),
		//									 "AT", &gsm_p->sem_response,
		//									 GSM_CMD_AT_TIMEOUT);
		//				if (ret < 0) {
		//					LOG_DBG("modem setup returned %d, %s",
		//							ret, "AT cmds failed");
		//				} else {
		//					LOG_INF("AT channel %d connected to %s",
		//							DLCI_AT, gsm_p->at_dev->name);
		//				}
		//			}
		//		}
#endif
		(void) k_delayed_work_submit_to_queue(&modem_work_q,
											  &gsm_p->modem_rssi_query_work,
											  K_MSEC(1));

	}
}

static void modem_power_on_work(struct k_work *work) {
	LOG_INF("Power On - Start");
	if (!gsm.power_ok) {
		gsm.power_ok = true;
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
		do {
			k_sleep(K_MSEC(200));
			gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 1);
			k_sleep(K_MSEC(3000));
			gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 0);
			k_sleep(K_MSEC(3000));
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
		} while (!gsm.input_pins.status);
#else
		} while (0);
#endif
	}
#endif
	LOG_INF("Power On - Ok");
	gsm.state = STATE_INIT;
	gsm.mux_enabled = false;
	gsm.setup_done = false;
	k_delayed_work_init(&gsm.gsm_configure_work, gsm_configure);
	(void) k_delayed_work_submit_to_queue(&modem_work_q,
										  &gsm.gsm_configure_work, K_NO_WAIT);
}

static void modem_power_off_work(struct k_work *work) {
	k_delayed_work_cancel(&gsm.gsm_configure_work);
	modem_disable_ppp();
	if (IS_ENABLED(CONFIG_GSM_MUX)) {
	}
	LOG_INF("Power Off - Ok");
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
	if (gsm.power_ok) {
		gsm.power_ok = false;
		do {
			k_sleep(K_MSEC(200));
			gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 1);
			k_sleep(K_MSEC(3000));
			gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 0);

			k_sleep(K_MSEC(500));
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
		} while (gsm.input_pins.status);
#else
		} while (0);
#endif
	}
#endif
	gsm.state = STATE_INIT;
	gsm.mux_enabled = false;
	gsm.setup_done = false;
}

static void modem_reset_work(struct k_work *work) {
	gsm.power_ok = false;
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), reset_gpios)
	gpio_pin_set(gsm.pins.power, GSM_RESET_HW_PIN, 1);
	k_sleep(K_MSEC(3000));
	gpio_pin_set(gsm.pins.power, GSM_RESET_HW_PIN, 0);
#else

#endif

}

static void modem_no_carrier(struct k_work *work) {
//    modem_disable_ppp();
//	k_delayed_work_submit_to_queue(&modem_work_q,
//								   &gsm.modem_connect_network_query_work,
//								   K_MSEC(1));
}

static void modem_rssi_query_work(struct k_work *work) {
	struct modem_cmd cmd = MODEM_CMD("+CSQ: ", on_cmd_atcmdinfo_rssi_csq, 2U,
									 ",");
//	static char *send_cmd = "AT+CSQ";
	int ret;
//	struct net_if *iface;

	ret = modem_cmd_send(&gsm.context.iface,
						 &gsm.context.cmd_handler,
						 &cmd,
						 1U,
						 "AT+CSQ", &gsm.sem_response,
						 GSM_CMD_AT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CSQ ret:%d", ret);
	}

	/* re-start RSSI query work */
	if (work && gsm.operator_connected) {
		k_delayed_work_submit_to_queue(&modem_work_q,
									   &gsm.modem_rssi_query_work,
									   GSM_RSSI_TIMEOUT);
	}
}

/*
static void modem_creg_query_work(struct k_work *work) {
	struct modem_cmd cmd = MODEM_CMD("+CREG: ", NULL, 2U, ",");
//    struct modem_cmd cmd = SETUP_CMD_NOHANDLE("AT+CREG?");
//	static char *send_cmd = "AT+CSQ";
	int ret;
//	struct net_if *iface;

	ret = modem_cmd_send(&gsm.context.iface,
						 &gsm.context.cmd_handler,
						 &cmd,
						 1U,
						 "AT+CREG?", &gsm.sem_response,
						 GSM_CMD_AT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CREG ret:%d", ret);
	}

	/ * re-start CREG query work * /
//    k_delayed_work_submit_to_queue(&modem_work_q, &gsm.modem_creg_query_work, GSM_CREG_TIMEOUT);
}
*/

static void modem_creg_change_status(struct k_work *work) {
	if (!(gsm.creg_status == 1 || gsm.creg_status == 5)) {
		if (gsm.operator_connected) {
			LOG_INF("Operator disconnect");
			k_delayed_work_cancel(&gsm.modem_rssi_query_work);
			//Disconnect operator network
			if (gsm.network_connected) {
				modem_disable_ppp();
			}
			gsm.operator_connected = false;
		}
	} else {
		if (!gsm.operator_connected) {
			LOG_INF("Operator connect");
			gsm.operator_connected = true;
			k_delayed_work_submit_to_queue(&modem_work_q,
										   &gsm.modem_rssi_query_work,
										   K_MSEC(1));
			k_delayed_work_submit_to_queue(&modem_work_q,
										   &gsm.modem_connect_network_query_work,
										   K_MSEC(2));
		}
	}
	//k_delayed_work_submit_to_queue(&modem_work_q, &gsm.modem_creg_query_work, GSM_CREG_TIMEOUT);
}

void pin_init(void) {
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
	gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 0);
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), reset_gpios)
	gpio_pin_set(gsm.pins.power, GSM_POWER_HW_PIN, 0);
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), enable_gpios)
	gpio_pin_set(gsm.pins.enable, GSM_ENABLE_HW_PIN, 0);
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), sim_select_gpios)
	gpio_pin_set(gsm.pins.sim_select, GSM_SIM_SELECT_HW_PIN, 0);
#endif
#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), spk_enable_gpios)
	gpio_pin_set(gsm.pins.spk_enable, GSM_SPK_ENABLE_HW_PIN, 0);
#endif
}

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)

static void interrupt_gsm_status(struct device *dev, struct gpio_callback *cb,
								 uint32_t pins) {
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	gsm.input_pins.status = gpio_pin_get(gsm.pins.status,
										 GSM_STATUS_HW_PIN);
	if (!gsm.input_pins.status && gsm.power_ok) {
		modem_disable_ppp();
		(void) k_delayed_work_submit_to_queue(&modem_work_q,
											  &gsm.modem_power_on_work,
											  K_NO_WAIT);
	}
	if (gsm.input_pins.status && !gsm.power_ok) {
		(void) k_delayed_work_submit_to_queue(&modem_work_q,
											  &gsm.modem_power_off_work,
											  K_NO_WAIT);
	}
}

#endif

static void modem_ppp_event_handler_connect(struct net_mgmt_event_callback *cb,
											uint32_t mgmt_event,
											struct net_if *iface) {

	if ((mgmt_event & (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED
					   | NET_EVENT_PPP_CARRIER_ON | NET_EVENT_PPP_CARRIER_OFF
					   | NET_EVENT_IF_UP | NET_EVENT_IF_DOWN)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		LOG_INF("Network connected");
		gsm.network_connected = true;
		LOG_INF("Link addr : %s",
				log_strdup(net_sprint_ll_addr(net_if_get_link_addr(iface)->addr,
											  net_if_get_link_addr(
													  iface)->len)));
		LOG_INF("MTU       : %d", net_if_get_mtu(iface));
		char buf[NET_IPV4_ADDR_LEN];
		LOG_INF("IP address: %s",
				log_strdup(net_addr_ntop(AF_INET,
										 &iface->config.ip.ipv4->unicast[0].address.in_addr,
										 buf, sizeof(buf))));
		return;
	}


	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		LOG_INF("Network disconnected");
		gsm.network_connected = false;
		return;
	}

}

static void modem_ppp_event_handler_ppp(struct net_mgmt_event_callback *cb,
										uint32_t mgmt_event,
										struct net_if *iface) {

	if ((mgmt_event & (NET_EVENT_PPP_CARRIER_ON | NET_EVENT_PPP_CARRIER_OFF |
					   NET_EVENT_PPP_LINK_DEAD)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_PPP_CARRIER_ON) {
		LOG_INF("EVT PPP carrier on");
	}
	if (mgmt_event == NET_EVENT_PPP_CARRIER_OFF) {
		LOG_INF("EVT PPP carrier off");
	}
	if (mgmt_event == NET_EVENT_PPP_LINK_DEAD) {
		LOG_INF("EVT PPP link terminated");
		k_delayed_work_submit_to_queue(&modem_work_q,
									   &gsm.modem_connect_network_query_work,
									   K_MSEC(1));
	}
}


static void modem_ppp_event_handler_if(struct net_mgmt_event_callback *cb,
									   uint32_t mgmt_event,
									   struct net_if *iface) {

	if ((mgmt_event & (NET_EVENT_IF_UP | NET_EVENT_IF_DOWN)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_IF_UP) {
		LOG_INF("EVT IF UP");
	}
	if (mgmt_event == NET_EVENT_IF_DOWN) {
		LOG_INF("EVT IF DOWN");
	}
}

static int gsm_init(struct device *device) {
	struct gsm_modem *gsm_p = device->driver_data;
	int r;

	LOG_DBG("GSM modem SIMCOM (%p)", gsm_p);

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), power_gpios)
	gsm.pins.power = device_get_binding(GSM_POWER_HW_PORT);
	gpio_pin_configure(gsm.pins.power, GSM_POWER_HW_PIN, GPIO_OUTPUT);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), reset_gpios)
	gsm.pins.reset = device_get_binding(GSM_RESET_HW_PORT);
	gpio_pin_configure(gsm.pins.reset, GSM_RESET_HW_PIN, GPIO_OUTPUT);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), status_gpios)
	gsm.pins.status = device_get_binding(GSM_STATUS_HW_PORT);
	gpio_pin_configure(gsm.pins.status, GSM_STATUS_HW_PIN, GPIO_INPUT);
	gpio_init_callback(&gsm.gsm_status_cb, interrupt_gsm_status,
					   BIT(GSM_STATUS_HW_PIN));
	gpio_pin_interrupt_configure(gsm.pins.status, GSM_STATUS_HW_PIN,
								 GPIO_INT_EDGE_BOTH);
	gpio_add_callback(gsm.pins.status, &gsm.gsm_status_cb);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), enable_gpios)
	gsm.pins.enable = device_get_binding(GSM_ENABLE_HW_PORT);
	gpio_pin_configure(gsm.pins.enable, GSM_ENABLE_HW_PIN, GPIO_OUTPUT);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), sim_select_gpios)
	gsm.pins.sim_select = device_get_binding(GSM_SIM_SELECT_HW_PORT);
	gpio_pin_configure(gsm.pins.sim_select, GSM_SIM_SELECT_HW_PIN, GPIO_OUTPUT);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), spk_enable_gpios)
	gsm.pins.spk_enable = device_get_binding(GSM_SPK_ENABLE_HW_PORT);
	gpio_pin_configure(gsm.pins.spk_enable, GSM_SPK_ENABLE_HW_PIN,
					   GPIO_OUTPUT);
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, simcom_sim800), ring_gpios)
	gsm.pins.ring = device_get_binding(GSM_RING_HW_PORT);
	gpio_pin_configure(gsm.pins.ring, GSM_RING_HW_PIN, GPIO_INPUT);
#endif
	pin_init();

	gsm_p->cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	gsm_p->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	gsm_p->cmd_handler_data.read_buf = &gsm_p->cmd_read_buf[0];
	gsm_p->cmd_handler_data.read_buf_len = sizeof(gsm_p->cmd_read_buf);
	gsm_p->cmd_handler_data.match_buf = &gsm_p->cmd_match_buf[0];
	gsm_p->cmd_handler_data.match_buf_len = sizeof(gsm_p->cmd_match_buf);
	gsm_p->cmd_handler_data.buf_pool = &gsm_recv_pool;
	gsm_p->cmd_handler_data.alloc_timeout = GSM_BUF_ALLOC_TIMEOUT;
	gsm_p->cmd_handler_data.eol = "\r";

	k_sem_init(&gsm_p->sem_response, 0, 1);

	/* initialize the work queue */
	k_work_q_start(&modem_work_q,
				   modem_work_q_stack,
				   K_THREAD_STACK_SIZEOF(modem_work_q_stack),
				   K_PRIO_COOP(9));

	r = modem_cmd_handler_init(&gsm_p->context.cmd_handler,
							   &gsm_p->cmd_handler_data);
	if (r < 0) {
		LOG_DBG("cmd handler error %d", r);
		return r;
	}

#if defined(CONFIG_MODEM_SHELL)
	/* modem information storage */
	gsm_p->context.data_manufacturer = minfo.mdm_manufacturer;
	gsm_p->context.data_model = minfo.mdm_model;
	gsm_p->context.data_revision = minfo.mdm_revision;
#endif
	gsm_p->context.data_imei = minfo.mdm_imei;

	gsm_p->gsm_data.isr_buf = &gsm_p->gsm_isr_buf[0];
	gsm_p->gsm_data.isr_buf_len = sizeof(gsm_p->gsm_isr_buf);
	gsm_p->gsm_data.rx_rb_buf = &gsm_p->gsm_rx_rb_buf[0];
	gsm_p->gsm_data.rx_rb_buf_len = sizeof(gsm_p->gsm_rx_rb_buf);

	r = modem_iface_uart_init(&gsm_p->context.iface, &gsm_p->gsm_data,
							  CONFIG_MODEM_GSM_UART_NAME);
	if (r < 0) {
		LOG_DBG("iface uart error %d", r);
		return r;
	}

	/* copy params to command interface */
	memcpy(&gsm_p->ppp_iface, &gsm_p->context.iface,
		   sizeof(struct modem_iface));


	r = modem_context_register(&gsm_p->context);
	if (r < 0) {
		LOG_DBG("context error %d", r);
		return r;
	}

	LOG_DBG("iface->read %p iface->write %p",
			gsm_p->context.iface.read, gsm_p->context.iface.write);

	k_thread_create(&gsm_rx_thread, gsm_rx_stack,
					K_THREAD_STACK_SIZEOF(gsm_rx_stack),
					(k_thread_entry_t) gsm_rx,
					gsm_p, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	// K_PRIO_COOP(7)
	k_thread_name_set(&gsm_rx_thread, "gsm_rx");

	k_delayed_work_init(&gsm_p->gsm_configure_work, gsm_configure);
	k_delayed_work_init(&gsm_p->modem_power_on_work,
						modem_power_on_work);
	k_delayed_work_init(&gsm_p->modem_power_off_work,
						modem_power_off_work);
	k_delayed_work_init(&gsm_p->modem_reset_work, modem_reset_work);
	k_delayed_work_init(&gsm.modem_creg_change_status,
						modem_creg_change_status);
	k_delayed_work_init(&gsm_p->modem_rssi_query_work,
						modem_rssi_query_work);
	k_delayed_work_init(&gsm_p->modem_no_carrier, modem_no_carrier);

//    k_delayed_work_init(&gsm.modem_creg_query_work, modem_creg_query_work);
	k_delayed_work_init(&gsm_p->modem_connect_network_query_work,
						modem_connect_network_query_work);

	k_delayed_work_init(&gsm_p->modem_start_ppp_work,
						modem_start_ppp_work);


	net_mgmt_init_event_callback(&modem_mgmt_cb_connect,
								 modem_ppp_event_handler_connect,
								 (NET_EVENT_L4_CONNECTED
								  | NET_EVENT_L4_DISCONNECTED));
	net_mgmt_init_event_callback(&modem_mgmt_cb_ppp,
								 modem_ppp_event_handler_ppp,
								 NET_EVENT_PPP_CARRIER_ON
								 | NET_EVENT_PPP_CARRIER_OFF
								 | NET_EVENT_PPP_LINK_DEAD);
	net_mgmt_init_event_callback(&modem_mgmt_cb_if, modem_ppp_event_handler_if,
								 NET_EVENT_IF_UP
								 | NET_EVENT_IF_DOWN);

	//(void) k_delayed_work_submit_to_queue(&modem_work_q,&gsm_p->gsm_configure_work, K_NO_WAIT);
	return 0;
}

DEVICE_INIT(gsm_ppp, "modem_gsm", gsm_init, &gsm, NULL, POST_KERNEL,
			CONFIG_MODEM_GSM_INIT_PRIORITY);


void z_impl_modem_power_on(void) {
	net_mgmt_add_event_callback(&modem_mgmt_cb_connect);
	net_mgmt_add_event_callback(&modem_mgmt_cb_ppp);
	net_mgmt_add_event_callback(&modem_mgmt_cb_if);
	(void) k_delayed_work_submit_to_queue(&modem_work_q,
										  &gsm.modem_power_on_work,
										  K_NO_WAIT);
}

void z_impl_modem_power_off(void) {
	net_mgmt_del_event_callback(&modem_mgmt_cb_connect);
	net_mgmt_del_event_callback(&modem_mgmt_cb_ppp);
	net_mgmt_del_event_callback(&modem_mgmt_cb_if);
	(void) k_delayed_work_submit_to_queue(&modem_work_q,
										  &gsm.modem_power_off_work,
										  K_NO_WAIT);
}

void z_impl_modem_power_reset(void) {
	(void) k_delayed_work_submit_to_queue(&modem_work_q,
										  &gsm.modem_reset_work,
										  K_NO_WAIT);
}

struct k_work_q *z_impl_modem_get_worker(void) {
	return &modem_work_q;
}
