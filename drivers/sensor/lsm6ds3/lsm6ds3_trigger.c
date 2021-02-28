/* ST Microelectronics LSM6DS3 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6ds3.pdf
 */

#define DT_DRV_COMPAT st_lsm6ds3

#include <kernel.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include "lsm6ds3.h"

LOG_MODULE_DECLARE(LSM6DS3, CONFIG_SENSOR_LOG_LEVEL);

#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
/**
 * lsm6ds3_enable_t_int - TEMP enable selected int pin to generate interrupt
 */
static int lsm6ds3_enable_t_int(const struct device *dev, int enable)
{
	const struct lsm6ds3_config *cfg = dev->config;
	struct lsm6ds3_data *lsm6ds3 = dev->data;
	lsm6ds3_int2_ctrl_t int2_ctrl;

	if (enable) {
		union axis1bit16_t buf;

		/* dummy read: re-trigger interrupt */
		lsm6ds3_temperature_raw_get(lsm6ds3->ctx, buf.u8bit);
	}

	/* set interrupt (TEMP DRDY interrupt is only on INT2) */
	if (cfg->int_pin == 1)
		return -EIO;

	lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
			 (uint8_t *)&int2_ctrl, 1);
	int2_ctrl.int2_drdy_temp = enable;
	return lsm6ds3_write_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
				 (uint8_t *)&int2_ctrl, 1);
}
#endif

/**
 * lsm6ds3_enable_xl_int - XL enable selected int pin to generate interrupt
 */
static int lsm6ds3_enable_xl_int(const struct device *dev, int enable)
{
	const struct lsm6ds3_config *cfg = dev->config;
	struct lsm6ds3_data *lsm6ds3 = dev->data;

	if (enable) {
		union axis3bit16_t buf;

		/* dummy read: re-trigger interrupt */
		lsm6ds3_acceleration_raw_get(lsm6ds3->ctx, buf.u8bit);
	}

	/* set interrupt */
	if (cfg->int_pin == 1) {
		lsm6ds3_int1_ctrl_t int1_ctrl;

		lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_INT1_CTRL,
				 (uint8_t *)&int1_ctrl, 1);

		int1_ctrl.int1_drdy_xl = enable;
		return lsm6ds3_write_reg(lsm6ds3->ctx, LSM6DS3_INT1_CTRL,
					 (uint8_t *)&int1_ctrl, 1);
	} else {
		lsm6ds3_int2_ctrl_t int2_ctrl;

		lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
				 (uint8_t *)&int2_ctrl, 1);
		int2_ctrl.int2_drdy_xl = enable;
		return lsm6ds3_write_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
					 (uint8_t *)&int2_ctrl, 1);
	}
}

/**
 * lsm6ds3_enable_g_int - Gyro enable selected int pin to generate interrupt
 */
static int lsm6ds3_enable_g_int(const struct device *dev, int enable)
{
	const struct lsm6ds3_config *cfg = dev->config;
	struct lsm6ds3_data *lsm6ds3 = dev->data;

	if (enable) {
		union axis3bit16_t buf;

		/* dummy read: re-trigger interrupt */
		lsm6ds3_angular_rate_raw_get(lsm6ds3->ctx, buf.u8bit);
	}

	/* set interrupt */
	if (cfg->int_pin == 1) {
		lsm6ds3_int1_ctrl_t int1_ctrl;

		lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_INT1_CTRL,
				 (uint8_t *)&int1_ctrl, 1);
		int1_ctrl.int1_drdy_g = enable;
		return lsm6ds3_write_reg(lsm6ds3->ctx, LSM6DS3_INT1_CTRL,
					 (uint8_t *)&int1_ctrl, 1);
	} else {
		lsm6ds3_int2_ctrl_t int2_ctrl;

		lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
				 (uint8_t *)&int2_ctrl, 1);
		int2_ctrl.int2_drdy_g = enable;
		return lsm6ds3_write_reg(lsm6ds3->ctx, LSM6DS3_INT2_CTRL,
					 (uint8_t *)&int2_ctrl, 1);
	}
}

/**
 * lsm6ds3_trigger_set - link external trigger to event data ready
 */
int lsm6ds3_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler)
{
	struct lsm6ds3_data *lsm6ds3 = dev->data;

	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		lsm6ds3->handler_drdy_acc = handler;
		if (handler) {
			return lsm6ds3_enable_xl_int(dev, LSM6DS3_EN_BIT);
		} else {
			return lsm6ds3_enable_xl_int(dev, LSM6DS3_DIS_BIT);
		}
	} else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		lsm6ds3->handler_drdy_gyr = handler;
		if (handler) {
			return lsm6ds3_enable_g_int(dev, LSM6DS3_EN_BIT);
		} else {
			return lsm6ds3_enable_g_int(dev, LSM6DS3_DIS_BIT);
		}
	}
#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
	else if (trig->chan == SENSOR_CHAN_DIE_TEMP) {
		lsm6ds3->handler_drdy_temp = handler;
		if (handler) {
			return lsm6ds3_enable_t_int(dev, LSM6DS3_EN_BIT);
		} else {
			return lsm6ds3_enable_t_int(dev, LSM6DS3_DIS_BIT);
		}
	}
#endif

	return -ENOTSUP;
}

/**
 * lsm6ds3_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void lsm6ds3_handle_interrupt(const struct device *dev)
{
	struct lsm6ds3_data *lsm6ds3 = dev->data;
	struct sensor_trigger drdy_trigger = {
		.type = SENSOR_TRIG_DATA_READY,
	};
	const struct lsm6ds3_config *cfg = dev->config;

	while (1) {
#if defined(CONFIG_LSM6DS3_ENABLE_FIFO)
		lsm6ds3_fifo_status2_t lsm6ds3_fifo_status2;
		if (lsm6ds3_read_reg(lsm6ds3->ctx, LSM6DS3_FIFO_STATUS2,
							   (uint8_t*)&lsm6ds3_fifo_status2, 1) < 0) {
			LOG_ERR("failed reading FIFO status reg");
			break;
		}
		if (lsm6ds3_fifo_status2.fth && (lsm6ds3->handler_drdy_acc != NULL)) {
			lsm6ds3->handler_drdy_acc(dev, &drdy_trigger);
			continue;
		} else {
			break;
		}

#else
		lsm6ds3_status_reg_t status;
		if (lsm6ds3_status_reg_get(lsm6ds3->ctx, &status) < 0) {
			LOG_ERR("failed reading status reg");
			goto HI_EXIT;
		}

		if ((status.xlda == 0) && (status.gda == 0)
#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
					&& (status.tda == 0)
#endif
					) {
			break;
		}

		if ((status.xlda) && (lsm6ds3->handler_drdy_acc != NULL)) {
			lsm6ds3->handler_drdy_acc(dev, &drdy_trigger);
		}

		if ((status.gda) && (lsm6ds3->handler_drdy_gyr != NULL)) {
			lsm6ds3->handler_drdy_gyr(dev, &drdy_trigger);
		}

#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
		if ((status.tda) && (lsm6ds3->handler_drdy_temp != NULL)) {
			lsm6ds3->handler_drdy_temp(dev, &drdy_trigger);
		}
#endif

#endif // CONFIG_LSM6DS3_ENABLE_FIFO
	}
	gpio_pin_interrupt_configure(lsm6ds3->gpio, cfg->int_gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);
}

static void lsm6ds3_gpio_callback(const struct device *dev,
				    struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	struct lsm6ds3_data *lsm6ds3 =
		CONTAINER_OF(cb, struct lsm6ds3_data, gpio_cb);
	const struct lsm6ds3_config *cfg = lsm6ds3->dev->config;

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure(lsm6ds3->gpio, cfg->int_gpio_pin,
				     GPIO_INT_DISABLE);

#if defined(CONFIG_LSM6DS3_TRIGGER_OWN_THREAD)
	k_sem_give(&lsm6ds3->gpio_sem);
#elif defined(CONFIG_LSM6DS3_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lsm6ds3->work);
#endif /* CONFIG_LSM6DS3_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_LSM6DS3_TRIGGER_OWN_THREAD
FUNC_NORETURN static void lsm6ds3_thread(struct lsm6ds3_data *lsm6ds3)
{

	while (1) {
		k_sem_take(&lsm6ds3->gpio_sem, K_FOREVER);
		lsm6ds3_handle_interrupt(lsm6ds3->dev);
	}
}
#endif /* CONFIG_LSM6DS3_TRIGGER_OWN_THREAD */

#ifdef CONFIG_LSM6DS3_TRIGGER_GLOBAL_THREAD
static void lsm6ds3_work_cb(struct k_work *work)
{
	struct lsm6ds3_data *lsm6ds3 =
		CONTAINER_OF(work, struct lsm6ds3_data, work);

	lsm6ds3_handle_interrupt(lsm6ds3->dev);
}
#endif /* CONFIG_LSM6DS3_TRIGGER_GLOBAL_THREAD */

int lsm6ds3_init_interrupt(const struct device *dev)
{
	struct lsm6ds3_data *lsm6ds3 = dev->data;
	const struct lsm6ds3_config *cfg = dev->config;
	int ret;

	/* setup data ready gpio interrupt (INT1 or INT2) */
	lsm6ds3->gpio = device_get_binding(cfg->int_gpio_port);
	if (lsm6ds3->gpio == NULL) {
		LOG_DBG("Cannot get pointer to %s device",
			    cfg->int_gpio_port);
		return -EINVAL;
	}

#if defined(CONFIG_LSM6DS3_TRIGGER_OWN_THREAD)
	k_sem_init(&lsm6ds3->gpio_sem, 0, UINT_MAX);

	k_thread_create(&lsm6ds3->thread, lsm6ds3->thread_stack,
			CONFIG_LSM6DS3_THREAD_STACK_SIZE,
			(k_thread_entry_t)lsm6ds3_thread, lsm6ds3,
			NULL, NULL, K_PRIO_COOP(CONFIG_LSM6DS3_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&lsm6ds3->thread, "lsm6ds3 trigger");
#elif defined(CONFIG_LSM6DS3_TRIGGER_GLOBAL_THREAD)
	lsm6ds3->work.handler = lsm6ds3_work_cb;
#endif /* CONFIG_LSM6DS3_TRIGGER_OWN_THREAD */

	ret = gpio_pin_configure(lsm6ds3->gpio, cfg->int_gpio_pin,
				 GPIO_INPUT | cfg->int_gpio_flags);
	if (ret < 0) {
		LOG_DBG("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&lsm6ds3->gpio_cb,
			   lsm6ds3_gpio_callback,
			   BIT(cfg->int_gpio_pin));

	if (gpio_add_callback(lsm6ds3->gpio, &lsm6ds3->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}

	/* enable interrupt on int1/int2 in pulse mode */
	if (lsm6ds3_int_notification_set(lsm6ds3->ctx,
					 //LSM6DS3_ALL_INT_PULSED) < 0) {
					 LSM6DS3_INT_PULSED) <0) {
		LOG_DBG("Could not set pulse mode");
		return -EIO;
	}

	return gpio_pin_interrupt_configure(lsm6ds3->gpio, cfg->int_gpio_pin,
					    GPIO_INT_EDGE_TO_ACTIVE);
}
