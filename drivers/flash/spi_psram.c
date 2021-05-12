/*
 * Copyright (c) 2021 Krivorot Oleg <krivorot.oleg@gmail.com>
 *
 * based on spi_nor.c
 * Emulate mtd flash device
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pseudo_mtd_sram

#include <errno.h>
#include <drivers/flash.h>
#include <drivers/spi.h>
#include <init.h>
#include <string.h>
#include <logging/log.h>
#include <sys/byteorder.h>

#include "spi_psram.h"

LOG_MODULE_REGISTER(spi_psram, CONFIG_FLASH_LOG_LEVEL);

/* Build-time data associated with the device. */
struct spi_psram_config {
	/* Runtime SFDP stores no static configuration. */
#ifndef CONFIG_SPI_PSRAM_SFDP_RUNTIME

	/* Size of device in bytes, from size property */
	uint32_t flash_size;

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	/* Flash page layout can be determined from devicetree. */
	struct flash_pages_layout layout;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	/* Device MF ID */
	uint8_t mf_id[SPI_PSRAM_MF_ID_LEN];
#endif /* CONFIG_SPI_PSRAM_SFDP_RUNTIME */
};

struct spi_psram_data {
	struct k_sem sem;
	const struct device *spi;
	struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */

#ifndef CONFIG_SPI_PSRAM_SFDP_RUNTIME
	uint8_t eid[6];
#endif /* CONFIG_SPI_PSRAM_SFDP_RUNTIME */

	bool write_protection;
};

static int spi_psram_write_protection_set(const struct device *dev, bool write_protect);

/* Get the size of the psram device.  Data for runtime, constant for
 * minimal and devicetree.
 */
static inline uint32_t dev_spi_psram_size(const struct device *dev)
{
	const struct spi_psram_config *cfg = dev->config;
	return cfg->flash_size;
}

/* Get the psram device page size.  Constant for minimal, data for
 * runtime and devicetree.
 */
static inline uint16_t dev_spi_psram_page_size(const struct device *dev)
{
	return 256;
}

/* Get the flash device page size.  Constant for minimal, data for
 * runtime and devicetree.
 */
static inline uint16_t dev_page_size(const struct device *dev)
{
	struct spi_psram_data *data = dev->data;
	uint16_t t = data->spi_cfg.frequency / 1000000;
	uint8_t page_size = t * DT_INST_PROP(0, tce_max_time);
	if (page_size * t == DT_INST_PROP(0, tce_max_time))
		page_size -= 1;
	return page_size;
}

static const struct flash_parameters spi_psram_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

/* Indicates that an access command includes bytes for the address.
 * If not provided the opcode is not followed by address bytes.
 */
#define NOR_ACCESS_ADDRESSED BIT(0)

/* Indicates that addressed access uses a 24-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_24BIT_ADDR BIT(1)

/* Indicates that addressed access uses a 32-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_32BIT_ADDR BIT(2)

/* Indicates that an access command is performing a write.  If not
 * provided access is a read.
 */
#define NOR_ACCESS_WRITE BIT(7)

/*
 * @brief Send an SPI command
 *
 * @param dev Device struct
 * @param opcode The command to send
 * @param access flags that determine how the command is constructed.
 *        See NOR_ACCESS_*.
 * @param addr The address to send
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @return 0 on success, negative errno code otherwise
 */
static int spi_psram_access(const struct device *const dev,
			  uint8_t opcode, unsigned int access,
			  off_t addr, void *data, size_t length)
{
	struct spi_psram_data *const driver_data = dev->data;
	bool is_addressed = (access & NOR_ACCESS_ADDRESSED) != 0U;
	bool is_write = (access & NOR_ACCESS_WRITE) != 0U;
	uint8_t buf[5] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = 1,
		},
		{
			.buf = data,
			.len = length
		}
	};

	buf[0] = opcode;
	if (is_addressed) {
		bool access_24bit = (access & NOR_ACCESS_24BIT_ADDR) != 0;
		bool access_32bit = (access & NOR_ACCESS_32BIT_ADDR) != 0;
		bool use_32bit = (access_32bit);
//			|| (!access_24bit
//				&& driver_data->flag_access_32bit));
		union {
			uint32_t u32;
			uint8_t u8[4];
		} addr32 = {
			.u32 = sys_cpu_to_be32(addr),
		};

		if (use_32bit) {
			memcpy(&buf[1], &addr32.u8[0], 4);
			spi_buf[0].len += 4;
		} else {
			memcpy(&buf[1], &addr32.u8[1], 3);
			spi_buf[0].len += 3;
		}
	};

	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (length != 0) ? 2 : 1,
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2,
	};

	if (is_write) {
		return spi_write(driver_data->spi,
				 &driver_data->spi_cfg, &tx_set);
	}

	return spi_transceive(driver_data->spi,
			      &driver_data->spi_cfg, &tx_set, &rx_set);
}

#define spi_psram_cmd_read(dev, opcode, dest, length) \
	spi_psram_access(dev, opcode, 0, 0, dest, length)
#define spi_psram_cmd_addr_read(dev, opcode, addr, dest, length) \
	spi_psram_access(dev, opcode, NOR_ACCESS_ADDRESSED, addr, dest, length)
#define spi_psram_cmd_write(dev, opcode) \
	spi_psram_access(dev, opcode, NOR_ACCESS_WRITE, 0, NULL, 0)
#define spi_psram_cmd_addr_write(dev, opcode, addr, src, length) \
	spi_psram_access(dev, opcode, NOR_ACCESS_WRITE | NOR_ACCESS_ADDRESSED, \
		       addr, (void *)src, length)

///*
// * @brief Send an SPI command
// *
// * @param dev Device struct
// * @param opcode The command to send
// * @param is_addressed A flag to define if the command is addressed
// * @param addr The address to send
// * @param data The buffer to store or read the value
// * @param length The size of the buffer
// * @param is_write A flag to define if it's a read or a write command
// * @return 0 on success, negative errno code otherwise
// */
//static int spi_psram_access(const struct device *const dev, uint8_t opcode, bool is_addressed,
//			    off_t addr, void *data, size_t length, bool is_write)
//{
//	struct spi_psram_data *const driver_data = dev->data;
//
//	uint8_t buf[4] = {
//		opcode,
//		(addr & 0xFF0000) >> 16,
//		(addr & 0xFF00) >> 8,
//		(addr & 0xFF),
//	};
//
//	struct spi_buf spi_buf[2] = { {
//					      .buf = buf,
//					      .len = (is_addressed) ? 4 : 1,
//				      },
//				      { .buf = data, .len = length } };
//	const struct spi_buf_set tx_set = { .buffers = spi_buf, .count = (length) ? 2 : 1 };
//
//	const struct spi_buf_set rx_set = { .buffers = spi_buf, .count = 2 };
//
//	if (is_write) {
//		return spi_write(driver_data->spi, &driver_data->spi_cfg, &tx_set);
//	}
//
//	return spi_transceive(driver_data->spi, &driver_data->spi_cfg, &tx_set, &rx_set);
//}
//
//#define spi_psram_cmd_read(dev, opcode, dest, length)                                              \
//	spi_psram_access(dev, opcode, false, 0, dest, length, false)
//#define spi_psram_cmd_addr_read(dev, opcode, addr, dest, length)                                   \
//	spi_psram_access(dev, opcode, true, addr, dest, length, false)
//#define spi_psram_cmd_addr_read_wait(dev, opcode, addr, dest, length)                              \
//	spi_psram_access(dev, opcode, true, addr, dest, length, false)
//#define spi_psram_cmd_write(dev, opcode) spi_psram_access(dev, opcode, false, 0, NULL, 0, true)
//#define spi_psram_cmd_addr_write(dev, opcode, addr, src, length)                                   \
//	spi_psram_access(dev, opcode, true, addr, (void *)src, length, true)

/* Everything necessary to acquire owning access to the device.
 *
 * This means taking the lock and, if necessary, waking the device
 * from deep power-down mode.
 */
static void acquire_device(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_psram_data *const driver_data = dev->data;

		k_sem_take(&driver_data->sem, K_FOREVER);
	}
}

/* Everything necessary to release access to the device.
 *
 * This means (optionally) putting the device into deep power-down
 * mode, and releasing the lock.
 */
static void release_device(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_psram_data *const driver_data = dev->data;

		k_sem_give(&driver_data->sem);
	}
}

static int spi_psram_read(const struct device *dev, off_t addr, void *dest, size_t size)
{
	struct spi_psram_data *data = dev->data;
	const size_t psram_size = dev_spi_psram_size(dev);

	/* should be between 0 and psram size */
	if ((addr < 0) || ((addr + size) > psram_size)) {
		return -EINVAL;
	}

	acquire_device(dev);

//	/* ToDo: realise wait read */
//	if (data->spi_cfg.frequency >= DT_INST_PROP(0, read_not_wait_max_frequency)) {
//		LOG_ERR("Not realise wait read");
//		return -EINVAL;
//	}

	int ret = 0;//spi_psram_cmd_addr_read(dev, SPI_PSRAM_CMD_READ, addr, dest, size);

	release_device(dev);
	return ret;
}

static int spi_psram_write(const struct device *dev, off_t addr, const void *src, size_t size)
{
	//struct spi_psram_data *data = dev->data;
	const size_t psram_size = dev_spi_psram_size(dev);
	const uint16_t page_size = dev_page_size(dev);
	int ret=0;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > psram_size)) {
		return -EINVAL;
	}

	acquire_device(dev);
	spi_psram_write_protection_set(dev, false);

	while (size > 0) {
		size_t to_write = size;

		/* Don't write more than a page. */
		if (to_write >= page_size) {
			to_write = page_size;
		}

		ret = spi_psram_cmd_addr_write(dev, SPI_PSRAM_CMD_WR, addr, src, to_write);
		if (ret != 0) {
			break;
		}

		size -= to_write;
		src = (const uint8_t *)src + to_write;
		addr += to_write;
	}

	spi_psram_write_protection_set(dev, true);
	release_device(dev);
	return 0;
}

static int spi_psram_erase(const struct device *dev, off_t addr, size_t size)
{
	const size_t psram_size = dev_spi_psram_size(dev);
	int ret = 0;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > psram_size)) {
		return -EINVAL;
	}

	return ret;
}

static int spi_psram_write_protection_set(const struct device *dev, bool write_protect)
{
	struct spi_psram_data *data = dev->data;

	//acquire_device(dev);
	data->write_protection = write_protect;
	//release_device(dev);

	return 0;
}

static int spi_psram_read_id(const struct device *dev, uint8_t *mf_id, uint8_t *kgd, uint8_t *eid)
{
	if (mf_id == NULL) {
		return -EINVAL;
	}
	uint8_t data[SPI_PSRAM_MAX_READ_ID_LEN];

	acquire_device(dev);

	int ret = spi_psram_cmd_addr_read(dev, SPI_PSRAM_CMD_RDID, 0x000000U, data,
					  SPI_PSRAM_MAX_READ_ID_LEN);

	release_device(dev);

	*mf_id = data[0];
	*kgd = data[1];
	memcpy(eid, &data[2], SPI_PSRAM_EID_LEN);

	return ret;
}

#ifdef CONFIG_FLASH_JESD216_API
static int spi_psram_read_jedec_id(const struct device *dev, uint8_t *id)
{
	if (id == NULL) {
		return -EINVAL;
	}
	acquire_device(dev);

	int ret = spi_psram_cmd_read(dev, SPI_PSRAM_CMD_RDID, id, SPI_PSRAM_MAX_READ_ID_LEN);

	release_device(dev);

	return ret;
}
#endif /*CONFIG_FLASH_JESD216_API*/

/**
 * @brief Configure the psram
 *
 * @param dev The psram device structure
 * @param info The psram info structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_psram_configure(const struct device *dev)
{
	struct spi_psram_data *data = dev->data;
	const struct spi_psram_config *cfg = dev->config;

	uint8_t mf_id;
	uint8_t kgd;

	int rc;

	data->spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!data->spi) {
		return -EINVAL;
	}

	data->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	data->spi_cfg.operation = SPI_WORD_SET(8);
	data->spi_cfg.slave = DT_INST_REG_ADDR(0);

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl.gpio_dev = device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	if (!data->cs_ctrl.gpio_dev) {
		return -ENODEV;
	}

	data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	data->cs_ctrl.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0);
	data->cs_ctrl.delay = CONFIG_SPI_NOR_CS_WAIT_DELAY;

	data->spi_cfg.cs = &data->cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */

	/* now the spi bus is configured, we can verify SPI
	 * connectivity by reading the ID.
	 */

	rc = spi_psram_read_id(dev, &mf_id, &kgd, data->eid);
	if (rc != 0) {
		LOG_ERR("ID read failed: %d", rc);
		return -ENODEV;
	}

	if (kgd != SPI_PSRAM_KGD_PASS) {
		LOG_ERR("Pseudo SRAM error tests passed: %d", kgd);
		return -ENODEV;
	}

	if (mf_id != cfg->mf_id[0]) {
		LOG_ERR("Device id %02x does not match config %02x ", mf_id, cfg->mf_id[0]);
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Initialize and configure the psram
 *
 * @param name The psram name
 * @return 0 on success, negative errno code otherwise
 */
static int spi_psram_init(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_psram_data *const driver_data = dev->data;

		k_sem_init(&driver_data->sem, 1, UINT_MAX);
	}

	return spi_psram_configure(dev);
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT

static void spi_psram_pages_layout(const struct device *dev,
				   const struct flash_pages_layout **layout, size_t *layout_size)
{
	const struct spi_psram_config *cfg = dev->config;

	*layout = &cfg->layout;

	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *spi_psram_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &spi_psram_parameters;
}

static const struct flash_driver_api spi_psram_api = {
	.read = spi_psram_read,
	.write = spi_psram_write,
	.erase = spi_psram_erase,
	.get_parameters = spi_psram_get_parameters,
//.write_protection = spi_psram_write_protection_set,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = spi_psram_pages_layout,
#endif
#ifdef CONFIG_FLASH_JESD216_API
	.sfdp_read = NULL,
	.read_jedec_id = spi_psram_read_jedec_id,
#endif
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT

/* For devicetree or minimal page layout we need to know the size of
 * the device.  We can't extract it from the raw BFP data, so require
 * it to be present in devicetree.
 */
BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size), "pseudo,sram size required");

/* instance 0 size in bytes */
#define INST_0_BYTES (DT_INST_PROP(0, size) / 8)

BUILD_ASSERT(SPI_PSRAM_IS_SECTOR_ALIGNED(CONFIG_SPI_PSRAM_LAYOUT_PAGE_SIZE),
	     "SPI_PSRAM_LAYOUT_PAGE_SIZE must be multiple of 4096");

/* instance 0 page count */
#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_SPI_PSRAM_LAYOUT_PAGE_SIZE)

BUILD_ASSERT((CONFIG_SPI_PSRAM_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT) == INST_0_BYTES,
	     "SPI_PSRAM_LAYOUT_PAGE_SIZE incompatible with flash size");

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct spi_psram_config spi_psram_config_0 = {
#ifdef CONFIG_FLASH_PAGE_LAYOUT
		.layout = {
				.pages_count = LAYOUT_PAGES_COUNT,
				.pages_size = CONFIG_SPI_PSRAM_LAYOUT_PAGE_SIZE,
		},
#undef LAYOUT_PAGES_COUNT
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

		.flash_size = DT_INST_PROP(0, size) / 8,
		.mf_id = DT_INST_PROP(0, mf_id),

};

static struct spi_psram_data spi_psram_data_0;

// CONFIG_SPI_PSRAM_INIT_PRIORITY
DEVICE_DT_INST_DEFINE(0, &spi_psram_init, NULL, &spi_psram_data_0, &spi_psram_config_0, POST_KERNEL,
		      CONFIG_SPI_NOR_INIT_PRIORITY, &spi_psram_api);
