/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NOR_H__
#define __SPI_NOR_H__

#include <sys/util.h>

#define SPI_NOR_MAX_ID_LEN	3

#define SPI_NOR_TYPE_DEVICE_DEF				0U
#define SPI_NOR_TYPE_DEVICE_AT25DF321A		1U

/* Status register bits */
#define SPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */
#define SPI_NOR_QE_BIT          BIT(6)  /* Enable quad mode */

/* Control register bits */
#define SPI_NOR_4BYTE_BIT       BIT(5)  /* 4B addressing */

/* Flash opcodes */
#define SPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define SPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define SPI_NOR_CMD_READ        0x03    /* Read data */
#define SPI_NOR_CMD_DREAD       0x3B    /* Read data (1-1-2) */
#define SPI_NOR_CMD_QREAD       0x6B    /* Read data (1-1-4) */
#define SPI_NOR_CMD_4READ       0xEB    /* Read data (1-4-4) */
#define SPI_NOR_CMD_WREN        0x06    /* Write enable */
#define SPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define SPI_NOR_CMD_PP          0x02    /* Page program */
#define SPI_NOR_CMD_4PP         0x38    /* Page program (1-4-4) */
#define SPI_NOR_CMD_RDCR        0x15    /* Read control register */
#define SPI_NOR_CMD_SE          0x20    /* Sector erase */
#define SPI_NOR_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NOR_CMD_BE          0xD8    /* Block erase */
#define SPI_NOR_CMD_CE          0xC7    /* Chip erase */
#define SPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NOR_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NOR_CMD_4BA         0xB7    /* Enter 4-Byte Address Mode */
#define SPI_NOR_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NOR_CMD_RDPD        0xAB    /* Release from Deep Power Down */

/* Page, sector, and block size are standard, not configurable. */
#define SPI_NOR_PAGE_SIZE    0x0100U
#define SPI_NOR_SECTOR_SIZE  0x1000U
#define SPI_NOR_BLOCK_SIZE   0x10000U

/* Test whether offset is aligned to a given number of bits. */
#define SPI_NOR_IS_ALIGNED(_ofs, _bits) (((_ofs) & BIT_MASK(_bits)) == 0)
#define SPI_NOR_IS_SECTOR_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 12)


#if DT_HAS_COMPAT_STATUS_OKAY(adesto_at25df321a)

#define SPI_NOR_AT25DF321A

/* JEDEC ID */
#define SPI_NOR_AT25DF321A_ID1      0x1f
#define SPI_NOR_AT25DF321A_ID2      0x47
#define SPI_NOR_AT25DF321A_ID3      0x01

/* Status register bits */
#define SPI_NOR_SB1_RDY_BIT         BIT(0)  /* Ready/Busy Status */
#define SPI_NOR_SB1_WEL_BIT         BIT(1)  /* Write Enable Latch Status */
#define SPI_NOR_SB1_SPF_BIT         (BIT(2) | BIT(3))  /* Software Protection Status */
#define SPI_NOR_SB1_WPP_BIT         BIT(4)  /* Write Protect (WP) Pin Status */
#define SPI_NOR_SB1_EPE_BIT         BIT(5)  /* Erase/Program Error */
#define SPI_NOR_SB1_SPRL_BIT        BIT(7)  /* Sector Protection Registers Locked */

#define SPI_NOR_SB2_RDY_BIT         BIT(0)  /* Ready/Busy Status */
#define SPI_NOR_SB2_ES_BIT          BIT(1)  /* Erase Suspend Status */
#define SPI_NOR_SB2_PS_BIT          BIT(2)  /* Program Suspend Status */
#define SPI_NOR_SB2_SLE_BIT         BIT(3)  /* Sector Lockdown Enabled */
#define SPI_NOR_SB2_RSTE_BIT        BIT(4)  /* Reset Enabled */

/* Protection Commands */
#define SPI_NOR_CMD_PSEN            0x36    /* Protect Sector */
#define SPI_NOR_CMD_PSDI            0x39    /* Unprotect Sector */

#endif

#endif /*__SPI_NOR_H__*/
