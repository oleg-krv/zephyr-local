/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NOR_AT25DF321A_H__
#define __SPI_NOR_AT25DF321A_H__

#if DT_HAS_COMPAT_STATUS_OKAY(adesto_at25df321a)
#define SPI_NOR_AT25DF321A
#define SPI_NOR_RESET
#define SPI_NOR_STATUS_BIT2
#endif

#ifdef SPI_NOR_AT25DF321A
#define SPI_NOR_CMD_RST_CB          0xD0
#define SPI_NOR_AT25DF321A_ID1      0x1f
#define SPI_NOR_AT25DF321A_ID2      0x47
#define SPI_NOR_AT25DF321A_ID3      0x01

/* Status register bits */
#define SPI_NOR_SB1_RDY_BIT         BIT(0)  /* Ready/Busy Status */
#define SPI_NOR_SB1_WEL_BIT         BIT(1)  /* Write Enable Latch Status */
#define SPI_NOR_SB1_SPF_BIT         BIT(2)  /* Software Protection Status */
#define SPI_NOR_SB1_WPP_BIT         BIT(4)  /* Write Protect (WP) Pin Status */
#define SPI_NOR_SB1_EPE_BIT         BIT(5)  /* Erase/Program Error */
#define SPI_NOR_SB1_SPRL_BIT        BIT(7)  /* Sector Protection Registers Locked */

#define SPI_NOR_SB2_RDY_BIT         BIT(0)  /* Ready/Busy Status */
#define SPI_NOR_SB2_ES_BIT          BIT(1)  /* Erase Suspend Status */
#define SPI_NOR_SB2_PS_BIT          BIT(2)  /* Program Suspend Status */
#define SPI_NOR_SB2_SLE_BIT         BIT(3)  /* Sector Lockdown Enabled */
#define SPI_NOR_SB2_RSTE_BIT        BIT(4)  /* Reset Enabled */

/* Protection Commands */
#define SPI_NOR_CMD_PSEN        0x36    /* Protect Sector */
#define SPI_NOR_CMD_PSDI        0x39    /* Unprotect Sector */
#define SPI_NOR_CMD_PSR         0x3C    /* Read Sector Protection Registers */
/* Security Commands */
#define SPI_NOR_CMD_SLD         0x33    /* Sector Lockdown */
#define SPI_NOR_CMD_FSLD        0x34    /* Freeze Sector Lockdown State */
#define SPI_NOR_CMD_RSLD        0x35    /* Read Sector Lockdown Registers */

/* Status Register Commands */
#define SPI_NOR_CMD_WRSR2       0x33    /* Write Status Register Byte 2 */

/* Miscellaneous Commands */
#define SPI_NOR_CMD_RST         0xF0    /* Reset */

#endif /* SPI_NOR_AT25DF321A */
#endif /*__SPI_NOR_AT25DF321A_H__*/
