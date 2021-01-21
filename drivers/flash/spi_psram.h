/*
 * Copyright (c) 2021 Krivorot Oleg <krivorot.oleg@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_PSRAM_LY68_H__
#define __SPI_PSRAM_LY68_H__

/* Max device ID */
#define SPI_PSRAM_MF_ID_LEN      1
#define SPI_PSRAM_KGD_LEN        1
#define SPI_PSRAM_EID_LEN        6

#define SPI_PSRAM_MAX_READ_ID_LEN       SPI_PSRAM_MF_ID_LEN + \
                                        SPI_PSRAM_KGD_LEN + \
                                        SPI_PSRAM_EID_LEN

#define SPI_PSRAM_KGD_PASS        0x5DU
#define SPI_PSRAM_KGD_FAIL        0x55U

/* PSRAM opcodes */
#define SPI_PSRAM_CMD_READ        0x03    /* Read data */
#define SPI_PSRAM_CMD_READF       0x0B    /* Fast read */
#define SPI_PSRAM_CMD_READFQ      0xEB    /* Fast read quad */

#define SPI_PSRAM_CMD_WR          0x02    /* Write data */
#define SPI_PSRAM_CMD_WRQ         0x38    /* Quad write */

#define SPI_PSRAM_CMD_QMEN        0x35    /* Enter quad mode */
#define SPI_PSRAM_CMD_QMEX        0xF5    /* Exit quad mode */

#define SPI_PSRAM_CMD_RESEN       0x66    /* Reset enable */
#define SPI_PSRAM_CMD_RES         0x99    /* Reset */

#define SPI_PSRAM_CMD_SE          0xC0    /* Set burst length */
#define SPI_PSRAM_CMD_RDID        0x9F    /* Read ID */

/* Page, sector, and block size are standard, not configurable. */
#define SPI_PSRAM_PAGE_SIZE       0x0100U
#define SPI_PSRAM_SECTOR_SIZE     0x1000U
#define SPI_PSRAM_BLOCK_SIZE      0x10000U

/* Max frequency read data command 0x03 */
#define SPI_PSRAM_READ_MAX_FREQ   33000000UL


/* Test whether offset is aligned to a given number of bits. */
#define SPI_PSRAM_IS_ALIGNED(_ofs, _bits) (((_ofs) & BIT_MASK(_bits)) == 0)
#define SPI_PSRAM_IS_SECTOR_ALIGNED(_ofs) SPI_PSRAM_IS_ALIGNED(_ofs, 12)

#endif
