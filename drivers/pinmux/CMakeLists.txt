# SPDX-License-Identifier: Apache-2.0

# Board initialization
zephyr_sources_ifdef(CONFIG_PINMUX_CC13XX_CC26XX   pinmux_cc13xx_cc26xx.c)
zephyr_sources_ifdef(CONFIG_PINMUX_ESP32           pinmux_esp32.c)
zephyr_sources_ifdef(CONFIG_PINMUX_HSDK            pinmux_hsdk.c)
zephyr_sources_ifdef(CONFIG_PINMUX_INTEL_S1000     pinmux_intel_s1000.c)
zephyr_sources_ifdef(CONFIG_PINMUX_ITE_IT8XXX2     pinmux_ite_it8xxx2.c)
zephyr_sources_ifdef(CONFIG_PINMUX_LPC11U6X        pinmux_lpc11u6x.c)
zephyr_sources_ifdef(CONFIG_PINMUX_MCUX            pinmux_mcux.c)
zephyr_sources_ifdef(CONFIG_PINMUX_MCUX_LPC        pinmux_mcux_lpc.c)
zephyr_sources_ifdef(CONFIG_PINMUX_RV32M1          pinmux_rv32m1.c)
zephyr_sources_ifdef(CONFIG_PINMUX_SAM0            pinmux_sam0.c)
zephyr_sources_ifdef(CONFIG_PINMUX_STM32           pinmux_stm32.c)
zephyr_sources_ifdef(CONFIG_PINMUX_XEC             pinmux_mchp_xec.c)
