:orphan:

.. _zephyr_3.0:

Zephyr 3.0.0 (Working draft)
############################

We are pleased to announce the release of Zephyr RTOS version 3.0.0.



The following sections provide detailed lists of changes by component.

Security Vulnerability Related
******************************

The following CVEs are addressed by this release:


Known issues
************

You can check all currently known issues by listing them using the GitHub
interface and listing all issues with the `bug label
<https://github.com/zephyrproject-rtos/zephyr/issues?q=is%3Aissue+is%3Aopen+label%3Abug>`_.

API Changes
***********

Changes in this release
=======================

* Following functions in UART Asynchronous API are using microseconds to represent
  timeout instead of milliseconds:
  * :c:func:`uart_tx`
  * :c:func:`uart_rx_enable`

* Replaced custom LwM2M :c:struct:`float32_value` type with a native double type.

* Added function for getting status of USB device remote wakeup feature.

* Added ``ranges`` and ``dma-ranges`` as invalid property to be used with DT_PROP_LEN()
  along ``reg`` and ``interrupts``.

Removed APIs in this release
============================

* The following Kconfig options related to radio front-end modules (FEMs) were
  removed:

  * ``CONFIG_BT_CTLR_GPIO_PA``
  * ``CONFIG_BT_CTLR_GPIO_PA_PIN``
  * ``CONFIG_BT_CTLR_GPIO_PA_POL_INV``
  * ``CONFIG_BT_CTLR_GPIO_PA_OFFSET``
  * ``CONFIG_BT_CTLR_GPIO_LNA``
  * ``CONFIG_BT_CTLR_GPIO_LNA_PIN``
  * ``CONFIG_BT_CTLR_GPIO_LNA_POL_INV``
  * ``CONFIG_BT_CTLR_GPIO_LNA_OFFSET``
  * ``CONFIG_BT_CTLR_FEM_NRF21540``
  * ``CONFIG_BT_CTLR_GPIO_PDN_PIN``
  * ``CONFIG_BT_CTLR_GPIO_PDN_POL_INV``
  * ``CONFIG_BT_CTLR_GPIO_CSN_PIN``
  * ``CONFIG_BT_CTLR_GPIO_CSN_POL_INV``
  * ``CONFIG_BT_CTLR_GPIO_PDN_CSN_OFFSET``

  This FEM configuration is hardware description, and was therefore moved to
  :ref:`devicetree <dt-guide>`. See the :dtcompatible:`nordic,nrf-radio`
  devicetree binding's ``fem`` property for information on what to do instead
  on the Nordic open source controller.

* Removed Kconfig option ``CONFIG_USB_UART_CONSOLE``.
  Option ``CONFIG_USB_UART_CONSOLE`` was only relevant for console driver
  when CDC ACM UART is used as backend. Since the behavior of the CDC ACM UART
  is changed so that it more closely mimics the real UART controller,
  option is no longer necessary.

* Removed Kconfig option ``CONFIG_OPENOCD_SUPPORT`` in favor of
  ``CONFIG_DEBUG_THREAD_INFO``.

Deprecated in this release
==========================

* :c:macro:`USBD_CFG_DATA_DEFINE` is deprecated in favor of utilizing
  :c:macro:`USBD_DEFINE_CFG_DATA`

Stable API changes in this release
==================================

New APIs in this release
========================

* Serial

  * Added new APIs to support datum wider than 8-bit.

    * :kconfig:`CONFIG_UART_WIDE_DATA` is added to enable this new APIs.

    * Following functions, mirroring similar functions for 8-bit datum,
      are added:

      * :c:func:`uart_tx_u16` to send a given number of datum from buffer.

      * :c:func:`uart_rx_enable_u16` to start receiving data.

      * :c:func:`uart_rx_buf_rsp_u16` to set buffer for receiving data
        in response to ``UART_RX_BUF_REQUEST`` event.

      * :c:func:`uart_poll_in_u16` to poll for input.

      * :c:func:`uart_poll_out_u16` to output datum in polling mode.

      * :c:func:`uart_fifo_fill_u16` to fill FIFO with data.

      * :c:func:`uart_fifo_read_u16` to read data from FIFO.


Kernel
******


Architectures
*************

* ARC


* ARM

  * AARCH32


  * AARCH64


* x86

* Xtensa

  * Introduced a mechanism to automatically figure out which scratch registers
    are used for internal code, instead of hard-coding. This is to accommodate
    the configurability of the architecture where some registers may exist in
    one SoC but not on another one.

  * Added coredump support for Xtensa.

  * Added GDB stub support for Xtensa.

Bluetooth
*********

* Audio

* Host

  * The :kconfig:`CONFIG_BT_SETTINGS_CCC_STORE_ON_WRITE` is enabled by default.
    Storing CCC right after it's written reduces risk of inconsistency of CCC values between bonded peers.

* Mesh

* Bluetooth LE split software Controller

* HCI Driver

Boards & SoC Support
********************

* Added support for these SoC series:


* Removed support for these SoC series:


* Made these changes in other SoC series:

  * stm32h7: Added SMPS support
  * stm32u5: Enabled TF-M

* Changes for ARC boards:


* Added support for these ARM boards:

  * OLIMEX-STM32-H405
  * ST Nucleo G031K8
  * ST Nucleo H7A3ZI Q
  * ST STM32G081B Evaluation

* Added support for these ARM64 boards:


* Removed support for these ARM boards:


* Removed support for these X86 boards:


* Made these changes in other boards:


* Added support for these following shields:


Drivers and Sensors
*******************

* ADC

  * Added support for stm32u5 series
  * stm32: Added shared IRQ support

* Bluetooth


* CAN


* Clock Control


* Console


* Counter

  * stm32: Added timer based counter driver (stm32f4 only for now).

* DAC

  * Added support for stm32u5 series

* Disk

  * stm32 sdmmc: Converted from polling to IT driven mode and added Hardware
    Flow Control option

* Display


* Disk


* DMA


* EEPROM


* Entropy

  * Added support for stm32u5 series

* ESPI


* Ethernet

  * Added support for Synopsys DesignWare MAC driver with implementation
    on stm32h7 series.
  * stm32 (hal based): Added promiscuous mode support
  * stm32 (hal based): Added PTP L2 timestamping support

* Flash

  * stm32g0: Added Dual Bank support
  * stm32_qspi: General enhancement (Generation of the reset pulse for SPI-NOR memory,
    Usage of 4IO for read / write (4READ/4PP), Support for different QSPI banks,
    Support for 4B addressing on spi-nor)

* GPIO


* Hardware Info


* I2C


* I2S


* IEEE 802.15.4


* Interrupt Controller


* LED


* LoRa


* MEMC

  *  Added support for stm32f7 series

* Modem


* Pinctrl

  * Added support for STM32

* PWM

  * stm32: DT bindings: `st,prescaler` property was moved from pwm
    to parent timer node.
  * stm32: Implemented PWM capture API

* Sensor

  * Added L5 die temperature sensor

* Serial

  * stm32: Implemented half-duplex option.

* SPI

  * stm32: Implemented Frame format option (TI vs Motorola).

* Timer

  * stm32 lptim: Added support for stm32h7

* USB

  * Added support for stm32u5 series (OTG full speed)

* Watchdog

  * Added support for stm32u5 series (Independent and Window)

* WiFi


Networking
**********

* CoAP:


* DHCPv4:


* DNS:


* HTTP:


* IPv4:


* LwM2M:


* Misc:


* OpenThread:


* Socket:


* TCP:


* TLS:


USB
***


Build and Infrastructure
************************

* Build system

  * New CMake extension functions:

    * ``dt_alias()``
    * ``target_sources_if_dt_node()``

  * The following CMake extension functions now handle devicetree aliases:

    * ``dt_node_exists()``
    * ``dt_node_has_status()``
    * ``dt_prop()``
    * ``dt_num_regs()``
    * ``dt_reg_addr()``
    * ``dt_reg_size()``

* Devicetree

  * Support for the devicetree compatible ``ti,ina23x`` has been removed.
    Instead, use :dtcompatible:`ti,ina230` or :dtcompatible:`ti,ina237`.

* West (extensions)


Libraries / Subsystems
**********************

* Disk


* Management

  * Fixed the mcumgr SMP protocol over serial not adding the length of the CRC16 to packet length.
  * Kconfig option OS_MGMT_TASKSTAT is now disabled by default.

* CMSIS subsystem


* Power management


* Logging


* Shell


* Storage


* Task Watchdog


* Tracing


* Debug

* OS


HALs
****

* STM32

  * stm32cube/stm32wb and its lib: Upgraded to version V1.12.1
  * stm32cube/stm32mp1: Upgraded to version V1.5.0
  * stm32cube/stm32u5: Upgraded to version V1.0.2

MCUboot
*******

* Fixed serial recovery skipping on nrf5340.
* Fixed issue which caused that progressive's erase feature was off although was selected by Kconfig (introduced by #42c985cead).
* Added check of reset address in incoming image validation phase, see ``CONFIG_MCUBOOT_VERIFY_IMG_ADDRESS``.
* Allow image header bigger than 1 KB for encrypted images.
* Support Mbed TLS 3.0.
* stm32: watchdog support.
* many documentation improvements.
* Fixed deadlock on cryptolib selectors in Kconfig.
* Fixed support for single application slot with serial recovery.
* Added various hooks to be able to change how image data is accessed, see ``CONFIG_BOOT_IMAGE_ACCESS_HOOKS``.
* Added custom commands support in serila recovery (PERUSER_MGMT_GROUP): storage erase ``CONFIG_BOOT_MGMT_CUSTOM_STORAGE_ERASE``, custo image status ``CONFIG_BOOT_MGMT_CUSTOM_IMG_LIST``.
* Added support for direct image upload, see ``CONFIG_MCUBOOT_SERIAL_DIRECT_IMAGE_UPLOAD`` in serial recovery.

Trusted Firmware-m
******************


Documentation
*************


Tests and Samples
*****************

* Drivers: clock_control: Added test suite for stm32 (u5, h7).

Issue Related Items
*******************

These GitHub issues were addressed since the previous 2.7.0 tagged
release:
