.. _modbus-rtu-server-sample:

Modbus RTU Server Sample
########################

Overview
********

This is a simple application demonstrating a Modbus RTU server implementation
in Zephyr RTOS.

Requirements
************

This sample has been tested with the nRF52840-DK and FRDM-K64F boards,
but it should work with any board that has a free UART interface.
Additionally the board should have three LEDs.

RTU server example is running on an evaluation board. Client is running
on a PC or laptop.
In addition to the evaluation board, an USB to RS-485 bus adapter and
a RS-485 shield are required. The shield converts UART TX, RX signals to RS-485.
An Arduino header compatible shield like `joy-it RS-485 shield for Arduino`_
can be used. This example uses DE signal, which is controlled by pin D9
on the JOY-IT shield. For other shields, ``de-gpios`` property must be adapted
or removed in the application overlay file
:zephyr_file:`samples/subsys/modbus/rtu_server/app.overlay`

The USB to RS-485 adapter connects to the USB port of a computer.
The two A+, B- lines should be connected to the RS-485 shield.

The description of this sample uses `PyModbus`_ (Pymodbus REPL).
The user can of course try out other client implementations with this sample.

Building and Running
********************

This sample can be found under
:zephyr_file:`samples/subsys/modbus/rtu_server` in the Zephyr tree.

The following commands build and flash RTU server sample.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/modbus/rtu_server
   :board: nrf52840dk_nrf52840
   :goals: build flash
   :compact:

On the client side, PC or laptop, the following command connects PyModbus
to the RTU server.

.. code-block:: console

   # pymodbus.console serial --port /dev/ttyUSB0 --baudrate 19200
                             --bytesize 8 --parity N --stopbits 2

The LEDs on the board are controlled by Modbus commands FC01, FC05, FC15.
For example, to set LED0 on use FC01 command (write_coil).

.. code-block:: console

   > client.write_coil address=0 value=1 unit=1

Client should type_and_id successful communication and LED0 should light.

.. code-block:: console

   {
       "address": 0,
       "value": true
   }

To set LED0 off but LED1 and LED2 on use FC15 command (write_coils).

.. code-block:: console

   > client.write_coils address=0 values=0,1,1 unit=1

To read LED0, LED1, LED2 state FC05 command (read_coils) can be used.

.. code-block:: console

   > client.read_coils address=0 count=3 unit=1
   {
       "bits": [
           false,
           true,
           true,
           false,
           false,
           false,
           false,
           false
       ]
   }

It is also possible to write and read the holding registers.
This however does not involve any special interaction
with the peripherals on the board yet.

To write single holding registers use FC06 command (write_register),

.. code-block:: console

   > client.write_register address=0 value=42 unit=1

or FC16 command (write_registers).

.. code-block:: console

   > client.write_registers address=0 values=42,42,42 unit=1

To read holding registers use FC03 command (read_holding_registers).

.. code-block:: console

   > client.read_holding_registers address=0 count=3 unit=1
   {
       "registers": [
           42,
           42,
           42
       ]
   }

.. _`joy-it RS-485 shield for Arduino`: https://joy-it.net/en/products/ARD-RS485
.. _`PyModbus`: https://github.com/riptideio/pymodbus
