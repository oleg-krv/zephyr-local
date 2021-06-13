# ST Microelectronics LSM6DS3 6-axis IMU sensor driver
#
# Copyright (c) 2019 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LSM6DS3            lsm6ds3.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3_SENSORHUB  lsm6ds3_shub.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3_TRIGGER    lsm6ds3_trigger.c)

zephyr_library_include_directories(../stmemsc)
