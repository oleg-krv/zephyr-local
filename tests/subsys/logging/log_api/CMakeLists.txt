# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(log_core)

target_sources(app PRIVATE src/main.c src/maincxx.cxx src/mock_frontend.c
               src/mock_backend.c src/test_module.c)
