# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# The nrfe source directory can be override through the definition of the NRFE_DIR symbol
# during the invocation of the build system
if(NOT DEFINED NRFE_DIR)
  set(NRFE_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/nrfe CACHE PATH "nrfe Directory")
endif()

set(INC_DIR ${NRFE_DIR}/include)

zephyr_include_directories(${NRFE_DIR})
zephyr_include_directories(${INC_DIR})
