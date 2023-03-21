#
# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(bhix60.c)
zephyr_library_sources(bhix60_conv.c) 
zephyr_library_sources(bhix60_dbg.c)
zephyr_library_sources(bhix60_pdr.c)
