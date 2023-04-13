# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_W25_SPIM_NAND
    w25_common.c
    w25_spim_nand.c
    w25m02gw.c
    w25n01gw.c
    w25n02jw.c
)
