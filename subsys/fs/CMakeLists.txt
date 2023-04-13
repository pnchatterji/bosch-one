# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FILE_SYSTEM_FLOGFS)
zephyr_library()

zephyr_library_sources(  
    flogfs.c
    flogfs_fs.c   
)
endif()