#
# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(bhy2.c)
zephyr_library_sources(bhy2_hif.c) 
zephyr_library_sources(bhy2_parse.c)
zephyr_library_sources(bhy2_swim.c)
zephyr_library_sources(bhy2_bsec.c)  
zephyr_library_sources(bhy2_klio.c)  
zephyr_library_sources(bhy2_pdr.c)
