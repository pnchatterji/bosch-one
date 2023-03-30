#
# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
# The following source files are installed by the west.yaml
# manifest file from the BHY2 Sensor API repository on Github

zephyr_library()

zephyr_library_sources(bhy2.c)
zephyr_library_sources(bhy2_hif.c) 
zephyr_library_sources(bhy2_parse.c)
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHY2_SWIM bhy2_swim.c)
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHY2_BSEC bhy2_bsec.c)  
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHY2_KLIO bhy2_klio.c)  
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHY2_PDR bhy2_pdr.c)
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHI3 bhi3.c)
zephyr_library_sources_ifdef(CONFIG_BOSCH_BHI3 bhi3_multi_tap.c)
