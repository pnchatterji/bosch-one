# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BQ25120 bq25120.c)
zephyr_library_sources_ifdef(CONFIG_BAT_LEVEL_ADC bat_level_adc.c)
