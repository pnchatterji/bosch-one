#
# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
# The following source files are installed by the west.yaml
# manifest file from the BMI270 Sensor API repository on Github

#TODO: support the variants as well, by adding ifdef configs for each variant

zephyr_library()

zephyr_library_sources(bmi2.c)
zephyr_library_sources(bmi270.c)

#for COINES apps, include common from examples as well
#as these are required by the COINES sensor examples

zephyr_library_sources_ifdef(CONFIG_COINES bmi270_examples/common/common.c)
zephyr_include_directories_ifdef(CONFIG_COINES bmi270_examples/common)