# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_BHIX60 bhix60)

zephyr_include_directories_ifdef(CONFIG_BHIX60 bhix60)