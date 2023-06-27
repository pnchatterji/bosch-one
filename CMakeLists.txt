# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0


zephyr_include_directories(include)
zephyr_include_directories(subsys)
add_subdirectory_ifdef(CONFIG_SENSOR sensor-api)
add_subdirectory(drivers)
add_subdirectory(lib)
add_subdirectory(subsys)
