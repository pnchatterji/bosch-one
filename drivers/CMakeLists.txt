# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory_ifdef(CONFIG_SENSOR temp)
add_subdirectory_ifdef(CONFIG_FLASH_HAS_DRIVER_ENABLED flash)
add_subdirectory_ifdef(CONFIG_BATTERY_SENSE  bat)
add_subdirectory_ifdef(CONFIG_LED  led)

