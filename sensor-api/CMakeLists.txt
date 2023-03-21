# Copyright (c) 2023 Bosch SensorTec GmbH
#
# SPDX-License-Identifier: Apache-2.0


add_subdirectory_ifdef(CONFIG_BOSCH_BHY2 bhy2)
zephyr_include_directories_ifdef(CONFIG_BOSCH_BHY2 bhy2)

