# SPDX-License-Identifier: Apache-2.0
# add new LED drivers to upstream Zephyr LED library using
# zephyr_library_amend. This avoids "no sources" build warnings

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_DS28E05 eeprom_ds28e05.c)
zephyr_library_sources_ifdef(CONFIG_DUMMY_EEPROM eeprom_dummy.c)
