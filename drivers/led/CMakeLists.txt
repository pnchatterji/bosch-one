# SPDX-License-Identifier: Apache-2.0
# add new LED drivers to upstream Zephyr LED library using
# zephyr_library_amend. This avoids "no sources" build warnings

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_FL3194 fl3194.c)
