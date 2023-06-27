# SPDX-License-Identifier: Apache-2.0

set(OPENOCD_NRF5_SUBFAMILY "nrf52")
board_runner_args(pyocd "--target=nrf52832" "--frequency=400000")
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
