/*
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @brief   Header for Zephyr driver for DS28e05 EEPROM via 1-wire bus.
 * Modified from original Dallas Semiconductor header for bare-metal
 * DS28e05 EEPROM driver
 */
//---------------------------------------------------------------------------
// Copyright (C) 2001 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//---------------------------------------------------------------------------

/*!
 * @addtogroup DS28E05
 * @brief This layer provides functions for supporting 1-wire EEPROM read/writes
 * @{*/



#ifndef DS28E05_H_
#define DS28E05_H_

#include <stdint.h>
#include <stdbool.h>

#define DS28E05_PAGE_MASK             			UINT8_C(0x70)
#define DS28E05_SEG_MASK              			UINT8_C(0x0E)
#define DS28E05_SEG_SIZE              			UINT8_C(8)
#define DS28E05_BYTES_PER_SEG         			UINT8_C(2)
#define DS28E05_BYTES_PER_PAGE        			UINT8_C((DS28E05_SEG_SIZE * DS28E05_BYTES_PER_SEG))
#define DS28E05_NUM_PAGES             			UINT8_C(7)

/* Commands to read and write */
#define DS28E05_READ_MEMORY_CMD   				UINT8_C(0xf0)
#define DS28E05_WRITE_MEMORY_CMD  				UINT8_C(0x55)

#define DS28E05_SKIP_ROM_CMD					UINT8_C(0xCC)
#define DS28E05_READ_ROM_CMD					UINT8_C(0x33)
#define DS28E05_MATCH_ROM_CMD					UINT8_C(0x55)
#define DS28E05_SEARCH_ROM_CMD					UINT8_C(0xF0)
#define DS28E05_RESUME_CMD						UINT8_C(0xA5)

/* Electrical Characteristics Definitions */
#define DS28E05_tPROG    						UINT8_C(16)

/* */
#define DS28E05_COMMAND_SUCCESS 				UINT8_C(0xAA)
#define DS28E05_COMMAND_FAILURE					UINT8_C(0x33)

#define DS28E05_ROM_ID_SIZE                    UINT8_C(0x08)

#endif /* DS28E05_H_ */
