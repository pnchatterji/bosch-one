/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    app30_eeprom.c
 * @date    Sep 25, 2018
 * @brief   This file contains read/write support for 1-wire EEPROM
 * NOTE: Temporary implementation. Will be replaced by calls to coines_eeprom_read/write
 * when COINES 2.0 is implemented in ZCOINES
 */

/**********************************************************************************/
/* header files */
/**********************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/*!
 *
 * @brief       : API to initiatialize APP3.0 EEPROM
 */

void app30_eeprom_init(void)
{
        //TODO: Implement on ZCOINES if required
}

/*!
 *
 * @brief       : API to read the internal ROM ID, and checks whether the read id is correct or not
 *
 */

bool app30_eeprom_romid(uint8_t *buffer)
{
        //TODO: Implement on ZCOINES if required
        return true;
}

/*!
 *
 * @brief       : API to read APP3.0 EEPROM
 */
bool app30_eeprom_read(uint16_t address, uint8_t *buffer, uint8_t length)
{
        const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(sprom));

        if (!device_is_ready(dev)) 
                return false;

        int rc = eeprom_read(dev, address, buffer,length);
        if (rc < 0) {
                return false;
        }
        return true;
}
/*!
 *
 * @brief       : API to write APP3.0 EEPROM
 */
bool app30_eeprom_write(uint8_t address, uint8_t *buffer, uint8_t length)
{
        const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(sprom));

        if (!device_is_ready(dev)) 
                return false;

        int rc = eeprom_write(dev, address, buffer,length);
        if (rc < 0) {
                return false;
        }
        return true;
}
