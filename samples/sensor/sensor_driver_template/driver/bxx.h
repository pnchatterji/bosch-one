/**
  * @file bxx.h
  * @brief BXX driver extended public API
  * @details 
  * 
  * Support API for low-level control of BXX device 
  * 
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */

#ifndef _BXX_H_
#define _BXX_H_
#ifdef __cplusplus
extern "C" {
#endif

/*
TIP: bxx.h can be used for BXX specific type declarations and constants 
if there are a large number that cannot be accomodated directly in bxx.c
Otherwise this file can be deleted.

bxx.h can also be used to declare a Raw API for low-level control of BXX
sensor in case there are device features which cannot be covered by the
generic Zephyr Sensor API.
Note, however, that in this case bxx.h would be placed in zephyr/include/sensor
, and not alongside bxx.c in zephyr/drivers/sensor, so that it is publically visible.
*/

#ifdef __cplusplus
}
#endif

#endif //_BXX_H_
