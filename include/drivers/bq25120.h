/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 * SPDX-License-Identifier: Apache-2.0
 * @brief Public API of BQ25120 Driver
*/
#ifndef BQ25120_H_
#define BQ25120_H_

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * @brief bit definitions for bq25120_status_cb callback function
 */
#define BQ25120_INT_STATUS_WAKE2(ISTAT)         (ISTAT & 0x0001)  /*WAKE2 Event Signalled*/
#define BQ25120_INT_STATUS_WAKE1(ISTAT)         (ISTAT & 0x0002)  /*WAKE1 Event Signalled*/
#define BQ25120_INT_STATUS_VIN_OV(ISTAT)        (ISTAT & 0x0080)  /*VIN overvoltage fault*/
#define BQ25120_INT_STATUS_VIN_UV(ISTAT)        (ISTAT & 0x0040)  /*VIN undervoltage fault*/
#define BQ25120_INT_STATUS_BAT_UVLO(ISTAT)      (ISTAT & 0x0020)  /*BAT_UVLO fault */
#define BQ25120_INT_STATUS_BAT_OCP(ISTAT)       (ISTAT & 0x0010)  /* BAT_OCP fault*/
#define BQ25120_INT_STATUS_TS_FAULT01(ISTAT)    (ISTAT & 0x0200)  /*TS temp < TCOLD or TS temp > THOT (Charging suspended)*/
#define BQ25120_INT_STATUS_TS_FAULT10(ISTAT)    (ISTAT & 0x0400)  /*TCOOL > TS temp > TCOLD (Charging current reduced by half)*/
#define BQ25120_INT_STATUS_TS_FAULT11(ISTAT)    (ISTAT & 0x0600)  /*TWARM < TS temp < THOT (Charging voltage reduced by 140 mV)*/
#define BQ25120_INT_STATUS_TS_FAULT_OPEN(ISTAT) (ISTAT & 0x0100)  /*TS OFF fault indicated*/

/*!
 * @brief to evaluate BQ25120 status byte read using bq25120_get_status()
 */
#define BQ25120_STATUS_READY(STATREG)                    ((STATREG & 0xA0) == 0x00)
#define BQ25120_STATUS_CHARGE_IN_PROGRESS(STATREG)       ((STATREG & 0xA0) == 0x40)
#define BQ25120_STATUS_CHARGE_DONE(STATREG)              ((STATREG & 0xA0) == 0x80)
#define BQ25120_STATUS_FAULT(STATREG)                    ((STATREG & 0xA0) == 0xA0)
#define BQ25120_STATUS_RESET_FAULT(STATREG)              ((STATREG & 0x10) == 0x10)
#define BQ25120_STATUS_TIMER_FAULT(STATREG)              ((STATREG & 0x08) == 0x08)
#define BQ25120_STATUS_VIN_DPM_ACTIVE(STATREG)           ((STATREG & 0x04) == 0x04)
#define BQ25120_STATUS_CHIP_DISABLED(STATREG)            ((STATREG & 0x02) == 0x02)
#define BQ25120_STATUS_SW_ENABLED(STATREG)               ((STATREG & 0x01) == 0x01)

/*!
 * @brief to evaluate BQ25120 fault byte read using bq25120_get_fault()
 */
#define BQ25120_FAULT_VIN_UOV(FAULT)                    ((FAULT & 0x80) == 0x80)
#define BQ25120_FAULT_VIN_UV(FAULT)                     ((FAULT & 0x40) == 0x40)
#define BQ25120_FAULT_BAT_UVLO(FAULT)                   ((FAULT & 0x20) == 0x20)
#define BQ25120_FAULT_BAT_OCP(FAULT)                    ((FAULT & 0x10) == 0x10)
#define BQ25120_FAULT_VIN_OK(FAULT)                     ((FAULT & 0xA0) == 0x00)
#define BQ25120_FAULT_BAT_OK(FAULT)                     ((FAULT & 0x30) == 0x00)

/*!
 * @brief callback function type for register reset callback
 */
typedef void (*bq25120_reset_cb_t)(const struct device *dev);

/*!
 * @brief callback function type for register int status callback
 * 	Use BQ25120_INT_STATUS_XXX macros to evaluate individual bits of int_status
 */
typedef void (*bq25120_int_status_cb_t)(const struct device *dev,uint16_t int_status);

/*!
 * @brief Register a callback function for getting  bq25120 status when the
 * device signals a status/fault event via INT pin
 * @param[in] status_cb: int status callback function 
 */
void bq25120_reg_status_cb(const struct device *dev, bq25120_int_status_cb_t int_status_cb);

/*!
 * @brief Register a callback function for getting informed when bq25120
 * device signals a RESET event via RESET pin
 * @param[in] reset_cb: reset callback function 
 */
void bq25120_reg_reset_cb(const struct device *dev, bq25120_reset_cb_t reset_cb);

/*!
 * @brief Register a callback function for getting  bq25120 status when the
 * device signals a status/fault event via INT pin
 * Use BQ25120_INT_STATUS_XXX macros to evaluate individual bits of int_status
 * in callback function
 * @param[in] status_cb: status callback function 
 */
void bq25120_reg_int_status_cb(const struct device *dev, bq25120_int_status_cb_t int_status_cb);

/*!
 * @brief Set Load Switch and LDO Voltage 
 * @param[in] ldov: ldo voltage in mv - 3300mv (3.3V)
 * @return
 * 	0 		:no error
 * -EINVAL :parameters out of range
 * -EIO 	:comm error
 */
 
int8_t bq25120_set_ls_ldo(const struct device *dev, uint32_t ldov);
/*!
 * @brief Set SYS Output Voltage (VOUT)
 * @param[in] vout: output voltage in mv - 3300mv (3.3V)
 * @return
 * 	0 		:no error
 * -EINVAL :parameters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_sys_vout(const struct device *dev, uint32_t vout);

/*!
 * @brief Set Operation Mode of BQ25120 to Ship Mode
 */
int bq25120_set_op_mode_ship(const struct device *dev);

/*!
 * @brief Set Operation Mode of BQ25120 to Normal Mode
 */
int bq25120_set_op_mode_normal(const struct device *dev);

/*!
 * @brief Reset all BQ25120 register to their default values.
 */
int bq25120_device_reset(const struct device *dev);

/*!
 * @brief Get BQ25120 Status
 * @param[out] status : status bits as per BQ25120 specifications ch9.6.1
 * 	Use BQ25120_STATUS_XXX macros to evaluate individual bits
 */
int bq25120_get_status(const struct device *dev, uint8_t *status);

/*!
 * @brief Get BQ25120 Fault
 * @param[out] status : fault bits as per BQ25120 specifications ch9.6.2
 * 	Use BQ25120_FAULT_XXX macros to evaluate individual bits
 */
int bq25120_get_fault(const struct device *dev, uint8_t *fault);

/*!
 * @brief Enable/Disable High Impedance Mode.
 * @param[in] status: 	0:	disable hz_mode 
 * 						>0:	enable hz_mode
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_set_hz_mode_status(const struct device *dev, uint8_t status);

/*!
 * @brief Get High Impedance Mode status.
 * @param[out] status: 	0: hz_mode disabled 
 * 						1: hz_mode enabled
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_get_hz_mode_status(const struct device *dev, uint8_t *status);

/*!
 * @brief Enable/Disable Charger.
 * @param[in] status: 	0:	disable charger 
 * 						>0:	enable charger
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_set_charger_status(const struct device *dev, uint8_t status);

/*!
 * @brief Get Charger Enabled status.
 * @param[out] status: 	0: charger disabled 
 * 						1: charger enabled
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_get_charger_status(const struct device *dev, uint8_t *status);

/*!
 * @brief Set pre-charge/termination charging current.
 * @param[in] ipreterm: Charging current in uA (500uA to 37000uA (37mA), Disable Termination:0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_ipreterm(const struct device *dev, uint32_t ipreterm);

/*!
 * @brief Set charging current.
 * @param[in] icharge: Charging current in mA (5-300mA, Disable Charger:0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_icharge(const struct device *dev, uint32_t icharge);

/*!
 * @brief Set input current limit and the Battery undervoltage-lockout.
 * @param[in] ilim: input Input current limit of device in mA. (50-400 mA)
 * @param[in] buvlo: Battery under-voltage lockout in mV (2.2-3 V, Disable UVLO :0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_ilim_bat_uvlo(const struct device *dev, uint32_t ilim, uint32_t buvlo);

/*!
 * @brief Set battery regulation voltage.
 * @param[in] bat_reg_mv: regulation voltage in mv range 3.6 - 4.65 V
 * @return 0: no error
 * 			-EINVAL: voltage out of range
 * 			-EIO: comm error
 */
int8_t bq25120_set_bat_reg_mv(const struct device *dev, const uint32_t bat_reg_mv);

/*!
 * @brief get current battery regulation voltage.
 * @param[out] bat_reg_mv: regulation voltage in mv
 * @return 0: no error
 * 			-EIO: comm error
 */
int8_t bq25120_get_bat_reg_mv(const struct device *dev, uint32_t *bat_reg_mv);

/*!
 * @brief Read current battery voltage and state-of-charge.
 * @param d ev: Zephyr device driver pointer 
 * @param  bat_mv: battery voltage in mv
 * @param  bat_soc: battery state-of-charge in %
 * @return
 *  0: no error
 *  -EIO: communication error
 */
int8_t bq25120_get_battery_mv_soc(const struct device *dev, uint32_t *bat_mv, uint32_t *bat_soc);

/*!
 * @brief Set BQ25120 chip disable (CD) pin state 
 * Only Battery:
 *		Drive CD low to place device in High-Z mode (low power consumption, I2C disabled)
 *		Drive CD high for Active Battery mode
 * VIN Valid: 
 *		Drive CD low to enable charging 
 *		Drive CD high to disable charging
 * @param  state: desired pin state 0 or 1
 * @return
 *  0: no error
 *  -EIO: GPIO error
 */
int bq25120_set_cd_state(const struct device *dev, uint8_t state);
/*!
 * @brief  Macros for setting/getting CD pin state in bq25120_set_cd_state() 
 * and bq25120_get_cd_state()
 */
#define BQ25120_CD_PIN_ENABLE 1
#define BQ25120_CD_PIN_DISABLE 0

/*!
 * @brief Get BQ25120 chip disable (CD) pin state 
 * @return  0 pin is in low state 
 * 			1 pin is in high state
 * 			-EIO GPIO error
 */
int bq25120_get_cd_state(const struct device *dev);

#ifdef __cplusplus
}
#endif 
#endif /* BQ25120_H_ */