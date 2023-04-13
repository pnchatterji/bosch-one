/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 * SPDX-License-Identifier: Apache-2.0
*/
#ifndef BQ25120_H_
#define BQ25120_H_

#ifdef __cplusplus
extern "C" {
#endif


/*BQ25120 configuration registers (Read/Write)*/	
#define BQ25120_STATUS_AND_MODE_CTRL_REG             0x00
#define BQ25120_FAULT_AND_FAULT_MASK_REG             0x01
#define BQ25120_TS_CTRL_AND_FAULT_MASK_REG           0x02
#define BQ25120_FAST_CHARGE_CTRL_REG                 0x03
#define BQ25120_TERM_PRECHARGE_REG                   0x04
#define BQ25120_BATTERY_VOLTAGE_CTRL_REG             0x05
#define BQ25120_SYS_VOUT_CTRL_REG                    0x06
#define BQ25120_LOAD_AND_LDO_CTRL_REG                0x07
#define BQ25120_PUSH_BUTTON_CTRL_REG                 0x08
#define BQ25120_ILIM_BATTERY_UVLO_CTRL_REG           0x09
#define BQ25120_VOLT_BASED_BATT_MONITOR_REG          0x0A
#define BQ25120_VIN_DPM_AND_TIMERS_REG               0x0B

/*BQ25120 Constants*/
/*battery regulation voltage should be in mv range 3.6 - 4.65 V*/
#define BQ25120_BATREG_MV_MAX  4650
#define BQ25120_BATREG_MV_MIN  3600
#define BQ25120_BATTERY_VOLTAGE_UPDATE 0x80
/*ILIM and Battery UVLO control register parameters*/
#define BQ25120_RESET_ALL_REGS      0x80
#define BQ25120_ILIM_MIN_MA         50
#define BQ25120_ILIM_MAX_MA         400
#define BQ25120_BUVLO_MIN_MV        2200
#define BQ25120_BUVLO_MAX_MV        3000
#define BQ25120_BUVLO_STEP_MV       200
#define BQ25120_BUVLO_2200MV_CODE   0x06
#define BQ25120_BUVLO_DISABLE_CODE  0x07
#define BQ25120_BUVLO_DISABLE_MV    0
#define BQ25120_ILIM_BUVLO_CODE(ILIMC,BUVLOC) (((ILIMC&0x07)<<3)|(BUVLOC&0x07)) 
/*Fast Charge Control Register parameters*/
#define BQ25120_ICHARGE_MIN_MA       5
#define BQ25120_ICHARGE_MAX_MA      300
#define BQ25120_ICHARGE_DISABLE_MA  0
#define BQ25120_ICHARGE_DISABLE_CODE(ICHARGE) ((ICHARGE ==BQ25120_ICHARGE_DISABLE_MA)?1:0)
#define BQ25120_ICHARGE_RANGE_CODE(ICHARGE) ((ICHARGE >35)?1:0)
#define BQ25120_ICHARGE_CODE_0(ICHARGE) ((ICHARGE-5)/1)
#define BQ25120_ICHARGE_CODE_1(ICHARGE) ((ICHARGE-40)/10)
#define BQ25120_ICHARGE_CODE(ICHARGE) ((ICHARGE >35)?BQ25120_ICHARGE_CODE_1(ICHARGE):BQ25120_ICHARGE_CODE_0(ICHARGE))
#define BQ25120_ICHARGE_REG_CODE(ICHARGE_RANGE,ICHARGE_CODE,ICHARGE_DISABLE,HZ_MODE) \
                        (((ICHARGE_RANGE & 0x01)<<7) | \
                        ((ICHARGE_CODE & 0x1F)<<2) | \
                        ((ICHARGE_DISABLE & 0x01)<<1) | \
                        (HZ_MODE & 0x01))
#define BQ25120_HZ_MODE_BIT         0x01
#define BQ25120_CHARGER_ENABLED_BIT 0x02

/*Termination PreCharge Register parameters*/
#define BQ25120_IPRETERM_MIN_UA       500
#define BQ25120_IPRETERM_MAX_UA      37000
#define BQ25120_IPRETERM_DISABLE_UA  0
#define BQ25120_IPRETERM_DISABLE_CODE(IPRETERM) ((IPRETERM ==BQ25120_IPRETERM_DISABLE_UA)?1:0)
#define BQ25120_IPRETERM_RANGE_CODE(IPRETERM) ((IPRETERM >5000)?1:0)
#define BQ25120_IPRETERM_CODE_0(IPRETERM) ((IPRETERM-500)/500)
#define BQ25120_IPRETERM_CODE_1(IPRETERM) ((IPRETERM-6000)/1000)
#define BQ25120_IPRETERM_CODE(IPRETERM) ((IPRETERM >5000)?BQ25120_IPRETERM_CODE_1(IPRETERM):BQ25120_IPRETERM_CODE_0(IPRETERM))
#define BQ25120_IPRETERM_REG_CODE(IPRETERM_RANGE,IPRETERM_CODE,IPRETERM_DISABLE) \
                        (((IPRETERM_RANGE & 0x01)<<7) | \
                        ((IPRETERM_CODE & 0x1F)<<2) | \
                        ((IPRETERM_DISABLE & 0x01)<<1))

/*Status and Mode control register bits*/
#define BQ25120_NORMAL_OP_ENABLE                     (0 <<5)
#define BQ25120_SHIP_MODE_ENABLE                     (1 << 5)

/*Macros to evaluate BQ25120 status byte read using bq25120_get_status()*/
#define BQ25120_STATUS_READY(STATREG)                    ((STATREG & 0xA0) == 0x00)
#define BQ25120_STATUS_CHARGE_IN_PROGRESS(STATREG)       ((STATREG & 0xA0) == 0x40)
#define BQ25120_STATUS_CHARGE_DONE(STATREG)              ((STATREG & 0xA0) == 0x80)
#define BQ25120_STATUS_FAULT(STATREG)                    ((STATREG & 0xA0) == 0xA0)
#define BQ25120_STATUS_RESET_FAULT(STATREG)              ((STATREG & 0x10) == 0x10)
#define BQ25120_STATUS_TIMER_FAULT(STATREG)              ((STATREG & 0x08) == 0x08)
#define BQ25120_STATUS_VIN_DPM_ACTIVE(STATREG)           ((STATREG & 0x04) == 0x04)
#define BQ25120_STATUS_CHIP_DISABLED(STATREG)            ((STATREG & 0x02) == 0x02)
#define BQ25120_STATUS_SW_ENABLED(STATREG)               ((STATREG & 0x01) == 0x01)

/*Macros to evaluate BQ25120 fault byte read using bq25120_get_fault()*/
#define BQ25120_FAULT_VIN_UOV(FAULT)                    ((FAULT & 0x80) == 0x80)
#define BQ25120_FAULT_VIN_UV(FAULT)                     ((FAULT & 0x40) == 0x40)
#define BQ25120_FAULT_BAT_UVLO(FAULT)                   ((FAULT & 0x20) == 0x20)
#define BQ25120_FAULT_BAT_OCP(FAULT)                    ((FAULT & 0x10) == 0x10)
#define BQ25120_FAULT_VIN_OK(FAULT)                     ((FAULT & 0xA0) == 0x00)
#define BQ25120_FAULT_BAT_OK(FAULT)                     ((FAULT & 0x30) == 0x00)

/*Macros for setting/getting CD pin state in bq25120_set_cd_state() 
and bq25120_get_cd_state()*/
#define BQ25120_CD_PIN_ENABLE 1
#define BQ25120_CD_PIN_DISABLE 0
#ifdef __cplusplus
}
#endif 
#endif /* BQ25120_H_ */