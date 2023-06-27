/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 * SPDX-License-Identifier: Apache-2.0
*/
#ifndef BQ25120_DEFS_H_
#define BQ25120_DEFS_H_

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
#define BQ25120_IPRETERM_ENABLE_CODE(IPRETERM) ((IPRETERM >BQ25120_IPRETERM_DISABLE_UA)?1:0)
#define BQ25120_IPRETERM_RANGE_CODE(IPRETERM) ((IPRETERM >5000)?1:0)
#define BQ25120_IPRETERM_CODE_0(IPRETERM) ((IPRETERM-500)/500)
#define BQ25120_IPRETERM_CODE_1(IPRETERM) ((IPRETERM-6000)/1000)
#define BQ25120_IPRETERM_CODE(IPRETERM) ((IPRETERM >5000)?BQ25120_IPRETERM_CODE_1(IPRETERM):BQ25120_IPRETERM_CODE_0(IPRETERM))
#define BQ25120_IPRETERM_REG_CODE(IPRETERM_RANGE,IPRETERM_CODE,IPRETERM_ENABLE) \
                        (((IPRETERM_RANGE & 0x01)<<7) | \
                        ((IPRETERM_CODE & 0x1F)<<2) | \
                        ((IPRETERM_ENABLE & 0x01)<<1))

/*SYS VOUT Register parameters*/
#define BQ25120_SYS_VOUT_MIN_MV 1100
#define BQ25120_SYS_VOUT_MAX_MV 3300
#define BQ25120_SYS_DISABLE_MV 0  
#define BQ25120_SYS_OUT_ENABLE_CODE(VOUT) ((VOUT >BQ25120_SYS_DISABLE_MV)?1:0)
#define BQ25120_SYS_SEL_CODE(VOUT) ((VOUT<1300)?0x00:(VOUT<2800)?0x01:0x11)
#define BQ25120_SYS_SEL_OFST(VOUT) ((VOUT<1300)?1100:(VOUT<2800)?1300:1800)
#define BQ25120_SYS_VOUT_CODE(VOUT) ((VOUT -BQ25120_SYS_SEL_OFST(VOUT))/100)
#define BQ25120_SYS_VOUT_REG_CODE(SYS_OUT_ENABLE,SYS_SEL_CODE,SYS_VOUT_CODE) \
                        (((SYS_OUT_ENABLE & 0x01)<<7) | \
                        ((SYS_SEL_CODE & 0x03)<<5)    | \
                        ((SYS_VOUT_CODE & 0x0F)<<1))
/*LS LDO Register parameters*/
#define  BQ25120_LDO_DISABLE_MV 0 
#define  BQ25120_LDO_MIN_MV 800
#define  BQ25120_LDO_MAX_MV 3300
#define  BQ25120_LS_DISABLE_CODE 0
#define  BQ25120_LS_ENABLE_CODE(ldov) ((ldov>BQ25120_LDO_DISABLE_MV)?1:0)
#define  BQ25120_LDO_CODE(ldov) ((ldov-800)/100)
#define  BQ25120_LS_LDO_REG_CODE(LS_ENAB,LDO_CODE,MRR_VIN) \
                        (((LS_ENAB & 0x01)<<7) | \
                        ((LDO_CODE & 0x1F)<<2)   | \
                        (MRR_VIN & 0x01))
/*MR Pushbutton Register parameters*/
#define  BQ25120_WAKE1_CODE(W1MS) ((W1MS>50)?0:1)
#define  BQ25120_WAKE2_CODE(W2MS) ((W2MS>1000)?0:1)
#define  BQ25120_RESET_CODE(RSEC) ((RSEC>10)?0x11:((RSEC>8)?0x10:((RSEC>4)?0x10:0x00)))
#define  BQ25120_MRREC_CODE(MRREC) ((MRREC>0)?1:0)
#define  BQ25120_PGMR_CODE(PGMR)   ((PGMR>0)?1:0)
#define  BQ25120_PB_CTRL_CODE(W1CODE,W2CODE,MRRCODE,RSTCODE,PGMRCODE) \
                    (W1CODE <<7)|(W2CODE <<6)|(MRRCODE<<5)|(RSTCODE<<3)|(PGMRCODE<2)
/*Status and Mode control register bits*/
#define BQ25120_NORMAL_OP_ENABLE                     (0 <<5)
#define BQ25120_SHIP_MODE_ENABLE                     (1 << 5)
/*TS Control &FAULTS Register parameters*/
#define  BQ25120_TS_FAULT_CODE(TSEN,ENINT,WAKEM,RSTM,TIMM) \
        (((TSEN&0x01)<<7) | ((ENINT&0x01)<<3) |            \
        ((WAKEM&0x01)<<2) | ((RSTM&0x01)<<1) | (TIMM&0x01))
#ifdef __cplusplus
}
#endif 
#endif /* BQ25120_DEFS_H_ */