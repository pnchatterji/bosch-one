/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Note: the code in this driver is based on the BQ2512x Design Specifications 
 * Ref SLUSBZ9C –AUGUST 2015–REVISED SEPTEMBER 2016
 * All section references in the code refer to this version of the specifications
 */

#define DT_DRV_COMPAT ti_bq25120

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
LOG_MODULE_REGISTER(bq25120, CONFIG_SENSOR_LOG_LEVEL);

#include "bq25120_defs.h"
#include <drivers/bq25120.h>
/* 
Battery SOC Look-Up Table, based on example in BQ25120 specs ch 9.3.4
TODO: Make this configurable via DTS
        VBMON%	SOC%
		91      100
		90		98
		89		96
		88		95
		87		85
		86		65
		85		50
		84		35
		83		25
		82		15
		81		0
*/
uint8_t bat_soc_lut[] ={0,15,25,35,50,65,85,95,96,98,100};
uint8_t bat_soc_lut_ofst =81;

//NOTE: configured values can be different from current values
//configured values are from DTS
//current values are what was read from device registers at the last reading
//which may or may not be fresh
struct bq25120_data {
	uint32_t battery_mv;			//current battery voltage in mv
	uint32_t bat_reg_mv;			//current battery regulation voltage in mv
	uint32_t battery_soc;			//current battery state-of-charge in %
	const struct device *dev;		//bq25120 device pointer for cb
	struct gpio_callback int_gpio_cb; //interrupt callback structure for INT pin
	struct gpio_callback rst_gpio_cb; //interrupt callback structure for RESET pin
	bq25120_int_status_cb_t status_cb;	  //callback function for int status
	bq25120_reset_cb_t  reset_cb;	  //callback function for reset event
};

struct bq25120_cfg {
	struct i2c_dt_spec bus;			 // i2c device used by bq25120
	struct gpio_dt_spec cd_gpios;	 // gpio pin connected to CD pin of bq25120
	struct gpio_dt_spec lsctrl_gpios;// gpio pin connected to LSCTRL pin of bq25120
	struct gpio_dt_spec	int_gpios;	 // gpio pin connected to BQ25120 INT pin, if any
	struct gpio_dt_spec  reset_gpios;// gpio pin connected to BQ25120 RESET pin, if any
	uint32_t sys_vout_mv;			 // Output voltage of SYS in mv - min 1100, max 3300, 0 to disable (0,1.1-3.3V)
	uint32_t ls_ldo_mv;				 // Output voltage of LS/LDO in mv - min 800, max 3300, 0 to disable (0,0.8-3.3V)
	uint32_t bat_reg_mv;			 // configured regulation voltage of battery in mv (3.6-4.65V)
	uint32_t ilim_ma;				 // configured input current limit of device in mA (50-400mA)
	uint32_t bat_uvlo_mv;			 // configured Battery under-voltage lockout in mV (2.2-3 V, Disable UVLO:0)
	uint32_t ichagre_ma;			 // configured Charging current in mA (5-300mA, Disable Charger:0)
	uint32_t ipreterm_ua;			 // configured pre-charging current/terminating current in uA 
									 // (500uA to 37000uA (37mA), Disable Termination:0)
	uint32_t wake1_ms;				 // Wake1 time setting in ms - 50 or 500 ms, 0 to disable 
	uint32_t wake2_ms;				 // Wake2 time setting in ms - 1000 or 1500 ms, 0 to disable 
	uint32_t reset_s;			 	 // Reset time setting in Sec - 4, 8, 10 or 14, 0 to disable 
	uint8_t	mrrec;					 // After Reset, device enters 0 - Ship mode, 1 - Hi-Z Mode
	uint8_t	mrreset_vin;			 // Reset when Reset time expires 0-Always 1-When Vin is healthy
	uint8_t	timer_int_en;			 // Enable interrupt signalling of timer events
    uint8_t	ts_int_en;		         // Enable interrupt signalling of temperature sensor (TS) events
	uint8_t charge_int_en;			 // Enable interrupt signalling of charging events
	uint8_t pg_mr;					 // Use PG Output as 0-Power Good Status, 1-MR Input Status

};

static int bq25120_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
	int status;
	const struct bq25120_cfg *cfg = dev->config;
	status = i2c_burst_read_dt(&cfg->bus,reg_addr,val,1);
	if (status < 0) {
		LOG_ERR("Unable to read register %x",reg_addr);
		return -EIO;
	}
	return 0;
}


static int bq25120_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	int status = 0;
	const struct bq25120_cfg *cfg = dev->config;
	status = i2c_reg_write_byte_dt(&cfg->bus, reg_addr,value);
	if (status < 0) {
		LOG_ERR("Failed to write into register %x",reg_addr);
		return -EIO;
	}

	return 0;
}

/*!
 * @brief Get BQ25120 chip disable (CD) pin state 
 * @return  0 pin is in low state 
 * 			1 pin is in high state
 * 			-EIO GPIO error
 */
int bq25120_get_cd_state(const struct device *dev)
{
	const struct bq25120_cfg *cfg = dev->config;
	return gpio_pin_get_dt(&cfg->cd_gpios);
}


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
int bq25120_set_cd_state(const struct device *dev, uint8_t state)
{
	const struct bq25120_cfg *cfg = dev->config;
	int status = gpio_pin_set_dt(&cfg->cd_gpios, state);
	if (status < 0) {
		LOG_ERR("Unable to set CD pin status");
		return status;
	}
	k_msleep(20);
	return 0;
}

/*!
 * @brief Read current battery voltage and state-of-charge.
 * @param d ev: Zephyr device driver pointer 
 * @param  bat_mv: battery voltage in mv
 * @param  bat_soc: battery state-of-charge in %
 * @return
 *  0: no error
 *  -EIO: communication error
 */
int8_t bq25120_get_battery_mv_soc(const struct device *dev, 
				uint32_t *bat_mv,
				uint32_t *bat_soc
)
{
    uint8_t reg_vbmon = 0;
	struct bq25120_data *devdata = dev->data;
	//Save CD initial state
	uint8_t cd_entry_state = bq25120_get_cd_state(dev); 
	//Push CD high to get an accurate reading of the battery voltage (BQ25120 specs ch 9.3.4)
	bq25120_set_cd_state(dev,1); 
	if(bq25120_reg_write(dev, BQ25120_VOLT_BASED_BATT_MONITOR_REG, BQ25120_BATTERY_VOLTAGE_UPDATE))
		return -EIO;
	k_msleep(2);//value is available 2 ms after initiating a read as per BQ25120 specs ch 9.3.4
	if(bq25120_reg_read(dev, BQ25120_VOLT_BASED_BATT_MONITOR_REG, &reg_vbmon))
		return -EIO;
	uint8_t vrange = (reg_vbmon & 0x60) >> 5;
	uint8_t vth = (reg_vbmon & 0x1C) >> 2;
	if(vrange ==0 && vth ==0)
	{
		*bat_soc = 0;
		*bat_mv = 0;
	}
	else
	{
		//simplification of the vbmon table in bq25120 specs ch9.6.11
		uint8_t vbat_per=0;
		int	soc_lut_idx =0;
		vbat_per = 60 +10*vrange;
		vbat_per += vth+1; 
		*bat_mv = (uint32_t)(vbat_per * devdata->bat_reg_mv) / 100;
		//map battery voltage % to soc % using LUT
		soc_lut_idx = vbat_per - bat_soc_lut_ofst;
		if(vbat_per < 0)
			*bat_soc = 0;
		else if (soc_lut_idx >= sizeof(bat_soc_lut))
			*bat_soc =100;
		else
			*bat_soc = bat_soc_lut[soc_lut_idx]; 
	}

	//Put CD back to its original state (assuming it was read without errors i.e. non-negative)
	if(cd_entry_state >=0)
	{
		bq25120_set_cd_state(dev,cd_entry_state);
	}
    return 0; 
}

/*!
 * @brief Reset all BQ25120 register to their default values.
 */
int bq25120_device_reset(const struct device *dev)
{
	uint8_t reg_data;
	if(bq25120_reg_read(dev, BQ25120_ILIM_BATTERY_UVLO_CTRL_REG, &reg_data))
		return -EIO;
	reg_data |= BQ25120_RESET_ALL_REGS;
	return bq25120_reg_write(dev, BQ25120_ILIM_BATTERY_UVLO_CTRL_REG, reg_data); 
}

/*!
 * @brief Set Operation Mode of BQ25120 to Ship Mode
 */
int bq25120_set_op_mode_ship(const struct device *dev)
{
	bq25120_set_cd_state(dev,1);
	int ret = bq25120_reg_write(dev, BQ25120_STATUS_AND_MODE_CTRL_REG,BQ25120_SHIP_MODE_ENABLE); 
	bq25120_set_cd_state(dev,0);
	return ret;
}

/*!
 * @brief Set Operation Mode of BQ25120 to Normal Mode
 */
int bq25120_set_op_mode_normal(const struct device *dev)
{
	return bq25120_reg_write(dev, BQ25120_STATUS_AND_MODE_CTRL_REG,BQ25120_NORMAL_OP_ENABLE); 
}

/*!
 * @brief Get BQ25120 Status
 * @param[out] status : status bits as per BQ25120 specifications ch9.6.1
 * 	Use BQ25120_STATUS_XXX macros to evaluate individual bits
 */
int bq25120_get_status(const struct device *dev, uint8_t *status)
{
	return bq25120_reg_read(dev, BQ25120_STATUS_AND_MODE_CTRL_REG,status); 
}

/*!
 * @brief Get BQ25120 Fault
 * @param[out] status : fault bits as per BQ25120 specifications ch9.6.2
 * 	Use BQ25120_FAULT_XXX macros to evaluate individual bits
 */
int bq25120_get_fault(const struct device *dev, uint8_t *fault)
{
	return bq25120_reg_read(dev, BQ25120_FAULT_AND_FAULT_MASK_REG,fault); 
}

/*!
 * @brief Set battery regulation voltage.
 * @param[in] bat_reg_mv: regulation voltage in mv range 3.6 - 4.65 V
 * @return 0: no error
 * 			-EINVAL: voltage out of range
 * 			-EIO: comm error
 */
int8_t bq25120_set_bat_reg_mv(const struct device *dev, const uint32_t bat_reg_mv)
{
	uint8_t bat_mv_code =0;
	if(bat_reg_mv < BQ25120_BATREG_MV_MIN || bat_reg_mv >BQ25120_BATREG_MV_MAX)
	{
		LOG_ERR("invalid regulation voltage specified:%d",bat_reg_mv);
		return -EINVAL;
	}
	bat_mv_code = ((bat_reg_mv - BQ25120_BATREG_MV_MIN)/10)<<1;	//using formula in bq25120 specs ch9.6.6
	return bq25120_reg_write(dev, BQ25120_BATTERY_VOLTAGE_CTRL_REG,bat_mv_code); 
}

/*!
 * @brief get current battery regulation voltage.
 * @param[out] bat_reg_mv: regulation voltage in mv
 * @return 0: no error
 * 			-EIO: comm error
 */
int8_t bq25120_get_bat_reg_mv(const struct device *dev, uint32_t *bat_reg_mv)
{
	uint8_t bat_mv_code =0;
	if(bq25120_reg_read(dev, BQ25120_BATTERY_VOLTAGE_CTRL_REG,&bat_mv_code))
		return -EIO;
	*bat_reg_mv = ((bat_mv_code >>1)*10 + BQ25120_BATREG_MV_MIN);	//using formula in bq25120 specs ch9.6.6
	return 0; 
}

/*!
 * @brief Set input current limit and the Battery undervoltage-lockout.
 * @param[in] ilim: input Input current limit of device in mA. (50-400 mA)
 * @param[in] buvlo: Battery under-voltage lockout in mV (2.2-3 V, Disable UVLO :0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_ilim_bat_uvlo(const struct device *dev, uint32_t ilim, uint32_t buvlo)
{
	if(ilim < BQ25120_ILIM_MIN_MA || ilim > BQ25120_ILIM_MAX_MA)
	{
		LOG_ERR("invalid ILIM specified:%d",ilim);
		return -EINVAL;
	}
	if(buvlo != BQ25120_BUVLO_DISABLE_MV && 
		(buvlo < BQ25120_BUVLO_MIN_MV || buvlo > BQ25120_BUVLO_MAX_MV))
	{
		LOG_ERR("invalid Battery UVLO specified:%d\n",buvlo);
		return -EINVAL;
	}
	uint8_t ilim_code = (ilim-50)/50; //using formula in bq25120 specs ch9.6.10
	uint8_t buvlo_code = (buvlo == BQ25120_BUVLO_DISABLE_MV)?
							BQ25120_BUVLO_DISABLE_CODE:
							(BQ25120_BUVLO_2200MV_CODE-((buvlo - BQ25120_BUVLO_MIN_MV)/BQ25120_BUVLO_STEP_MV)); 
							//using formula in bq25120 specs ch9.6.10
	uint8_t regval=BQ25120_ILIM_BUVLO_CODE(ilim_code,buvlo_code);
	return bq25120_reg_write(dev, BQ25120_ILIM_BATTERY_UVLO_CTRL_REG,regval); 
}

/*!
 * @brief Set charging current.
 * @param[in] icharge: Charging current in mA (5-300mA, Disable Charger:0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_icharge(const struct device *dev, uint32_t icharge)
{
	if(icharge != BQ25120_ICHARGE_DISABLE_MA && 
		(icharge < BQ25120_ICHARGE_MIN_MA || icharge > BQ25120_ICHARGE_MAX_MA))
	{
		LOG_ERR("invalid charging current specified:%d",icharge);
		return -EINVAL;
	}
	//read HZ_mode bit to change icharger related bits without affecting hz_mode 
	uint8_t regval=0;
	if(bq25120_reg_read(dev, BQ25120_FAST_CHARGE_CTRL_REG,&regval))
	{
		return -EIO;
	}
	uint8_t	hz_mode= regval & BQ25120_HZ_MODE_BIT;
	uint8_t icharge_disab = BQ25120_ICHARGE_DISABLE_CODE(icharge);
	uint8_t icharge_range = BQ25120_ICHARGE_RANGE_CODE(icharge);
	uint8_t icharge_code = BQ25120_ICHARGE_CODE(icharge);
	regval=BQ25120_ICHARGE_REG_CODE(icharge_range,icharge_code,icharge_disab,hz_mode);
	return bq25120_reg_write(dev, BQ25120_FAST_CHARGE_CTRL_REG,regval);
}

/*!
 * @brief Enable/Disable High Impedance Mode.
 * @param[in] status: 	0:	disable hz_mode 
 * 						>0:	enable hz_mode
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_set_hz_mode_status(const struct device *dev, uint8_t status)
{
	//read register to change hz_mode without affectiong icharge settings 
	uint8_t regval=0;
	if(bq25120_reg_read(dev, BQ25120_FAST_CHARGE_CTRL_REG,&regval))
	{
		return -EIO;
	}
	regval &= ~(BQ25120_HZ_MODE_BIT);
	uint8_t hz_mode = (status >0)?BQ25120_HZ_MODE_BIT:0x00;
	regval |= hz_mode;
	return bq25120_reg_write(dev, BQ25120_FAST_CHARGE_CTRL_REG,regval);
}

/*!
 * @brief Get High Impedance Mode status.
 * @param[out] status: 	0: hz_mode disabled 
 * 						1: hz_mode enabled
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_get_hz_mode_status(const struct device *dev, uint8_t *status)
{
	uint8_t regval=0;
	if(bq25120_reg_read(dev, BQ25120_FAST_CHARGE_CTRL_REG,&regval))
	{
		return -EIO;
	}
	*status = ((regval&BQ25120_HZ_MODE_BIT)>0)?1:0;
	return 0;
}

/*!
 * @brief Enable/Disable Charger.
 * @param[in] status: 	0:	disable charger 
 * 						>0:	enable charger
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_set_charger_status(const struct device *dev, uint8_t status)
{
	//read register to change charger enabled bit without affectiong icharge settings 
	uint8_t regval=0;
	if(bq25120_reg_read(dev, BQ25120_FAST_CHARGE_CTRL_REG,&regval))
	{
		return -EIO;
	}
	regval &= ~(BQ25120_CHARGER_ENABLED_BIT);
	uint8_t cebit = (status >0)?BQ25120_CHARGER_ENABLED_BIT:0x00;
	regval |= cebit;
	return bq25120_reg_write(dev, BQ25120_FAST_CHARGE_CTRL_REG,regval);
}

/*!
 * @brief Get Charger Enabled status.
 * @param[out] status: 	0: charger disabled 
 * 						1: charger enabled
 * @return
 * 	0 		:no error
 * -EIO 	:comm error
 */
int8_t bq25120_get_charger_status(const struct device *dev, uint8_t *status)
{
	uint8_t regval=0;
	if(bq25120_reg_read(dev, BQ25120_FAST_CHARGE_CTRL_REG,&regval))
	{
		return -EIO;
	}
	*status = ((regval&BQ25120_CHARGER_ENABLED_BIT)>0)?1:0;
	return 0;
}

/*!
 * @brief Set pre-charge/termination charging current.
 * @param[in] ipreterm: Charging current in uA (500uA to 37000uA (37mA), Disable Termination:0)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_ipreterm(const struct device *dev, uint32_t ipreterm)
{
	if(ipreterm != BQ25120_IPRETERM_DISABLE_UA && 
		(ipreterm < BQ25120_IPRETERM_MIN_UA || ipreterm > BQ25120_IPRETERM_MAX_UA))
	{
		LOG_ERR("invalid charging current specified:%d\n",ipreterm);
		return -EINVAL;
	}
	uint8_t ipreterm_enab = BQ25120_IPRETERM_ENABLE_CODE(ipreterm);
	uint8_t ipreterm_range = BQ25120_IPRETERM_RANGE_CODE(ipreterm);
	uint8_t ipreterm_code = BQ25120_IPRETERM_CODE(ipreterm);
	uint8_t regval=	BQ25120_IPRETERM_REG_CODE(ipreterm_range,ipreterm_code,ipreterm_enab);
	return bq25120_reg_write(dev, BQ25120_FAST_CHARGE_CTRL_REG,regval);
}

/*!
 * @brief Set SYS Output Voltage (VOUT)
 * @param[in] vout: output voltage in mv - 3300mv (3.3V)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_sys_vout(const struct device *dev, uint32_t vout)
{
	if(vout != BQ25120_SYS_DISABLE_MV && 
		(vout < BQ25120_SYS_VOUT_MIN_MV || vout > BQ25120_SYS_VOUT_MAX_MV))
	{
		LOG_ERR("invalid SYS VOUT specified:%d\n",vout);
		return -EINVAL;
	}
	uint8_t sys_out_enab = BQ25120_SYS_OUT_ENABLE_CODE(vout);
	uint8_t sys_sel_code = BQ25120_SYS_SEL_CODE(vout);
	uint8_t sys_vout_code = BQ25120_SYS_VOUT_CODE(vout);
	uint8_t regval=	BQ25120_SYS_VOUT_REG_CODE(sys_out_enab,sys_sel_code,sys_vout_code);
	return bq25120_reg_write(dev, BQ25120_SYS_VOUT_CTRL_REG,regval);
}

/*!
 * @brief Set Load Switch and LDO Voltage 
 * @param[in] ldov: ldo voltage in mv - 3300mv (3.3V)
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
int8_t bq25120_set_ls_ldo(const struct device *dev, uint32_t ldov)
{
	int8_t rc =0;
	uint8_t regval=0;
	const struct bq25120_cfg *cfg = dev->config;
	if(ldov != BQ25120_LDO_DISABLE_MV && 
		(ldov < BQ25120_LDO_MIN_MV || ldov > BQ25120_LDO_MAX_MV))
	{
		LOG_ERR("invalid LDO voltage specified:%d\n",ldov);
		return -EINVAL;
	}
	/*disable LS and pins before changing LDO Voltage*/
	regval=	BQ25120_LS_LDO_REG_CODE(BQ25120_LS_DISABLE_CODE,0,cfg->mrreset_vin);
	rc = bq25120_reg_write(dev, BQ25120_LOAD_AND_LDO_CTRL_REG,regval);
	/*disable LSCTRL pin, if defined*/
	if(cfg->lsctrl_gpios.port)
	{
		gpio_pin_configure_dt(&cfg->lsctrl_gpios, GPIO_OUTPUT_INACTIVE);
	}
	if( ldov == BQ25120_LDO_DISABLE_MV)
	{
		/*Already disabled, no further work*/
		return rc;
	}
	k_msleep(10);
	uint8_t ls_enab = BQ25120_LS_ENABLE_CODE(ldov);
	uint8_t ldo_code = BQ25120_LDO_CODE(ldov);
	regval=	BQ25120_LS_LDO_REG_CODE(ls_enab,ldo_code,cfg->mrreset_vin);
	rc= bq25120_reg_write(dev, BQ25120_LOAD_AND_LDO_CTRL_REG,regval);
	k_msleep(10);
	/*re-enable LSCTRL pin, if defined*/
	if(cfg->lsctrl_gpios.port)
	{
		gpio_pin_configure_dt(&cfg->lsctrl_gpios, GPIO_OUTPUT_ACTIVE);
	}
	k_msleep(10);
	return rc;
}

/*!
 * @brief Register a callback function for getting  bq25120 status when the
 * device signals a status/fault event via INT pin
 * @param[in] status_cb: status callback function 
 */
void bq25120_reg_status_cb(const struct device *dev, bq25120_int_status_cb_t int_status_cb)
{
	struct bq25120_data *devdata = dev->data;
	devdata->status_cb = int_status_cb;
}

/*!
 * @brief Register a callback function for getting informed when bq25120
 * device signals a RESET event via RESET pin
 * @param[in] reset_cb: reset callback function 
 */
void bq25120_reg_reset_cb(const struct device *dev, bq25120_reset_cb_t reset_cb)
{
	struct bq25120_data *devdata = dev->data;
	devdata->reset_cb = reset_cb;
}

/*!
 * @brief Interrupt callback handler for INT pin 
 */
static void bq25120_int_cb(const struct device *gpio_dev, struct gpio_callback *cb,uint32_t pins)
{
	ARG_UNUSED(gpio_dev);
	uint8_t wakereg =0,faultreg=0,tsreg=0;
	uint16_t status=0;
	struct bq25120_data *devdata =
		CONTAINER_OF(cb, struct bq25120_data, int_gpio_cb);
	const struct device *dev = devdata->dev;
	const struct bq25120_cfg *cfg = dev->config;

	if ((pins & BIT(cfg->int_gpios.pin)) == 0U) {
		return;
	}
	/*Get faults and events*/
	bq25120_reg_read(dev, BQ25120_PUSH_BUTTON_CTRL_REG,&wakereg);
	bq25120_reg_read(dev, BQ25120_FAULT_AND_FAULT_MASK_REG,&faultreg);
	bq25120_reg_read(dev, BQ25120_TS_CTRL_AND_FAULT_MASK_REG,&tsreg);
	/*build status bitfield*/
	status = (wakereg&0x03) | (faultreg&0x80) | ((tsreg&0x70) <<4);
	if(devdata->status_cb != NULL)
	{
		devdata->status_cb(dev,status);
	}
}
/*!
 * @brief Interrupt callback handler for RESET pin 
 */
static void bq25120_reset_cb(const struct device *gpio_dev, struct gpio_callback *cb,uint32_t pins)
{
	ARG_UNUSED(gpio_dev);
	struct bq25120_data *devdata =
		CONTAINER_OF(cb, struct bq25120_data, rst_gpio_cb);
	const struct device *dev = devdata->dev;
	const struct bq25120_cfg *cfg = dev->config;

	if ((pins & BIT(cfg->reset_gpios.pin)) == 0U) {
		return;
	}
	if(devdata->reset_cb != NULL)
	{
		devdata->reset_cb(dev);
	}
}

/*!
 * @brief Initialize MR pushbutton related features 
 * @return
 * 	0 		:no error
 * -EINVAL :parmeters out of range
 * -EIO 	:comm error
 */
static int8_t bq25120_init_pb(const struct device *dev)
{
	int8_t rc =0;
	uint8_t regval=0;
	const struct bq25120_cfg *cfg = dev->config;
	struct bq25120_data *devdata = dev->data;
	devdata->dev = dev;
	uint8_t wake1_code = BQ25120_WAKE1_CODE(cfg->wake1_ms); 
	uint8_t wake2_code = BQ25120_WAKE2_CODE(cfg->wake2_ms); 
	uint8_t reset_code = BQ25120_RESET_CODE(cfg->reset_s);
	uint8_t mrrec_code = BQ25120_MRREC_CODE(cfg->mrrec);
	uint8_t pgmr_code = BQ25120_PGMR_CODE(cfg->pg_mr);
	regval=	BQ25120_PB_CTRL_CODE(wake1_code,wake2_code,mrrec_code,reset_code,pgmr_code);
	rc |= bq25120_reg_write(dev, BQ25120_PUSH_BUTTON_CTRL_REG,regval);
	uint8_t ts_en_code  = (cfg->ts_int_en)?1:0; 	/*TS Function Disabled*/
	uint8_t en_int_code = (cfg->charge_int_en)?1:0; /* INT on faults only or fault+ charge events*/
	uint8_t wake_m_code  = ((cfg->wake1_ms ==0) 
						 && (cfg->wake1_ms ==0))
						 		?1:0;/*Mask Wake Condition from MR*/
	uint8_t reset_m_code = (cfg->reset_s ==0)
								?1:0; /*Mask RESET condition from MR*/
	uint8_t timer_m_code = (cfg->timer_int_en)?0:1; /*Mask Timer fault (safety)*/
	regval=	BQ25120_TS_FAULT_CODE(ts_en_code,en_int_code,wake_m_code,reset_m_code,timer_m_code);
	rc |= bq25120_reg_write(dev, BQ25120_TS_CTRL_AND_FAULT_MASK_REG,regval);
	/*attach interrupt callbacks for INT and RESET if gpios are defined*/
	if(cfg->int_gpios.port)
	{
		const struct gpio_dt_spec *int_gpios_p =&cfg->int_gpios;
		gpio_pin_configure_dt(int_gpios_p, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(int_gpios_p,GPIO_INT_EDGE_TO_ACTIVE);
		gpio_init_callback(&devdata->int_gpio_cb, bq25120_int_cb, BIT(int_gpios_p->pin));
		gpio_add_callback(int_gpios_p->port, &devdata->int_gpio_cb);
	}

	if(cfg->reset_gpios.port)
	{
		const struct gpio_dt_spec *rst_gpios_p =&cfg->reset_gpios;
		gpio_pin_configure_dt(rst_gpios_p, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(rst_gpios_p,GPIO_INT_EDGE_TO_ACTIVE);
		gpio_init_callback(&devdata->rst_gpio_cb, bq25120_reset_cb, BIT(rst_gpios_p->pin));
		gpio_add_callback(rst_gpios_p->port, &devdata->int_gpio_cb);
	}
	return rc;
}


/**
 * @brief sensor value get
 * sensor value : val1: integral part val2:decimal part, multiplied by 1 million
 * @return -ENOTSUP for unsupported channels
 */
static int bq25120_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct bq25120_data *devdata = dev->data;


	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = ((devdata->battery_mv / 1000));
		val->val2 = ((devdata->battery_mv % 1000) * 1000U);
		break;
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = devdata->battery_soc;
		val->val2 = 0;
		break;
	//TODO: support other channels like charging current etc., if possible 
	//(see \zephyr\drivers\sensor\bq274xx\bq274xx.c for other possibilities)
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bq25120_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct bq25120_data *devdata = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		if(bq25120_get_bat_reg_mv(dev, &devdata->bat_reg_mv)) {
			LOG_ERR("Failed to read battery regulation voltage");
			return -EIO;
		}
		if(bq25120_get_battery_mv_soc(dev, &devdata->battery_mv,&devdata->battery_soc)) {
			LOG_ERR("Failed to read battery voltage and soc");
			return -EIO;
		}
		break;
	//TODO: support other channels like charging current etc., if possible 
	//(see \zephyr\drivers\sensor\bq274xx\bq274xx.c for other possibilities)
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief initialise the battery manager
 *
 * @return 0 for success
 */
static int bq25120_init(const struct device *dev)
{
	int rslt=0;
    uint8_t status = 0, fault =0;
	const struct bq25120_cfg *cfg = dev->config;
	rslt = gpio_pin_configure_dt(&cfg->cd_gpios,
			   GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	if (rslt < 0) {
		LOG_ERR("Unable to configure CD pin to output and open drain");
		return rslt;
	}

	rslt = bq25120_set_cd_state(dev, BQ25120_CD_PIN_ENABLE);
	if (rslt < 0) {
		LOG_ERR("Unable to set CD pin to Enable");
	 	return status;
	}

	/* Read the BQ25120 status register */
	if(bq25120_get_status(dev, &status))
		return -EIO;
	bq25120_get_fault(dev, &fault);
	LOG_INF("BQ25120 status: %x fault %x",status, fault);


	if(bq25120_device_reset(dev))
		return -EIO;
	/* Give time to driver to reset */
	k_msleep(50);
	/*Set LDO and SYS Output voltages*/
	/*Can be initialized via config variable or DTS*/
#ifdef CONFIG_BQ25120_SYS_MV
	rslt |=bq25120_set_sys_vout(dev,CONFIG_BQ25120_SYS_MV);
#else
	rslt |=bq25120_set_sys_vout(dev,cfg->sys_vout_mv);
#endif
#ifdef CONFIG_BQ25120_LDO_MV
	rslt |=bq25120_set_ls_ldo(dev,CONFIG_BQ25120_LDO_MV);
#else
	rslt |=bq25120_set_ls_ldo(dev,cfg->ls_ldo_mv);
#endif
	/* Set BQ normal operation mode:*/
	rslt |= bq25120_set_op_mode_normal(dev);

	/*set battery regulation voltage as per configured value in DTS*/
	bq25120_set_bat_reg_mv(dev, cfg->bat_reg_mv);

	/*Set input current limit and cut-off voltage as per configured value in DTS*/
	rslt |= bq25120_set_ilim_bat_uvlo(dev, cfg->ilim_ma,cfg->bat_uvlo_mv);

	/*Set charging current limit as per configured value in DTS*/
	rslt |= bq25120_set_icharge(dev, cfg->ichagre_ma);

	/*Set pre-charging and termination current limit as per configured value in DTS*/
	rslt |= bq25120_set_ipreterm(dev, cfg->ipreterm_ua);

	/*initialize MR pushbutton related features*/
	rslt |= bq25120_init_pb(dev);

	bq25120_get_status(dev, &status);
	bq25120_get_fault(dev, &fault);

	/*Enable charging if Vin is healthy, or HiZ mode if on battery
	Note that SYS and LDO will continue to be powered by Battery in HiZ*/
	bq25120_set_cd_state(dev, BQ25120_CD_PIN_DISABLE);

	if(rslt !=0)
	{
		LOG_INF("BQ25120 configured with errors: status %x fault %x",status,fault);
		return -EFAULT;
	}
	else
	{
		LOG_INF("BQ25120 configured successfully: status %x fault %x",status,fault);
		return 0;
	} 
}

static const struct sensor_driver_api bq25120_batlevel_api = {
	.sample_fetch = bq25120_sample_fetch,
	.channel_get = bq25120_channel_get,
};


const static struct bq25120_cfg bq25120_cfg_0index = {
	.bus = I2C_DT_SPEC_INST_GET(0),
	.cd_gpios = GPIO_DT_SPEC_INST_GET(0, cd_gpios),
	.lsctrl_gpios = GPIO_DT_SPEC_INST_GET_OR(0, lsctrl_gpios,{}),
	.sys_vout_mv =DT_PROP(DT_INST(0,DT_DRV_COMPAT),sys_vout_mv),
	.int_gpios = GPIO_DT_SPEC_INST_GET_OR(0, int_gpios,{}),
	.reset_gpios = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios,{}),
	.ls_ldo_mv =  DT_PROP(DT_INST(0,DT_DRV_COMPAT),ls_ldo_mv),
	.bat_reg_mv = DT_PROP(DT_INST(0,DT_DRV_COMPAT),bat_reg_mv),
	.ilim_ma = DT_PROP(DT_INST(0,DT_DRV_COMPAT),ilim_ma),
	.bat_uvlo_mv = DT_PROP(DT_INST(0,DT_DRV_COMPAT),bat_uvlo_mv),
	.ichagre_ma = DT_PROP(DT_INST(0,DT_DRV_COMPAT),icharge_ma),
	.ipreterm_ua = DT_PROP(DT_INST(0,DT_DRV_COMPAT),ipreterm_ua),
	.wake1_ms = DT_PROP(DT_INST(0,DT_DRV_COMPAT),wake1_ms), 
	.wake2_ms = DT_PROP(DT_INST(0,DT_DRV_COMPAT),wake2_ms), 
	.reset_s = DT_PROP(DT_INST(0,DT_DRV_COMPAT),reset_s), 
	.mrrec = DT_PROP(DT_INST(0,DT_DRV_COMPAT),mrrec),
	.mrreset_vin = DT_PROP(DT_INST(0,DT_DRV_COMPAT),mrreset_vin),
	.timer_int_en = DT_PROP(DT_INST(0,DT_DRV_COMPAT),timer_int_en),
    .ts_int_en = DT_PROP(DT_INST(0,DT_DRV_COMPAT),ts_int_en),
	.charge_int_en = DT_PROP(DT_INST(0,DT_DRV_COMPAT),charge_int_en),	
	.pg_mr = DT_PROP(DT_INST(0,DT_DRV_COMPAT),pg_mr),
};

static struct bq25120_data bq25120_data_0index = {0};

DEVICE_DT_INST_DEFINE(0, &bq25120_init, NULL, 
		      &bq25120_data_0index,&bq25120_cfg_0index, 
			  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &bq25120_batlevel_api);
