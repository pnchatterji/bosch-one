/*
 * Copyright (c) 2022 Bosch Sensortec GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 * board-specific init file for arduino nicla sense me
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>


static int board_arduino_nicla_sense_me_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	bool resetRequired = false;

	/* Make COINES-specific changes to the UICR Non-volatile registers if necessary*/

	/* Use Reset button as normal pushbutton by disabling the reset function. This is 
	   done by setting the registers PSELRESET0 and PSELRESET1 to different values 
	   which invalidates the reset function. The "correct" way to do disable reset is to 
	   set both PSELRESET registers to 0xFFFFFFFF, but that is not possible without 
	   erasing the full UICR block if these registers are already set to a valid pin value (ie 21) 
	   This is because 1s can be made 0s by writing to the register, but 0s cant be made 1, 
	   due to the nature of Flash  
	   As a workaround, to avoid the complications of erasing the block, the registers 
	   are set to incompatible values by switching a few 1s to 0s in one of them*/
    if (NRF_UICR->PSELRESET[0] == NRF_UICR->PSELRESET[1] &&
		NRF_UICR->PSELRESET[1] !=  0xFFFFFFFF &&
		NRF_UICR->PSELRESET[1] !=  0)
    {
		/*if the registers are set to identical values and that value is not 0xFFFFFFFF
		  it means the reset function is properly set. So now we mess that up!
		  NOTE: Check for 0 is also done, as in that case this techinique cant be used
		  and System Reset will trigger continuously*/
    	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    	NRF_UICR->PSELRESET[0] = ~(NRF_UICR->PSELRESET[1]);
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		printk ("Reset PSELRESET regs to %x %x\n",NRF_UICR->PSELRESET[0],NRF_UICR->PSELRESET[1]);
	  	resetRequired = true;
    }

    /* Configure NFCT pins as GPIOs. UART RX is assigned to P0.9, same pin that is 
		used as the antenna input for NFC by default*/
    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == 
		(UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){
    	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    	NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		printk ("Reset NFCPINS reg to %x\n",NRF_UICR->NFCPINS);
	  	resetRequired = true;
    }

	if(resetRequired)
	{
		printk("Restarting CPU after changing UICR register settings\n");
    	NVIC_SystemReset();
	}
	return 0;
}

SYS_INIT(board_arduino_nicla_sense_me_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
