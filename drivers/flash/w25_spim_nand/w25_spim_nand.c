/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT winbond_spim_nand

#include <errno.h>
#include <zephyr/drivers/flash.h>
#include <drivers/flash_nand.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include <stdlib.h>

#include "w25_common.h"
#include "w25n02jw.h"

#include "w25n01gw.h"
#include "w25m02gw.h"
/*for Zephyr specific operations*/
#include <zephyr/kernel.h> 

/**
 * @brief Initialization priority of W25 nand flash driver
 * 
 */

#define W25_NAND_INIT_PRIO 80

LOG_MODULE_REGISTER(w25_spim_nand, CONFIG_FLASH_LOG_LEVEL);

/**
 * @brief W25 NAND flash parameters structure
 * this structure is returned by the get_paramaters() Flash API function
 */
static const struct flash_parameters w25_flash_parameters = {
	.write_block_size = 128,//needs to be modified??
	.erase_value = 0xff,
};

/**
 * @brief layout of flash page
 * This structure is returned by the Page_layout function pointer in the 
 * Flash API
 * "Page" in Zephyr Flash API terms means the smallest erasable unit, 
 * which in W25 terms is equivalent to a "block". Proper definition of
 * This struct is key to correct operation of the upper layer FS code.
 * LittleFS depends on this info for all operations, and it needs to be 
 * initialized before the LittleFS is initialized
 * This structure is initialized based on the connected device in the
 * init function, after getting JDEC id from device
 */
static struct flash_pages_layout w25_dev_layout;

/**
 * @brief Structure for defining the W25 NAND access
 */
struct w25_nand_data {
#ifdef CONFIG_MULTITHREADING
	/* The semaphore to control exclusive access to the device. */
	struct k_sem sem;
#endif /* CONFIG_MULTITHREADING */
};
/**
 * @brief Main configuration structure
 * TODO: Use the semaphore to control multithread access to functions??
 * TODO: Check if Littlefs already has mutex support, so this wont be necessary
 */
static struct w25_nand_data w25_nand_memory_data = {
#ifdef CONFIG_MULTITHREADING
	.sem = Z_SEM_INITIALIZER(w25_nand_memory_data.sem, 1, 1),
#endif /* CONFIG_MULTITHREADING */
};

static bool w25_nand_initialized = false;
flash_nand_config_t flash;
static inline int w25_flash_initialize(void){

	uint16_t device_id = 0;

	if(w25_init(&device_id) == W25_NAND_INITIALIZED)
	{
		flash.vcode = NAND_EXT_VCODE;
		flash.inode0_max_block =32;
		flash.pre_allocate_size =10;
		if((device_id == W25N01GW_DEVICE_ID) || (device_id == W25M02GW_DEVICE_ID))
		{
			flash.ext_api.get_memory_params = w25n01gw_get_memory_params;
			flash.ext_api.get_device_info = w25n01gw_get_manufacture_and_devid;
			if(device_id == W25N01GW_DEVICE_ID) /* Hearable device */
			{
				flash.ext_api.init = w25n01gw_init;
				flash.ext_api.get_init_status = w25n01gw_get_device_init_status;
				flash.ext_api.erase_block = w25n01gw_erase_block;
				flash.ext_api.erase_all = w25n01gw_mass_erase;
				flash.ext_api.read = w25n01gw_read;
				flash.ext_api.read_spare = w25n01gw_read_spare;
				flash.ext_api.write_sector_with_spare = w25n01gw_write_sector_with_spare;
				flash.ext_api.load_sector = w25n01gw_load_sector;
				flash.ext_api.load_sector_spare = w25n01gw_load_sector_spare;
				w25_dev_layout.pages_count=W25N01GW_AVAILABLE_BLOCKS;/*zephyr page => w25 block*/
				w25_dev_layout.pages_size=W25N01GW_BLOCK_SIZE; /*zephyr page => w25 block*/
				flash.id[0]=W25_MANUFACTURER_ID;
				flash.id[1]=(W25N01GW_DEVICE_ID>>8);
				flash.id[2]=(W25N01GW_DEVICE_ID&0xFF);
				flash.available_size=W25N01GW_FLASH_SIZE;
				flash.block_size=W25N01GW_BLOCK_SIZE;
				flash.num_blocks=W25N01GW_AVAILABLE_BLOCKS;
				flash.page_size=W25N01GW_PAGE_SIZE;
				flash.sector_size=W25N01GW_SECTOR_SIZE;
				flash.sectors_per_page = W25N01GW_NO_OF_SEC;
				flash.pages_per_block = W25N01GW_NO_OF_PAGES;
			}
			else /*APP3.0 - W25M02GW Old Flash chip */
			{
				flash.ext_api.init = w25m02gw_init;
				flash.ext_api.erase_all = w25m02gw_mass_erase;
				flash.ext_api.get_init_status = w25m02gw_get_device_init_status;
				flash.ext_api.erase_block = w25m02gw_erase_block;
				flash.ext_api.read = w25m02gw_read;
				flash.ext_api.write_sector_with_spare = w25m02gw_write_sector_with_spare;
				flash.ext_api.load_sector = w25m02gw_load_sector;
				flash.ext_api.read_spare = w25m02gw_read_spare;
				flash.ext_api.load_sector_spare = w25m02gw_load_sector_spare;
				w25_dev_layout.pages_count=W25M02GW_AVAILABLE_BLOCKS;/*zephyr page => w25 block*/
				w25_dev_layout.pages_size=W25M02GW_BLOCK_SIZE; /*zephyr page => w25 block*/
				flash.id[0]=W25_MANUFACTURER_ID;
				flash.id[1]=(W25M02GW_DEVICE_ID>>8);
				flash.id[2]=(W25M02GW_DEVICE_ID&0xFF);
				flash.available_size=W25M02GW_FLASH_SIZE;
				flash.block_size=W25M02GW_BLOCK_SIZE;
				flash.num_blocks=W25M02GW_AVAILABLE_BLOCKS;
				flash.page_size=W25M02GW_PAGE_SIZE;
				flash.sector_size=W25M02GW_SECTOR_SIZE;
				flash.sectors_per_page = W25M02GW_NO_OF_SEC_PER_PAGE;
				flash.pages_per_block = W25M02GW_NO_OF_PAGES_PER_BLOCK;
			}
		}
		else if(device_id == W25N02JW_DEVICE_ID || /*APP3.0 - W25N02JW Latest Flash chip */
				device_id == W25N02KW_DEVICE_ID)   /*APP3.1 - W25N02KW Latest Flash chip */
		{
			flash.ext_api.get_memory_params = w25n02jw_get_memory_params;
			flash.ext_api.get_device_info = w25n02jw_get_manufacture_and_devid;
			flash.ext_api.init = w25n02jw_init;
			flash.ext_api.get_init_status = w25n02jw_get_device_init_status;
			flash.ext_api.erase_block = w25n02jw_erase_block;
			flash.ext_api.erase_all = w25n02jw_mass_erase;
			flash.ext_api.read = w25n02jw_read;
			flash.ext_api.read_spare = w25n02jw_read_spare;
			flash.ext_api.write_sector_with_spare = w25n02jw_write_sector_with_spare;
			flash.ext_api.load_sector = w25n02jw_load_sector;
			flash.ext_api.load_sector_spare = w25n02jw_load_sector_spare;
			w25_dev_layout.pages_count=W25N02JW_AVAILABLE_BLOCKS;/*zephyr page => w25 block*/
			w25_dev_layout.pages_size=W25N02JW_BLOCK_SIZE; /*zephyr page => w25 block*/
			flash.id[0]=W25_MANUFACTURER_ID;
			flash.id[1]=(device_id>>8);
			flash.id[2]=(device_id&0xFF);
			flash.available_size=W25N02JW_FLASH_SIZE;
			flash.block_size=W25N02JW_BLOCK_SIZE;
			flash.num_blocks=W25N02JW_AVAILABLE_BLOCKS;
			flash.page_size=W25N02JW_PAGE_SIZE;
			flash.sector_size=W25N02JW_SECTOR_SIZE;
			flash.sectors_per_page = W25N02JW_NO_OF_SEC;
			flash.pages_per_block = W25N02JW_NO_OF_PAGES;
		}
		else
		{
			flash.vcode = 0; //mark ext api as invalid
			return -ENODEV;
		}
		/* Init buffers */
		flash.ext_api.init();
		w25_nand_initialized =true;	
		return 0;
	}
	else
	{
		flash.vcode = 0; //mark ext api as invalid
		return -ENODEV;
	}
	return 0;
}

/*
 *	Standard Flash API implememnation
 *  ===================================
 */
#if defined(CONFIG_FLASH_JESD216_API)
static int w25_read_jedec_id(const struct device *dev,
				     uint8_t *id)
{
	//Not Necessary to implement, as not used by upper FS layer
	return 0;
}

static int w25_sfdp_read(const struct device *dev, off_t offset,
			  void *data, size_t len)
{
	//Not Necessary to implement, as not used by upper FS layer
	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */


static int w25_nand_read(const struct device *dev, off_t addr, void *dest,
			 size_t size)
{
	if(!w25_nand_initialized)
	{
		if(w25_flash_initialize())
			return -ENODEV;
	}
	//here read_loc is address in overall address space (ignoring blockification & pageination)
	//i.e. same as addr argument 	
	if(flash.ext_api.read(dest,size,addr) == W25_NAND_READ_SUCCESS)
	{
		return 0;
	}
	else
	{
		return -EBUSY;
	}
}

static int w25_nand_write(const struct device *dev, off_t addr,
			  const void *src,
			  size_t size)
{
	int npage,nsector;
	if(!w25_nand_initialized)
	{
		if(w25_flash_initialize())
			return -ENODEV;
	}

	//only sector-sized writes are handled. This should be ensured by passing proper
	//config parameters to the upper file system e.g. in littlefs prog_size= FS_SECTOR_SIZE
	if(size <= 0 || size > flash.sector_size)
		return -EINVAL;
	if(addr > flash.available_size)
		return -EINVAL;
	//load_sector(data_ptr,no_of_bytes,write_loc)
	//where write_loc is address in overall address space (ignoring blockification & pageination)
	//i.e. same as addr argument 	
	flash.ext_api.load_sector(src,size, addr);
	//write_sector_with_spare(page_num, sector_num)
	//Where page_num is continuous page numering from block 0
	//sector_num is sector in the page to be programmed
	npage = addr / flash.page_size;
	nsector = (addr % flash.page_size)/flash.sector_size;
	if(flash.ext_api.write_sector_with_spare(npage,nsector) == W25_NAND_WRITE_SUCCESS)
	{
		//As per API, 0 returned on success (not number of bytes written)
		return 0;
	}
	else
	{
		return -EBUSY;
	}
}

static int w25_nand_erase(const struct device *dev, off_t addr, size_t size)
{
	//Note: addr and size should be multiples of block size, else W25_NAND_ERR_LOCATION_INVALID error
	//this is checked in the driver, so no need to check again here.
	//appropriate configurations should be made in upper file system 
	//eg. in littlefs block_size = FS_BLOCK_SIZE
	//first arg is a 1 based position in total address space where erasing is to start, so addr+1
	if(!w25_nand_initialized)
	{
		if(w25_flash_initialize())
			return -ENODEV;
	}

	switch(flash.ext_api.erase_block(addr+1,size))
	{
		case W25_NAND_ERASE_SUCCESS:
			return 0;
		case W25_NAND_ERR_LOCATION_INVALID:
			return -EINVAL;
		case W25_NAND_ERASE_FAILURE:
			return -EBUSY;
		default:
			return -EIO;
	}
}

/* static int w25_nand_write_protection_set(const struct device *dev,
					 bool write_protect)
{
	//Not necessary to implement
	return 0;
} */

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
static int w25_nand_init(const struct device *dev)
{ 
	if(w25_nand_initialized)
		return 0;
	return w25_flash_initialize();
}

static const struct flash_parameters *
w25_flash_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &w25_flash_parameters;
}

static void w25_nand_pages_layout(const struct device *dev,
				  const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	if(!w25_nand_initialized)
	{
		w25_flash_initialize();
	}
	*layout = &w25_dev_layout;
	*layout_size = 1;
}

static const struct flash_driver_api w25_nand_api = {
	.read = w25_nand_read,
	.write = w25_nand_write,
	.erase = w25_nand_erase,
	.get_parameters = w25_flash_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = w25_nand_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = w25_sfdp_read,
	.read_jedec_id = w25_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

DEVICE_DT_INST_DEFINE(0, &w25_nand_init, NULL,
		&w25_nand_memory_data, &flash,
		POST_KERNEL, W25_NAND_INIT_PRIO,
		&w25_nand_api);
