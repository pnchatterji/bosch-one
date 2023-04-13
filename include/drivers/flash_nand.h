/*
 * Copyright (c) 2020 Bosch Sensortech GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extention of Public API for FLASH drivers for NAND Flash
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FLASH_NAND_H_
#define ZEPHYR_INCLUDE_DRIVERS_FLASH_NAND_H_

/**
 * @brief FLASH internal Interface Extention for NAND
 * @defgroup flash_internal_interface FLASH internal Interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Structure to return the characteristics of the storage device
 */
typedef struct flash_nand_params
{
    uint32_t memory_size;       /**< Total memory size of the storage device in bytes*/
    uint16_t sector_size;       /**< Sector size*/
    uint32_t no_of_sectors;     /**< Total number of Sectors in storage device*/
    uint16_t erase_block_units; /**< Erase block size in unit of pages*/
} flash_nand_params_t;

/**
 * @brief structure to return the nand flash device information
 */
typedef struct flash_nand_deviceinfo
{
    uint8_t mfg_id;     /**< Manufacturer Id*/
    uint16_t device_id; /**< Device Id*/
}flash_nand_deviceinfo_t;

/**
 * @brief Initialize the NAND flash driver
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_api_init)(void);
typedef int (*flash_nand_api_get_init_status)(void);

/**
 * @brief Erase blocks based on input position and length
 * @param[in] pos : Position of the flash from where the erase should happen
 * @param[in] len : No of bytes to be erased
 * Note: the entire set of blocks containing this memory range is erased,
 * not just the specific memory range that is passed.
 *  @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_erase_block)(uint32_t pos, uint32_t len);

/**
 * @brief Erase the entire nand flash device
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_erase_all)(void);

/**
 * @brief Read from nand flash
 * @param[in,out] data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]   num_of_bytes_to_read : No of bytes to read
 * @param[in]   read_loc : Location from which the data is to be read
 *
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_read)(uint8_t* data_ptr, uint32_t num_of_bytes_to_read, uint32_t read_loc);

/**
 * @brief Read the spare bytes from a page
 * @param[in,out]  data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]  no_of_bytes_to_read : No of bytes to read
 * @param[in]  page_num : Page number
 * @param[in]  page_off : Location from which the data is to be read
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_read_spare)(uint8_t* dataPtr, int8_t num_of_bytes_to_read, uint32_t page_num, uint16_t page_off);

/**
 * @brief Write spare bytes to page
 * @param[in]   data_ptr : Pointer to the buffer which holds the data that is to be written
 * @param[in]   no_of_bytes_to_write : No of bytes to write
 * @param[in]   page_num : Page number
 * @param[in]   page_off : Location to which the data is to be written
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_write_spare)(const uint8_t* data_ptr, uint8_t no_of_bytes_to_write, uint32_t page_num, uint16_t page_off);

/**
 * @brief Write the bytes to page
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_write)(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);

/**
 * @brief get nand flash characteristics
 * @param[in]   flash_nand_params_t* : pointer to params structuer that will be filled 
 */
typedef void (*flash_nand_get_memory_params)(flash_nand_params_t* params);

/**
 * @brief get nand flash device info (device id and manufacturer id)
 * @param[in,out]   nand_flash_device_info_t* - pointer to structure that will be filled with the nand flash device info
 */
typedef void (*flash_nand_get_device_info)(flash_nand_deviceinfo_t* devinfo);

/**
 * @brief Load sector data into a buffer
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_load_sector)(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);

/**
 * @brief load the sector spare data into a buffer
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_load_sector_spare)(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write,uint32_t page_num);

/**
 * @brief  This function performs the actual write of the buffer that is 
 * filled by the load_sector function
 * @param[in]  page_num : Page num to which the data is to be written
 * @param[in]  sector_num : Sector number to which the data is to be written
 * @return  0 on success, negative errno code on fail.
 */
typedef int (*flash_nand_write_sector_with_spare)(uint32_t page_num, uint8_t sector_num);


struct flash_nand_driver_api {
    flash_nand_get_memory_params	get_memory_params;
    flash_nand_get_device_info	get_device_info;
    flash_nand_api_init	init;
    flash_nand_api_get_init_status	get_init_status;
    flash_nand_erase_block	erase_block;
    flash_nand_erase_all	erase_all;
    flash_nand_read	read;
    flash_nand_read_spare	read_spare;
    flash_nand_load_sector	load_sector;
    flash_nand_load_sector_spare	load_sector_spare;
    flash_nand_write_sector_with_spare	write_sector_with_spare;
};

#define NAND_EXT_VCODE_DEF(A,B,C,D)  ((A<<24)|(B<<16)|(C<<8)|D)
#define NAND_EXT_VCODE NAND_EXT_VCODE_DEF('N','A','N','1')

/**
 * @brief NAND flash device configuration structure
 * The configuration structure starts with a verification code to check 
 * whether a driver supports the NAND extention API, and if the version
 * of API is as expected
 */
typedef struct {
        /*API verfication code to check existance and version */
        uint32_t vcode;
        /* JEDEC id of Flash device */
        uint8_t id[3];
        /* Available memory size in bytes (after adjusting for ) */
        uint32_t available_size;
        /* Size of one block (Block as per W25 Terminology)*/
        uint32_t block_size;
        /* Number of available blocks (Block as per W25 Terminology)*/
        uint32_t num_blocks;
        /* Size of one page (Page as per W25 Terminology)*/
        uint32_t page_size;
        /* No. of sectors in one page (Page, Sector as per W25 Terminology)*/
        uint32_t sectors_per_page;
        /* No. of pages in one block (Page, Block as per W25 Terminology)*/
        uint32_t pages_per_block;        
        /* Size of one sector (Sector as per W25 Terminology)*/
        uint32_t sector_size;
        /*Number of blocks to preallocate*/
        uint32_t pre_allocate_size;
        /*FlogFS specific constant*/
        uint32_t inode0_max_block;
       /*Extention API for NAND flash drivers*/
       struct flash_nand_driver_api ext_api;

} flash_nand_config_t;


#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif


#endif /* ZEPHYR_INCLUDE_DRIVERS_FLASH_NAND_H_ */
