/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file        w25_nand_error_codes.h
 *
 * @brief
 */

/*!
 * @addtogroup w25_nand_error_codes
 * @brief
 * @{*/


#ifndef W25_NAND_ERROR_CODES_H_
#define W25_NAND_ERROR_CODES_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**********************************************************************************/
/* header includes */
/**********************************************************************************/


/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/


/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/

/**
 * @brief Enum which holds the error codes
 */

/* 
Changed to adapt to NAND API
typedef enum w25_nand_errorcode_enum_type
{
    W25_NAND_ERROR,
    W25_NAND_ERR_LOCATION_INVALID,
    W25_NAND_ERR_BUFFER_INVALID,
    W25_NAND_ERR_BYTE_LEN_INVALID,
    W25_NAND_UNINITIALIZED,
    W25_NAND_INITIALIZED,
    W25_NAND_INITIALIZATION_FAILED,
    W25_NAND_ERASE_SUCCESS,
    W25_NAND_ERASE_FAILURE,
    W25_NAND_READ_SUCCESS,
    W25_NAND_READ_FAILURE,
    W25_NAND_ECC_FAILURE,
    W25_NAND_WRITE_SUCCESS,
    W25_NAND_WRITE_FAILURE,
    W25_NAND_BUSY,
    W25_NAND_BBM_FAILURE,
    W25_NAND_BBM_SUCCESS
} w25_nand_error_t;
 */
typedef int w25_nand_error_t;
#define W25_NAND_ERROR -1
#define W25_NAND_ERR_LOCATION_INVALID   -2
#define W25_NAND_ERR_BUFFER_INVALID -3
#define W25_NAND_ERR_BYTE_LEN_INVALID   -4
#define W25_NAND_UNINITIALIZED  -5
#define W25_NAND_INITIALIZED 0
#define W25_NAND_INITIALIZATION_FAILED  -6
#define W25_NAND_ERASE_SUCCESS  0
#define W25_NAND_ERASE_FAILURE  -7
#define W25_NAND_READ_SUCCESS   0
#define W25_NAND_READ_FAILURE   -8
#define W25_NAND_ECC_FAILURE    -9
#define W25_NAND_WRITE_SUCCESS  0
#define W25_NAND_WRITE_FAILURE  -10
#define W25_NAND_BUSY   -11
#define W25_NAND_BBM_FAILURE    -12
#define W25_NAND_BBM_SUCCESS    0

/**********************************************************************************/
/* inline function definitions */
/**********************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* W25_NAND_ERROR_CODES_H_ */

/** @}*/
