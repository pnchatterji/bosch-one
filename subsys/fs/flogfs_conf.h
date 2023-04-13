/*
Copyright (c) 2013, Ben Nahill <bnahill@gmail.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FLogFS Project.
*/

/*!
 * @file flogfs_conf.h
 * @author Ben Nahill <bnahill@gmail.com>
 * @ingroup FLogFS
 *
 * @brief Platform-specific interface details
 */

#ifndef __FLOGFS_CONF_H_
#define __FLOGFS_CONF_H_
#include <zephyr/kernel.h>
//undef certain Zephyr macros which are redefined in flogfs code
//to remove warning messages
#undef MAX
#undef MIN
#undef assert
#include <drivers/flash_nand.h>

#define DT_DRV_COMPAT zephyr_fstab_flogfs
extern flash_nand_config_t *nand_cfg;

static inline void flogfs_configure(flash_nand_config_t *cfg)
{   
   nand_cfg = cfg;
}

//! @addtogroup FLogConf
//! @{

//! @name Flash module parameters
//! @{
//NOTE: values in bracket are typical for w25m02gw, provided here for reference.
//Direct access is made to constants defined in DTS in these macros
//as these constants are used for array allocation in the flogfs code
//, so access is needed at compile time.
#define FS_SECTOR_SIZE      (DT_INST_PROP(0, sector_size))        //(512)
#define FS_SECTORS_PER_PAGE  (DT_INST_PROP(0, sectors_per_page))  //(4)
#define FS_PAGES_PER_BLOCK   (DT_INST_PROP(0, pages_per_block))   //(64)
#define FS_NUM_BLOCKS        (DT_INST_PROP(0, num_blocks))        //(2008) //Total no of blocks available in w25m02gw/w25n02jw variant is 2048 in which 40 blocks are reserved
#define FS_PAGE_SIZE          (FS_SECTORS_PER_PAGE * FS_SECTOR_SIZE)
#define FS_BLOCK_SIZE         (FS_PAGES_PER_BLOCK * FS_PAGE_SIZE)


//! @}

#define FS_SECTORS_PER_BLOCK (FS_SECTORS_PER_PAGE * FS_PAGES_PER_BLOCK)

//! The number of blocks to preallocate
#define FS_PREALLOCATE_SIZE  (DT_INST_PROP(0, pre_allocate_size))  //(10)
#define FS_INODE0_MAX_BLOCK (DT_INST_PROP(0, inode0_max_block))   //(32)

//! @} // FLogConf

#endif
