/**
  * Copyright (c) 2022 Bosch Sensortec GmbH
  *
  * SPDX-License-Identifier: Apache-2.0
  * @brief provides an adapter layer to adapt FlogFS source code to a Zephyr compatible filesystem
  */
 
#ifndef FLOGFS_FS_H_
#define FLOGFS_FS_H_

#include <zephyr/fs/fs.h>
#include <drivers/flash_nand.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  *	@brief Zephyr File System Type Code for FLogFS
  * External Filesystems codes in Zephyr have to be defined as greater than FS_TYPE_EXTERNAL_BASE
  * It is the application's responsibility to ensure there is no clash with other filesystems
  * For FLogFS, we define it as 1+FS_TYPE_EXTERNAL_BASE on the assumption that there 
  * will be no other external filesystems used in Bosch Sensortec applications 
  * (other than Zephyr internal filesystems like FAT and LittleFS)
  */
enum { 
	/** Identifier for external file systems FLogFS. */
	FS_FLOGFS  = FS_TYPE_EXTERNAL_BASE +1
};

/** @brief Filesystem info structure for FLogFS mount */
struct fs_flogfs {
  flash_nand_config_t *ncfg;
};

/**
 * @brief for forced formatting and remounting. To be used in application for making a 
 * fresh start after clearing the flash drive
 * @retval 0 Success
 * @retval-EFAULT  format or remount failed 
 */
int flogfs_format_remount(void);
#ifdef __cplusplus
}
#endif

#endif /* FLOGFS_FS_H_ */
