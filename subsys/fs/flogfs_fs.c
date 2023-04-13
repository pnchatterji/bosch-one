/**
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * @brief provides an adapter layer to adapt FlogFS API to a Zephyr compatible filesystem
 * Note that RW mode is not supported in FLogFS. If RW flag is passed to file open operation,
 * it will return a NOT SUPPORTED error. Sync, Mkdir and Truncate operations are also not
 * supported. When a file is opened for write operations, APPEND and CREATE flags are assumed.
 * The actual state of these flags is ignored. This is because FLogFS always appends to files.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/logging/log.h>

#include "flogfs_fs.h"
#include "flogfs.h"

#define LOG_MODULE_NAME flogfs
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
struct flogfs_file_data {
	union{
	flog_write_file_t wfile;
	flog_read_file_t  rfile;
	}f;
	fs_mode_t zflags; /* flags with which file was opened*/
};

/* Global memory pool for open files and iterators*/
K_MEM_SLAB_DEFINE(file_data_pool, sizeof(struct flogfs_file_data),
			 CONFIG_FS_FLOGFS_NUM_FILES, 4);
K_MEM_SLAB_DEFINE(flog_itr_pool, sizeof(flogfs_ls_iterator_t),
			 CONFIG_FS_FLOGFS_NUM_ITRS, 4);
static void release_file_data(struct fs_file_t *fp)
{

	k_mem_slab_free(&file_data_pool, &fp->filep);
	fp->filep = NULL;
}

/* fs_lock and fs_unlock functions are called prior to each file system
	operation. It provides a coarse level of locking, as opposed to a 
	finer level of locking available in the FLogFS code.
	The following functions are not implemented as the locking in FlogFS
	code is preffered
*/
static inline void fs_init_lock(struct fs_flogfs *fs)
{
}

static inline void fs_lock(struct fs_flogfs *fs)
{
}

static inline void fs_unlock(struct fs_flogfs *fs)
{
}

/*
	FlogFS Implementation API
	Implementation of platform-specific functions required by FlogFS
*/

flog_result_t flogfs_impl_init(void) 
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_erase_block(uint32_t a, uint32_t b)
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_read(uint8_t* a, uint32_t b, uint32_t v)
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_write_sector_with_spare(uint32_t a, uint8_t b)
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_load_sector(const uint8_t* a, uint32_t b, uint32_t c)
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_read_spare(uint8_t* a, int8_t b, uint32_t c, uint16_t d )
{
	return FLOG_SUCCESS;
}

flog_result_t flogfs_impl_load_sector_spare(const uint8_t* a, uint32_t b, uint32_t c)
{
	return FLOG_SUCCESS;
}

/*strip volume ID from path. Differes from version in fs_impl.c used by other
file systems in that leading '/' is also removed*/
const char *flogfs_strip_prefix(const char *path,
				 const struct fs_mount_t *mp)
{
	/*there is no root directory in flogfs*/
	static const char *const root = "";
	if ((path == NULL) || (mp == NULL)) {
		return path;
	}
	/*replace "/flog0/foo.txt" with "foo.txt"*/
	path += mp->mountp_len;
	return *path ? ++path : root;
}

/*Zephyr filesystem callback functions used in fs_file_system_t definition*/

static int flogfs_open_(struct fs_file_t *fp, const char *path,
			 fs_mode_t zflags)
{
	flog_result_t retf;
	struct fs_flogfs *fs = fp->mp->fs_data;
	int ret = 0;
	
	if((zflags & FS_O_READ) && (zflags & FS_O_WRITE))
	{
		/*RW Mode not supported*/
		LOG_ERR("RW Mode not supported for file open %s",path);
		return -ENOTSUP;
	}
	ret = k_mem_slab_alloc(&file_data_pool, &fp->filep, K_NO_WAIT);

	if (ret != 0) {
		return ret;
	}
	struct flogfs_file_data *fdp = fp->filep;

	memset(fdp, 0, sizeof(*fdp));
	fdp->zflags = zflags;
	path = flogfs_strip_prefix(path, fp->mp);
	fs_lock(fs);
    if(zflags & FS_O_WRITE)
    {
        retf = flogfs_open_write(&fdp->f.wfile, path);
    }
	else if (zflags & FS_O_READ)
	{
        retf = flogfs_open_read(&fdp->f.rfile, path);
	}
	else
	{
		LOG_ERR("Neither Read nor Write mode for file open %s",path);
		retf = FLOG_FAILURE;
	}
	fs_unlock(fs);
	if (retf == FLOG_FAILURE) {
		release_file_data(fp);
		LOG_ERR("File open failed %s",path);
		return -EFAULT;
	}
	return 0;
}

static int flogfs_close_(struct fs_file_t *fp)
{
	flog_result_t retf;
	struct fs_flogfs *fs = fp->mp->fs_data;
	struct flogfs_file_data *fdp = fp->filep;
	fs_lock(fs);

	if(fdp->zflags & FS_O_WRITE)
    {
        retf = flogfs_close_write(&fdp->f.wfile);
    }
	else
	{
        retf = flogfs_close_read(&fdp->f.rfile);
	}

	fs_unlock(fs);

	release_file_data(fp);

	return (retf != FLOG_SUCCESS)?-EFAULT:0;
}

static int flogfs_unlink_(struct fs_mount_t *mountp, const char *path)
{
	struct fs_flogfs *fs = mountp->fs_data;

	path = flogfs_strip_prefix(path, mountp);

	fs_lock(fs);

	//flog_result_t retf = flogfs_rm(path);
	flog_result_t retf = flogfs_invlaidate(path);
	flog_delete_invalidated_block();
	fs_unlock(fs);
	return (retf != FLOG_SUCCESS)?-EFAULT:0;
}

static int flogfs_rename_(struct fs_mount_t *mountp, const char *from,
			   const char *to)
{
	/*this service does not exist in flogfs*/
	LOG_ERR("File Rename not available in FLogfs %s",from);
	return -EIO;
}

static ssize_t flogfs_read_(struct fs_file_t *fp, void *ptr, size_t len)
{
	struct fs_flogfs *fs = fp->mp->fs_data;
	struct flogfs_file_data *fdp = fp->filep;
	uint32_t ret=0;
	fs_lock(fs);
	/*read only if read file pointer, else do nothing and return 0*/
	if((fdp->zflags & FS_O_WRITE)==0) 
	{
		ret = flogfs_read(&fdp->f.rfile, (uint8_t*)ptr, (uint32_t)len);
	}
	fs_unlock(fs);
	return (ssize_t)ret;
}

static ssize_t flogfs_write_(struct fs_file_t *fp, const void *ptr, size_t len)
{
	struct fs_flogfs *fs = fp->mp->fs_data;
	struct flogfs_file_data *fdp = fp->filep;
	uint32_t ret=0;

	/*write only if write file pointer, else do nothing and return 0*/
	if(fdp->zflags & FS_O_WRITE) 
	{
		fs_lock(fs);
		ret = flogfs_write(&fdp->f.wfile, (const uint8_t*)ptr, (uint32_t)len);
		fs_unlock(fs);
	}
	return (ssize_t)ret;
}

static int flogfs_seek_(struct fs_file_t *fp, off_t off, int whence)
{
	struct fs_flogfs *fs = fp->mp->fs_data;
	struct flogfs_file_data *fdp = fp->filep;
	flog_result_t retf = FLOG_FAILURE;
	/*seek can only be from beginning of file for flogfs, 
	otherwise signal argument error*/
	if(whence != FS_SEEK_SET) 
		return -EINVAL;
	/*seek only if read file pointer, else do nothing and return failure
	write files are append only */
	if((fdp->zflags & FS_O_WRITE)==0)
	{
		fs_lock(fs);
		retf = flogfs_read_seek(&fdp->f.rfile, (uint32_t) off);
		fs_unlock(fs);
	}
	return (retf != FLOG_SUCCESS)?-EFAULT:0;
}

static off_t flogfs_tell_(struct fs_file_t *fp)
{
	struct fs_flogfs *fs = fp->mp->fs_data;
	struct flogfs_file_data *fdp = fp->filep;
	uint32_t ofst=0;
	/*tell only if read file pointer 
	for write files, return file size which is same as current position
	*/
	if((fdp->zflags & FS_O_WRITE)==0)
	{
		fs_lock(fs);
		ofst = flogfs_read_tell(&fdp->f.rfile);
		fs_unlock(fs);
	}
	else
	{
		ofst = fdp->f.wfile.file_size;
	}
	return (off_t)ofst;
}

static int flogfs_truncate_(struct fs_file_t *fp, off_t length)
{
	/*this service does not exist in flogfs*/
	LOG_ERR("File Truncate not available in FLogfs %s","");
	return -ENOTSUP;
}

static int flogfs_sync_(struct fs_file_t *fp)
{
	/*this service does not exist in flogfs*/
	LOG_ERR("File Sync not available in FLogfs %s","");
	return -ENOTSUP;
}

static int flogfs_mkdir_(struct fs_mount_t *mountp, const char *path)
{
	/*this service does not exist in flogfs*/
	return -ENOTSUP;
}

static int flogfs_opendir_(struct fs_dir_t *dp, const char *path)
{
	struct fs_flogfs *fs = dp->mp->fs_data;
	if (k_mem_slab_alloc(&flog_itr_pool, &dp->dirp, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	memset(dp->dirp, 0, sizeof(flogfs_ls_iterator_t));
	fs_lock(fs);
	flogfs_start_ls(dp->dirp);
	fs_unlock(fs);
	return 0;
}

static int file_name_to_dirent(const char *file_name, struct fs_dirent *entry)
{
	uint32_t rsize =0;
	flog_result_t retf = FLOG_FAILURE;
	flog_read_file_t rfile;
	retf = flogfs_open_read(&rfile, file_name);
	if(retf == FLOG_SUCCESS)
	{
		rsize = flogfs_read_file_size(&rfile);
		flogfs_close_read(&rfile);
	}
	/*fill up the dirent even in case of error (rsize=0)*/
	entry->type = FS_DIR_ENTRY_FILE;
	entry->size = rsize;
	strncpy(entry->name, file_name, sizeof(entry->name));
	entry->name[sizeof(entry->name) - 1] = '\0';
	//return (retf == FLOG_SUCCESS)?0:-EIO;
	/*signal success even if getting file size operation failed
	  , because file name is still useful data (TBD?)*/
	return 0;
}

static int flogfs_readdir_(struct fs_dir_t *dp, struct fs_dirent *entry)
{
	int ret=0;
	char fname_dst[FLOG_MAX_FNAME_LEN+1];
	struct fs_flogfs *fs = dp->mp->fs_data;
	fs_lock(fs);
	ret = flogfs_ls_iterate(dp->dirp, fname_dst);
	if (ret > 0) { 			/*1=> valid data*/
		ret = file_name_to_dirent(fname_dst, entry);
	} else if (ret == 0) {	/*0=> end of data*/
		entry->name[0] = 0;
	} else {				/* should never get here*/
		ret = -EIO;
	}
	fs_unlock(fs);
	return ret;
}

static int flogfs_closedir_(struct fs_dir_t *dp)
{
	struct fs_flogfs *fs = dp->mp->fs_data;
	fs_lock(fs);
	flogfs_stop_ls(dp->dirp);
	fs_unlock(fs);
	k_mem_slab_free(&flog_itr_pool, &dp->dirp);
	return 0;
}

static int flogfs_stat_(struct fs_mount_t *mountp,
			 const char *path, struct fs_dirent *entry)
{
	path = flogfs_strip_prefix(path, mountp);
    if (flogfs_check_exists(path) == FLOG_FAILURE)
	{
		LOG_ERR("File %s does not exist",path);
        return -ENOENT;	
	}
	return file_name_to_dirent(path,entry);
}

static int flogfs_statvfs_(struct fs_mount_t *mountp,
			    const char *path, struct fs_statvfs *stat)
{
	const uint32_t BLOCK_SIZE = FS_PAGES_PER_BLOCK*FS_SECTORS_PER_PAGE*FS_SECTOR_SIZE;
	uint32_t free_space = flogfs_available_space();
	stat->f_bsize =BLOCK_SIZE; 			// Optimal transfer block size
	stat->f_frsize =BLOCK_SIZE;			// Allocation unit size
	stat->f_blocks=FS_NUM_BLOCKS;		// Size of FS in f_frsize units
	stat->f_bfree=free_space/BLOCK_SIZE;// Number of free blocks
	return 0;
}

/*
mount and data structure for FLogFS are directly defined here.
A single partition is assumed, as FlogFS currently does not 
support multiple partitions
*/

#define DT_DRV_COMPAT zephyr_fstab_flogfs
#define inst 0
#define FS_PARTITION(inst) DT_PHANDLE_BY_IDX(DT_DRV_INST(inst), partition, 0)
#define FS_PARTITION_LABEL(inst) DT_STRING_TOKEN(FS_PARTITION(inst), label)

static struct fs_flogfs fs_flogfs_data_partion_0 ={
	.ncfg = NULL,	//initialized with pointer from driver in init function
};

//static const struct flogfs_config *flogfs_config_p = &fs_flogfs_cfg;

struct fs_mount_t fs_flogfs_mp_0 = {
	.type = FS_FLOGFS,
	.mnt_point = DT_INST_PROP(inst, mount_point),
	.fs_data = &fs_flogfs_data_partion_0,
	.storage_dev = DT_FIXED_PARTITION_ID(FS_PARTITION(inst)),
	.flags = FSTAB_ENTRY_DT_MOUNT_FLAGS(DT_DRV_INST(inst)),
};

//Note: The following structure is required for the flogfs_initialize() call
//The structure is initialized from the DTS, if definitions are available in the DTS, 
//otherwise default values are taken from the driver in the below flogfs_init() function
flog_initialize_params_t params = {
	.number_of_blocks = DT_INST_PROP(inst, num_blocks),		
	.pages_per_block = DT_INST_PROP(inst, pages_per_block),
};

/*Expose FLogFs format and mount commands to application
for forced formatting and remounting. To be used in application for 
making a fresh start after clearing the flash drive*/
int flogfs_format_remount(void)
{
	if(fs_flogfs_data_partion_0.ncfg !=NULL)
		fs_flogfs_data_partion_0.ncfg->ext_api.erase_all();
	if(flogfs_format() == FLOG_FAILURE)
		return -EFAULT;
	// if (FLOG_FAILURE == flogfs_initialize(&params))
	// 	return -EFAULT;
	if (flogfs_mount() == FLOG_FAILURE)
		return -EFAULT;
	return 0;
}

/*Get NAND flash extention API from Flash driver configuration structure,
if it exists, and configure FlogFS with it. Else signal an error*/
static int flogfs_configure_(struct fs_mount_t *mountp)
{
	unsigned int area_id = (uintptr_t)mountp->storage_dev;
	const struct flash_area *fapp = NULL;
	const struct device *dev;
	int ret;

	/* Open flash area */
	ret = flash_area_open(area_id, &fapp);
	if ((ret < 0) || (fapp == NULL)) {
		LOG_ERR("can't open flash area %d", area_id);
		return -ENODEV;
	}

	LOG_DBG("FS area %u at 0x%x for %u bytes", area_id,
		(uint32_t)fapp->fa_off, (uint32_t)fapp->fa_size);

	dev = flash_area_get_device(fapp);
	if (dev == NULL) {
		LOG_ERR("can't get flash device: %s",
			fapp->fa_dev->name);
		return -ENODEV;
	}
	flash_nand_config_t *nand_cfg = (flash_nand_config_t *)dev->config;
	if(nand_cfg == NULL || nand_cfg->vcode != NAND_EXT_VCODE)
	{
		LOG_ERR("Flash driver %s does not support the NAND Extention API, or API version incorrect\n",fapp->fa_dev->name);
		return -ENODEV;
	}
	flogfs_configure(nand_cfg);
	fs_flogfs_data_partion_0.ncfg = nand_cfg;
	return 0;
}

static int flogfs_mount_(struct fs_mount_t *mountp)
{
	struct fs_flogfs *fs = mountp->fs_data;
	if(flogfs_configure_(mountp))
		return -EFAULT;
	if (FLOG_FAILURE == flogfs_initialize(&params))
		return -EFAULT;
	if (FLOG_FAILURE == flogfs_mount())
	{
		if( (mountp->flags & FS_MOUNT_FLAG_NO_FORMAT) ==0)
		{
			/*Retry mounting after formatting, else fail permanently*/
			LOG_ERR("Mount failed. Attempting to format(Auto-format is enabled).%s","");
			if(flogfs_format_remount()!=0)
			{
				LOG_ERR("Remounting after formatting failed.%s","");
				return -EFAULT;
			}
		}
		else
		{
			LOG_ERR("Mount failed. The drive may need to be formatted (Auto-format is disabled).%s","");
			return -EFAULT;
		}
	}
	fs_init_lock(fs);
	return 0;
}

static int flogfs_unmount_(struct fs_mount_t *mountp)
{
	/*Not supported by FLogFS*/
	return -ENOTSUP;
}

/* File system interface */
static const struct fs_file_system_t flogfs_fs = {
	.open = flogfs_open_,
	.close = flogfs_close_,
	.read = flogfs_read_,
	.write = flogfs_write_,
	.lseek = flogfs_seek_,
	.tell = flogfs_tell_,
	.truncate = flogfs_truncate_,
	.sync = flogfs_sync_,
	.opendir = flogfs_opendir_,
	.readdir = flogfs_readdir_,
	.closedir = flogfs_closedir_,
	.mount = flogfs_mount_,
	.unmount = flogfs_unmount_,
	.unlink = flogfs_unlink_,
	.rename = flogfs_rename_,
	.mkdir = flogfs_mkdir_,
	.stat = flogfs_stat_,
	.statvfs = flogfs_statvfs_,
};

static void mount_init(struct fs_mount_t *mp)
{

	LOG_DBG("FlogFS partition at %s", mp->mnt_point);
	if ((mp->flags & FS_MOUNT_FLAG_AUTOMOUNT) != 0) {
		int rc = fs_mount(mp);

		if (rc < 0) {
			LOG_ERR("Automount %s failed: %d",
				mp->mnt_point, rc);
		} else {
			LOG_DBG("Automount %s succeeded",
				mp->mnt_point);
		}
	}
}

static int flogfs_init(const struct device *dev)
{
	static struct fs_mount_t *partitions[] = {
		/*Use DT_INST_FOREACH_STATUS_OKAY(REFERENCE_MOUNT) for multiple partitions configured from DTS
		see Implementation Note above*/
		&fs_flogfs_mp_0,
	};
	
	int rc = fs_register(FS_FLOGFS, &flogfs_fs);

	if (rc == 0) {
		struct fs_mount_t **mpi = partitions;

		while (mpi < (partitions + ARRAY_SIZE(partitions))) {
			mount_init(*mpi++);
		}
	}

	return rc;
}

SYS_INIT(flogfs_init, POST_KERNEL, 99);
