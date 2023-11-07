#pragma once

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "littlefs/lfs.h"

// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
#define HW_FLASH_STORAGE_BYTES  (1408 * 1024)
#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) // 655360

#if 0
int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int pico_erase(const struct lfs_config *c, lfs_block_t block);
int pico_sync(const struct lfs_config *c);
#endif

// configuration of the filesystem is provided by this struct
extern const struct lfs_config PICO_FLASH_CFG;
