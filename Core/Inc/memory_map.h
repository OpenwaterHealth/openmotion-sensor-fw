/*
 * memory_map.h
 *
 *  Created on: Mar 4, 2024
 *      Author: gvigelet
 */

#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_

#ifdef __cplusplus
 extern "C" {
#endif
 /* Base address of the Flash pages */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Camera FPGA bitstream is ~160 KB, so it occupies sectors 5 AND 6 of bank 2. */
#define ADDR_CAMERA_BITSTREAM       ADDR_FLASH_SECTOR_5_BANK2
#define ADDR_FLASH_END_ADDRESS      ((uint32_t)0x08200000)
#define FLASH_USER_START_ADDR       ADDR_FLASH_SECTOR_7_BANK2   /* motion_config */

/* Hardware serial number: a 32-byte record at the TOP of the motion_config
 * sector (sector 7). Sector 7 is the only flash region ABOVE the firmware
 * image (the merged image runs from 0x08000000 through the FPGA bitstream end
 * in sector 6), so it is the only place a value survives a full firmware flash
 * — which is exactly why motion_config lives there too. The two tenants share
 * one 128 KB sector (the minimum erase granularity); motion_config.c owns the
 * sector and preserves this slot across its erase/rewrite. */
#define FLASH_SERIAL_RECORD_SIZE    32U
#define FLASH_SERIAL_START_ADDR     (ADDR_FLASH_END_ADDRESS - FLASH_SERIAL_RECORD_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* INC_MEMORY_MAP_H_ */
