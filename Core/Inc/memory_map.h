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

#define ADDR_CAMERA_BITSTREAM       ADDR_FLASH_SECTOR_5_BANK2
#define ADDR_FLASH_END_ADDRESS      ((uint32_t)0x08200000)
#define FLASH_USER_START_ADDR       ADDR_FLASH_SECTOR_7_BANK2

/* Boot flags in the top words of SRAM4 (0x38000000 + 64 KB). They must NOT
 * live at the bottom of SRAM4: spi6_buffer (the FPGA histogram DMA buffer)
 * is linked at 0x38000000, and the histogram SOF marker is the same
 * 0xDEADBEEF value as the DFU magic, so streamed data could spoof a
 * bootloader request across a warm reset. ram_scrub() zeroes all of SRAM4
 * on every normal boot, so the flags only persist across the resets that
 * set them deliberately. */
#define BOOT_FLAG_DFU_REQUEST_ADDR    ((uint32_t)0x3800FFF8) /* app asks SystemInit to enter ROM bootloader */
#define BOOT_FLAG_DFU_REQUEST_MAGIC   ((uint32_t)0xDEADBEEF)
#define BOOT_FLAG_JUMP_ENTRY_ADDR     ((uint32_t)0x3800FFFC) /* set entering bootloader; if still set at app start, app was entered by jump, not reset */
#define BOOT_FLAG_JUMP_ENTRY_MAGIC    ((uint32_t)0xB00710AD)

#ifdef __cplusplus
}
#endif

#endif /* INC_MEMORY_MAP_H_ */
