/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_FLASH_STORE_H__
#define __ECAM_FLASH_STORE_H__

/*
 * The last uploaded profile, kept in the top 32 KB of the processor's flash so it survives a reboot.
 * The linker scripts in Device_Startup end the firmware below this area. Flashing new firmware
 * erases it (the uploader erases the chip), so a firmware update needs one upload.
 *
 *   header  magic 'ECPS', version, length of the text, FNV-1a of the text
 *   text    the upload lines between "load" and "commit", each ending in '\n'
 *
 * Writing erases 8 KB blocks and programs 512-byte pages. While the controller erases or writes,
 * code in the same flash bank stalls, so the engine only saves with nothing moving.
 */

#include <stddef.h>
#include <stdint.h>

#define ECAM_STORE_ADDRESS 0x00078000u  /* 512 KB flash - 32 KB */
#define ECAM_STORE_SIZE 0x8000u
#define ECAM_STORE_TEXT_MAX (ECAM_STORE_SIZE - 16u)

/* nullptr on success, or why it failed. */
const char *FlashStoreSave(const char *text, uint32_t length);
/* The stored text (points into flash), or nullptr when nothing valid is stored. */
const char *FlashStoreText(uint32_t *length);
/* Forget the stored profile (erases the header block). */
const char *FlashStoreErase();
/* Called while waiting on the flash controller (erase, page write), so the firmware can keep its network
 * alive. It must not use the flash controller. Code it runs from the store's bank stalls until the
 * controller is done. */
void FlashStoreSetYield(void (*fn)());

#endif
