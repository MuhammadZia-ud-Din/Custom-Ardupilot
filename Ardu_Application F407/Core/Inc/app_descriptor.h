/* ArduPilot bootloader application descriptor.
 * The bootloader uses memmem() to scan the entire app flash region for
 * the 8-byte signature, then validates CRCs and board_id before jumping.
 * CRCs must be patched into the .bin after build using patch_descriptor.py.
 */
#ifndef APP_DESCRIPTOR_H
#define APP_DESCRIPTOR_H

#include <stdint.h>

/* Unsigned firmware signature – matches AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED */
#define APP_DESCRIPTOR_SIG_UNSIGNED { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 }

/* APJ_BOARD_ID for sw-spar-f407 (board_types.txt line AP_HW_sw-spar-f407 6000) */
#define APP_BOARD_ID  6000U

/* App version */
#define APP_VERSION_MAJOR  1U
#define APP_VERSION_MINOR  0U

/* Total size of this struct must stay 36 bytes */
typedef struct __attribute__((packed))
{
    uint8_t  sig[8];          /* APP_DESCRIPTOR_SIG_UNSIGNED              */
    uint32_t image_crc1;      /* CRC32 from app start to here (excl.)     */
    uint32_t image_crc2;      /* CRC32 from version_major to image end    */
    uint32_t image_size;      /* Total firmware size in bytes             */
    uint32_t git_hash;        /* 0 for custom builds                      */
    uint8_t  version_major;   /* CRC2 region starts at this field         */
    uint8_t  version_minor;
    uint16_t board_id;        /* Must be 6000 – checked by bootloader     */
    uint8_t  reserved[8];
} app_descriptor_t;

/* Placed in .app_descriptor linker section (see app_descriptor.c) */
extern const app_descriptor_t app_descriptor;

#endif /* APP_DESCRIPTOR_H */
