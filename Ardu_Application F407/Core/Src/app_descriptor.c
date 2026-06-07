#include "app_descriptor.h"

/* Placed immediately after the vector table (.app_descriptor section in linker script).
 * image_crc1, image_crc2, and image_size are left as zero here and patched into the
 * .bin file by patch_descriptor.py as a post-build step.
 */
__attribute__((section(".app_descriptor"), used))
const app_descriptor_t app_descriptor =
{
    .sig          = APP_DESCRIPTOR_SIG_UNSIGNED,
    .image_crc1   = 0x00000000U,   /* patched by patch_descriptor.py */
    .image_crc2   = 0x00000000U,   /* patched by patch_descriptor.py */
    .image_size   = 0x00000000U,   /* patched by patch_descriptor.py */
    .git_hash     = 0x00000000U,
    .version_major = APP_VERSION_MAJOR,
    .version_minor = APP_VERSION_MINOR,
    .board_id     = APP_BOARD_ID,   /* 6000 = AP_HW_sw-spar-f407 */
    .reserved     = {0}
};
