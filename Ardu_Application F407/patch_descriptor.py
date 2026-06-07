#!/usr/bin/env python3
"""
patch_descriptor.py – ArduPilot bootloader app-descriptor CRC patcher.

Flash layout (STM32F407, bootloader = 64 KB):
  0x08000000 – 0x0800FFFF  Bootloader (64 KB, sectors 0-3)
                            hwdef-bl.dat: FLASH_BOOTLOADER_LOAD_KB 64
  0x08010000               APP_START_ADDRESS – application starts here
                            ISR vector table at offset 0  (SP + Reset_Handler)
                            App descriptor immediately after ISR vector
  0x08010000+              Application code (960 KB)

The arm-none-eabi-objcopy binary output starts at 0x08010000 (the lowest
load address in the single FLASH region).  The .bin file therefore begins
with the ISR vector table, followed immediately by the app descriptor, then
the rest of the application code.

Usage (Post-build step in CubeIDE):
  python3 "${ProjDirPath}/patch_descriptor.py" "${ProjName}.bin"

The script:
  1. Finds the 8-byte unsigned descriptor signature in the binary.
  2. Calculates CRC1  (from byte 0 of the .bin up to the image_crc1 field).
  3. Calculates CRC2  (from the version_major field to the end of the binary).
  4. Patches image_crc1, image_crc2, and image_size into the binary in-place.

CRC algorithm: IEEE 802.3 CRC32, initial value 0, no final XOR.
This matches crc32_small(0, ...) used in AP_CheckFirmware.cpp.
"""

import sys
import os
import struct

# ---- CRC32 table (IEEE 802.3 / reflected 0xEDB88320, init=0) ----
_CRC32_TABLE = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ (0xEDB88320 if (_c & 1) else 0)
    _CRC32_TABLE.append(_c)

def crc32(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc = _CRC32_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc & 0xFFFFFFFF


# ---- Descriptor constants (must match app_descriptor.h) ----
SIGNATURE = bytes([0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06])
EXPECTED_BOARD_ID = 6000  # AP_HW_sw-spar-f407

# Flash base address where the .bin file originates (APP_START_ADDRESS)
BIN_FLASH_BASE = 0x08010000

# Descriptor field byte offsets relative to start of signature:
#  [0 ] sig[8]
#  [8 ] image_crc1   (uint32 LE)   <-- CRC1 covers [bin_offset_0 .. here)
#  [12] image_crc2   (uint32 LE)
#  [16] image_size   (uint32 LE)
#  [20] git_hash     (uint32 LE)
#  [24] version_major (uint8)      <-- CRC2 covers [here .. image_size)
#  [25] version_minor (uint8)
#  [26] board_id     (uint16 LE)
#  [28] reserved[8]
# Total: 36 bytes
OFFSET_CRC1          = 8
OFFSET_CRC2          = 12
OFFSET_IMAGE_SIZE    = 16
OFFSET_VERSION_MAJOR = 24
OFFSET_BOARD_ID      = 26


def patch(bin_path: str) -> None:
    if not os.path.isfile(bin_path):
        print(f"ERROR: file not found: {bin_path}")
        sys.exit(1)

    with open(bin_path, "rb") as f:
        fw = bytearray(f.read())

    pos = fw.find(SIGNATURE)
    if pos < 0:
        print("ERROR: app descriptor signature not found in binary!")
        print("       Make sure app_descriptor.c is compiled and linked via .app_descriptor section.")
        sys.exit(1)

    print(f"  Descriptor found at .bin offset 0x{pos:06X}  "
          f"(flash address 0x{BIN_FLASH_BASE + pos:08X})")

    # Verify board_id
    board_id = struct.unpack_from("<H", fw, pos + OFFSET_BOARD_ID)[0]
    if board_id != EXPECTED_BOARD_ID:
        print(f"ERROR: board_id in descriptor is {board_id}, expected {EXPECTED_BOARD_ID}")
        sys.exit(1)

    image_size = len(fw)

    # CRC1: bytes [0 .. pos+OFFSET_CRC1)
    #   .bin starts at 0x08010000 (ISR vector), descriptor follows after the vector table.
    #   CRC1 covers everything from byte 0 of the .bin up to (not including) image_crc1.
    len1 = pos + OFFSET_CRC1

    # CRC2: bytes [pos+OFFSET_VERSION_MAJOR .. image_size)
    #   Covers version, board_id, reserved, the 32 KB gap (0xFF), and all app code.
    crc2_start = pos + OFFSET_VERSION_MAJOR
    len2 = image_size - crc2_start

    if len2 <= 0:
        print("ERROR: firmware too small – descriptor is at the very end of the binary.")
        sys.exit(1)

    if len1 > image_size:
        print("ERROR: image_size / descriptor layout sanity check failed.")
        sys.exit(1)

    crc1 = crc32(fw[:len1])
    crc2 = crc32(fw[crc2_start: crc2_start + len2])

    print(f"  image_size = {image_size} bytes  (0x{image_size:06X})")
    print(f"  CRC1 region [0x000000 .. 0x{len1-1:06X}] ({len1} bytes) = 0x{crc1:08X}")
    print(f"  CRC2 region [0x{crc2_start:06X} .. 0x{crc2_start+len2-1:06X}] ({len2} bytes) = 0x{crc2:08X}")

    # Patch the three fields in-place
    struct.pack_into("<I", fw, pos + OFFSET_CRC1,       crc1)
    struct.pack_into("<I", fw, pos + OFFSET_CRC2,       crc2)
    struct.pack_into("<I", fw, pos + OFFSET_IMAGE_SIZE, image_size)

    with open(bin_path, "wb") as f:
        f.write(fw)

    print(f"  Descriptor patched successfully -> {bin_path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin>")
        sys.exit(1)

    print(f"[patch_descriptor] Patching: {sys.argv[1]}")
    patch(sys.argv[1])
    print("[patch_descriptor] Done.")