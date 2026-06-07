#!/usr/bin/env python3
"""
make_apj.py – Wrap a patched CubeIDE .bin into an ArduPilot .apj file.

The .apj format is the container Mission Planner uses for DroneCAN firmware
uploads.  It is a JSON file whose "image" field holds the firmware binary
compressed with zlib and then base64-encoded.

Run this AFTER patch_descriptor.py so the descriptor CRCs are already correct.

Flash layout (STM32F407, bootloader = 64 KB):
  0x08010000  APP_START_ADDRESS – .bin starts here (ISR vector + app descriptor)
  0x080FFFFF  End of 960 KB application space

Usage (post-build, second step after patch_descriptor.py):
  python3 "${ProjDirPath}/make_apj.py" "${ProjName}.bin"

Outputs:
  <firmware>.apj  in the same directory as the input .bin

Board-specific constants (must match hwdef.dat for sw-spar-f407):
  board_id      = 6000
  flash_total   = 983040  (0x08010000 – end of 1 MB flash = 960 KB = 983040 B)
"""

import sys
import os
import json
import zlib
import base64
import struct

# ---- Board constants – must match hwdef.dat ---------------------
BOARD_ID     = 6000
FLASH_TOTAL  = 983040   # 960 KB application space in bytes
SUMMARY      = "sw-spar-f407"
DESCRIPTION  = "Firmware for a STM32F407xx board"
MAGIC        = "APJFWv1"

# ---- Descriptor signature (to verify the binary is patched) -----
SIGNATURE    = bytes([0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06])


def make_apj(bin_path: str) -> None:
    if not os.path.isfile(bin_path):
        print(f"ERROR: file not found: {bin_path}")
        sys.exit(1)

    with open(bin_path, "rb") as f:
        binary = f.read()

    # Sanity: descriptor must already be patched by patch_descriptor.py
    pos = binary.find(SIGNATURE)
    if pos < 0:
        print("ERROR: descriptor signature not found.")
        print("       Run patch_descriptor.py before make_apj.py.")
        sys.exit(1)

    # Check CRCs are non-zero (i.e. descriptor was patched)
    crc1 = struct.unpack_from("<I", binary, pos + 8)[0]
    crc2 = struct.unpack_from("<I", binary, pos + 12)[0]
    if crc1 == 0 or crc2 == 0:
        print("ERROR: descriptor CRCs are zero – run patch_descriptor.py first.")
        sys.exit(1)

    image_size = len(binary)

    # Compress with zlib (level 9 to match ArduPilot toolchain output)
    compressed = zlib.compress(binary, level=9)

    # Base64-encode (standard alphabet, no newlines)
    image_b64 = base64.b64encode(compressed).decode("ascii")

    # Build the APJ JSON – field order matches AP_Periph.apj exactly
    apj = {
        "board_id":         BOARD_ID,
        "magic":            MAGIC,
        "description":      DESCRIPTION,
        "summary":          SUMMARY,
        "version":          "1.0",
        "image_size":       image_size,
        "flash_total":      FLASH_TOTAL,
        "image_maxsize":    FLASH_TOTAL,
        "image":            image_b64,
    }

    # Write .apj alongside the .bin
    apj_path = os.path.splitext(bin_path)[0] + ".apj"
    with open(apj_path, "w") as f:
        json.dump(apj, f, indent=2)

    print(f"[make_apj] Input .bin:    {bin_path}  ({image_size} bytes)")
    print(f"[make_apj] Compressed:    {len(compressed)} bytes  "
          f"({100*len(compressed)//image_size}% of original)")
    print(f"[make_apj] Output .apj:   {apj_path}")
    print(f"[make_apj] board_id:      {BOARD_ID}")
    print(f"[make_apj] flash_total:   {FLASH_TOTAL}")
    print("[make_apj] Done – use this .apj in Mission Planner for CAN firmware upload.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin>")
        sys.exit(1)

    make_apj(sys.argv[1])
