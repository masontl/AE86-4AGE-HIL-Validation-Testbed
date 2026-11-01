# ==============================================================================
# STEP 1: PURE-PYTHON 32MB FAT16 SD CARD IMAGE BUILDER (create_fat16_sd_img.py)
# Creates a standards-compliant MBR + FAT16 disk image ('hachi_sd_fat16.img')
# ready to load into SimulIDE's SPI SD Card component!
# ==============================================================================

import struct
import os

IMG_FILENAME = "hachi_sd_fat16.img"
TOTAL_SECTORS = 65536          # 65,536 sectors * 512 bytes = 32 MB
BYTES_PER_SECTOR = 512
PARTITION_START_LBA = 63       # Standard MBR partition offset
PART_SECTORS = TOTAL_SECTORS - PARTITION_START_LBA
SECTORS_PER_CLUSTER = 4        # 2 KB clusters (~16,300 clusters -> True FAT16!)
RESERVED_SECTORS = 1
NUM_FATS = 2
ROOT_DIR_ENTRIES = 512
SECTORS_PER_FAT = 64


def main():
    print(f"[*] Building 32 MB FAT16 SD Card Image: '{IMG_FILENAME}'...")
    with open(IMG_FILENAME, "wb") as f:
        #Pre-allocate 32 MB zeroed image
        f.seek(TOTAL_SECTORS * BYTES_PER_SECTOR - 1)
        f.write(b"\x00")

        # 1. Write Master Boot Record (MBR) at LBA 0
        mbr = bytearray(512)
        # Partition 1 entry at offset 446 (0x1BE): Active=0x80, Type=0x06 (FAT16B)
        struct.pack_into("<BBBBBBBBII", mbr, 446,
                         0x80, 0x01, 0x01, 0x00, 0x06, 0x0F, 0x3F, 0x20,
                         PARTITION_START_LBA, PART_SECTORS)
        mbr[510:512] = b"\x55\xAA"
        f.seek(0)
        f.write(mbr)

        # 2. Write FAT16 Volume Boot Record (PBR) at LBA 63
        pbr = bytearray(512)
        pbr[0:3] = b"\xEB\x3C\x90"
        pbr[3:11] = b"MSDOS5.0"
        struct.pack_into("<HBHBHHBHHHII", pbr, 11,
                         BYTES_PER_SECTOR,     # 512
                         SECTORS_PER_CLUSTER,  # 4
                         RESERVED_SECTORS,     # 1
                         NUM_FATS,             # 2
                         ROOT_DIR_ENTRIES,     # 512
                         0,                    # TotalSectors16 (0 -> use 32-bit field)
                         0xF8,                 # Media Descriptor (Fixed/SD)
                         SECTORS_PER_FAT,      # 64
                         63,                   # Sectors per track
                         16,                   # Heads
                         PARTITION_START_LBA,  # Hidden sectors (63)
                         PART_SECTORS)         # TotalSectors32
        struct.pack_into("<BBBI11s8s", pbr, 36,
                         0x80, 0x00, 0x29, 0x19860086,
                         b"HACHI_FDR  ", b"FAT16   ")
        pbr[510:512] = b"\x55\xAA"
        f.seek(PARTITION_START_LBA * BYTES_PER_SECTOR)
        f.write(pbr)

        # 3. Initialize both FAT16 tables with Media Descriptor (F8 FF FF FF)
        for fat_idx in range(NUM_FATS):
            fat_offset = (PARTITION_START_LBA + RESERVED_SECTORS + fat_idx * SECTORS_PER_FAT) * BYTES_PER_SECTOR
            f.seek(fat_offset)
            f.write(b"\xF8\xFF\xFF\xFF")

    print(f"[+] SUCCESS! Created '{os.path.abspath(IMG_FILENAME)}' (32.0 MB FAT16).")
    print("    -> Right-click the SD Card in SimulIDE -> Properties -> Image File -> Select this .img!")


if __name__ == "__main__":
    main()