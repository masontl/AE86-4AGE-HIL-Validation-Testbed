# ==============================================================================
# LESSON 11: FAT16 SPI SD IMAGE EXTRACTOR (hachi_read_SDimg_create_csv.py)
# Reads 'HACHI11.CSV' (13 channels including Loop_CL) directly out of
# SimulIDE's 'hachi_sd_fat16.img' binary and writes 'VIRTUAL_SD_512MB.CSV'.
# ==============================================================================

import struct
import csv

IMG_FILENAME = "hachi_sd_fat16.img"
OUT_CSV_FILENAME = "VIRTUAL_SD_512MB.CSV"

PARTITION_START_LBA = 63
RESERVED_SECTORS = 1
NUM_FATS = 2
SECTORS_PER_FAT = 64
ROOT_DIR_ENTRIES = 512
SECTORS_PER_CLUSTER = 4
BYTES_PER_SECTOR = 512


def main():
    root_dir_lba = PARTITION_START_LBA + RESERVED_SECTORS + (NUM_FATS * SECTORS_PER_FAT)
    data_start_lba = root_dir_lba + ((ROOT_DIR_ENTRIES * 32) // BYTES_PER_SECTOR)
    fat1_offset = (PARTITION_START_LBA + RESERVED_SECTORS) * BYTES_PER_SECTOR

    with open(IMG_FILENAME, "rb") as f:
        f.seek(root_dir_lba * BYTES_PER_SECTOR)
        root_data = f.read(ROOT_DIR_ENTRIES * 32)

        target_cluster = None
        file_size = 0
        found_name = ""

        # Prefer HACHI11.CSV, fallback to HACHI10.CSV if needed
        for target_file in ("HACHI11.CSV", "HACHI10.CSV"):
            for i in range(0, len(root_data), 32):
                entry = root_data[i:i+32]
                if entry[0] in (0x00, 0xE5) or entry[11] == 0x0F:
                    continue
                name = entry[0:8].decode("ascii", errors="ignore").strip()
                ext  = entry[8:11].decode("ascii", errors="ignore").strip()
                if f"{name}.{ext}".upper() == target_file:
                    target_cluster = struct.unpack_from("<H", entry, 26)[0]
                    file_size = struct.unpack_from("<I", entry, 28)[0]
                    found_name = target_file
                    break
            if target_cluster and file_size > 0:
                break

        if not target_cluster or file_size == 0:
            print("[!] HACHI11.CSV not found (or 0 bytes) inside hachi_sd_fat16.img.")
            print("    -> Make sure you ran SimulIDE for at least 1 second (20 frames) so logFile.flush() executed!")
            return

        # Follow FAT16 cluster chain
        raw_bytes = bytearray()
        cluster = target_cluster
        while 2 <= cluster < 0xFFF8 and len(raw_bytes) < file_size:
            cluster_lba = data_start_lba + (cluster - 2) * SECTORS_PER_CLUSTER
            f.seek(cluster_lba * BYTES_PER_SECTOR)
            raw_bytes.extend(f.read(SECTORS_PER_CLUSTER * BYTES_PER_SECTOR))

            f.seek(fat1_offset + cluster * 2)
            cluster = struct.unpack("<H", f.read(2))[0]

    csv_text = raw_bytes[:file_size].replace(b"\x00", b"").decode("utf-8", errors="ignore")
    lines = [line.strip() for line in csv_text.splitlines() if line.strip()]

    valid_frames = 0
    skipped_torn_lines = 0

    with open(OUT_CSV_FILENAME, "w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow([
            "Host_Timestamp", "SimTime_ms", "RPM", "TPS_Pct", "Raw_THW",
            "Coolant_F", "TVIS", "Fan", "Live_DTC", "Latched_DTC",
            "OX_mV", "VF_mV", "Fuel_Trim_Pct", "Loop_CL"
        ])

        for line in lines:
            if line.startswith("SimTime_ms"):
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) not in (12, 13):
                skipped_torn_lines += 1
                continue

            try:
                int_vals = [int(x) for x in parts]
            except ValueError:
                skipped_torn_lines += 1
                continue

            # Backwards-compatibility if reading a 12-column log
            if len(int_vals) == 12:
                loop_cl = 1 if (int_vals[4] >= 140 and int_vals[10] >= 350) else 0
                int_vals.append(loop_cl)

            sim_ms, rpm = int_vals[0], int_vals[1]
            if sim_ms < 0 or rpm < 0 or rpm > 10000:
                skipped_torn_lines += 1
                continue

            sim_s = f"{sim_ms / 1000.0:.3f}"
            writer.writerow([sim_s] + int_vals)
            valid_frames += 1

    print("=" * 78)
    print(f"[+] SPI SD IMAGE EXTRACTION COMPLETE: '{found_name}' -> '{OUT_CSV_FILENAME}'")
    print(f"    - FAT16 File Size Read       : {file_size:,} bytes")
    print(f"    - Valid 20Hz Frames Extracted: {valid_frames:,} frames ({valid_frames * 0.05:.2f}s)")
    if skipped_torn_lines > 0:
        print(f"    - Torn/Null Boundary Lines   : {skipped_torn_lines} sanitized & skipped")
    print("=" * 78)
    print("    -> Now run 'sd_post_processor.py' to run the 3-Layer Mission Dissector!")


if __name__ == "__main__":
    main()