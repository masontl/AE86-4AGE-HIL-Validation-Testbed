# ==============================================================================
# LESSON 13.1: FAT16 SPI SD IMAGE EXTRACTOR WITH SECTOR-SPLICE SANITIZATION
# Reads 'HACHI12.CSV' out of 'hachi_sd_fat16.img', sanitizes torn FAT16 sector
# splices using physical range + monotonic clock gates, and writes 'VIRTUAL_SD_512MB.CSV'.
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


def is_physically_valid_frame(vals, last_sim_ms):
    """
    Rejects torn/spliced ASCII lines caused by 512-byte SPI SD sector boundaries.
    """
    sim_ms, rpm, tps, raw_thw, coolant_f, tvis, fan, live_dtc, latched_dtc, ox_mv, vf_mv, trim_pct, loop_cl, igf_ok = vals

    # 1. Monotonic clock & realistic 20Hz step check (allows brief SPI flush jitter up to 250ms)
    if sim_ms <= last_sim_ms:
        return False
    if last_sim_ms > 0 and (sim_ms - last_sim_ms) > 250:
        return False

    # 2. Physical sensor & actuator domain bounds (1986 4A-GE hardware limits)
    if not (500 <= rpm <= 8500):
        return False
    if not (0 <= tps <= 100):
        return False
    if not (0 <= raw_thw <= 1023):
        return False
    if not (coolant_f == -99 or (55 <= coolant_f <= 240)):
        return False
    if tvis not in (0, 1) or fan not in (0, 1) or loop_cl not in (0, 1) or igf_ok not in (0, 1):
        return False
    if live_dtc not in (0, 4, 7, 14) or latched_dtc not in (0, 4, 7, 14):
        return False
    if not (0 <= ox_mv <= 5000) or not (0 <= vf_mv <= 5000):
        return False
    if not (-25 <= trim_pct <= 25):
        return False

    return True


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

        for target_file in ("HACHI12.CSV", "HACHI11.CSV", "HACHI10.CSV"):
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
            print("[!] HACHI12.CSV not found (or 0 bytes) inside hachi_sd_fat16.img.")
            return

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
    last_sim_ms = 0

    with open(OUT_CSV_FILENAME, "w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow([
            "Host_Timestamp", "SimTime_ms", "RPM", "TPS_Pct", "Raw_THW",
            "Coolant_F", "TVIS", "Fan", "Live_DTC", "Latched_DTC",
            "OX_mV", "VF_mV", "Fuel_Trim_Pct", "Loop_CL", "IGf_OK"
        ])

        for line in lines:
            if line.startswith("SimTime_ms"):
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) not in (12, 13, 14):
                skipped_torn_lines += 1
                continue

            try:
                int_vals = [int(x) for x in parts]
            except ValueError:
                skipped_torn_lines += 1
                continue

            if len(int_vals) == 12:
                loop_cl = 1 if (int_vals[4] >= 140 and int_vals[10] >= 350) else 0
                int_vals.append(loop_cl)
            if len(int_vals) == 13:
                igf_ok = 0 if int_vals[7] == 14 else 1
                int_vals.append(igf_ok)

            if not is_physically_valid_frame(int_vals, last_sim_ms):
                skipped_torn_lines += 1
                continue

            last_sim_ms = int_vals[0]
            sim_s = f"{last_sim_ms / 1000.0:.3f}"
            writer.writerow([sim_s] + int_vals)
            valid_frames += 1

    print("=" * 78)
    print(f"[+] SPI SD IMAGE EXTRACTION COMPLETE: '{found_name}' -> '{OUT_CSV_FILENAME}'")
    print(f"    - FAT16 File Size Read       : {file_size:,} bytes")
    print(f"    - Valid 20Hz Frames Extracted: {valid_frames:,} frames ({valid_frames * 0.05:.2f}s)")
    print(f"    - Torn/Spliced Sector Lines  : {skipped_torn_lines} caught & sanitized!")
    print("=" * 78)


if __name__ == "__main__":
    main()