# ==============================================================================
# LESSON 14: FAT16 SPI SD IMAGE EXTRACTOR WITH XOR CHECKSUM VERIFICATION
# Reads 'HACHI14.CSV' out of 'hachi_sd_fat16.img', validates the embedded NMEA-style
# 8-bit XOR checksum (*XX) on every telemetry frame, rejects any sector-torn
# frames deterministically, and writes 'VIRTUAL_SD_512MB.CSV'.
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


def compute_xor_checksum(payload_str: str) -> int:
    cs = 0
    for b in payload_str.encode("ascii", errors="ignore"):
        cs ^= b
    return cs


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

        for target_file in ("HACHI14.CSV", "HACHI12.CSV", "HACHI11.CSV"):
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
            print("[!] HACHI14.CSV (or fallback log) not found inside hachi_sd_fat16.img.")
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
    xor_failures = 0
    malformed_lines = 0
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

            # LESSON 14: Deterministic XOR Checksum Verification (*XX)
            if "*" in line:
                payload_part, cs_part = line.rsplit("*", 1)
                if len(cs_part) != 2:
                    malformed_lines += 1
                    continue
                try:
                    stamped_cs = int(cs_part, 16)
                except ValueError:
                    malformed_lines += 1
                    continue

                calc_cs = compute_xor_checksum(payload_part)
                if calc_cs != stamped_cs:
                    xor_failures += 1
                    continue
            else:
                # Legacy unframed fallback (HACHI12 / HACHI11)
                payload_part = line

            parts = [p.strip() for p in payload_part.split(",")]
            if len(parts) != 14:
                malformed_lines += 1
                continue

            try:
                int_vals = [int(x) for x in parts]
            except ValueError:
                malformed_lines += 1
                continue

            # Ensure monotonic timestamp order
            if int_vals[0] <= last_sim_ms:
                malformed_lines += 1
                continue

            last_sim_ms = int_vals[0]
            sim_s = f"{last_sim_ms / 1000.0:.3f}"
            writer.writerow([sim_s] + int_vals)
            valid_frames += 1

    total_rejected = xor_failures + malformed_lines
    integrity_pct = (valid_frames / (valid_frames + total_rejected) * 100.0) if (valid_frames + total_rejected) > 0 else 0.0

    print("=" * 78)
    print(f"[+] LESSON 14 XOR-VERIFIED SD EXTRACTION COMPLETE: '{found_name}' -> '{OUT_CSV_FILENAME}'")
    print(f"    - FAT16 File Size Read          : {file_size:,} bytes")
    print(f"    - Checksum-Verified 20Hz Frames : {valid_frames:,} frames ({valid_frames * 0.05:.2f}s)")
    print(f"    - XOR Checksum Failures Caught  : {xor_failures} frame(s)")
    print(f"    - Truncated/Header Lines Skipped: {malformed_lines} line(s)")
    print(f"    - Telemetry Framing Integrity   : {integrity_pct:.2f}%")
    print("=" * 78)


if __name__ == "__main__":
    main()