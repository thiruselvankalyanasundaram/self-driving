"""
Robust Multi-Threaded CARLA Simulator Downloader with Auto-Resume and Retries.
Downloads CARLA 0.9.15 for Windows using concurrent HTTP range requests,
verifying exact part sizes before extraction.
"""

import os
import sys
import time
import zipfile
import urllib.request

CARLA_WIN_URL = "https://carla-releases.s3.us-east-005.backblazeb2.com/Windows/CARLA_0.9.15.zip"
TARGET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "carla_simulator")
ZIP_PATH = os.path.join(TARGET_DIR, "CARLA_0.9.15.zip")

NUM_WORKERS = 4  # 4 stable parallel streams

def get_remote_file_size(url: str) -> int:
    req = urllib.request.Request(url, method="HEAD")
    req.add_header("User-Agent", "Mozilla/5.0")
    with urllib.request.urlopen(req, timeout=30) as resp:
        return int(resp.headers.get("Content-Length", 0))

def download_part(url: str, start: int, end: int, part_path: str, max_retries: int = 15):
    """Downloads a specific byte range with auto-resume on disconnect."""
    expected_size = end - start + 1

    for attempt in range(1, max_retries + 1):
        existing_size = os.path.getsize(part_path) if os.path.exists(part_path) else 0
        if existing_size >= expected_size:
            return True

        curr_start = start + existing_size
        headers = {
            "Range": f"bytes={curr_start}-{end}",
            "User-Agent": "Mozilla/5.0"
        }
        req = urllib.request.Request(url, headers=headers)

        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                with open(part_path, "ab") as f:
                    while True:
                        buf = resp.read(512 * 1024)  # 512 KB buffer
                        if not buf:
                            break
                        f.write(buf)
                        f.flush()

            if os.path.getsize(part_path) >= expected_size:
                return True
        except Exception as e:
            time.sleep(min(attempt * 2, 10))

    return os.path.exists(part_path) and os.path.getsize(part_path) >= expected_size

def main():
    os.makedirs(TARGET_DIR, exist_ok=True)

    # Check if already extracted
    carla_exe = os.path.join(TARGET_DIR, "CarlaUE4.exe")
    if os.path.exists(carla_exe):
        print("[CARLA Downloader] CarlaUE4.exe already extracted and ready!")
        return

    print("=" * 60)
    print(" CARLA 0.9.15 (Windows) Robust Downloader & Extractor")
    print("=" * 60)

    total_size = get_remote_file_size(CARLA_WIN_URL)
    total_gb = total_size / (1024 ** 3)
    print(f"Total archive size: {total_gb:.2f} GB ({total_size:,} bytes)")
    print(f"Connecting with {NUM_WORKERS} parallel streams (with auto-resume)...")

    chunk_size = total_size // NUM_WORKERS
    ranges = []
    part_files = []

    for i in range(NUM_WORKERS):
        start = i * chunk_size
        end = (start + chunk_size - 1) if i < NUM_WORKERS - 1 else (total_size - 1)
        part_file = f"{ZIP_PATH}.part{i}"
        ranges.append((start, end, part_file))
        part_files.append(part_file)

    from concurrent.futures import ThreadPoolExecutor
    start_time = time.time()

    with ThreadPoolExecutor(max_workers=NUM_WORKERS) as executor:
        futures = [
            executor.submit(download_part, CARLA_WIN_URL, r[0], r[1], r[2])
            for r in ranges
        ]

        while True:
            downloaded = sum(os.path.getsize(p) for p in part_files if os.path.exists(p))
            elapsed = max(time.time() - start_time, 0.1)
            speed_mb = (downloaded / (1024 * 1024)) / elapsed
            percent = (downloaded / total_size) * 100 if total_size > 0 else 0

            sys.stdout.write(
                f"\rProgress: [{downloaded / (1024**3):.2f}/{total_gb:.2f} GB] "
                f"({percent:.1f}%) | Speed: {speed_mb:.2f} MB/s | Elapsed: {int(elapsed)}s"
            )
            sys.stdout.flush()

            if all(f.done() for f in futures):
                break
            time.sleep(1.0)

    # Validate all parts
    all_ok = all(f.result() for f in futures)
    if not all_ok:
        print("\n[Error] Some download chunks failed. Please rerun the script to resume.")
        sys.exit(1)

    print("\n\nAll parts verified! Merging chunks into CARLA_0.9.15.zip...")
    if os.path.exists(ZIP_PATH):
        os.remove(ZIP_PATH)

    with open(ZIP_PATH, "wb") as outfile:
        for part_file in part_files:
            with open(part_file, "rb") as infile:
                while True:
                    buf = infile.read(32 * 1024 * 1024)  # 32 MB buffer
                    if not buf:
                        break
                    outfile.write(buf)
            os.remove(part_file)

    print(f"Merged archive size: {os.path.getsize(ZIP_PATH) / (1024**3):.2f} GB")
    print("Verifying ZIP archive integrity before extraction...")
    if not zipfile.is_zipfile(ZIP_PATH):
        print("[Error] Merged file is not a valid ZIP archive (possible disk-full or merge error).")
        print("        Please delete the partial file and rerun the script to restart the download.")
        sys.exit(1)

    print("Extracting CarlaUE4 simulator into carla_simulator/...")
    with zipfile.ZipFile(ZIP_PATH, 'r') as zip_ref:
        zip_ref.extractall(TARGET_DIR)

    print("\n Extraction complete! CarlaUE4.exe is fully ready in carla_simulator/.")

if __name__ == "__main__":
    main()
