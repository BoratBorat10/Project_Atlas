#!/usr/bin/env python3

from picamera2 import Picamera2
from time import sleep
from pathlib import Path

# Configuration
DURATION_SEC = 30
INTERVAL_SEC = 1
OUTPUT_DIR   = Path.home() / "timelapse"

def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()
    sleep(2)  # sensor warm-up

    shots = DURATION_SEC // INTERVAL_SEC
    for i in range(shots):
        filename = OUTPUT_DIR / f"img_{i:03d}.jpg"
        picam2.capture_file(str(filename))
        print("Captured", filename)
        sleep(INTERVAL_SEC)

    picam2.stop()

if __name__ == "__main__":
    main()
