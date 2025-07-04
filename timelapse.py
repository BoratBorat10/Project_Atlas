#!/usr/bin/env python3
import time
import os
import shutil
from datetime import datetime
from picamera2 import Picamera2


def main():
    # Parameters
    interval = 1           # seconds between shots
    total_duration = 30    # total run time in seconds
    frames = total_duration // interval

    # Create output folder named with start datetime
    start_dt = datetime.now().strftime("%Y%m%d_%H%M%S")
    # folder = f"timelapse_{start_dt}"
    # Wipe each time for testing:
    folder = "test_folder"
    if os.path.isdir(folder):
        shutil.rmtree(folder)

    os.makedirs(folder, exist_ok=True)

    # Configure and start camera
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1920, 1080)})

    picam2.configure(config)
    picam2.align_configuration(config)

    picam2.start()

    # Capture loop
    for i in range(int(frames)):
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(folder, f"{now}.jpg")
        picam2.capture_file(filename)
        print(f"Captured {filename} ({i+1}/{int(frames)})")
        time.sleep(interval)

    # Clean up
    picam2.stop()
    print(f"Time-lapse complete, images in ./{folder}")

if __name__ == "__main__":
    main()
