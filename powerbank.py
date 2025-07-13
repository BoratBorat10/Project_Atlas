#!/usr/bin/env python3
import os
import time
import subprocess
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
from picamera2 import Picamera2

def setup_logger(log_folder: str):
    os.makedirs(log_folder, exist_ok=True)
    logfile = os.path.join(log_folder, "timelapse.log")
    handler = RotatingFileHandler(logfile, maxBytes=1_000_000, backupCount=3)
    fmt = "%(asctime)s [%(levelname)s] %(message)s"
    handler.setFormatter(logging.Formatter(fmt))
    logger = logging.getLogger("timelapse")
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    return logger

def get_free_mb(path: str) -> float:
    stat = os.statvfs(path)
    return stat.f_bavail * stat.f_frsize / 1024**2

def main():
    # setup
    start_dt = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_folder = f"timelapse_{start_dt}"
    log_folder    = "logs"
    logger = setup_logger(log_folder)
    logger.info("===== Starting timelapse =====")
    logger.info(f"Output folder: {output_folder}")

    os.makedirs(output_folder, exist_ok=True)

    # camera
    picam2 = Picamera2()
    cfg    = picam2.create_still_configuration(main={"size": (1920,1080)})
    picam2.configure(cfg)
    picam2.align_configuration(cfg)
    picam2.start()
    logger.info("Camera started")

    # timing
    interval   = 1               # seconds between frames
    total_secs = 30 * 60
    end_time   = time.time() + total_secs
    shot_count = 0

    try:
        while time.time() < end_time:
            loop_start = time.time()

            # check disk
            free_mb = get_free_mb(output_folder)
            if free_mb < 50:
                logger.warning(f"Low disk space: {free_mb:.1f} MB free, stopping early")
                break

            # capture
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(output_folder, f"{ts}.jpg")
            picam2.capture_file(path)
            shot_count += 1
            logger.info(f"Captured {shot_count}: {path} ({free_mb:.1f} MB free)")

            # sleep to maintain 1 fps
            elapsed = time.time() - loop_start
            to_sleep = interval - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except Exception as e:
        logger.exception("Error during capture loop")

    finally:
        picam2.stop()
        logger.info(f"Camera stopped, total frames: {shot_count}")

        # shutdown
        if os.geteuid() == 0:
            logger.info("Shutting down system")
            subprocess.run(["shutdown","-h","now"], check=True)
        else:
            logger.warning("Not root, skipping shutdown")

if __name__ == "__main__":
    main()
