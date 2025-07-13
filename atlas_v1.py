#!/usr/bin/env python3
import os
import time
import threading
import subprocess
import logging
from datetime import datetime
import serial
import pynmea2
import gpxpy
import gpxpy.gpx
from picamera2 import Picamera2

# 1) wait for GPS time, then set system clock
def sync_time_and_mkdir(base_path="/home/pi"):
    ser = serial.Serial("/dev/serial0", 9600, timeout=1)
    print("Starting serial connection")
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        print(line)
        if not line or not line.startswith("$GPRMC"):
            continue
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            continue
        if msg.status == "A":  # data valid
            print("Got a fix!")
            # build datetime from RMC
            dt = datetime.combine(msg.datestamp, msg.timestamp)
            print("Time from GPS", dt)
            # set system time (requires sudo)
            timestr = dt.strftime("%Y-%m-%d %H:%M:%S")
            subprocess.run(["sudo", "timedatectl", "set-time", timestr], check=True)
            print("set system time to:", timestr)
            # make folder named YYYYMMDD_HHMMSS
            folder = os.path.join(base_path, dt.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(folder, exist_ok=True)
            print("Created Folder:", folder)
            ser.close()
            return folder

'''# 2) camera thread
def camera_job(folder):
    cam = PiCamera()
    cam.resolution = (1920, 1080)
    cam.framerate = 1
    for frame in cam.capture_continuous(os.path.join(folder, "img_{timestamp:%Y%m%d_%H%M%S}.jpg")):
        # picamera inserts timestamp automatically
        time.sleep(1)'''


def camera_job(folder):
    # Parameters
    interval = 1           # seconds between shots
    total_duration = 30    # total run time in min
    frames = (total_duration * 60) // interval

    print("Starting Camera:", total_duration, " min. At", interval, "fps. Total:", frames )

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

# 3) GPS→GPX thread
def gps_job(folder):
    ser = serial.Serial("/dev/serial0", 9600, timeout=1)
    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    seg = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(seg)
    gpx.tracks.append(track)
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            continue
        if isinstance(msg, pynmea2.types.talker.GGA):
            t = datetime.utcnow()
            lat, lon, ele = msg.latitude, msg.longitude, msg.altitude
            seg.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon, elevation=ele, time=t))
            # overwrite GPX each fix
            with open(os.path.join(folder, "track.gpx"), "w") as f:
                f.write(gpx.to_xml())

if __name__ == "__main__":
    base = "/home/pi"
    fld = sync_time_and_mkdir(base)
    # start both jobs
    threading.Thread(target=camera_job, args=(fld,), daemon=True).start()
    threading.Thread(target=gps_job,    args=(fld,), daemon=True).start()
    # keep the main thread alive
    while True:
        time.sleep(60)
