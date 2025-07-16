#!/usr/bin/env python3
import os
import time
import signal
import threading
import logging
from datetime import datetime, timezone
import serial
import pynmea2
import gpxpy
import gpxpy.gpx
from picamera2 import Picamera2

# Thread class to continuously read NMEA sentences from the GPS serial port
class GPSReader(threading.Thread):
    def __init__(self, port='/dev/ttyS0', baudrate=9600):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
        self.lock = threading.Lock()
        self.lat = None
        self.lon = None
        self.alt = None
        self.fix = False
        self.gps_time = None # To store GPS timestamp


    def run(self):
        while True:
            line = self.serial.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)
                    # Check for valid fix
                    valid = False
                    if line.startswith('$GPGGA') and hasattr(msg, 'gps_qual'):
                        valid = int(msg.gps_qual) > 0
                    elif line.startswith('$GPRMC') and hasattr(msg, 'status'):
                        valid = (msg.status == 'A')

                    if valid and hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        with self.lock:
                            self.lat = msg.latitude
                            self.lon = msg.longitude
                            self.fix = True
                            if hasattr(msg, 'altitude'):
                                self.alt = msg.altitude
                except pynmea2.ParseError as e:
                    logging.warning(f"NMEA parse error: {e}")


def main():
    # Set up session directory
    session_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    base_dir = os.path.join('/home/pi/project/data', session_time)
    print(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(base_dir, 'session.log')),
            logging.StreamHandler()
        ]
    )

    # Start GPS reader thread
    gps_reader = GPSReader()
    gps_reader.start()

    # Prepare GPX track
    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    segment = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(segment)

    # Initialize camera
    camera = Picamera2()
    config = camera.create_still_configuration()
    camera.configure(config)
    camera.start()

    logging.info("Starting logging; press Ctrl+C to stop.")
    try:
        while True:
            with gps_reader.lock:
                lat = gps_reader.lat
                lon = gps_reader.lon
                alt = gps_reader.alt
                fix = gps_reader.fix

            if fix and lat is not None and lon is not None:
                timestamp = datetime.utcnow().replace(tzinfo=timezone.utc)
                # Add point to GPX
                pt = gpxpy.gpx.GPXTrackPoint(lat, lon,
                                            elevation=alt,
                                            time=timestamp)
                segment.points.append(pt)
                logging.info(f"Logged {lat:.6f}, {lon:.6f} at {timestamp.isoformat()}")

                # Capture an image alongside GPS point
                img_filename = f"img_{timestamp.strftime('%Y%m%d_%H%M%S')}.jpg"
                img_path = os.path.join(base_dir, img_filename)
                camera.capture_file(img_path)
                logging.info(f"Captured image {img_path}")
            else:
                logging.info("Waiting for GPS fix...")

            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Interrupt received, stopping logging and saving track...")
        # Ignore further Ctrl+C during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # Cleanup camera
        try:
            camera.stop()
            camera.close()
        except Exception:
            pass

        # Save GPX file
        gpx_filename = f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.gpx"
        gpx_path = os.path.join(base_dir, gpx_filename)
        try:
            with open(gpx_path, 'w') as f:
                f.write(gpx.to_xml())
            logging.info(f"Saved GPX track to {gpx_path}")
        except Exception as e:
            logging.error(f"Failed to save GPX track: {e}")

        logging.info("Exiting.")


if __name__ == '__main__':
    main()
