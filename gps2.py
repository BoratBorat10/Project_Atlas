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
import subprocess # Added for time synchronization

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
            try:
                line = self.serial.readline().decode('ascii', errors='replace').strip()
                if not line:
                    continue # Skip empty lines

                if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                    msg = pynmea2.parse(line)
                    # Check for valid fix
                    valid = False
                    current_lat = None
                    current_lon = None
                    current_alt = None
                    current_gps_time = None

                    if isinstance(msg, pynmea2.types.talker.GGA) and hasattr(msg, 'gps_qual'):
                        if int(msg.gps_qual) > 0:
                            valid = True
                            current_lat = msg.latitude
                            current_lon = msg.longitude
                            current_alt = msg.altitude
                            if msg.timestamp: # GGA also has timestamp
                                current_gps_time = datetime.combine(datetime.today().date(), msg.timestamp, tzinfo=timezone.utc)
                                logging.info(f"Current GPS time: {current_gps_time}")
                    elif isinstance(msg, pynmea2.types.talker.RMC) and hasattr(msg, 'status'):
                        if msg.status == 'A':
                            valid = True
                            current_lat = msg.latitude
                            current_lon = msg.longitude
                            # RMC doesn't have altitude directly, but it's okay if GGA provides it
                            if msg.datestamp and msg.timestamp:
                                current_gps_time = datetime.combine(msg.datestamp, msg.timestamp, tzinfo=timezone.utc)

                    if valid and current_lat is not None and current_lon is not None:
                        with self.lock:
                            self.lat = current_lat
                            self.lon = current_lon
                            self.fix = True
                            if current_alt is not None: # Update altitude only if provided by GGA
                                self.alt = current_alt
                            if current_gps_time:
                                self.gps_time = current_gps_time # Store the most recent GPS timestamp
                    else:
                        with self.lock:
                            self.fix = False # No valid fix or missing data

            except pynmea2.ParseError as e:
                logging.warning(f"NMEA parse error: {e} - Line: '{line}'")
            except serial.SerialException as e:
                logging.error(f"Serial error: {e}. Attempting to re-open port...")
                time.sleep(5) # Wait before retrying
                try:
                    self.serial.close()
                    self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
                except Exception as re_e:
                    logging.error(f"Failed to re-open serial port: {re_e}")
                    time.sleep(10) # Longer wait if re-open fails
            except Exception as e:
                logging.error(f"Unexpected error in GPSReader: {e}")
            time.sleep(0.1) # Small sleep to prevent busy-waiting if no data

def sync_system_time_with_gps(gps_reader_instance):
    logging.info("Waiting for valid GPS time to synchronize system clock...")
    start_time = time.time()
    while not gps_reader_instance.fix or gps_reader_instance.gps_time is None:
        if time.time() - start_time > 60: # Timeout after 60 seconds
            logging.warning("Timeout: Could not get a valid GPS time sync. Proceeding with current system time.")
            return False
        time.sleep(1) # Wait for GPS data to come in

    with gps_reader_instance.lock:
        gps_dt = gps_reader_instance.gps_time
    
    if gps_dt:
        timestr = gps_dt.strftime("%Y-%m-%d %H:%M:%S")
        logging.info(f"Attempting to set system time to GPS time: {timestr}")
        try:
            # Need to ensure this script runs with appropriate permissions (e.g., sudo)
            subprocess.run(["sudo", "timedatectl", "set-time", timestr], check=True)
            logging.info(f"System time synchronized to GPS: {timestr}")
            return True
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to set system time with timedatectl: {e}. Please ensure script is run with sudo or appropriate permissions.")
            return False
        except Exception as e:
            logging.error(f"An error occurred while setting system time: {e}")
            return False
    return False

def main():
    # preliminary logging to console only
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        handlers=[logging.StreamHandler()]
    )

    # start GPS reader
    gps_reader = GPSReader()
    gps_reader.start()

    # sync time
    sync_system_time_with_gps(gps_reader)

    # now create the real session dir
    session_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    base_dir = os.path.join('/home/pi/project/data', session_time)
    os.makedirs(base_dir, exist_ok=True)

    # add file handler
    file_handler = logging.FileHandler(os.path.join(base_dir, 'session.log'))
    file_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
    logging.getLogger().addHandler(file_handler)

    logging.info(f"Session directory created: {base_dir}")

    # Set up logging (moved after base_dir creation)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(base_dir, 'session.log')),
            logging.StreamHandler()
        ]
    )
    logging.info("Logging initialized.")

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
    logging.info("Camera initialized.")

    logging.info("Starting logging; press Ctrl+C to stop.")
    try:
        while True:
            with gps_reader.lock:
                lat = gps_reader.lat
                lon = gps_reader.lon
                alt = gps_reader.alt
                fix = gps_reader.fix
                # Use the GPS time for the timestamp if available and reliable
                timestamp_for_point = gps_reader.gps_time if gps_reader.gps_time else datetime.utcnow().replace(tzinfo=timezone.utc)
                print("Time Compare: GPS:", gps_reader.gps_time, "SYSTEM:", datetime.utcnow().replace(tzinfo=timezone.utc))

            if fix and lat is not None and lon is not None:
                # Add point to GPX
                pt = gpxpy.gpx.GPXTrackPoint(lat, lon,
                                             elevation=alt,
                                             time=timestamp_for_point)
                segment.points.append(pt)
                logging.info(f"Logged {lat:.6f}, {lon:.6f} (Alt: {alt if alt is not None else 'N/A'}) at {timestamp_for_point.isoformat()}")

                # Capture an image alongside GPS point
                # Ensure image filename matches the GPS point time as closely as possible
                img_filename = f"img_{timestamp_for_point.strftime('%Y%m%d_%H%M%S')}.jpg"
                img_path = os.path.join(base_dir, img_filename)
                try:
                    camera.capture_file(img_path)
                    logging.info(f"Captured image {img_path}")
                except Exception as e:
                    logging.error(f"Failed to capture image {img_path}: {e}")
            else:
                logging.info("Waiting for GPS fix or valid data...")

            time.sleep(1) # Capture rate, adjust as needed
    except KeyboardInterrupt:
        logging.info("Interrupt received, stopping logging and saving track...")
        # Ignore further Ctrl+C during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # Cleanup camera
        try:
            camera.stop()
            camera.close()
            logging.info("Camera stopped and closed.")
        except Exception as e:
            logging.error(f"Error stopping/closing camera: {e}")

        # Save GPX file
        gpx_filename = f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.gpx" # Use current system time for filename if cleanup is later
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