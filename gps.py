#!/usr/bin/env python3
"""
GPS logger for Raspberry Pi Zero

Polls GPS every second via gpsd, records track points to a GPX file.

Dependencies:
  pip3 install gpsd-py3 gpxpy
Ensure gpsd is running and your GPS module is connected.
"""
import time
from datetime import datetime, timezone
import gpsd
import gpxpy
gpxpy.gpx

def main():
    # Connect to local gpsd
    gpsd.connect(host="127.0.0.1", port=2947)

    # Create GPX object and track structure
    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    segment = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(segment)

    print("Starting GPS logging, press Ctrl+C to stop...")
    try:
        while True:
            packet = gpsd.get_current()
            # Use fix only if 2D or better
            if packet.mode >= 2 and packet.lat is not None and packet.lon is not None:
                point_time = datetime.now(timezone.utc)
                point = gpxpy.gpx.GPXTrackPoint(
                    packet.lat,
                    packet.lon,
                    elevation=packet.alt if packet.alt is not None else None,
                    time=point_time
                )
                segment.points.append(point)
                print(f"[{point_time.isoformat()}] Logged {packet.lat:.6f}, {packet.lon:.6f}")
            else:
                print("No fix yet, waiting...")
            time.sleep(1)
    except KeyboardInterrupt:
        # Write to file on exit
        filename = f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.gpx"
        with open(filename, "w") as f:
            f.write(gpx.to_xml())
        print(f"GPX track saved to {filename}")

if __name__ == '__main__':
    main()
