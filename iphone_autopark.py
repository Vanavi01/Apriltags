'''
iPhone autopark: same closed-loop parking control as autopark.py, but reads
frames from an iPhone's MJPEG stream (IP Camera Lite) instead of a local
webcam - the car drives itself using a fixed AprilTag (tag36h11, ID 14) as
its landmark instead of carrying the tag itself.

The tag is stationary somewhere in the room; the iPhone is mounted on the
car, broadside to the tag, same physical relationship autopark.py relies on
(the car's drive axis parallel to the tag's line of sight) - only which end
carries the camera and which carries the tag is swapped. That means the
control math is identical: the car can't turn, so the only thing driving
forward/backward changes is the tag's horizontal position in frame, and
that's the only feedback signal needed.

Usage:
    python iphone_autopark.py

You'll be prompted for the MJPEG stream URL (see iphone_tracking.py for how
to find it), e.g. http://admin:admin@10.243.97.67:8081/video
'''
import time

import cv2
import numpy as np

from lelib import doubleMotor

CARD_SERIAL = "0999"                       # same Double Motor as autopark.py
TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_TAG_ID = 14

# The two motors are mounted mirrored on this chassis - same correction, same car.
RIGHT_FLIP = -1
LEFT_FLIP = 1

# Whether "tag right of image-center" means "drive forward" or "drive
# backward" depends on which end of the car is facing which way once it's
# set down broadside to the tag - flip this to -1 if the car drives away
# from center instead of toward it.
DRIVE_SIGN = 1

MAX_SPEED = 50          # -100..100, kept modest for a maneuver that ends close to the tag
DRIVE_GAIN = 0.25       # motor-speed units per pixel of horizontal offset

CENTER_TOLERANCE_PX = 20
HOLD_FRAMES = 5          # consecutive in-tolerance frames before declaring parked
LOST_TIMEOUT = 1.5       # seconds with no tag seen before the car is stopped as a failsafe
SEND_INTERVAL = 0.1      # BLE write throttle

MAX_CONSECUTIVE_READ_FAILURES = 30     # ~1s of dropped frames over Wi-Fi before giving up


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def tag_metrics(corners):
    """corners: (4,2) array, detector order. Returns (centroid_xy, side_length_px).
    side_px isn't used for control - it's shown on screen purely as a sanity readout."""
    centroid = corners.mean(axis=0)
    side_px = np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])
    return centroid, side_px


class ParkMotor:
    '''Throttled, deduped BLE speed sends - same pattern as autopark.py.'''

    def __init__(self, card_serial):
        self.card_serial = card_serial
        self.dm = doubleMotor()
        self._last_send = 0.0
        self._last_cmd = None

    def connect(self):
        print(f"Connecting to double motor {self.card_serial}...")
        self.dm.connect(card_serial=self.card_serial)
        print("Connected.")

    def send(self, speed, now):
        speed = int(speed)
        if now - self._last_send > SEND_INTERVAL and speed != self._last_cmd:
            self.dm.set_speed_left(LEFT_FLIP * speed)
            self.dm.set_speed_right(RIGHT_FLIP * speed)
            self.dm.run_left()
            self.dm.run_right()
            self._last_send, self._last_cmd = now, speed

    def stop(self):
        self.dm.stop()

    def disconnect(self):
        self.dm.disconnect()


stream_url = input("iPhone stream URL (e.g. http://admin:admin@10.243.97.67:8081/video): ").strip()
cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

aruco_dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

car = ParkMotor(CARD_SERIAL)
car.connect()

last_seen = time.time()
tolerance_streak = 0
parked = False
consecutive_failures = 0

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            # A Wi-Fi stream drops frames now and then; tolerate brief drops and
            # only give up if they don't stop (autopark.py's wired webcam never
            # needed this - a dropped read there is fatal).
            consecutive_failures += 1
            if consecutive_failures > MAX_CONSECUTIVE_READ_FAILURES:
                print("Lost the stream - too many consecutive failed reads.")
                break
            continue
        consecutive_failures = 0

        now = time.time()
        h, w, _ = frame.shape
        center_x = w / 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        target_corners = None
        if ids is not None:
            for c, tag_id in zip(corners_list, ids.flatten()):
                if tag_id == TARGET_TAG_ID:
                    target_corners = c.reshape(4, 2)
                    break

        if target_corners is not None:
            last_seen = now
            centroid, side_px = tag_metrics(target_corners)

            cv2.polylines(frame, [target_corners.astype(int)], isClosed=True,
                          color=(0, 0, 255), thickness=2)
            cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({centroid[0]:.0f}, {centroid[1]:.0f})  size={side_px:.0f}px",
                        (int(centroid[0]) + 10, int(centroid[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            x_error = centroid[0] - center_x

            in_tolerance = abs(x_error) < CENTER_TOLERANCE_PX
            tolerance_streak = tolerance_streak + 1 if in_tolerance else 0
            if tolerance_streak >= HOLD_FRAMES:
                parked = True

            speed = 0 if parked else clamp(DRIVE_GAIN * x_error * DRIVE_SIGN, -MAX_SPEED, MAX_SPEED)
            car.send(speed, now)

        if now - last_seen > LOST_TIMEOUT:
            car.send(0, now)
            parked = False
            tolerance_streak = 0

        status = "PARKED" if parked else ("TRACKING" if target_corners is not None else "NO TAG")
        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow("iPhone autopark - press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    car.stop()
    car.disconnect()
    cap.release()
    cv2.destroyAllWindows()
    print("Stopped.")
