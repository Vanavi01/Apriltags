'''
Autopark: a LEGO Education Double Motor car with an AprilTag (tag36h11, ID 14)
mounted on it drives itself in front of a stationary laptop webcam. OpenCV's
built-in AprilTag detector (cv2.aruco, family 36h11 - the same family used by
https://ftc-docs.firstinspires.org/.../AprilTag_0-20_family36h11.pdf) finds
the tag every frame; its image position and apparent size are used to steer
and to judge distance, closing the loop over the same BLE link drive.py in
the ceciLego project uses (lelib.py is vendored here the same way it is
there - see README).

Usage:
    python autopark.py
'''
import time

import cv2
import numpy as np

from lelib import doubleMotor

CARD_SERIAL = "0999"                       # same Double Motor as the ceciLego gesture car
TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_TAG_ID = 14

# The two motors are mounted mirrored on this chassis (see ceciLego/CLAUDE.md) -
# same correction, same car.
RIGHT_FLIP = -1
LEFT_FLIP = 1

# Whether "tag left of center in the image" means "speed up the left wheel"
# or the right one depends on which way the tag faces the webcam, and isn't
# something you can derive from a single 2D corner set - flip this to -1 if
# the car steers away from center instead of toward it (same idea as
# ceciLego's SWAP_HANDS).
STEER_SIGN = 1

MAX_SPEED = 50          # -100..100, kept modest for a maneuver that ends close to the laptop
TURN_GAIN = 0.15        # motor-speed units per pixel of horizontal offset
DIST_GAIN = 0.35        # motor-speed units per pixel of tag-size error

TARGET_SIDE_PX = 220    # apparent tag side length (px) that counts as "parked"
CENTER_TOLERANCE_PX = 20
SIZE_TOLERANCE_PX = 15
HOLD_FRAMES = 5          # consecutive in-tolerance frames before declaring parked
LOST_TIMEOUT = 1.5       # seconds with no tag seen before the car is stopped as a failsafe
SEND_INTERVAL = 0.1      # BLE write throttle, same reasoning as ceciLego/motor.py


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def tag_metrics(corners):
    """corners: (4,2) array, detector order. Returns (centroid_xy, side_length_px)."""
    centroid = corners.mean(axis=0)
    side_px = np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])
    return centroid, side_px


class ParkMotor:
    '''Throttled, deduped BLE speed sends - same pattern as ceciLego/motor.py.'''

    def __init__(self, card_serial):
        self.card_serial = card_serial
        self.dm = doubleMotor()
        self._last_send = 0.0
        self._last_cmd = None

    def connect(self):
        print(f"Connecting to double motor {self.card_serial}...")
        self.dm.connect(card_serial=self.card_serial)
        print("Connected.")

    def send(self, left, right, now):
        left, right = int(left), int(right)
        if now - self._last_send > SEND_INTERVAL and (left, right) != self._last_cmd:
            self.dm.set_speed_left(LEFT_FLIP * left)
            self.dm.set_speed_right(RIGHT_FLIP * right)
            self.dm.run_left()
            self.dm.run_right()
            self._last_send, self._last_cmd = now, (left, right)

    def stop(self):
        self.dm.stop()

    def disconnect(self):
        self.dm.disconnect()


cams = []
for i in range(2):
    c = cv2.VideoCapture(i)
    if c.isOpened():
        cams.append(i)
        c.release()
print("Available cameras:", cams)
cap = cv2.VideoCapture(int(input("Which camera index? ")))

aruco_dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

car = ParkMotor(CARD_SERIAL)
car.connect()

last_seen = time.time()
tolerance_streak = 0
parked = False

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

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
            cv2.putText(frame, f"({centroid[0]:.0f}, {centroid[1]:.0f})",
                        (int(centroid[0]) + 10, int(centroid[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            x_error = centroid[0] - center_x
            size_error = TARGET_SIDE_PX - side_px

            in_tolerance = abs(x_error) < CENTER_TOLERANCE_PX and abs(size_error) < SIZE_TOLERANCE_PX
            tolerance_streak = tolerance_streak + 1 if in_tolerance else 0
            if tolerance_streak >= HOLD_FRAMES:
                parked = True

            if parked:
                left_speed = right_speed = 0
            else:
                forward = clamp(DIST_GAIN * size_error, -MAX_SPEED, MAX_SPEED)
                turn = clamp(TURN_GAIN * x_error, -MAX_SPEED, MAX_SPEED) * STEER_SIGN
                left_speed = clamp(forward - turn, -MAX_SPEED, MAX_SPEED)
                right_speed = clamp(forward + turn, -MAX_SPEED, MAX_SPEED)

            car.send(left_speed, right_speed, now)

        if now - last_seen > LOST_TIMEOUT:
            car.send(0, 0, now)
            parked = False
            tolerance_streak = 0

        status = "PARKED" if parked else ("TRACKING" if target_corners is not None else "NO TAG")
        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow("Autopark - press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    car.stop()
    car.disconnect()
    cap.release()
    cv2.destroyAllWindows()
    print("Stopped.")
