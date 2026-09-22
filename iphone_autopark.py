'''
iPhone autopark: same closed-loop parking setup as autopark.py, but reads
frames from an iPhone's MJPEG stream (IP Camera Lite) instead of a local
webcam - the car drives itself using a fixed AprilTag (tag36h11, ID 14) as
its landmark instead of carrying the tag itself.

The tag is stationary somewhere in the room; the iPhone is mounted on the
car, broadside to the tag, same physical relationship autopark.py relies on
(the car's drive axis parallel to the tag's line of sight) - only which end
carries the camera and which carries the tag is swapped. That means the car
still can't turn and the only feedback signal is the tag's horizontal
position in frame - but unlike autopark.py's pure-P controller, this one is
PD: it also reacts to how fast that horizontal offset is changing, to damp
the overshoot/oscillation that motor lag and the car's momentum would
otherwise cause as it approaches center.

Usage:
    python iphone_autopark.py

You'll be prompted for the MJPEG stream URL (see iphone_tracking.py for how
to find it), e.g. http://admin:admin@10.243.97.67:8081/video
'''
import time

import cv2
import numpy as np

from lelib import doubleMotor

CARD_SERIAL = "1130"                       # same Double Motor as autopark.py
TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_TAG_ID = 14

# The two motors are mounted mirrored on this chassis - same correction, same car.
RIGHT_FLIP = -1
LEFT_FLIP = 1

# Whether "tag right of image-center" means "drive forward" or "drive
# backward" depends on which end of the car is facing which way once it's
# set down broadside to the tag - flip this to -1 if the car drives away
# from center instead of toward it.
DRIVE_SIGN = -1

MAX_SPEED = 25          # -100..100, kept modest for a maneuver that ends close to the tag
KP_GAIN = 0.12          # motor-speed units per pixel of horizontal offset - lower than
                        # autopark.py's because detection runs on the cropped corner
                        # (~32% of frame width), so the same real-world offset covers far
                        # fewer pixels and would otherwise saturate almost immediately
KD_GAIN = 0.02          # motor-speed units per (pixel/second) of offset rate of change -
                        # starting placeholder, tune empirically like KP_GAIN/DRIVE_SIGN:
                        # raise it if the car oscillates around center, lower it if it's
                        # sluggish to stop
D_SMOOTHING_ALPHA = 0.3  # EMA weight on the raw per-frame derivative (0 = fully smoothed/
                         # laggy, 1 = unsmoothed/noisy) - damps jitter from frame-to-frame
                         # detection noise so it doesn't turn into a jerky speed signal

CENTER_TOLERANCE_PX = 20
HOLD_FRAMES = 5          # consecutive in-tolerance frames before declaring parked
LOST_TIMEOUT = 1.5       # seconds with no tag seen before the car is stopped as a failsafe
SEND_INTERVAL = 0.1      # BLE write throttle

MAX_CONSECUTIVE_READ_FAILURES = 30     # ~1s of dropped frames over Wi-Fi before giving up

# IP Camera Lite has no setting to disable its front-camera picture-in-picture
# overlay, and the rest of the main frame has UI chrome (buttons, status text)
# drawn over it - only the small upper-right corner is a clean, uncluttered
# camera image. So instead of using the full frame, we crop down to just that
# corner and treat it as the whole working feed (display + detection both use
# it) - smaller, but the only part that's actually pure. As fractions of the
# original frame's width/height; tune these against what's shown on screen.
CROP_WIDTH_FRAC = 0.32
CROP_HEIGHT_FRAC = 0.32

# Nudges the crop window inward from the top-right corner, as fractions of
# frame width/height, in case the clean corner image doesn't sit flush against
# the edges. OFFSET_X shifts it left (away from the right edge), OFFSET_Y
# shifts it down (away from the top edge).
CROP_OFFSET_X_FRAC = 0.04
CROP_OFFSET_Y_FRAC = 0.04


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
            # run_left/run_right issue a "start moving" command - only needed when
            # coming from a stop or reversing direction, not on every speed tweak.
            # Resending them every time was adding 2 extra blocking BLE writes per
            # send, which is what was stalling the video loop once tracking started.
            starting = (self._last_cmd is None
                        or (self._last_cmd == 0) != (speed == 0)
                        or (self._last_cmd > 0) != (speed > 0))
            if starting:
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
prev_error = None
prev_time = None
smoothed_d = 0.0

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
        orig_h, orig_w, _ = frame.shape
        crop_w = int(orig_w * CROP_WIDTH_FRAC)
        crop_h = int(orig_h * CROP_HEIGHT_FRAC)
        offset_x = int(orig_w * CROP_OFFSET_X_FRAC)
        offset_y = int(orig_h * CROP_OFFSET_Y_FRAC)
        x_end = orig_w - offset_x
        y_start = offset_y
        frame = frame[y_start:y_start + crop_h, x_end - crop_w:x_end]
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

            if prev_error is not None and prev_time is not None and now > prev_time:
                raw_d = (x_error - prev_error) / (now - prev_time)
                smoothed_d = D_SMOOTHING_ALPHA * raw_d + (1 - D_SMOOTHING_ALPHA) * smoothed_d
            else:
                smoothed_d = 0.0
            prev_error, prev_time = x_error, now

            in_tolerance = abs(x_error) < CENTER_TOLERANCE_PX
            tolerance_streak = tolerance_streak + 1 if in_tolerance else 0
            if tolerance_streak >= HOLD_FRAMES:
                parked = True

            speed = 0 if parked else clamp(
                DRIVE_SIGN * (KP_GAIN * x_error + KD_GAIN * smoothed_d), -MAX_SPEED, MAX_SPEED
            )
            car.send(speed, now)

        if now - last_seen > LOST_TIMEOUT:
            car.send(0, now)
            parked = False
            tolerance_streak = 0
            prev_error = None
            prev_time = None
            smoothed_d = 0.0

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
