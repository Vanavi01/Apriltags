'''
Autopark: a LEGO Education Double Motor car with an AprilTag (tag36h11, ID 14)
mounted on it parallel-parks itself in front of a stationary laptop webcam.
The car sits broadside to the webcam and only ever drives straight forward or
backward - because of that orientation, straight driving is exactly what
appears as left/right motion on screen (like a car sliding along a curb), so
there's no turning involved anywhere in this control loop.

OpenCV's built-in AprilTag detector (cv2.aruco, family 36h11 - the same
family used by
https://ftc-docs.firstinspires.org/.../AprilTag_0-20_family36h11.pdf) finds
the tag every frame; only its horizontal image position is used as feedback.
Motor control goes over the same BLE link drive.py in the ceciLego project
uses (lelib.py is vendored here the same way it is there - see README).

Usage:
    python autopark.py
'''
import time

import cv2
import numpy as np

from lelib import doubleMotor

CARD_SERIAL = "1130"                       # same Double Motor as the ceciLego gesture car
TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_TAG_ID = 14

# The two motors are mounted mirrored on this chassis (see ceciLego/CLAUDE.md) -
# same correction, same car.
RIGHT_FLIP = -1
LEFT_FLIP = 1

# Whether "tag right of image-center" means "drive forward" or "drive
# backward" depends on which end of the car is facing which way when it's
# set on the table broadside to the webcam - flip this to -1 if the car
# drives away from center instead of toward it (same idea as ceciLego's
# SWAP_HANDS).
DRIVE_SIGN = 1

MAX_SPEED = 20          # -100..100, lowered further so the car stays slow enough for the
                        # webcam's framerate to keep up - it no longer stops on a single missed
                        # frame (see the tracking loop below), so speed has to be conservative
                        # enough that misses are rare in the first place, not just short-lived
DRIVE_GAIN = 0.15       # motor-speed units per pixel of horizontal offset - proportional-only
                        # (no integral/derivative term), same shape as the original controller.
                        # Real BLE/motor latency means it still overshoots center slightly before
                        # correcting back, without the full-speed-to-the-line violence of a
                        # bang-bang controller.

CENTER_TOLERANCE_PX = 20            # error band that counts as "centered" for parking
RECALIBRATE_TOLERANCE_PX = 40       # wider than CENTER_TOLERANCE_PX (hysteresis) - how far the
                                     # tag has to drift once parked before the car re-approaches
HOLD_FRAMES = 5          # consecutive in-tolerance frames before declaring parked
DRIFT_HOLD_FRAMES = 5    # consecutive frames past RECALIBRATE_TOLERANCE_PX, while parked,
                         # before giving up "parked" and driving back to center
LOST_TIMEOUT = 1.5       # seconds with no tag seen before parked/streak state resets and a
                         # search sweep starts (see SEARCH_SPEED below)
SEARCH_SPEED = 10        # slow, deliberate speed while blind-sweeping for a lost tag
SEARCH_LEG_SECONDS = 1.5 # how long to drive each direction before reversing, so the sweep stays
                         # within a bounded patch of desk instead of driving off the edge
SEND_INTERVAL = 0.1      # BLE write throttle, same reasoning as ceciLego/motor.py


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pid_speed(x_error):
    '''Proportional speed toward the tag, clamped to MAX_SPEED - the original controller
    shape. No slow-down zone, so it still overshoots center a little before correcting back,
    but it decelerates as the error shrinks instead of driving at full speed to the line.'''
    return clamp(DRIVE_GAIN * x_error * DRIVE_SIGN, -MAX_SPEED, MAX_SPEED)


def tag_metrics(corners):
    """corners: (4,2) array, detector order. Returns (centroid_xy, side_length_px).
    side_px isn't used for control (the car can't change its distance from
    the webcam by driving along its own parking line) - it's shown on screen
    purely as a sanity readout."""
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
drift_streak = 0
parked = False
ever_seen = False        # whether the tag has ever been acquired - gates the search sweep so
                         # the car doesn't go blind-searching before it's found the tag once
last_direction = 1       # sign of the most recent x_error, used to pick which way to start
                         # sweeping when the tag is lost (continue the way it was last heading)
searching = False
search_dir = 1
search_leg_start = 0.0

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
            ever_seen = True
            searching = False
            centroid, side_px = tag_metrics(target_corners)

            cv2.polylines(frame, [target_corners.astype(int)], isClosed=True,
                          color=(0, 0, 255), thickness=2)
            cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({centroid[0]:.0f}, {centroid[1]:.0f})  size={side_px:.0f}px",
                        (int(centroid[0]) + 10, int(centroid[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            x_error = centroid[0] - center_x
            last_direction = 1 if x_error > 0 else -1
            in_tolerance = abs(x_error) < CENTER_TOLERANCE_PX
            drifted = abs(x_error) > RECALIBRATE_TOLERANCE_PX

            if parked:
                # Stay parked through small jitter, but a sustained drift - the car or the
                # camera got bumped - should make it re-approach instead of sitting still
                # forever. RECALIBRATE_TOLERANCE_PX is wider than CENTER_TOLERANCE_PX so this
                # doesn't chatter right at the edge of the tolerance band.
                drift_streak = drift_streak + 1 if drifted else 0
                if drift_streak >= DRIFT_HOLD_FRAMES:
                    parked = False
                    tolerance_streak = 0
                    drift_streak = 0
            else:
                tolerance_streak = tolerance_streak + 1 if in_tolerance else 0
                if tolerance_streak >= HOLD_FRAMES:
                    parked = True
                    drift_streak = 0

            speed = 0 if parked else pid_speed(x_error)
            car.send(speed, now)
        elif now - last_seen <= LOST_TIMEOUT:
            # Tag not found this single frame - at MAX_SPEED/DRIVE_GAIN tuned low enough for the
            # webcam's framerate, this should be rare and momentary, not sustained motion blur.
            # Deliberately don't touch the motors here: stopping on every one-frame blip is the
            # stop-start jitter we're trying to avoid, so the car keeps coasting at its last
            # commanded speed until either the tag reappears or LOST_TIMEOUT gives up on it.
            pass
        else:
            # Genuinely lost (past LOST_TIMEOUT, not just a blurred frame) - reset parking state
            # so a fresh approach starts clean once the tag is found again.
            parked = False
            tolerance_streak = 0
            drift_streak = 0

            if ever_seen:
                # Failsafe: the tag was on-screen before and is gone now, most likely because
                # the car drove it out of frame. Sweep slowly back and forth (continuing the
                # direction it was last heading, then reversing every SEARCH_LEG_SECONDS) to
                # scan the desk for it, instead of just sitting stopped and hoping.
                if not searching:
                    searching = True
                    search_dir = last_direction
                    search_leg_start = now
                elif now - search_leg_start > SEARCH_LEG_SECONDS:
                    search_dir *= -1
                    search_leg_start = now
                car.send(SEARCH_SPEED * search_dir * DRIVE_SIGN, now)
            else:
                car.send(0, now)

        if target_corners is not None:
            status = "PARKED" if parked else "TRACKING"
        elif searching:
            status = "SEARCHING"
        else:
            status = "NO TAG"
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
