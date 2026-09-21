# AprilTag Autopark

A LEGO Education Double Motor car, with an AprilTag mounted on it, parallel-
parks itself in front of a stationary laptop webcam - like a car sliding
along a curb, it sits broadside to the webcam and only ever drives straight
forward/backward, which is exactly what appears as left/right motion on
screen. There's no turning anywhere in this project.

## Hardware

- LEGO Education Double Motor (same car/Connection Card serial `0999` as the
  gesture-controlled car in [ceciLego](../ceciLego))
- A printed AprilTag, family **tag36h11**, **ID 14**, from
  [FTC's AprilTag 0-20 family36h11 sheet](https://ftc-docs.firstinspires.org/en/latest/_downloads/ba0d87cc0d392ad0bad054d4b81e9077/AprilTag_0-20_family36h11.pdf),
  mounted on the car facing the webcam
- Any Mac/PC webcam, stationary, facing the area the car drives in
- The car set up **broadside** to the webcam - its forward/backward drive
  axis parallel to the camera's image plane, free to slide along that line

## Setup

Requires Python 3.11+ (same constraint as ceciLego - `legoeducation` won't
run on macOS system Python).

```bash
python3 -m venv my_env
source my_env/bin/activate          # Windows: my_env\Scripts\activate
pip install --upgrade pip
pip install legoeducation opencv-contrib-python numpy
```

`opencv-contrib-python` (not plain `opencv-python`) is required - AprilTag
detection lives in `cv2.aruco`, which only ships in the contrib build. Don't
install both packages at once; they conflict.

`lelib.py` is copied in from the ceciLego project, same as every other
hardware script there - see its README for where it originally comes from.

## Running

```bash
python autopark.py
```

Pick a camera index when prompted. A red outline is drawn around the tag
whenever it's detected, with its centroid coordinates printed next to it.
Press `q` to quit.

## How it parks

The car is set on the table **broadside** to the webcam, like a car already
alongside a curb - so driving it straight forward or backward doesn't change
its distance from the camera, it slides the car left/right in the camera's
view. That means parking only needs one feedback signal: how far the tag's
centroid is from the image's horizontal center.

Every frame, `autopark.py` looks for AprilTag ID 14 and, if found, feeds that
horizontal offset through proportional control into a single forward/backward
speed (both wheels the same speed - no differential turning, since none is
needed). Once the offset stays within tolerance for `HOLD_FRAMES` consecutive
frames, it's declared `PARKED` and the motors are held at zero. If the tag
isn't seen for `LOST_TIMEOUT` (1.5s), the car stops as a failsafe, mirroring
the lost-hands failsafe in ceciLego's `drive.py`.

The tag's apparent size in pixels is still shown on screen next to its
centroid, but it's informational only - the car has no way to correct its
distance from the webcam by driving along this line, so size isn't part of
the control loop.

See [CLAUDE.md](CLAUDE.md) for the constants, tuning knobs, and known
limitations of this approach.

## iPhone camera mode

An alternate setup that inverts which end carries the camera vs. the tag:
the AprilTag stays fixed somewhere in the room, and an iPhone - mounted on
the car, broadside to the tag, same geometry as above - is the moving
camera. Two scripts cover this:

- `iphone_tracking.py` - perception only. Detects and draws the tag from
  the iPhone's stream; no motor control. Useful for checking the camera
  setup on its own before wiring up the car.
- `iphone_autopark.py` - the full closed loop, same control logic as
  `autopark.py` (same proportional control / tolerance-latch / lost-tag
  failsafe), just reading frames from the iPhone instead of a local webcam.

### Getting the iPhone's video feed

No Mac is required. Install **IP Camera Lite** (free, App Store) on the
iPhone, open its **Server** section, and turn on the IP Camera Server. It
lists several stream URLs - use the **MJPEG** one, e.g.:

```
http://<phone-ip>:8081/video
```

If it prompts for a username/password, the app's default is `admin`/`admin`
unless changed; embed credentials directly in the URL:

```
http://admin:admin@<phone-ip>:8081/video
```

Both scripts prompt for this URL at startup, the same way `autopark.py`
prompts for a camera index.

`.local` hostnames shown in the app (e.g. `dans-iphone.local`) often don't
resolve through OpenCV's ffmpeg backend even when a browser can load them
fine - use the phone's plain IP address instead (Settings > Wi-Fi > tap the
ⓘ next to your network) if the hostname fails.

### The picture-in-picture workaround

IP Camera Lite has no setting to broadcast the back camera alone - its
stream is the full on-screen composite: back camera as the main image,
front camera as a small picture-in-picture overlay in one corner, plus UI
chrome (buttons, status text) drawn over the rest. Since the corner overlay
is the only clean, uncluttered image, `iphone_autopark.py` crops down to
just that corner and uses it as the entire working feed - display and
detection both run on the crop, not the full frame. `CROP_WIDTH_FRAC` /
`CROP_HEIGHT_FRAC` and `CROP_OFFSET_X_FRAC` / `CROP_OFFSET_Y_FRAC` at the
top of the file control the crop's size and position - tune them against
what's shown in the display window until it lines up exactly.

Because detection runs on a much smaller image than `autopark.py`'s full
frame, `iphone_autopark.py` uses a lower `DRIVE_GAIN`/`MAX_SPEED` - the same
real-world offset covers far fewer pixels in the cropped view, so
`autopark.py`'s original gain would saturate to full speed almost
immediately.

### The control policy

`iphone_autopark.py`'s policy is a memoryless proportional (P) controller on
a single scalar observation, `x_error` - no integral/derivative term, no
learning, no state estimation:

```python
speed = clamp(DRIVE_GAIN * x_error * DRIVE_SIGN, -MAX_SPEED, MAX_SPEED)
```

`DRIVE_SIGN` is an empirically-set polarity flip (not derived from
geometry), and `clamp(...)` is what made the car "take off" earlier when
`DRIVE_GAIN` was tuned too high - almost any offset saturated straight to
`MAX_SPEED`, turning the proportional controller into an effectively
bang-bang one.

Two discrete overrides sit on top of that continuous law:

- **Parked latch** - once `x_error` stays within `CENTER_TOLERANCE_PX` for
  `HOLD_FRAMES` consecutive frames, `speed` is forced to `0` and stays that
  way even if the tag later drifts back out of tolerance. This is the only
  memory anywhere in the policy.
- **Lost-tag failsafe** - if the tag isn't seen for `LOST_TIMEOUT`, `speed`
  is forced to `0` and both the latch and tolerance streak reset.

Whatever speed results is sent identically to both wheels (no differential
steering) through `ParkMotor.send()`, which is a separate BLE throttling/
dedup layer, not part of the decision logic itself.

### Known issues

- Frame rate visibly drops once the tag is detected and the car starts
  moving, since BLE motor commands block the video loop while they're sent.
  Only reissuing `run_left`/`run_right` (which actually start the motor) on
  a stop/direction change rather than on every speed adjustment helps, but
  some slowdown remains.
- Streaming over Wi-Fi occasionally drops frames; both iPhone scripts
  tolerate a run of failed reads (`MAX_CONSECUTIVE_READ_FAILURES`) before
  giving up, unlike `autopark.py`'s wired webcam, which treats any failed
  read as fatal.
