# AprilTag Autopark

A LEGO Education Double Motor car, with an AprilTag mounted on it, drives
itself to a stop in front of a stationary laptop webcam.

## Hardware

- LEGO Education Double Motor (same car/Connection Card serial `0999` as the
  gesture-controlled car in [ceciLego](../ceciLego))
- A printed AprilTag, family **tag36h11**, **ID 14**, from
  [FTC's AprilTag 0-20 family36h11 sheet](https://ftc-docs.firstinspires.org/en/latest/_downloads/ba0d87cc0d392ad0bad054d4b81e9077/AprilTag_0-20_family36h11.pdf),
  mounted facing the webcam
- Any Mac/PC webcam, stationary, facing the area the car drives in

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

Every frame, `autopark.py` looks for AprilTag ID 14 and, if found, measures:

- **how far its centroid is from the image's horizontal center** (steering
  error)
- **its apparent side length in pixels** (a stand-in for distance - a closer
  tag looks bigger; no camera calibration is needed since we only care about
  reaching a *target* apparent size, not a real-world distance in cm)

Those two errors are fed through proportional control into left/right motor
speeds, so the car simultaneously centers the tag and closes the distance.
Once both errors stay within tolerance for `HOLD_FRAMES` consecutive frames,
it's declared `PARKED` and the motors are held at zero. If the tag isn't seen
for `LOST_TIMEOUT` (1.5s), the car stops as a failsafe, mirroring the
lost-hands failsafe in ceciLego's `drive.py`.

See [CLAUDE.md](CLAUDE.md) for the constants, tuning knobs, and known
limitations of this approach.
