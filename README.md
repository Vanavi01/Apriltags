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
