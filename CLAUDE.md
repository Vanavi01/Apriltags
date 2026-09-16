# AprilTag Autopark

A LEGO Education Double Motor car with an AprilTag mounted on it drives
itself in front of a stationary laptop webcam. Sibling project to
[ceciLego](../ceciLego) (same car hardware, same `legoeducation`/`lelib.py`
BLE wrapper), different sensing/control problem: instead of a human steering
via hand gestures, the car steers itself off a fixed external camera's view
of a tag mounted on it.

## Hardware

- **LEGO Education Double Motor**, Connection Card serial `0999` - the same
  physical car used in `ceciLego`. Motors are mounted **mirrored** on the
  chassis, same as there; `autopark.py` uses the same `RIGHT_FLIP = -1`,
  `LEFT_FLIP = 1` correction.
- **AprilTag, family tag36h11, ID 14**, printed from FTC's
  `AprilTag_0-20_family36h11.pdf`, mounted on the car facing the webcam.
- A **stationary** webcam (unlike `ceciLego`, which doesn't care where the
  camera is - here the camera's position defines the parking target, so it
  must not move during a run).

## Files

| File | Role |
|---|---|
| `autopark.py` | The whole thing: tag detection, drawing, and closed-loop BLE motor control. |
| `lelib.py` | Vendored, unmodified copy of `ceciLego/lelib.py` - same BLE wrapper, same "not ready" connect-retry behavior. See `ceciLego/CLAUDE.md`'s BLE quirks section for the details; nothing about it changed for this project. |

## Design decisions in `autopark.py`

- **Tag size is a distance proxy, not a calibrated measurement.** There's no
  camera calibration or known real-world tag size fed in, so `autopark.py`
  never computes an actual distance in cm. It only compares the tag's
  apparent side length in pixels (`side_px`, averaged over all four edges so
  a bit of rotation doesn't throw it off) against `TARGET_SIDE_PX` — a value
  you tune empirically by parking the car where you want it and reading off
  the printed centroid/side length. This is deliberately the simplest thing
  that works, not a proper pose estimate.
- **Steering direction is a sign flag, not derived geometry.** Whether "tag
  left of image-center" should speed up the left or right wheel depends on
  which way the tag is facing the camera - a single 2D corner set doesn't
  tell you the car's heading, only where it is in the image. Rather than
  attempt real pose estimation (`solvePnP` + calibration) for this, the
  controller just steers to center the tag in x and closes distance via
  apparent size, with `STEER_SIGN` as an empirically-set flag if that steers
  the wrong way for a given camera/tag mounting - same idea as `ceciLego`'s
  `SWAP_HANDS`. **This works well when the car approaches roughly head-on;
  it is not a general solution for arbitrary starting orientations** (e.g. a
  car starting sideways to the camera can't be centered by x-error alone).
- **Proportional control on both axes simultaneously**, not staged
  (center-then-approach). `left/right = forward ∓ turn` blends a forward term
  from `DIST_GAIN * size_error` with a turn term from
  `TURN_GAIN * x_error`, both clamped to `MAX_SPEED` (50, deliberately lower
  than `ceciLego`'s 100 - this maneuver ends close to expensive laptop
  hardware).
- **`PARKED` is sticky once reached**, the same debounce-then-latch pattern
  as `ceciLego`'s gesture streak: `HOLD_FRAMES` (5) consecutive in-tolerance
  frames sets `parked = True`, and it stays `True` (holding zero speed) even
  if a later frame drifts out of tolerance, on the theory that "parked" means
  the maneuver is done, not "currently within tolerance." The only thing that
  clears it is `LOST_TIMEOUT` - if the tag disappears entirely, both `parked`
  and the tolerance streak reset so a fresh approach starts clean.
- **BLE send is throttled/deduped exactly like `ceciLego/motor.py`** - see
  its design notes. Same `SEND_INTERVAL` (0.1s) reasoning: this is about not
  flooding the BLE link, not a UI framerate limit.

## Known limitations

- No pose estimation - see "steering direction" above. Works for
  roughly-head-on approaches; can get stuck oscillating or fail to converge
  from steep starting angles.
- No obstacle/crash awareness - unlike `ceciLego`, this project doesn't wire
  up the Color Sensor. If you need that, port `ceciLego/crash_guard.py`
  over the same way `lelib.py` was vendored.
- `TARGET_SIDE_PX` is specific to one camera's field of view and one tag
  print size - re-tune it if either changes.
