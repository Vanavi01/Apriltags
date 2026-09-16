# AprilTag Autopark

A LEGO Education Double Motor car with an AprilTag mounted on it parallel-
parks itself in front of a stationary laptop webcam. Sibling project to
[ceciLego](../ceciLego) (same car hardware, same `legoeducation`/`lelib.py`
BLE wrapper), different sensing/control problem: instead of a human steering
via hand gestures, the car centers itself off a fixed external camera's view
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
- The car is set up **broadside to the webcam** - this is the load-bearing
  physical assumption of the whole project (see below).

## Files

| File | Role |
|---|---|
| `autopark.py` | The whole thing: tag detection, drawing, and closed-loop BLE motor control. |
| `lelib.py` | Vendored, unmodified copy of `ceciLego/lelib.py` - same BLE wrapper, same "not ready" connect-retry behavior. See `ceciLego/CLAUDE.md`'s BLE quirks section for the details; nothing about it changed for this project. |

## Design decisions in `autopark.py`

- **The car parallel-parks; it never turns.** This is the key physical setup
  the whole control loop depends on: the car sits **broadside** to the
  webcam, with its forward/backward drive axis parallel to the camera's
  image plane - like a car already pulled up alongside a curb. Because of
  that orientation, driving straight forward or backward doesn't change the
  car's distance from the camera; it slides the car left/right in the
  camera's view, exactly the way parallel parking works. So the controller
  never sends different speeds to the two wheels - `car.send(speed, now)`
  takes one speed and applies it to both, straight-line motion only. If the
  car is ever set up facing the camera instead of broadside to it, this
  entire approach doesn't apply - see "Not general-purpose" below.
- **One feedback signal: horizontal pixel offset.** Since the only degree of
  freedom the car can actuate is position along its parking line, the only
  error that matters is `x_error = centroid_x - center_x` (how far the tag's
  centroid is from the image's horizontal center). `DRIVE_GAIN * x_error`
  (clamped to `MAX_SPEED`) is the entire controller - there's no distance/
  depth term, because the car has no way to correct depth by driving along
  this line anyway.
- **Tag apparent size (`side_px`) is display-only.** `tag_metrics()` still
  computes it (averaged over all four edges so slight rotation doesn't throw
  it off) and it's drawn next to the centroid on screen, purely as a sanity
  readout for whoever's watching - it plays no role in the parked/not-parked
  decision or the speed calculation.
- **`DRIVE_SIGN` is an empirically-set flag, not derived geometry** - same
  idea as `ceciLego`'s `SWAP_HANDS`. Whether "tag right of image-center"
  should drive the car forward or backward depends on which end of the car
  ends up facing which way once it's set on the table broadside to the
  webcam, which isn't something the code can know in advance. Flip it if the
  car drives away from center instead of toward it.
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

- **Not general-purpose - assumes broadside setup.** This is a 1-D parking
  controller, not a 2-D navigation/pose-estimation system. It has no idea
  where the car actually is relative to the camera besides "how far left/
  right does the tag look" - it works because the broadside setup makes that
  the only thing that needs correcting. Set the car up facing the camera (or
  at any other angle) and this controller will not park it correctly.
- No obstacle/crash awareness - unlike `ceciLego`, this project doesn't wire
  up the Color Sensor. If you need that, port `ceciLego/crash_guard.py`
  over the same way `lelib.py` was vendored.
- No pose estimation or camera calibration anywhere - by design, given the
  above. If a future version needs the car to also correct real distance
  from the camera (not just left/right position), that's a different,
  harder problem (`solvePnP` + calibration + actual turning) and isn't what
  this project does.
