# cone_control

Two nodes: a pure-pursuit path tracker that turns a local path into drive
commands, and a command mux that arbitrates between that and the human.

```
/path (nav_msgs/Path, base_link)
   -> pure_pursuit -> /cmd/auto (AckermannDriveStamped)
                                        \
/cmd/manual (AckermannDriveStamped) ----+-> cmd_mux -> /cmd (AckermannDriveStamped)
/autonomy_enable (std_msgs/Bool, latched, SPACE toggle) --/
```

## `pure_pursuit`

Subscribes to `/path` (already in `base_link`, so the robot is at the origin
facing +x — the geometry is trivial) and publishes `/cmd/auto`.

Each control tick:

1. **Lookahead point** — the first path point at least `lookahead_distance`
   away; if the path is shorter than that, use its last point.
2. **Pure-pursuit steering** — with the target at `(x, y)` in `base_link`:
   `alpha = atan2(y, x)` (bearing to target),
   `delta = atan2(2 * wheelbase * sin(alpha), ld)` (bicycle steering angle),
   where `ld` is the distance to the lookahead point.
3. **Speed** — constant `speed` by default. If `adaptive_speed` is true,
   speed is instead scheduled off the just-clamped steering angle: normalize
   `|steer| / max_steering_angle` to `s in [0, 1]`, then
   `v = max_speed - s^speed_curve_exponent * (max_speed - min_speed)` —
   `max_speed` on straights (`s=0`) down to a `min_speed` floor at full lock
   (`s=1`, never crawls to a stop). `speed_curve_exponent` shapes the curve:
   `1.0` linear, `>1` stays fast through mild steering and drops sharply near
   lock, `<1` slows down earlier. This is a geometric heuristic, not a
   vehicle-dynamics model.
4. **Slew-rate limiting** on both steering and speed against the previously
   published values, using the actual measured tick period.
5. Publishes the command, plus an RViz `MarkerArray` on `/pure_pursuit/markers`
   (lookahead point, line to it, steering arrow, speed/steering readout, and
   the active speed mode/target when adaptive).

**Path loss handling**: the planner publishes a path every scan, including
empty ones on frames with too few cones to triangulate. Treating an empty
path as "path gone" made the command flicker between cruise speed and zero
on every dropped frame, so `pure_pursuit` only caches *non-empty* paths —
`path_timeout` is checked against the age of that cache, not against every
message. If no non-empty path has arrived within `path_timeout`, target
speed/steering drop to zero and slew-limit down from there like any other
command change.

### Configuration (`config/control.yaml`, `pure_pursuit` block)

| Key | Default | Description |
|---|---|---|
| `lookahead_distance` | `0.6 m` | Smaller = tighter but twitchier tracking |
| `wheelbase` | `0.25 m` | **MEASURE** — front wheel to rear axle distance |
| `max_steering_angle` | `0.5 rad` | Servo limit (~29°) |
| `speed` | `1.0 m/s` | Constant cruise speed, used when `adaptive_speed` is false. Start slow |
| `adaptive_speed` | `false` | `true`: schedule speed off steering angle (see step 3 above) instead of constant `speed` |
| `max_speed` | `0.8 m/s` | Adaptive mode: speed on straights (`s=0`) — only takes effect if `adaptive_speed` is true; currently lower than the constant-mode `speed` above, so switching modes changes cruise speed too |
| `min_speed` | `0.2 m/s` | Adaptive mode: floor at full lock (`s=1`); never crawls to a stop |
| `speed_curve_exponent` | `1.0` | Adaptive mode: `1.0` linear (current default); `>1` stays fast through mild steer then drops near lock; `<1` slows earlier |
| `control_rate` | `50.0 Hz` | Feeds the firmware watchdog |
| `path_timeout` | `1.5 s` | No non-empty path cached for this long -> stop |
| `min_path_points` | `2` | Minimum points required to compute a lookahead |
| `max_steering_rate` | `1.0 rad/s` | Slew limit on published steering angle |
| `max_speed_rate` | `0.5 m/s²` | Slew limit on published speed |
| `publish_markers` | `true` | RViz markers on `/pure_pursuit/markers` |
| `output_topic` | `/cmd/auto` | Where commands are published |

## `cmd_mux`

Latched source selector, toggled by pressing SPACE in the terminal running
`autonomy_bringup.launch.py` (see [`robot_bringup`'s
README](../robot_bringup/README.md)):

- `/autonomy_enable` latched `True` -> forward `/cmd/auto` (the robot drives
  itself).
- `/autonomy_enable` latched `False` (the default) -> forward `/cmd/manual`
  (you drive, or it stops if teleop is also idle — teleop has its own
  deadman).

Publishes on a fixed timer regardless of message arrival (feeds the firmware
watchdog). If the currently-selected source hasn't published within
`cmd_timeout`, it publishes **zero**, not the last stale command.

### Configuration (`config/control.yaml`, `cmd_mux` block)

| Key | Default | Description |
|---|---|---|
| `control_rate` | `50.0 Hz` | Publish rate for `/cmd` |
| `cmd_timeout` | `0.3 s` | Selected source stale for this long -> publish zero |

## Run standalone

Needs `/path` (from `cone_planner`, or published manually for bench testing)
and `/cmd/manual` (from `robot_teleop`) already available. `/autonomy_enable`
is only published by `autonomy_bringup.launch.py`'s keyboard latch, so
running `cmd_mux` on its own means it stays in manual mode unless you publish
that topic yourself, e.g.:

```bash
ros2 topic pub /autonomy_enable std_msgs/msg/Bool "{data: true}" --once
```

```bash
ros2 launch cone_control control.launch.py
```

This is what `robot_bringup`'s `autonomy_bringup.launch.py` includes; see the
[workspace README](../../README.md#run-the-robot) for the full stack.
