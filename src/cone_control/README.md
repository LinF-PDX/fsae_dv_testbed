# cone_control

Two nodes: a pure-pursuit path tracker that turns a local path into drive
commands, and a command mux that arbitrates between that and the human.

```
/path (nav_msgs/Path, base_link)
   -> pure_pursuit -> /cmd/auto (AckermannDriveStamped)
                                        \
/cmd/manual (AckermannDriveStamped) ----+-> cmd_mux -> /cmd (AckermannDriveStamped)
/joy (sensor_msgs/Joy, for the auto-enable button) --/
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
3. **Slew-rate limiting** on both steering and speed against the previously
   published values, using the actual measured tick period.
4. Publishes the command, plus an RViz `MarkerArray` on `/pure_pursuit/markers`
   (lookahead point, line to it, steering arrow, speed/steering readout).

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
| `speed` | `0.7 m/s` | Constant cruise speed. Start slow |
| `control_rate` | `50.0 Hz` | Feeds the firmware watchdog |
| `path_timeout` | `1.5 s` | No non-empty path cached for this long -> stop |
| `min_path_points` | `2` | Minimum points required to compute a lookahead |
| `max_steering_rate` | `1.0 rad/s` | Slew limit on published steering angle |
| `max_speed_rate` | `0.5 m/s²` | Slew limit on published speed |
| `publish_markers` | `true` | RViz markers on `/pure_pursuit/markers` |
| `output_topic` | `/cmd/auto` | Where commands are published |

## `cmd_mux`

Deadman-style source selector, safest for early autonomous testing:

- **HOLD** the auto-enable button (`auto_enable_button`, default `0` = A on
  Xbox) -> forward `/cmd/auto` (the robot drives itself).
- **RELEASE** it -> forward `/cmd/manual` (you drive, or it stops if teleop
  is also idle — teleop has its own deadman).

Publishes on a fixed timer regardless of message arrival (feeds the firmware
watchdog). If the currently-selected source hasn't published within
`cmd_timeout`, it publishes **zero**, not the last stale command — releasing
everything always returns to a safe stop.

### Configuration (`config/control.yaml`, `cmd_mux` block)

| Key | Default | Description |
|---|---|---|
| `auto_enable_button` | `0` (A) | `/joy` button index that hands control to `/cmd/auto` while held |
| `control_rate` | `50.0 Hz` | Publish rate for `/cmd` |
| `cmd_timeout` | `0.3 s` | Selected source stale for this long -> publish zero |

## Run standalone

Needs `/path` (from `cone_planner`, or published manually for bench testing)
and `/joy` + `/cmd/manual` (from `robot_teleop`) already available:

```bash
ros2 launch cone_control control.launch.py
```

This is what `robot_bringup`'s `autonomy_bringup.launch.py` includes; see the
[workspace README](../../README.md#run-the-robot) for the full stack.
