# robot_teleop

Gamepad teleop: maps an Xbox-style controller to
`ackermann_msgs/AckermannDriveStamped` commands.

```
joy_node -> /joy (sensor_msgs/Joy) -> joy_to_ackermann_node -> /cmd/manual
```

`joy_to_ackermann_node` publishes to `/cmd` internally; the launch file
remaps it to `/cmd/manual` so it feeds `cmd_mux` (in
[`cone_control`](../cone_control/README.md)) rather than `can_bridge`
directly — teleop alone is not enough to drive the robot, it needs `cmd_mux`
in the loop too (which is exactly what `robot_bringup`'s
`teleop_bringup.launch.py` sets up — see the
[workspace README](../../README.md#run-the-robot)).

## `joy_to_ackermann_node`

- Publishes on a fixed timer (default 50 Hz) from the latest joystick state,
  not only on `/joy` callbacks — this gives the firmware a steady command
  stream for its watchdog and lets it emit a safe zero the instant the
  controller goes quiet (`joy_timeout`).
- **Deadman button**: speed is zero unless the configured button is held.
  Let go and the robot stops. This is the software half of "don't hit the
  wall"; the firmware watchdog is the hardware half.
- All axis/button indices, scales, and inversions are parameters — the
  correct mapping depends on the physical controller, and you will almost
  certainly need to flip at least one invert flag.

## Configuration (`config/teleop.yaml`)

| Key | Default | Description |
|---|---|---|
| `speed_axis` / `steering_axis` | `5` / `0` | `/joy` axis indices — verify with `ros2 topic echo /joy` (move a stick, see which index changes) |
| `deadman_button` | `-1` (disabled) | Button held to enable driving; `-1` disables the deadman entirely (not recommended for first tests) |
| `max_speed` | `1.0 m/s` | Speed at full stick |
| `max_steering_angle` | `0.35 rad` | Steering at full stick (~20°) |
| `invert_speed` / `invert_steering` | `false` / `false` | Flip if the robot drives/steers backwards from the stick |
| `publish_rate` | `50.0 Hz` | Command rate (feeds the firmware watchdog) |
| `joy_timeout` | `0.5 s` | No `/joy` for this long -> publish zero (safe stop) |
| `frame_id` | `base_link` | Header frame stamped on published `AckermannDriveStamped` messages |

## Run standalone

```bash
ros2 launch robot_teleop teleop.launch.py
```

To sanity-check the joystick mapping without any of the rest of the stack:

```bash
ros2 topic echo /joy          # confirm axis/button indices
ros2 topic echo /cmd/manual   # confirm speed/steering respond as expected
```
