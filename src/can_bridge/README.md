# can_bridge

Bridges `/cmd` (`ackermann_msgs/AckermannDriveStamped`) to CAN frames for the
STM32 via `ros2_socketcan`, and unpacks encoder feedback back into ROS.

```
/cmd (AckermannDriveStamped)
   -> can_bridge_node   -- pack DV_Command (0x200) --> /to_can_bus -> can_sender -> STM32
/from_can_bus <- can_receiver <- STM32
   -> can_bridge_node   -- unpack DV_Feedback (0x201) --> /encoder
```

All CAN IDs, signal layouts, scaling, and units are parameters, so the DBC
can change without recompiling. Speed conversion (m/s -> PWM) is a single
configurable ratio.

## Nodes

- **`can_receiver`** / **`can_sender`** — `ros2_socketcan` lifecycle nodes,
  bound to the given interface. Brought up and auto-activated by the launch
  file (see below) — you should not need to run `ros2 lifecycle set` by hand.
- **`can_bridge_node`** — plain node, does the actual `/cmd` <-> CAN frame
  translation described above.

## Configuration (`config/can_bridge.yaml`)

| Key | Default | Description |
|---|---|---|
| `cmd_can_id` / `fb_can_id` | `0x200` / `0x201` | DV_Command / DV_Feedback CAN IDs |
| `steering_scale` | `1800/pi ≈ 572.96` | rad -> DBC units (deg × 10) |
| `max_pwm` / `speed_at_max_pwm` | `1000.0` / `0.2 m/s` | m/s -> PWM calibration point |
| `max_steering_dbc` | `300.0` (±30 deg) | Clamp applied after conversion |
| `max_speed_pwm` | `1000.0` | Clamp applied after conversion |
| `cmd_topic` | `/cmd` | Input command topic — this is `cmd_mux`'s output (see [`cone_control`](../cone_control/README.md)), not teleop or the controller directly |

## Run standalone

Needs the CAN interface up first (see
[`robot_bringup/scripts/can_up.sh`](../robot_bringup/README.md#scriptscan_upsh)
or run `sudo ip link set can0 up type can bitrate 500000` by hand):

```bash
ros2 launch can_bridge can_bridge.launch.py interface:=can0
```

This brings up `can_receiver`, `can_sender`, and `can_bridge_node`, and
drives the lifecycle nodes through `configure` -> `activate` automatically.

## Lifecycle activation

`can_receiver`/`can_sender` are ROS 2 lifecycle nodes — they must reach the
`active` state before they actually open the CAN socket and pass traffic. The
launch file drives this with `launch_ros` event handlers: `OnProcessStart`
fires `configure` for each node as soon as its process starts, and
`OnStateTransition(goal_state='inactive')` fires `activate` once `configure`
actually completes. This is what should happen on every launch, with no
fixed delay involved.

### Troubleshooting

**Everything else works (RViz, `/cmd` is being published) but the robot
doesn't move.** Check the lifecycle state directly:

```bash
ros2 lifecycle get /can_receiver
ros2 lifecycle get /can_sender
```

Both should say `active`. If one is stuck at `unconfigured` or `inactive`,
nothing published to `/to_can_bus` is actually reaching the bus — the
`ros2_socketcan` lifecycle publisher/socket only turns on once `active`, so
it fails silently rather than erroring.

This used to be a real, reproducible bug: an earlier version of this launch
file used `TimerAction`s to fire one-shot `ros2 lifecycle set ... configure`
/ `activate` CLI calls at fixed t=2s / t=3s delays. `ros2 lifecycle set` does
not retry — if the node wasn't discoverable yet in that window (e.g. under
the CPU/DDS-discovery load of the full `autonomy_bringup.launch.py` stack
starting LiDAR + perception + planning all at once), the call just failed
("Node not found" / "Unknown transition requested") and the node was stuck
unconfigured forever, with no further attempt to fix it. It reliably worked
when `can_bridge` was launched alone (low contention, plenty of margin
inside 2-3s) and reliably failed bundled with everything else — which is
exactly why "it works when I launch nodes one by one" was misleading. The
current event-driven approach (above) doesn't guess a delay, so it isn't
sensitive to how loaded the system is at startup.

**If you still see it fail**: check for a stale, orphaned `ros2 launch`
process from an earlier session still holding the same node names and the
CAN interface (`ps aux | grep -E "ros2 launch|socket_can"`) — two nodes named
`/can_receiver` at once makes the lifecycle service ambiguous and produces
the same symptoms.
