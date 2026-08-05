# robot_bringup

Top-level launch files that bring up the whole robot with one command. No
nodes of its own — it only composes the launch files of the other packages.
See the [workspace README](../../README.md#run-the-robot) for the quick-start
commands; this file covers what each launch does internally and how to debug
it when something doesn't come up right.

## `scripts/can_up.sh`

Brings the SocketCAN interface up. Requires `sudo`, so it's a plain shell
script run once per boot, outside of any launch file (`ros2 launch` should
never need root):

```bash
./scripts/can_up.sh [interface] [bitrate]   # defaults: can0 500000
```

## `launch/autonomy_bringup.launch.py`

Full stack: `LiDAR -> perception -> planning -> control -> CAN`, with teleop
running alongside as the manual-override / deadman path.

Starts, in order of the data flow:

| Package | What it starts |
|---|---|
| `rslidar_sdk` | LiDAR driver -> `/rslidar_points` |
| `cone_detector` | static TF (`base_link` -> `rslidar`) + `cone_detector_node` + `cone_mapper_node` -> `/cones/observed` |
| `cone_planner` | `delaunay_planner_node` -> `/path` |
| `cone_control` | `pure_pursuit` -> `/cmd/auto`, `cmd_mux` -> `/cmd` |
| `robot_teleop` | `joy_node` + `joy_to_ackermann` -> `/cmd/manual` |
| `can_bridge` | `can_receiver`/`can_sender` (lifecycle) + `can_bridge_node` -> CAN -> STM32 |
| *(this launch file)* | background keyboard listener -> latched `/autonomy_enable`, toggled by SPACE |

Launch arguments:

| Arg | Default | Description |
|---|---|---|
| `interface` | `can0` | SocketCAN interface name, forwarded to `can_bridge` |
| `lidar` | `true` | Set `false` to skip the live LiDAR driver (e.g. replaying a bag on `/rslidar_points` instead) |

```bash
ros2 launch robot_bringup autonomy_bringup.launch.py
ros2 launch robot_bringup autonomy_bringup.launch.py lidar:=false
```

**Control handoff**: this launch file starts a background keyboard listener
in the terminal it's run from — **press SPACE** to engage autonomy (forwards
`/cmd/auto`, pure_pursuit drives), **press SPACE again** to drop back to
manual (forwards `/cmd/manual`, you drive). It's a latch, not a deadman: the
state sticks until you press SPACE again, it does not require holding
anything down. `cmd_mux` picks the source every tick based on that latched
state, and if neither source has published recently it publishes zero
regardless. See [`cone_control`'s README](../cone_control/README.md) for the
exact timeout parameters, and the note at the top of
`launch/autonomy_bringup.launch.py` for why the key listener lives in the
launch file instead of as its own node.

## `launch/teleop_bringup.launch.py`

Lighter bringup for driving by hand only — no LiDAR, perception, or
planning:

| Package | What it starts |
|---|---|
| `can_bridge` | `can_receiver`/`can_sender` (lifecycle, auto-activated) + `can_bridge_node` |
| `robot_teleop` | `joy_node` + `joy_to_ackermann` -> `/cmd/manual` |
| `cone_control` | `cmd_mux` only (no `/cmd/auto` publisher in this mode, so it always forwards `/cmd/manual`) |

```bash
ros2 launch robot_bringup teleop_bringup.launch.py
ros2 launch robot_bringup teleop_bringup.launch.py interface:=can1
```

## Troubleshooting

**RViz shows detections/path/markers fine, but the robot doesn't respond to
the gamepad or drive itself.** This is almost always the CAN lifecycle nodes
never reaching `active` — perception, planning, and control are entirely
separate nodes from CAN actuation, so they can look completely healthy while
`/cmd` messages are silently dropped after `can_bridge_node`.

Check:

```bash
ros2 lifecycle get /can_receiver
ros2 lifecycle get /can_sender
```

Both must say `active`. If either says `unconfigured` or `inactive`, see
[`can_bridge`'s README](../can_bridge/README.md#troubleshooting) — this used
to be a real bug (a fixed-delay auto-activation that raced under the full
stack's startup load) and has since been fixed to be event-driven, but if you
ever see it again, that's the place to look.

**A previous run is still alive.** If you `Ctrl+C` a launch and the terminal
doesn't fully clean up (or the terminal itself was closed), the child
processes can be orphaned and keep holding the CAN interface and node names,
which then collides with your next launch in confusing ways (duplicate
`/can_receiver`, etc.). Check for leftovers before re-launching:

```bash
ps aux | grep -E "ros2 launch|socket_can|can_bridge"
```
