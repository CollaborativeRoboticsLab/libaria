# Omron Robot CLI

This document describes the standalone non-ROS command-line test client built
from [tools/omron_robot_cli.cpp](../tools/omron_robot_cli.cpp).

The tool is intended for first-contact robot checks before involving ROS 2. It
connects to the robot over ArNetworking, prints live robot state, and lets you
send a small set of operator commands for motion, navigation and docking.

## What It Tests

- Network connection to the robot server
- ArNetworking protocol compatibility using `D6MTX` first, then `5MTX`
- Robot state updates such as mode, status, pose, velocity and battery
- Basic operator commands such as stop, safe drive, ratio drive and goto pose
- Dock request support when the server exposes the dock interface

## Build

Build the package from the workspace root:

```bash
cd /home/ubuntu/colcon_ws
colcon build --packages-select libaria
```

Then source the workspace so the installed binary and libraries are available:

```bash
source /home/ubuntu/colcon_ws/install/setup.bash
```

## Launch

Run the installed executable:

```bash
/home/ubuntu/colcon_ws/install/libaria/bin/omron_robot_cli
```

The CLI uses the same default connection settings as the existing ROS 1 driver:

- Host: `172.19.21.203`
- Port: `7272`
- User: `steve`
- Password mode: `-np` (no password)

If your robot uses different settings, override them on the command line:

```bash
/home/ubuntu/colcon_ws/install/libaria/bin/omron_robot_cli \
	-host 192.168.0.50 \
	-p 7272 \
	-u steve \
	-np
```

If the server requires a password, use `-pw` (or `-pwd`) instead of `-np`:

```bash
/home/ubuntu/colcon_ws/install/libaria/bin/omron_robot_cli \
	-host 192.168.0.50 \
	-p 7272 \
	-u operator \
	-pw secret
```

## Startup Behavior

On successful connection the tool:

- Tries `D6MTX`, then `5MTX`, unless you override it with `-protocol <value>`
- Starts the ArNetworking client asynchronously
- Requests robot update packets
- Subscribes to dock state updates if the server provides `dockInfoChanged`
- Enters an interactive prompt: `omron>`

To disable protocol enforcement entirely and accept the server-advertised version, pass an empty protocol string:

```bash
/home/ubuntu/colcon_ws/install/libaria/bin/omron_robot_cli \
	-host 192.168.0.50 \
	-p 7272 \
	-u operator \
	-pw secret \
	-protocol ""
```

## Commands

Type `help` at the prompt to print the built-in command summary.

### `status`

Print one snapshot of current robot state:

- Mode
- Status string
- Pose in meters and heading in degrees
- Translational, lateral and rotational velocity
- Battery voltage
- Dock state, when available

Example:

```text
status
```

### `watch [count] [interval_ms]`

Print repeated status snapshots.

Examples:

```text
watch
watch 20 500
```

The first form prints 10 updates at 1 second intervals.

### `stop`

Requests stop mode through `ArClientRatioDrive::stop()`.

Example:

```text
stop
```

Use this as the first recovery command if the robot is moving unexpectedly.

### `safe`

Requests safe drive mode on the server.

Example:

```text
safe
```

### `unsafe`

Requests unsafe drive mode on the server.

Example:

```text
unsafe
```

Do not use unsafe drive unless you are certain the test area is clear and you
understand how the server is configured.

### `ratio <trans_pct> <rot_pct> [duration_ms] [throttle_pct] [lat_pct]`

Sends ratio-drive commands using `ArClientRatioDrive`. Values are percentages of
 the server-configured limits.

Examples:

```text
ratio 10 0 1000
ratio 0 15 1000
ratio 5 -10 1500 50
```

Notes:

- `trans_pct` is forward/backward percentage
- `rot_pct` is rotational percentage
- `duration_ms` is optional; if provided, the client waits that long and then sends `stop`
- `throttle_pct` defaults to `100`
- `lat_pct` defaults to `0`

For initial testing, start with small values such as `5` or `10`.

### `cmdvel <linear> <angular> [duration_ms] [throttle_pct] [lat]`

Sends a raw `ratioDrive` request packet directly, matching the pattern used by
the ROS 1 node.

Examples:

```text
cmdvel 0.1 0.0 1000
cmdvel 0.0 0.2 1000
```

This command is useful when you want to validate the exact network request path
used by the ROS 1 driver instead of the `ArClientRatioDrive` helper class.

### `goto <x_m> <y_m> <theta_deg>`

Sends a `gotoPose` request to the server in meters and degrees.

Example:

```text
goto 1.0 0.0 0
```

If the server does not advertise `gotoPose`, the command reports that instead of
sending the request.

### `dock`

Sends a `dock` request to the server.

Example:

```text
dock
```

If the server does not advertise `dock`, the command reports that instead of
sending the request.

### `quit`

Exits the prompt, requests stop, disconnects the client and shuts down ARIA.

Examples:

```text
quit
exit
```

## Recommended First Test Sequence

Use a conservative sequence on real hardware:

1. Start the CLI and confirm it connects.
2. Run `status`.
3. Run `watch 10 500`.
4. Run `safe`.
5. Run `ratio 5 0 1000`.
6. Run `stop`.
7. Run `ratio 0 5 1000`.
8. Run `stop` again.

Only after those succeed should you test `goto` or `dock`.

## Troubleshooting

### Connection rejected

Check host, port, username and password options. If the robot server uses a
password, do not use `-np`.

### Connected but command is unavailable

The CLI checks whether the server advertises interfaces such as `gotoPose`,
`dock`, or `ratioDrive`. If a command is unavailable, verify that the robot-side
server actually exposes that request.

### ARIA prints `could not find where it is located`

That message may appear when running directly from a build tree or without the
expected installed layout. Prefer running the installed binary after sourcing the
workspace setup file.

## Source

- CLI implementation: [tools/omron_robot_cli.cpp](../tools/omron_robot_cli.cpp)
- Top-level package notes: [README.md](../README.md)
