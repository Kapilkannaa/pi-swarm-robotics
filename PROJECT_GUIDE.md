# Project guide

Pi Swarm combines external camera sensing, adaptive laptop control, ESP8266 motor
actuation and optional Raspberry Pi computation. This is the final project archive.

## Review the implementation

| Area | Start here |
|---|---|
| Main application | `scripts/click_go.py` |
| Marker position and heading | `swarm/vision/tracker.py` |
| Goal controller | `swarm/marker_go.py` |
| Adaptive motion model | `swarm/control.py` |
| Compute workload | `swarm/offload.py` |
| Worker HTTP interface | `swarm/offload_server.py` |
| Nonblocking client | `swarm/offload_client.py` |
| Embedded command and safety logic | `firmware/bot/logic.h` |
| Recorded software demonstration | `media/README.md` |
| Final report | `reports/review2/review2.pdf` |

The camera feeds the laptop. Wheel commands go directly from laptop to robot.
The worker evaluates candidate profiles and returns advice; it cannot send wheel commands.

## Robot identities

The team's pictured printed marker decoded as ArUco ID 1. Printed and firmware
identities are independent: `--bot 2 --marker-id 1` tracks that print and addresses
firmware bot 2. Each visible robot needs a distinct marker.

Recorded wiring flags: bot 1 swap=true, invert-left=false, invert-right=true;
bot 2 swap=true, invert-left=false, invert-right=false. Motor inputs are D1, D2,
D5 and D6. These records describe the earlier build, not a newly commissioned robot.

## Reproduce without hardware

Install `requirements.txt`, then run `python -m pytest -q` or launch
`python scripts/click_go.py --config config.example.toml --bot 2 --tracking aruco --marker-id 1 --dry-run`.
For the MP4 recording, install `requirements-demo.txt` and run `python scripts/render_demo.py`.

The robots were dismantled before final documentation. The archive preserves
software evaluation and earlier bench records. No physical-Pi timing, new hardware
video or quantitative floor-trial dataset is included.

## Configuration reference

Use a private copy of `config.example.toml` for camera URL, robot address and arena
dimensions. A phone MJPEG stream or laptop webcam can feed the laptop. The camera
is calibrated using four floor corners. See the README for the archived live command.
The worker service is installed separately using `scripts/install_offload.sh`.

The provided firmware is motor-disabled by default. Reassembly and motor enablement
require deliberate power, direction and calibration checks. Never run the old Pi
motor sender together with the direct laptop controller.
