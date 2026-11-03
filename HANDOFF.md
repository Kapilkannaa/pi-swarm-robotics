# Handoff: current Review 2 architecture

## Start with this mental model

**Phone overhead camera → laptop → robot.** The operator clicks a destination in
the picture. The laptop estimates where the robot is and sends wheel commands
directly. **The Pi is a side worker**, not a video relay or mandatory motor sender.

The worker evaluates three burst-control profiles under randomized motor speed,
wheel mismatch, camera delay and packet loss. Its completion/time comparison is
advice shown in the interface; it is not automatically applied to physical motors.
This preserves your teammate's working click-to-go behavior.

## Which files to use

- `scripts/click_go.py`: main laptop runner; default `--tracking led` preserves the
  old red-light tracker. Use `--tracking aruco --marker-id 1` for the supplied print.
- `scripts/install_offload.sh`: installs the compute-only Pi service and stops the
  older Pi motor sender. Run on the Pi only after copying the code there.
- `swarm/offload_server.py`: HTTP worker on 8770; knows no robot network addresses.
- `swarm/offload_client.py`: background thread, latest-goal queue and stale-result rejection.
- `reports/review2/review2.pdf`: course report; `.tex` is beside it.

## Identity and wiring

The photographed marker was decoded as **ArUco ID 1**. It may be on firmware bot 2.
Use `--bot 2 --marker-id 1`; do not change the board ID or motor flags just to match
paper. If both robots are visible, give them different printed marker IDs.

Recorded flags: bot 1 swap=true, invert-left=false, invert-right=true; bot 2
swap=true, invert-left=false, invert-right=false. Fixed D1/D2/D5/D6 motor inputs.
Do not use boot pins for external motor wiring or tie USB power to the L298N 5V feed.
Keep a working commissioned teammate robot's firmware unless a change is actually needed.

## What to run

Copy `config.example.toml` to `config.toml`; put the actual camera URL, robot IP and
measured arena dimensions in the private copy. Calibrate four floor corners, then
verify a 20 cm displacement against a ruler.

```sh
python scripts/click_go.py --bot 2 --tracking aruco --marker-id 1 --source http://PHONE_IP:8080/video --recalibrate
```

Left click = goal, right click / x = stop, q = quit. Camera 0 is the laptop webcam.
For the older LED tracker use `--tracking led`. To add Pi advice, append
`--offload http://PI_IP:8770` after installing the worker and privately copying its token.
If Pi is down, the local interface stays usable. If camera tracking is lost, stop.

## Evidence and honest claims

Fresh full suite: 181 tests passed. New marker mapping/stop and asynchronous-worker
behavior are tested. New simulated comparisons and ablation data are in the report.
Historical actual-board records cover zero-speed commands and reported LED/watchdog
states. The user reports the teammate's phone click-to-go works, but no quantitative
physical trial series was supplied. Current physical-Pi worker benchmarking,
calibration/arrival error, wheel direction and repeated floor trials remain pending.
No current hardware was reflashed or moved while preparing the report/repository.

## Next practical steps

1. Keep only one motor sender; stop old Pi-central services before direct laptop control.
2. Confirm the camera/robot identities and freshness; use a clear supervised arena.
3. Commission motion with raised wheels before a floor demo. Bench firmware is motor-disabled.
4. Install the Pi worker; show a completed advisory job and test Pi-offline behavior.
5. Record at least ten real clicked-goal trials with time, error, failures and stop observations.
6. Check the final report with the institution's similarity tool and present the actual model.
