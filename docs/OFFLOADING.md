# Background Pi computation

Install on Pi with `sh scripts/install_offload.sh`. It stops the older Pi motor
sender and enables pi-offload.service on port 8770. The worker imports no camera
libraries and has no robot endpoint configuration. Python 3.11+ stdlib is sufficient.

Privately copy the generated local_token.txt to the laptop. Add
`--offload http://PI_IP:8770` to the direct click_go.py command. No raw video leaves
the laptop for this worker. A snapshot includes arena, current pose, goal and
assumed control settings. A default job runs 12 trials for each of three profiles.

The randomized cases cover speed, wheel mismatch, camera latency and packet loss.
Results include simulated successes, mean completion time, worst error and near-edge
counts. These are model estimates, not physical safety predictions. Current physical
Pi runtime was not measured in the final evaluation; the saved benchmark is from the laptop.

The client uses a background thread and a latest-goal mailbox. It discards results
for cancelled/superseded goals. Offline/busy workers leave local control operational.
The UI never applies a recommendation to moving hardware automatically.
