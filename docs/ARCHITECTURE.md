# Architecture

Phone/laptop camera -> laptop vision and adaptive control -> direct UDP -> ESP8266.
The Pi receives compact goal snapshots and returns advisory Monte Carlo comparisons.
It receives no video and sends no robot motor packets in the current architecture.

The control loop never waits for a worker response. Latest-goal IDs prevent old
advice from being attached to new clicks. Recommendations are displayed/logged,
not automatically applied. Worker outage does not bypass camera freshness checks.

Use scripts/click_go.py. LED tracking is the original default; ArUco uses
--tracking aruco --marker-id N. Firmware and printed IDs are independent.
See ../PROJECT_GUIDE.md and OFFLOADING.md for identities and compute-only setup.
