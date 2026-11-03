# Architecture history

The original team project ran vision/control on the laptop. An intermediate
experiment placed a Pi coordinator in the motor-command path and demonstrated
Pi computation and an LED indicator. The team subsequently requested the earlier
phone-camera click-to-go behavior with the Pi used for background computation.

The Review 2 architecture follows that request: camera -> laptop -> robot, plus a
separate Pi compute branch. The older coordinator/preview modules are retained for
historical regression tests; they are not the main operating instructions. Do not
start them alongside the current direct-control runner.

Historical evidence is retained under evidence/historical with private addresses
redacted. A successful old Pi-central simulation is not proof of a new offload
deployment or a physical navigation accuracy measurement.
