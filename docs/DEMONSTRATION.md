# Recorded simulation demonstration

The final demonstration is the software recording in [media](../media/README.md).
The robots were dismantled before final documentation; no new hardware session is required.

1. Observe the generated marker and selected goal.
2. Follow the burst controller's commands and the resulting trajectory.
3. Inspect the lost-marker scene: the controller emits zero wheel commands.
4. Inspect the authenticated background HTTP result; local control continues independently.
5. Compare continuous/burst control and learning-on/off using the saved result tables.

Rebuild with `python scripts/render_demo.py` after installing `requirements-demo.txt`.
The recorded worker runs locally. This does not measure physical-Pi performance.
For code and architecture details see the report, PROJECT_GUIDE.md and OFFLOADING.md.
