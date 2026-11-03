# Final submission record

## Published contents

- [Repository](https://github.com/Kapilkannaa/pi-swarm-robotics)
- [Release downloads](https://github.com/Kapilkannaa/pi-swarm-robotics/releases/tag/review2-2026-11-03)
- Report: `reports/review2/review2.pdf`, compiled from editable LaTeX.
- Demonstration: `media/pi-swarm-simulation.mp4`, with source and numerical outputs.
- Project guide: `PROJECT_GUIDE.md`.

## Recorded checks

The complete software suite passed 181 tests. The archived full test log and seeded
scenario results are in `reports/review2/evidence/`. The recorded video completes
its goal, emits zero commands during the explicit marker loss, and obtains a
background HTTP result. All PDF pages were rendered and checked. Public files
were screened for private settings, tokens, Wi-Fi credentials and firmware binaries.

## Final architecture

Phone or webcam -> laptop vision and adaptive control -> direct UDP -> ESP8266.
The Pi branch runs advisory Monte Carlo evaluation asynchronously. The worker
does not relay video or issue wheel commands.

## Evaluation scope

The robots were dismantled before final documentation. This archive contains
software simulations, historical bench records and clearly identified team-reported
operation. New physical trials, hardware video and current physical-Pi runtime
measurements are outside the submitted evaluation. No similarity percentage is
claimed without an actual institutional checker report.

## Reproduction

Use the README and `PROJECT_GUIDE.md` for entry points. Run `python -m pytest -q`
for software checks, `python scripts/reproduce_review2.py` for numerical tables,
and `python scripts/render_demo.py` for the video after installing
`requirements-demo.txt`. No physical hardware is needed for these commands.
