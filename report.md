# Review 2 submission status - 3 November 2026

## Published deliverables

- Repository: https://github.com/Kapilkannaa/pi-swarm-robotics
- Release: https://github.com/Kapilkannaa/pi-swarm-robotics/releases/tag/review2-2026-11-03
- Report: reports/review2/review2.pdf (16 pages, compiled from review2.tex)
- Complete LaTeX bundle: attached to the release.
- Team/course/guide attribution and acknowledgement: README and report front matter.

## Validation

- Fresh complete local suite: **181 passed**, 243.58 seconds.
- Published clean tree: 181 tests collected; 10 offload/marker tests passed independently.
- GitHub Actions: **success**, run 41585904898.
- PDF: all pages rendered and inspected; required sections/team details/links checked.
- Published PDF checksum matches the reviewed local copy.
- Public stage checked for private config files, credentials and firmware binaries.

## Architecture

Phone/laptop camera -> laptop vision and adaptive control -> direct UDP -> robot.
Pi computes advisory Monte Carlo profile comparisons asynchronously. It is not a
camera relay or motor sender. Printed marker and firmware IDs are independent.

## Evidence limits and remaining physical work

Simulation, historical bench records and teammate-reported operation are labelled
separately. Physical arrival-error/stopping-distance trials, repeated floor-run
measurements and the new compute-only worker's current Pi benchmark remain pending.
No new robot was moved/reflashed while preparing this submission. The institutional
plagiarism checker must determine similarity; no certified percentage is claimed.

## Reproduction

Start with HANDOFF.md. Run pytest for software checks; use scripts/reproduce_review2.py
for scenario tables. Source/figures/evidence for the report are under reports/review2.
