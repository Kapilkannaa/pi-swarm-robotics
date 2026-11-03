# Review 2 report

The editable source is `review2.tex`; team/course details are in `academic.tex`.
The PDF is compiled from this source. The five main sections and subsection names
follow the supplied course format. Tables are backed by the CSV/JSON files in `evidence/`.

Build with Tectonic:

```sh
cd reports/review2
tectonic review2.tex
```

Or run XeLaTeX twice. The report uses Times New Roman, Arial and Consolas; install
these fonts or replace the font names with compatible available fonts.

The bibliography uses labelled references in the .tex source; no BibTeX step is needed.
No physical navigation measurements or similarity-check percentage are invented.
The final evaluation uses archived bench records and software simulation; the hardware was dismantled before this documentation stage.

`evidence/test_results.txt` is the fresh full 181-test result. Simulation tables
are distinct from historical bench records. The compute-worker timing in the submission was measured on the laptop. The simulation video and its reproduction script are in `media/` and `scripts/render_demo.py`.
