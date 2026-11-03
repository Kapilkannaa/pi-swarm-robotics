"""Write docs/markers/markers_a4.pdf (and .png): bot markers 1..N (default 2) and corner markers 10-13.

    python scripts/make_markers.py [--bot-mm 60] [--corner-mm 80]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.vision.markers import DPI, build_sheet  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bots", type=int, default=2, help="number of bot markers (IDs 1..N); 2 fit on one page with corners")
    ap.add_argument("--bot-mm", type=float, default=60)
    ap.add_argument("--corner-mm", type=float, default=55)
    a = ap.parse_args()
    sheet = build_sheet(list(range(1, a.bots + 1)), [10, 11, 12, 13], a.bot_mm, a.corner_mm)
    out = ROOT / "docs" / "markers"
    out.mkdir(parents=True, exist_ok=True)
    sheet.save(out / "markers_a4.pdf", resolution=DPI)
    sheet.save(out / "markers_a4.png", dpi=(DPI, DPI))
    print(f"wrote {out / 'markers_a4.pdf'} (print at 100% on A4)")


if __name__ == "__main__":
    main()
