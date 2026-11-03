"""Rebuild Review 2 simulation tables. No hardware, camera or network required."""
import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.sim.experiments import trials, learning_ablation
from swarm.offload import evaluate


def clean(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, list): return [clean(v) for v in value]
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', type=int, default=10)
    a = ap.parse_args()
    if not 1 <= a.trials <= 100: ap.error('trials must be 1..100')
    out = ROOT / 'reports/review2'; (out/'evidence').mkdir(parents=True, exist_ok=True)
    cases = [('Fast camera / continuous', dict(latency_s=0, camera_latency_s=0)),
        ('Delayed camera / continuous', dict(latency_s=0, camera_latency_s=.3)),
        ('Delayed camera / burst', dict(latency_s=.4, camera_latency_s=.3)),
        ('Burst + 15% packet loss', dict(latency_s=.4, camera_latency_s=.3, packet_loss=.15)),
        ('Burst + 25% wheel mismatch', dict(latency_s=.4, camera_latency_s=.3, gain_spread=.25))]
    rows = [asdict(trials(name, a.trials, **kw)) for name, kw in cases]
    with (out/'evidence/scenario_results.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    ablation = learning_ablation(a.trials)
    job = evaluate({'job_id': 'review2-local-validation', 'arena': [1, .75],
                    'pose': [.2, .2, 0], 'goal': [.8, .55], 'trials': 12})
    job['execution_host'] = 'local execution; do not attribute this timing to Pi unless run there'
    for name, data in [('learning_ablation', ablation), ('offload_validation', job),
                       ('results', {'scenario_rows': rows, 'ablation': ablation, 'offload': job})]:
        (out/f'evidence/{name}.json').write_text(json.dumps(clean(data), indent=2, allow_nan=False))
    fmt = lambda x: '--' if not math.isfinite(x) else f'{x:.2f}'
    table = [r'\begin{table}[ht]\centering\small',
        r'\begin{tabularx}{\textwidth}{@{}Yrrrr@{}}\toprule',
        r'Scenario & Success & Mean (s) & Worst (s) & Contacts\\\midrule']
    for r in rows:
        table.append(r['name'].replace('%', r'\%') + f" & {r['success']}/{r['n']} & {fmt(r['mean_t'])} & {fmt(r['max_t'])} & {r['collisions']}" + r'\\')
    table += [r'\bottomrule\end{tabularx}',
        rf'\caption{{Simulation scenarios, {a.trials} seeds per row. Not physical robot measurements.}}\label{{tab:scenarios}}\end{{table}}']
    (out/'scenario_table.tex').write_text('\n'.join(table))
    table = [r'\begin{table}[ht]\centering\small',
        r'\begin{tabularx}{\textwidth}{@{}Yrr@{}}\toprule',
        r'Variant & Completed & Mean time (s)\\\midrule']
    for label, n, success, t in ablation:
        table.append(f'{label} & {success}/{n} & {fmt(t)}' + r'\\')
    table += [r'\bottomrule\end{tabularx}',
        r'\caption{Learning ablation; means use completed courses only.}\label{tab:ablation}\end{table}']
    (out/'ablation_table.tex').write_text('\n'.join(table))
    print(f'Rebuilt {len(rows)} scenario rows and learning/offload evidence in {out}')


if __name__ == '__main__': main()
