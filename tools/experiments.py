"""Run recorded, paired-seed experiments. Every failed/timeout run is retained."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time


def wilson(success, count):
    z = 1.96
    p = success / count
    denominator = 1 + z * z / count
    center = (p + z * z / (2 * count)) / denominator
    radius = z * math.sqrt(p * (1-p)/count + z*z/(4*count*count)) / denominator
    return [center-radius, center+radius]


def summarize(rows):
    solved = [r for r in rows if r['status'] == 'solved']
    times = [r['total_seconds'] for r in solved]
    search_times = [r['search_seconds'] for r in solved]
    optimized = [r for r in solved if (r.get('optimized_metrics') or {}).get('complete')]
    reasons = {}
    for row in rows:
        reasons[row['status']] = reasons.get(row['status'], 0) + 1
    return {
        'runs': len(rows), 'solved': len(solved), 'status_counts': reasons,
        'success_fraction': len(solved)/len(rows),
        'success_wilson_95': wilson(len(solved), len(rows)),
        'median_solved_total_seconds': statistics.median(times) if times else None,
        'solved_search_seconds_distribution': {
            'min': min(search_times), 'median': statistics.median(search_times),
            'max': max(search_times), 'quartiles': statistics.quantiles(search_times, n=4) if len(search_times) > 1 else [],
        } if search_times else None,
        'median_original_objective_m': statistics.median(r['metrics']['objective_m'] for r in solved) if solved else None,
        'optimized_count': len(optimized),
        'median_optimized_objective_m': statistics.median(r['optimized_metrics']['objective_m'] for r in optimized) if optimized else None,
        'total_all_run_seconds': sum(r['total_seconds'] for r in rows),
        'median_solved_translation_m': statistics.median(r['metrics']['translation_m'] for r in solved) if solved else None,
        'median_solved_rotation_rad': statistics.median(r['metrics']['rotation_rad'] for r in solved) if solved else None,
        'mean_narrow_queries_all_runs': statistics.mean(r['queries']['narrow'] for r in rows),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exe', default='build/windows/Release/passagelab_cli.exe')
    p.add_argument('--out', default='results/experiments-0.2.0')
    p.add_argument('--runs', type=int, default=30)
    p.add_argument('--seconds', type=float, default=5)
    p.add_argument('--suite', choices=['baseline','comparisons'], default='baseline')
    args = p.parse_args()
    if args.runs < 1 or not math.isfinite(args.seconds) or args.seconds < 0:
        p.error('runs must be positive and seconds must be finite and nonnegative')
    out = Path(args.out)
    if out.exists() and any(out.iterdir()):
        p.error('output directory already contains evidence; choose a new --out')
    out.mkdir(parents=True, exist_ok=True)
    variants = [('self', [])] if args.suite == 'baseline' else [
        ('brute', ['--brute']), ('fine', ['--step', '0.04']), ('ompl', ['--planner', 'ompl'])]
    groups = {}
    manifest = {'suite': args.suite, 'runs_per_configuration': args.runs, 'seeds': list(range(1000,1000+args.runs)),
                'wall_time_limit_seconds': args.seconds, 'started_unix': time.time(),
                'notes': 'Release, headless. Solved-run timing is conditional on success. Seeds are paired, RNG streams differ by planner. No cached reference paths passed to planners.'}
    (out/'experiment.json').write_text(json.dumps(manifest, indent=2))
    for variant, flags in variants:
        for demo in range(3):
            rows = []
            for seed in manifest['seeds']:
                run_dir = out/f'{variant}-demo-{demo}'/str(seed)
                cmd = [args.exe, 'plan', '--demo', str(demo), '--seed', str(seed), '--seconds', str(args.seconds), '--goal-bias', '0', '--out', str(run_dir), *flags]
                result = subprocess.run(cmd, capture_output=True, text=True, errors='replace', timeout=max(120, args.seconds*10))
                if result.returncode not in (0,2):
                    raise RuntimeError(f'{cmd}: {result.stdout}\n{result.stderr}')
                row = json.loads((run_dir/'report.json').read_text())
                rows.append(row)
                print(f'{variant} demo={demo} seed={seed}: {row["status"]}', flush=True)
            key=f'{variant}-demo-{demo}'
            (out/key/'runs.json').write_text(json.dumps(rows, indent=2))
            groups[key]=summarize(rows)
            (out/'summary.json').write_text(json.dumps(groups, indent=2))
    try:
        os.environ['MPLCONFIGDIR'] = str((out / '.matplotlib').resolve())
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        labels=list(groups); values=[groups[k]['success_fraction'] for k in labels]
        fig, ax=plt.subplots(figsize=(max(8,len(labels)*1.1),4.5))
        ax.bar(labels,values,color='#258b91')
        low=[v-groups[k]['success_wilson_95'][0] for k,v in zip(labels,values)]
        high=[groups[k]['success_wilson_95'][1]-v for k,v in zip(labels,values)]
        ax.errorbar(labels,values,yerr=[low,high],fmt='none',ecolor='#182b42',capsize=4)
        ax.set_ylim(0,1.08);ax.set_ylabel('Solved fraction (95% Wilson interval)');ax.set_title(f'PassageLab: {args.runs} seeds per configuration, {args.seconds:g}s search budget')
        ax.tick_params(axis='x',rotation=25);fig.tight_layout();fig.savefig(out/'success-rate.png',dpi=160);plt.close(fig)
    except ImportError:
        print('matplotlib absent; raw results and summaries saved without chart.')


if __name__ == '__main__':
    main()
