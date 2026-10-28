"""One-factor geometry variants and paired SE(3)/planar experiments.

This script changes JSON input parameters only; the C++ executable performs all
geometry queries, planning, optimization and validation.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
from experiments import summarize


def plot_summary(root, groups):
    os.environ['MPLCONFIGDIR'] = str((root / '.matplotlib').resolve())
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    labels = list(groups)
    values = [groups[k]['success_fraction'] for k in labels]
    low = [max(0, v-groups[k]['success_wilson_95'][0]) for k, v in zip(labels, values)]
    high = [max(0, groups[k]['success_wilson_95'][1]-v) for k, v in zip(labels, values)]
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.barh(labels, values, color='#258b91', xerr=[low, high], capsize=3)
    ax.invert_yaxis()
    ax.set_xlim(0, 1.1)
    ax.set_xlabel('Solved fraction (95% Wilson interval)')
    ax.set_title('Geometry and motion-space comparisons; paired seeds and budgets')
    for y, key in enumerate(labels):
        ax.text(1.015, y, f"{groups[key]['solved']}/{groups[key]['runs']}", va='center', fontsize=9)
    fig.tight_layout()
    fig.savefig(root/'success-rate.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', default='build/worktree/Release/passagelab_cli.exe')
    parser.add_argument('--out', default='results/design-0.2.0')
    parser.add_argument('--runs', type=int, default=30)
    parser.add_argument('--seconds', type=float, default=1)
    args = parser.parse_args()
    if args.runs < 1 or args.seconds <= 0:
        parser.error('runs and seconds must be positive')
    root = Path(args.out)
    if root.exists():
        parser.error('choose a new output directory')
    root.mkdir(parents=True)
    subprocess.run([args.exe, 'generate-scenes', '--out', str(root/'scenes')], check=True)
    names = ['A-offset-passage', 'B-L-corner', 'C-staggered-apertures']
    scenes = [json.loads((root/'scenes'/f'{name}.json').read_text()) for name in names]
    variants = [(f'{chr(65+i)}-control', copy.deepcopy(scene), 'se3', 'unchanged geometry; same search budget')
                for i, scene in enumerate(scenes)]
    for label, index, factor in [('A-thicker', 0, 1.25), ('B-larger-corner', 1, 1.15)]:
        scene = copy.deepcopy(scenes[index])
        if index == 0:
            for axis in (1, 2):
                scene['moving_body'][0]['half_extents'][axis] *= factor
        else:
            # Keep its outer faces fixed; move only the inner-corner faces.
            obstacle = scene['obstacles'][0]
            for axis, sign in ((0, 1), (1, -1)):
                delta = obstacle['half_extents'][axis] * (factor - 1)
                obstacle['half_extents'][axis] += delta
                obstacle['pose']['position'][axis] += sign * delta
        variants.append((label, scene, 'se3', f'one geometry factor={factor}'))
    for factor, offset in ((.85, 0), (1.15, 0), (1, -.25), (1, .25)):
        scene = copy.deepcopy(scenes[2])
        # Rebuild each opening's surrounding boxes while keeping outer walls fixed.
        for prefix, cy, cz, hy, hz in [('vertical-opening', -.35, 1.35, .48, .8),
                                      ('horizontal-opening', .45, 1.7, .85, .38)]:
            if prefix == 'horizontal-opening':
                cy += offset
            hy *= factor
            hz *= factor
            for obstacle in scene['obstacles']:
                if not obstacle['id'].startswith(prefix):
                    continue
                p, h = obstacle['pose']['position'], obstacle['half_extents']
                side = obstacle['id'][len(prefix)+1:]
                if side == 'left':
                    p[1], h[1] = (-2+cy-hy)/2, (cy-hy+2)/2
                elif side == 'right':
                    p[1], h[1] = (2+cy+hy)/2, (2-cy-hy)/2
                elif side == 'bottom':
                    p[1], p[2], h[1], h[2] = cy, (cz-hz)/2, hy, (cz-hz)/2
                elif side == 'top':
                    p[1], p[2], h[1], h[2] = cy, (3.2+cz+hz)/2, hy, (3.2-cz-hz)/2
        label = f'C-offset-{offset}' if offset else f'C-opening-{factor}'
        note = f'second opening y offset={offset}m' if offset else f'opening half-width/height scale={factor}'
        variants.append((label, scene, 'se3', note))
    paired = copy.deepcopy(scenes[2])
    paired['scene_id'] = 'C-common-endpoints'
    for key in ('start_pose', 'goal_pose'):
        paired[key]['position'][2] = 1.55
        paired[key]['quaternion'] = [0, 0, 0, 1]
    for mode in ('se3', 'planar-xy'):
        variants.append((f'C-common-{mode}', copy.deepcopy(paired), mode,
                         'same geometry/endpoints; only motion space changes'))
    manifest = {'seeds': list(range(1000, 1000+args.runs)), 'search_seconds': args.seconds,
                'comparison_contract': 'One factor per comparison; original C endpoints are NOT silently projected.',
                'interpretation': 'budget_exhausted is not a proof of infeasibility', 'variants': []}
    groups = {}
    for label, scene, mode, note in variants:
        folder = root/label
        folder.mkdir()
        source = folder/'input.json'
        source.write_text(json.dumps(scene, indent=2))
        rows = []
        manifest['variants'].append({'name': label, 'motion_space': mode, 'change': note})
        for seed in manifest['seeds']:
            dest = folder/str(seed)
            cmd = [args.exe, 'plan', '--scene', str(source), '--seed', str(seed),
                   '--seconds', str(args.seconds), '--goal-bias', '0', '--motion-space', mode, '--out', str(dest)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            if result.returncode not in (0, 2):
                raise RuntimeError(result.stdout + result.stderr)
            row = json.loads((dest/'report.json').read_text())
            rows.append(row)
            (folder/'runs.json').write_text(json.dumps(rows, indent=2))
            print(label, seed, row['status'], flush=True)
        groups[label] = summarize(rows)
        (root/'summary.json').write_text(json.dumps(groups, indent=2))
        (root/'experiment.json').write_text(json.dumps(manifest, indent=2))
    plot_summary(root, groups)


if __name__ == '__main__':
    main()
