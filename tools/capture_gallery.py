"""Record README media from the real viewer: full-window screenshots and viewport-only GIFs.

Every frame is rendered by passagelab_desktop; nothing is composited or retouched beyond
cropping to the 3D viewport, scaling and GIF palette reduction. Requires Pillow.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from PIL import Image

# The 3D viewport inside the fixed 1440x900 capture window (see apps/desktop.cpp glViewport).
VIEWPORT = (330, 2, 1110, 706)
SCENES = [
    ('a', ['--demo', '0']),
    ('b', ['--demo', '1']),
    ('c', ['--demo', '2']),
    ('d', ['--scene', 'assets/cad/D-step-aperture.json']),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', default='build/windows/Release/passagelab_desktop.exe')
    parser.add_argument('--frames', type=int, default=72)
    parser.add_argument('--out', default='docs/media/current')
    parser.add_argument('--workspace', required=True, help='new, empty directory for raw frames')
    args = parser.parse_args()
    media, workspace = Path(args.out), Path(args.workspace)
    exe = str(Path(args.exe).resolve())
    if workspace.exists() and any(workspace.iterdir()):
        parser.error(f'Workspace already contains files: {workspace}')
    media.mkdir(parents=True, exist_ok=True)
    for label, source in SCENES:
        frames = workspace / label
        frames.mkdir(parents=True, exist_ok=True)
        subprocess.run([exe, *source, '--capture-dir', str(frames), '--frames', str(args.frames)], check=True)
        images = sorted(frames.glob('frame*.ppm'))
        if len(images) != args.frames:
            raise RuntimeError(f'Expected {args.frames} frames, got {len(images)}')
        with Image.open(images[args.frames * 2 // 5]) as shot:
            shot.save(media / f'scene-{label}.png', optimize=True)
        clip = []
        for source_frame in images:
            with Image.open(source_frame) as im:
                view = im.crop(VIEWPORT).resize((640, 578), Image.Resampling.LANCZOS)
                # Octree keeps the small accent colours (start, goal, axes) that median cut drops.
                clip.append(view.quantize(colors=160, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.FLOYDSTEINBERG))
        # Hold the last pose briefly so the loop does not snap back immediately.
        durations = [83] * (len(clip) - 1) + [900]
        clip[0].save(media / f'scene-{label}.gif', save_all=True, append_images=clip[1:], duration=durations,
                     loop=0, optimize=True)
    (media / 'provenance.json').write_text(json.dumps({
        'source': 'Actual PassageLab OpenGL frames; GIFs are the 3D viewport cropped from the full window',
        'scenes': {label: source for label, source in SCENES},
        'seed_policy': 'First solved seed in 42..49, fixed operation limits, wall-clock disabled',
        'frames_per_scene': args.frames,
        'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'git_tracked_dirty': bool(subprocess.check_output(
            ['git', 'status', '--porcelain', '--untracked-files=no'], text=True).strip()),
        'executable_sha256': hashlib.sha256(Path(exe).read_bytes()).hexdigest()}, indent=2) + '\n')


if __name__ == '__main__':
    main()
