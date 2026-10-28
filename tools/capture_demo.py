"""Record real OpenGL output. Requires Pillow; FFmpeg executable supplied explicitly."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', default='build/windows/Release/passagelab_desktop.exe')
    parser.add_argument('--ffmpeg', required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--out', default='docs/media/0.2.0')
    parser.add_argument('--workspace', default='results/capture-0.2.0')
    args = parser.parse_args()
    if args.frames < 2:
        parser.error('frames must be at least 2')
    media = Path(args.out)
    workspace = Path(args.workspace)
    for directory in (media, workspace):
        if directory.exists() and any(directory.iterdir()):
            parser.error(f'Output already contains artifacts: {directory}')
        directory.mkdir(parents=True, exist_ok=True)
    for demo, label in enumerate(('a', 'b', 'c')):
        frames = workspace/label; frames.mkdir(parents=True, exist_ok=True)
        subprocess.run([args.exe, '--demo', str(demo), '--capture-dir', str(frames), '--frames', str(args.frames)], check=True)
        images = sorted(frames.glob('frame*.ppm'))
        if len(images) != args.frames:
            raise RuntimeError(f'Expected {args.frames} frames, got {len(images)}')
        with Image.open(images[args.frames*2//5]) as shot:
            shot.save(media/f'scene-{label}.png')
        subprocess.run([args.ffmpeg, '-y', '-loglevel', 'error', '-framerate', '12', '-i', str(frames/'frame%04d.ppm'), '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(workspace/f'{label}.mp4')], check=True)
        if demo == 2:
            previews = []
            for source in images[::2]:
                with Image.open(source) as im:
                    previews.append(im.resize((960,600),Image.Resampling.LANCZOS).convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
            previews[0].save(media/'passagelab.gif',save_all=True,append_images=previews[1:],duration=167,loop=0,optimize=True)
    manifest=workspace/'concat.txt'
    manifest.write_text("file 'a.mp4'\nfile 'b.mp4'\nfile 'c.mp4'\n")
    subprocess.run([args.ffmpeg,'-y','-loglevel','error','-f','concat','-safe','0','-i',str(manifest),'-c','copy','-movflags','+faststart',str(media/'passagelab-demo.mp4')],check=True)
    (media/'provenance.json').write_text(json.dumps({'source':'Actual PassageLab OpenGL frames; no reference motion substitution', 'demo_order':['A','B','C'],'seed_policy':'First solved seed in 42..49, fixed operation limits, wall-clock disabled','frames_per_demo':args.frames,'fps':12,'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(), 'git_tracked_dirty': bool(subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],text=True).strip()), 'executable_sha256': hashlib.sha256(Path(args.exe).read_bytes()).hexdigest()},indent=2))


if __name__ == '__main__':
    main()
