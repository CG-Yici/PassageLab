"""Build a self-contained Chinese HTML guide from the reviewed Markdown sources.

Documentation-only dependency: markdown-it-py. Does not modify software packages.
"""
import argparse
import base64
import hashlib
import json
import re
import zipfile
from pathlib import Path
from urllib.parse import urlsplit
from markdown_it import MarkdownIt

ROOT = Path(__file__).resolve().parent.parent
SECTIONS = {
    'README.zh-CN.md': ('project', '项目介绍'),
    'docs/demo.zh-CN.md': ('demo', '演示与操作'),
    'docs/releases/v0.2.0.zh-CN.md': ('release', '发布说明'),
    'docs/chinese-docs-review.zh-CN.md': ('review', '文档验收'),
}


def data_url(path):
    mime = {'.png': 'image/png', '.gif': 'image/gif', '.mp4': 'video/mp4'}[path.suffix]
    return f'data:{mime};base64,' + base64.b64encode(path.read_bytes()).decode('ascii')


def render(path):
    parser = MarkdownIt('commonmark', {'html': False}).enable('table')
    tokens = parser.parse(path.read_text(encoding='utf-8'))

    def transform(items):
        for token in items:
            if token.children:
                transform(token.children)
            if token.type == 'image':
                src = (path.parent / token.attrGet('src')).resolve()
                src.relative_to(ROOT)
                token.attrSet('src', data_url(src))
            if token.type == 'link_open':
                href = token.attrGet('href')
                if urlsplit(href).scheme or href.startswith('#'):
                    continue
                target = (path.parent / href.split('#')[0]).resolve().relative_to(ROOT).as_posix()
                token.attrSet('href', '#' + SECTIONS[target][0] if target in SECTIONS
                              else 'https://github.com/CG-Yici/PassageLab/blob/main/' + target)
    transform(tokens)
    return parser.renderer.render(tokens, parser.options, {})


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--out', default='dist/chinese-guide-0.2.0')
    options = args.parse_args()
    out = (ROOT / options.out).resolve()
    out.relative_to(ROOT)
    if out.exists():
        args.error('Choose a new output directory; existing artifacts are preserved.')
    out.mkdir(parents=True)
    subtitles = (ROOT/'docs/media/0.2.0/passagelab-demo.zh-CN.srt').read_text(encoding='utf-8')
    vtt = 'WEBVTT\n\n' + re.sub(r'(\d{2}:\d{2}:\d{2}),(\d{3})', r'\1.\2', subtitles)
    navigation = ''.join(f'<a href="#{key}">{label}</a>' for key, label in SECTIONS.values())
    sections = ''.join(f'<section id="{key}">{render(ROOT/source)}</section>'
                       for source, (key, label) in SECTIONS.items())
    video = data_url(ROOT/'docs/media/0.2.0/passagelab-demo.mp4')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PassageLab 0.2.0 中文项目与演示指南</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f3f6f8;color:#182b3a;font:17px/1.85 system-ui,"Microsoft YaHei",sans-serif}
nav{background:#142b3d;padding:14px 24px;display:flex;gap:24px;flex-wrap:wrap}nav a{color:white;text-decoration:none}
main{max-width:1080px;margin:auto;padding:28px}section,.intro{background:white;border:1px solid #dce5eb;border-radius:10px;padding:32px;margin-bottom:28px;overflow-wrap:anywhere}
h1{font-size:30px;line-height:1.4}h2{font-size:24px;margin-top:36px;border-bottom:1px solid #dce5eb;padding-bottom:8px}h3{font-size:20px}
a{color:#087080}img,video{max-width:100%;height:auto;border-radius:5px}video{width:100%}
table{border-collapse:collapse;display:block;overflow:auto;max-width:100%;font-size:15px;margin:18px 0}td,th{border:1px solid #ccd9e2;padding:10px 12px;vertical-align:top;min-width:130px}th{background:#eaf2f7;text-align:left}
pre{overflow:auto;background:#132c3d;color:#e9f3f8;padding:18px;border-radius:6px;font-size:14px}code{font-family:Consolas,monospace;font-size:.92em}li{margin:8px 0}strong{color:#163e53}
.note{color:#536877;font-size:15px}@media(max-width:650px){main{padding:10px}section,.intro{padding:18px}h1{font-size:25px}nav{gap:14px}}
</style>'''
    page += f'<nav aria-label="章节导航">{navigation}</nav><main><div class="intro">'
    page += '<h1>PassageLab 中文项目与演示指南</h1><p>适用软件：v0.2.0 Windows x64。此页面内置正文、图片、视频和字幕，断网也可阅读与播放。外部资料链接需要联网。</p>'
    page += f'<video controls preload="metadata" src="{video}"><track id="captions" kind="subtitles" srclang="zh-CN" label="简体中文" default></video>'
    page += '<p class="note">点击播放观看 15 秒演示：0–5 秒场景 A，5–10 秒场景 B，10–15 秒场景 C。无旁白；播放后可在视频控件中切换字幕。完整点击步骤见“演示与操作”。</p></div>'
    page += sections + '</main>'
    page += '<script>document.getElementById("captions").src=URL.createObjectURL(new Blob([' + json.dumps(vtt, ensure_ascii=True) + '],{type:"text/vtt"}));</script></html>'
    (out/'index.html').write_text(page, encoding='utf-8')
    archive = out/'PassageLab-0.2.0-Chinese-Guide.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        z.write(out/'index.html', 'index.html')
    archive.with_suffix('.zip.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest() + '  ' + archive.name + '\n')
    print(archive)


if __name__ == '__main__':
    main()
