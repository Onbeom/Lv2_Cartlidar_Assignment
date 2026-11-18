#!/usr/bin/env python3
"""report.md → report.html (영상 · 그림 포함). @img 경로 | 설명, @video 경로 | 설명 줄을 그림 · 영상으로 바꾼다."""
import html, re, sys
src, out = sys.argv[1], sys.argv[2]
lines = open(src, encoding='utf-8').read().split('\n')
def inl(t):
    t = html.escape(t, quote=False)
    t = re.sub(r'`([^`]+)`', r'<code>\1</code>', t)
    t = re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', t)
    return t
body, i, title = [], 0, ''
while i < len(lines):
    l = lines[i]
    if not l.strip(): i += 1; continue
    m = re.match(r'(#{1,3}) (.*)', l)
    if m:
        n = len(m.group(1)); title = title or m.group(2)
        body.append(f'<h{n}>{inl(m.group(2))}</h{n}>'); i += 1; continue
    if l.startswith('@img ') or l.startswith('@video '):
        kind, rest = l.split(' ', 1); p, cap = [x.strip() for x in rest.split('|', 1)]
        if kind == '@img':
            body.append(f'<figure><a href="{p}"><img src="{p}" loading="lazy" alt="{html.escape(cap)}"></a><figcaption>{inl(cap)}</figcaption></figure>')
        else:
            body.append(f'<figure><video controls preload="metadata" src="{p}"></video><figcaption>{inl(cap)}</figcaption></figure>')
        i += 1; continue
    if l.startswith('|'):
        rows = []
        while i < len(lines) and lines[i].startswith('|'):
            rows.append([c.strip() for c in lines[i].strip().strip('|').split('|')]); i += 1
        h, data = rows[0], [r for r in rows[1:] if not all(re.fullmatch(r':?-+:?', c) for c in r)]
        t = '<div class="tw"><table><thead><tr>' + ''.join(f'<th>{inl(c)}</th>' for c in h) + '</tr></thead><tbody>'
        t += ''.join('<tr>' + ''.join(f'<td>{inl(c)}</td>' for c in r) + '</tr>' for r in data)
        body.append(t + '</tbody></table></div>'); continue
    if re.match(r'\s*(- |\d+\. )', l):
        items = []
        while i < len(lines) and re.match(r'\s*(- |\d+\. )', lines[i]):
            ind = len(lines[i]) - len(lines[i].lstrip()); txt = re.sub(r'\s*(- |\d+\. )', '', lines[i], count=1)
            items.append((ind, txt, bool(re.match(r'\s*\d+\. ', lines[i])))); i += 1
        tag = 'ol' if items[0][2] else 'ul'
        s = f'<{tag}>'; open_sub = False
        for k, (ind, txt, _) in enumerate(items):
            if ind == 0:
                if open_sub: s += '</ul></li>'; open_sub = False
                elif k: s += '</li>'
                s += f'<li>{inl(txt)}'
            else:
                if not open_sub: s += '<ul>'; open_sub = True
                s += f'<li>{inl(txt)}</li>'
        s += ('</ul></li>' if open_sub else '</li>') + f'</{tag}>'
        body.append(s); continue
    para = []
    while i < len(lines) and lines[i].strip() and not re.match(r'(#|\||@img |@video |\s*- |\d+\. )', lines[i]):
        para.append(lines[i]); i += 1
    body.append(f'<p>{inl(" ".join(para))}</p>')
css = """
:root{--fg:#1d1d1f;--bg:#fff;--mut:#6b6b70;--line:#dcdce0;--th:#f3f3f5;--acc:#2a5db0}
@media (prefers-color-scheme:dark){:root{--fg:#e8e8ea;--bg:#18181a;--mut:#a0a0a8;--line:#3a3a40;--th:#26262a;--acc:#8ab4f8}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.7 "Noto Sans KR","Apple SD Gothic Neo","Malgun Gothic",sans-serif}
main{max-width:980px;margin:0 auto;padding:32px 16px 80px}
h1{font-size:1.7em;line-height:1.3;margin:0 0 .6em}h2{font-size:1.35em;margin:2.2em 0 .6em;padding-top:.6em;border-top:1px solid var(--line)}h3{font-size:1.1em;margin:1.6em 0 .5em}
p,li{word-break:keep-all}code{font-size:.9em;background:var(--th);padding:1px 5px;border-radius:4px}
.tw{overflow-x:auto;margin:1em 0}table{border-collapse:collapse;width:100%;font-size:.92em}th,td{border:1px solid var(--line);padding:6px 9px;text-align:left;vertical-align:top;word-break:keep-all}th{background:var(--th)}
figure{margin:1.2em 0}img,video{display:block;max-width:100%;border:1px solid var(--line);border-radius:6px;background:#000}img{background:#fff}
figcaption{color:var(--mut);font-size:.9em;margin-top:.35em}a{color:var(--acc)}
"""
doc = f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{html.escape(title)}</title><style>{css}</style></head><body><main>{"".join(body)}</main></body></html>'
open(out, 'w', encoding='utf-8').write(doc)
print(out, len(body))
