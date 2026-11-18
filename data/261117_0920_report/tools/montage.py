#!/usr/bin/env python3
"""실험 묶음 폴더의 mp4 를 3x2 칸 영상으로 모은다. montage.py <출력.mp4> <제목> <폴더>... """
import glob, os, subprocess, sys, tempfile, math
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
W, H, COLS, ROWS, BAR = 320, 240, 3, 2, 36
def dur(f):
    return float(subprocess.check_output(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', f]).strip())
def esc(t):
    return t.replace('\\', '\\\\').replace(':', '\\:').replace("'", '').replace(',', '\\,')
def page(clips, title, out, sp):
    L = max(dur(f) for f in clips) / sp
    args = ['ffmpeg', '-v', 'error', '-y']
    for f in clips: args += ['-i', f]
    fl = []
    n = len(clips)
    for i, f in enumerate(clips):
        lab = esc(os.path.basename(os.path.dirname(f)) if os.path.basename(f) == 'video.mp4' else os.path.splitext(os.path.basename(f))[0])
        fl.append(f"[{i}:v]setpts=PTS/{sp},fps=15,scale={W}:{H}:force_original_aspect_ratio=decrease,pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,setsar=1,"
                  f"tpad=stop_mode=clone:stop_duration={L+1:.2f},trim=duration={L:.2f},"
                  f"drawtext=fontfile={FONT}:text='{lab}':fontsize=13:fontcolor=white:box=1:boxcolor=black@0.6:x=4:y=4[v{i}]")
    for i in range(n, COLS * ROWS):
        fl.append(f"color=c=gray:s={W}x{H}:r=15:d={L:.2f},setsar=1[v{i}]")
    lay = '|'.join(f'{(i % COLS) * W}_{(i // COLS) * H}' for i in range(COLS * ROWS))
    fl.append(''.join(f'[v{i}]' for i in range(COLS * ROWS)) + f'xstack=inputs={COLS * ROWS}:layout={lay}[g]')
    fl.append(f"[g]pad={COLS * W}:{ROWS * H + BAR}:0:{BAR}:color=black,drawtext=fontfile={FONT}:text='{esc(title)}':fontsize=18:fontcolor=white:x=8:y=9[o]")
    args += ['-filter_complex', ';'.join(fl), '-map', '[o]', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '32', '-pix_fmt', 'yuv420p', out]
    subprocess.check_call(args)
out, title, dirs = sys.argv[1], sys.argv[2], sys.argv[3:]
clips = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '**', '*.mp4'), recursive=True))
mx = max(dur(f) for f in clips)
sp = 1 if mx <= 15 else 2
td = tempfile.mkdtemp(dir=os.path.dirname(os.path.abspath(out)))
parts = []
for k in range(math.ceil(len(clips) / (COLS * ROWS))):
    p = os.path.join(td, f'p{k:02d}.mp4')
    page(clips[k * 6:(k + 1) * 6], f'{title}  ({k + 1}/{math.ceil(len(clips) / 6)}, x{sp})', p, sp); parts.append(p)
lst = os.path.join(td, 'list.txt'); open(lst, 'w').write(''.join(f"file '{p}'\n" for p in parts))
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', lst, '-c', 'copy', '-movflags', '+faststart', out])
for p in parts: os.remove(p)
os.remove(lst); os.rmdir(td)
print(f'{out}: 영상 {len(clips)} 개, 배속 x{sp}, {os.path.getsize(out) / 1e6:.1f} MB')
