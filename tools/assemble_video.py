"""Assemble the team video from the members' recordings (run inside Docker).

    docker/hub.sh run 'python3 tools/assemble_video.py'

Order and titles follow docs/demo/VIDEO_SCRIPT.md. For every part:
  * pauses longer than KEEP_PAUSE are shortened to KEEP_PAUSE (silence is
    judged against each recording's own mean level, so quiet speakers keep
    their words);
  * picture scaled and padded to 1920x1080, 30 fps; audio loudness-normalised
    (EBU R128, -16 LUFS) to 48 kHz stereo;
  * a 3 s title card before every part, an end card at the end;
  * if the total would exceed TARGET, everything is sped up by the same
    small factor (atempo keeps the voices' pitch).
Writes videos/final/team_video.mp4 and videos/final/assembly.json (what was
cut and why). Recordings are read from project-video-recs/.
"""
import json
import os
import re
import subprocess

SRC = 'project-video-recs'
WORK = 'output/edit/parts'
OUT = 'videos/final/team_video.mp4'
KEEP_PAUSE = 0.4          # s of pause left where a longer pause was
MIN_PAUSE = 0.6           # s: shorter pauses are left alone
TARGET = 29.5 * 60        # s, safely under the 30 min limit
CARD = 3.0                # s per title card
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

PARTS = [
    ('Introduction', 'Saidumar · team leader, M5', ['intro_saidumar.mov']),
    ('Architecture and how to run it', 'Saidumar · M5',
     ['architecture and how to run it_saidumar_pt1.mov',
      'architecture and how to run it _saidumar_pt2.mov']),
    ('Passenger detection (S1)', 'Husan · M1-A', ['m1apassengerdetection_husan.mov']),
    ('Demand prediction (S2)', 'Tuymurod · M1-B', ['M1B_Tuymurod_demand_prediction.mp4']),
    ('Hub coordinator (T04, crossing)', 'Arkar · M2-B', ['M2-B Hub coordinator_arkar.mov']),
    ('Bus docking (S3)', 'Fazliddin · M2-A', ['m2abusdocking_fazliddin.mp4']),
    ('Parking and traffic (S4, S5)', 'Carla · M3', ['m3parkingandtraffic_carla.mov']),
    ('Routing and escort robot (S6, S7)', 'Aidil · M4', ['M4_Aidil.mov', 'M4_AidilCN.mov']),
    ('Simulator and safety (S9)', 'Saidumar · M5',
     ['m5simulator_saidumar_pt1.mov', 'm5simulator_saidumar_pt2.mov',
      'm5simulator_saidumar_pt3.mov']),
    ('S8 rush hour and all results', 'Arkar · M2-B',
     ['S8 — everything together, and all results_arkar.mov']),
]
END_LINES = ['Thanks for watching', 'Team Shenanigans',
             'Husan (M1-A) · Tuymurod (M1-B) · Fazliddin (M2-A) · Arkar (M2-B)',
             'Carla (M3) · Aidil (M4) · Saidumar (M5, team leader)',
             'Smart Mobility Engineering Lab · Inha University']

VIDEO = ['-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21', '-pix_fmt', 'yuv420p',
         '-r', '30', '-profile:v', 'high']
AUDIO = ['-c:a', 'aac', '-b:a', '160k', '-ar', '48000', '-ac', '2']


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise SystemExit('ffmpeg failed: %s\n%s' % (' '.join(cmd[:6]), p.stderr[-2000:]))
    return p.stderr


def duration(path):
    return float(subprocess.check_output(
        ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0',
         path]).decode())


def keep_intervals(path):
    """Speech intervals: everything except pauses, pauses cut to KEEP_PAUSE."""
    mean = float(re.search(r'mean_volume: (-?[\d.]+)', run(
        ['ffmpeg', '-hide_banner', '-nostats', '-i', path, '-vn', '-af', 'volumedetect',
         '-f', 'null', '-'])).group(1))
    log = run(['ffmpeg', '-hide_banner', '-nostats', '-i', path, '-vn', '-af',
               'silencedetect=noise=%ddB:d=%.2f' % (round(mean - 14), MIN_PAUSE),
               '-f', 'null', '-'])
    starts = [float(x) for x in re.findall(r'silence_start: (-?[\d.]+)', log)]
    ends = [float(x) for x in re.findall(r'silence_end: ([\d.]+)', log)]
    total = duration(path)
    ends += [total] * (len(starts) - len(ends))
    keep, t = [], 0.0
    for s, e in zip(starts, ends):
        s = max(0.0, s)
        half = KEEP_PAUSE / 2
        if s + half > t:
            keep.append((t, s + half))
        t = max(t, e - half)
    if total - t > 0.05:
        keep.append((t, total))
    return [(a, b) for a, b in keep if b - a > 0.05], total, round(mean - 14)


def card(path, lines, seconds):
    """Dark title card with centred lines (first line bold and large).

    The text goes through small files (textfile=) so it needs no filtergraph
    escaping.
    """
    draws, y = [], 540 - 60 * (len(lines) - 1) // 2 - 20
    for k, line in enumerate(lines):
        size, font = (64, BOLD) if k == 0 else (38, FONT)
        txt = '%s.line%d.txt' % (path, k)
        with open(txt, 'w', encoding='utf-8') as fh:
            # ffmpeg 6.1 drawtext drops one trailing character per multibyte
            # character (and one more), so pad with that many spaces
            fh.write(line + ' ' * (1 + len(line.encode()) - len(line)))
        draws.append("drawtext=fontfile=%s:textfile=%s:fontsize=%d:fontcolor=white:"
                     "x=(w-text_w)/2:y=%d" % (font, txt, size, y))
        y += 90 if k == 0 else 60
    run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
         '-f', 'lavfi', '-i', 'color=c=0x14181d:s=1920x1080:r=30:d=%.2f' % seconds,
         '-f', 'lavfi', '-i', 'anullsrc=r=48000:cl=stereo', '-t', '%.2f' % seconds,
         '-vf', ','.join(draws)] + VIDEO + AUDIO + [path])


def render(path, sources, speed):
    """Render every source on its own, then join them (same codecs, no re-encode).

    Each source is one ffmpeg run with a plain video filter and a plain audio
    filter: select/aselect keep the speech intervals. Joining video and audio
    inside one filter graph (concat filter) queued frames until the container
    ran out of memory on the 2880x1800 screen recordings.
    """
    pieces = []
    for k, (src, keep) in enumerate(sources):
        expr = '+'.join('between(t,%.3f,%.3f)' % (a, b) for a, b in keep)
        vf = ("fps=30,select='%s',setpts=N/30/TB/%.4f,scale=1920:1080:"
              "force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:"
              "color=0x14181d,setsar=1" % (expr, speed))
        af = ("aresample=48000,aformat=channel_layouts=stereo,aselect='%s',"
              "asetpts=N/SR/TB,atempo=%.4f,loudnorm=I=-16:TP=-1.5:LRA=11,aresample=48000"
              % (expr, speed))
        piece = '%s.src%d.mp4' % (path, k)
        for name, text in (('vf', vf), ('af', af)):
            with open('%s.%s' % (piece, name), 'w') as fh:
                fh.write(text)
        run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-i', src,
             '-filter_script:v', piece + '.vf', '-filter_script:a', piece + '.af'] +
            VIDEO + AUDIO + [piece])
        pieces.append(piece)
    if len(pieces) == 1:
        os.replace(pieces[0], path)
        return
    lst = path + '.list'
    with open(lst, 'w') as fh:
        fh.writelines("file '%s'\n" % os.path.abspath(p) for p in pieces)
    run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-f', 'concat', '-safe', '0',
         '-i', lst, '-c', 'copy', path])


def main():
    os.makedirs(WORK, exist_ok=True)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    plan, total = [], 0.0
    for title, who, files in PARTS:
        sources, raw, kept = [], 0.0, 0.0
        for f in files:
            keep, dur, thr = keep_intervals(os.path.join(SRC, f))
            sources.append((os.path.join(SRC, f), keep))
            raw += dur
            kept += sum(b - a for a, b in keep)
        plan.append({'title': title, 'who': who, 'files': files, 'raw_s': round(raw, 1),
                     'after_pauses_s': round(kept, 1), 'sources': sources})
        total += kept + CARD
    total += CARD * 2                                   # end card
    speed = max(1.0, total / TARGET)
    pieces = []
    for i, p in enumerate(plan):
        c = os.path.join(WORK, '%02d_card.mp4' % i)
        card(c, ['Part %d of %d · %s' % (i + 1, len(plan), p['title']), p['who'],
                 'Team Shenanigans · Smart Mobility Hub'], CARD)
        v = os.path.join(WORK, '%02d_part.mp4' % i)
        render(v, p['sources'], speed)
        pieces += [c, v]
        print('part %d done: %s' % (i + 1, p['title']), flush=True)
    end = os.path.join(WORK, '99_end.mp4')
    card(end, END_LINES, CARD * 2)
    pieces.append(end)
    lst = os.path.join(WORK, 'list.txt')
    with open(lst, 'w') as fh:
        fh.writelines("file '%s'\n" % os.path.abspath(x) for x in pieces)
    run(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-f', 'concat', '-safe', '0',
         '-i', lst, '-c', 'copy', '-movflags', '+faststart', OUT])
    final = duration(OUT)
    report = {'output': OUT, 'duration_s': round(final, 1),
              'duration_min': round(final / 60, 2), 'speed_factor': round(speed, 4),
              'size_mb': round(os.path.getsize(OUT) / 1e6, 1),
              'parts': [{k: v for k, v in p.items() if k != 'sources'} for p in plan]}
    with open(os.path.join(os.path.dirname(OUT), 'assembly.json'), 'w') as fh:
        json.dump(report, fh, indent=2)
    print(json.dumps({k: report[k] for k in ('duration_min', 'speed_factor', 'size_mb')}))


if __name__ == '__main__':
    main()
