r"""video_eval (M1-A tool): score passenger_detector on a recorded video (8.3).

    ros2 run smart_hub video_eval --video lab.mp4 --labels labels.csv \\
        --image-points u1 v1 u2 v2 u3 v3 u4 v4 --map-points x1 y1 x2 y2 x3 y3 x4 y4 \\
        --detector hog --out output/video_eval

Runs the same pipeline as the ROS node (detector -> foot point -> homography
-> tracker -> queue logic) on every frame, offline and repeatable, and
compares it with hand labels.

labels.csv   frame,count[,crossing]      one row per labelled frame (>= 100)
boxes.csv    frame,x,y,w,h               optional, person boxes -> precision/recall

Writes to --out: results.json (MAE, RMSE, exact-count rate, crossing
accuracy, precision/recall), per_frame.csv, count_vs_truth.png and
annotated frames (frame_<n>.png: boxes, foot points, count).

    --prefill labels_draft.csv   writes the detector's own counts for every
                                 --every-th frame, for a human to CORRECT;
                                 the assignment requires manual labels.
"""
import argparse
import csv
import json
import math
import os

import cv2
from smart_hub.perception.detectors import foot, make_detector
from smart_hub.perception.homography import fit_homography, image_to_map
from smart_hub.perception.queue_logic import QueueLogic


def read_csv(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def match_boxes(dets, truth, threshold=0.5):
    """Greedy IoU matching; returns (true positives, false pos, false neg)."""
    pairs = sorted(((iou(d, t), i, j) for i, d in enumerate(dets)
                    for j, t in enumerate(truth)), reverse=True)
    used_d, used_t = set(), set()
    for score, i, j in pairs:
        if score >= threshold and i not in used_d and j not in used_t:
            used_d.add(i)
            used_t.add(j)
    tp = len(used_d)
    return tp, len(dets) - tp, len(truth) - tp


def run_video(video, detector, h, every=1):
    """Yield (frame_index, time_s, bgr, detections, state) for every frame."""
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise ValueError('cannot open video %r' % video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    logic = QueueLogic()
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = index / fps
        dets = detector.detect(frame)
        people = []
        for d in dets:
            try:
                people.append(image_to_map(h, *foot(d, detector.foot_offset)) + (d[4],))
            except ValueError:
                continue
        logic.on_frame(people, t, t)
        yield index, t, frame, dets, logic.state(t), logic
        index += 1


def annotate(frame, dets, state, foot_offset):
    img = frame.copy()
    for d in dets:
        x, y, w, h, _ = d
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 200, 0), 2)
        u, v = foot(d, foot_offset)
        cv2.circle(img, (int(u), int(v)), 4, (0, 0, 255), -1)
    text = 'count %d  crossing %s  valid %s' % (state['count'], state['crossing_occupied'],
                                                state['valid'])
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return img


def evaluate(args):
    detector = make_detector(args.detector)
    h = fit_homography(args.image_points, args.map_points)
    os.makedirs(args.out, exist_ok=True)
    if args.prefill:
        with open(args.prefill, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(['frame', 'count', 'crossing'])
            for i, _, _, _, s, _ in run_video(args.video, detector, h):
                if i % args.every == 0:
                    writer.writerow([i, s['count'], int(s['crossing_occupied'])])
        return {'prefill': args.prefill,
                'note': 'detector output, NOT ground truth: correct every row by hand'}

    labels = {int(r['frame']): r for r in read_csv(args.labels)}
    boxes = {}
    if args.boxes:
        for r in read_csv(args.boxes):
            boxes.setdefault(int(r['frame']), []).append(
                tuple(float(r[k]) for k in ('x', 'y', 'w', 'h')))
    rows, errors, cross_ok, cross_n = [], [], 0, 0
    tp = fp = fn = 0
    keep = set(sorted(labels)[::max(1, len(labels) // 6)][:6]) if labels else set()
    for i, t, frame, dets, s, _ in run_video(args.video, detector, h):
        if i not in labels:
            continue
        truth = int(labels[i]['count'])
        row = {'frame': i, 't': round(t, 2), 'truth': truth, 'count': s['count'],
               'valid': s['valid'], 'crossing': int(s['crossing_occupied'])}
        errors.append(abs(s['count'] - truth))
        if labels[i].get('crossing', '') != '':
            cross_n += 1
            cross_ok += int(int(labels[i]['crossing']) == int(s['crossing_occupied']))
            row['truth_crossing'] = int(labels[i]['crossing'])
        if i in boxes:
            a, b, c = match_boxes([d[:4] for d in dets], boxes[i])
            tp, fp, fn = tp + a, fp + b, fn + c
        rows.append(row)
        if i in keep:
            cv2.imwrite(os.path.join(args.out, 'frame_%05d.png' % i),
                        annotate(frame, dets, s, detector.foot_offset))
    n = len(errors)
    result = {
        'video': os.path.basename(args.video), 'detector': detector.name,
        'labelled_frames': n, 'enough_frames_for_8_3': n >= 100,
        'count_mae': round(sum(errors) / n, 4) if n else None,
        'count_rmse': round(math.sqrt(sum(e * e for e in errors) / n), 4) if n else None,
        'exact_count_rate': round(sum(e == 0 for e in errors) / n, 4) if n else None,
        'crossing_accuracy': round(cross_ok / cross_n, 4) if cross_n else None,
        'precision': round(tp / (tp + fp), 4) if tp + fp else None,
        'recall': round(tp / (tp + fn), 4) if tp + fn else None,
        'target_mae_le_1': (sum(errors) / n <= 1.0) if n else None,
    }
    with open(os.path.join(args.out, 'per_frame.csv'), 'w', newline='') as fh:
        writer = csv.DictWriter(fh, ['frame', 't', 'truth', 'count', 'valid', 'crossing',
                                     'truth_crossing'], extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(args.out, 'results.json'), 'w') as fh:
        json.dump(result, fh, indent=2)
    plot(rows, os.path.join(args.out, 'count_vs_truth.png'), result)
    return result


def plot(rows, path, result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.step([r['t'] for r in rows], [r['truth'] for r in rows], where='post',
            label='hand label', linewidth=2)
    ax.plot([r['t'] for r in rows], [r['count'] for r in rows], 'o', markersize=3,
            label='passenger_detector')
    ax.set_xlabel('time in video (s)')
    ax.set_ylabel('people in queue Q')
    ax.set_title('%s: MAE %s over %d frames'
                 % (result['detector'], result['count_mae'], result['labelled_frames']))
    ax.legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--video', required=True)
    ap.add_argument('--labels', help='frame,count[,crossing] CSV (hand labels)')
    ap.add_argument('--boxes', help='frame,x,y,w,h CSV for precision/recall')
    ap.add_argument('--image-points', type=float, nargs=8, required=True)
    ap.add_argument('--map-points', type=float, nargs=8, required=True)
    ap.add_argument('--detector', choices=('blob', 'hog'), default='hog')
    ap.add_argument('--out', default='output/video_eval')
    ap.add_argument('--prefill', help='write a draft labels CSV instead of scoring')
    ap.add_argument('--every', type=int, default=10, help='prefill every N-th frame')
    args = ap.parse_args(argv)
    if not args.prefill and not args.labels:
        ap.error('--labels is required (or use --prefill to make a draft)')
    print(json.dumps(evaluate(args), indent=2))


if __name__ == '__main__':
    main()
