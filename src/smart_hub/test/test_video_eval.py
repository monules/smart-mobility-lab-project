"""video_eval on a real MP4 rendered from the S1 scenario (M1-A tool)."""
import csv
import json
import os

import pytest

cv2 = pytest.importorskip('cv2')

from smart_hub import video_eval  # noqa: E402
from smart_hub.camera_model import PinholeCamera  # noqa: E402
from smart_hub.passenger_detector import DEFAULT_IMAGE_POINTS, DEFAULT_MAP_POINTS  # noqa: E402
from smart_hub.perception.zones import point_in_polygon, QUEUE_Q  # noqa: E402
from smart_hub.sim_core import HubSim  # noqa: E402
import yaml  # noqa: E402

FLOOR = ((2.0, 5.0, 6.0, 7.0), (4.5, 1.25, 5.5, 4.75))


@pytest.fixture(scope='module')
def s1_video(tmp_path_factory):
    """Render 25 s of S1 at 10 fps into an MP4 plus truth labels."""
    d = tmp_path_factory.mktemp('video')
    with open(os.path.join(os.path.dirname(__file__), '..', 'scenarios', 'S1.yaml')) as fh:
        sim = HubSim(yaml.safe_load(fh), seed=1)
    cam = PinholeCamera()
    writer = cv2.VideoWriter(str(d / 's1.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10.0,
                             (640, 480))
    labels = []
    for frame in range(250):
        writer.write(cam.render(sim.camera_people(), FLOOR))
        if frame % 10 == 5:                                # label at t = k + 0.5 s
            q = sum(point_in_polygon(p.x, p.y, QUEUE_Q) for p in sim.pedestrians.values())
            labels.append((frame, q))
        sim.step()
        sim.step()                                         # 2 sim steps = 0.1 s
    writer.release()
    with open(d / 'labels.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['frame', 'count'])
        w.writerows(labels)
    return d


def args_for(d, *extra):
    pts = ['--image-points'] + [str(v) for v in DEFAULT_IMAGE_POINTS] + \
        ['--map-points'] + [str(v) for v in DEFAULT_MAP_POINTS]
    return ['--video', str(d / 's1.mp4'), '--detector', 'blob', '--out', str(d / 'out')] + \
        pts + list(extra)


def test_scores_a_video_against_labels(s1_video):
    video_eval.main(args_for(s1_video, '--labels', str(s1_video / 'labels.csv')))
    with open(s1_video / 'out' / 'results.json') as fh:
        r = json.load(fh)
    assert r['labelled_frames'] == 25 and r['enough_frames_for_8_3'] is False
    assert r['count_mae'] == 0.0 and r['exact_count_rate'] == 1.0
    assert (s1_video / 'out' / 'count_vs_truth.png').stat().st_size > 1000
    assert any(p.name.startswith('frame_') for p in (s1_video / 'out').iterdir())


def test_prefill_writes_a_draft_to_correct(s1_video):
    draft = s1_video / 'draft.csv'
    video_eval.main(args_for(s1_video, '--prefill', str(draft), '--every', '10'))
    with open(draft) as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 25 and set(rows[0]) == {'frame', 'count', 'crossing'}


def test_box_matching_counts_true_and_false_positives():
    tp, fp, fn = video_eval.match_boxes([(0, 0, 10, 20), (50, 50, 10, 20)],
                                        [(1, 1, 10, 20), (100, 100, 10, 20)])
    assert (tp, fp, fn) == (1, 1, 1)


def test_export_frames_and_occlusion_scores(s1_video):
    kit = s1_video / 'kit'
    video_eval.main(args_for(s1_video, '--export-frames', str(kit), '--every', '10'))
    with open(kit / 'labels.csv') as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 25 and list(rows[0]) == ['frame', 't', 'count', 'crossing', 'occluded']
    assert len(list(kit.glob('frame_*.jpg'))) == 25
    # score with an occlusion column: frames split into occluded / not occluded
    labels = s1_video / 'labels_occ.csv'
    with open(s1_video / 'labels.csv') as src, open(labels, 'w', newline='') as dst:
        reader, writer = csv.DictReader(src), csv.writer(dst)
        writer.writerow(['frame', 'count', 'occluded'])
        for k, r in enumerate(reader):
            writer.writerow([r['frame'], r['count'], k % 2])
    video_eval.main(args_for(s1_video, '--labels', str(labels), '--stabilize'))
    with open(s1_video / 'out' / 'results.json') as fh:
        r = json.load(fh)
    assert r['frames_partial_occlusion'] == 12 and r['mae_partial_occlusion'] == 0.0
    assert r['stabilized'] is True and r['frames_aligned'] is not None
