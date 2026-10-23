# M1-A — the people counter 👀 (Husan, scenario S1)

Branch: `feature/m1a-passenger-detector` (also merged into
`integration/current-team-work`). Husan said yes to this being written for
him — so it's his now. Read it, get it, own it before the defence. 🫡

---

## tl;dr

There's a camera looking at the bus stop. Your program looks at every
picture, finds the people, and says **how many are waiting in the line** and
**if someone is standing on the road crossing**. 10 pictures in, 5 answers
out, every second. That's it. That's the job.

## how it works (no cap, just 5 steps)

1. **Spot people** 🔍 — find every person in the picture and draw a box
   around them.
2. **Look at their feet** 🦶 — the bottom middle of the box = where they
   stand on the floor.
3. **Picture → real floor** 🗺️ — turn "pixel (300, 140)" into "3.3 m, 6.6 m
   on the map". This trick is called a *homography*. You teach it once with
   4 floor points you know.
4. **Remember who's who** 🧠 — give each person a number and follow them
   from picture to picture, so the same person never gets counted twice.
5. **Count & check** ✅ — how many feet are inside the queue box Q? Is
   anyone inside the crossing box C? Send the answer on `/hub/queue`.

## the rules that keep it honest 📏

| rule | what it means in normal words | number |
|---|---|---|
| don't trust a flash | a new person only counts after being seen for a bit | 0.4 s |
| don't forget too fast | someone who vanishes still counts for a bit (maybe they got blocked) | 0.4 s |
| peekaboo is fine | if someone hides behind another person and comes back, they keep their number | up to 0.5 s |
| crossing stays "busy" | after the crossing gets empty, wait before saying it's free | 2 s |
| camera froze? say so | no new pictures (or the same old picture again) → answer "I don't know" (`valid=false`), never "0 people" | after 1 s |
| unknown crossing = busy | when we don't know, the crossing counts as occupied so nobody drives in | always |

Why? A wrong "0 people" or a wrong "crossing is free" could let a bus drive
into someone. "I don't know" is always safer than a guess. 🛑

## the files (where stuff lives)

| file | what it does |
|---|---|
| `src/smart_hub/smart_hub/passenger_detector.py` | the main program (the ROS node) |
| `perception/detectors.py` | finds people: `blob` (dark shapes, for the fake camera) or `hog` (a real people detector built into OpenCV, for real video) |
| `perception/homography.py` | pixels → metres |
| `perception/tracker.py` | remembers who's who |
| `perception/zones.py` | "is this point inside box Q / box C?" |
| `perception/queue_logic.py` | counting, the 2 s crossing rule, the "camera froze" rule |
| `s1_check.py`, `launch/s1.launch.py` | runs the whole S1 test and checks it |
| `video_publisher.py` | plays a real video as if it was the camera |
| `video_eval.py` | scores your detector on a real video vs your hand labels |
| `config/passenger_detector.yaml` | all the settings + the 4 calibration points |

The fake camera itself is M5's (`camera_model.py`): it draws little people on
a grey floor with the queue and crossing painted on.

## run it 🚀

```bash
docker/hub.sh s1
```
→ prints `S1 verdict: PASSED` and saves proof in `output/`:
`s1_check.json` (the checks), `s1_count_vs_truth.png` (the graph),
`dashboard/` (pictures + video of the map).

Try the real-video detector on the fake camera:
```bash
docker/hub.sh s1 hog
```

## what's already proven ✅ (fake camera, simulation)

- counts go **0 → 3 → 5 → 4** exactly like the assignment script (§8.3), error
  **0.000** over ~127 checks
- 6 people → exactly 6 ids (no double counting, even with a 0.3 s hide)
- camera freezes → "I don't know" after **1.04 s** (limit 1.2 s)
- crossing stays busy **2.5 s** after the person leaves (0.4 s + 2 s rule)
- 🐛 a bug we found in the graph and fixed: right after a freeze the count
  used to drop to 0 while still saying "valid" — now it never does
- the `hog` detector on the fake people: error 1.0, finds 4 of 6 people —
  it's made for real humans, not cartoon ones, so that's expected

## 🎥 the real video — where we are

✅ **Filmed** (`IMG_0572.MOV`, 70 s, 1080p, 30 fps; kept in `videos/lab/`, not
in git). The floor tape was not measured, so the two taped boxes simply *are*
queue Q and crossing C: their 8 corners are the calibration points.

✅ **Processed.** Two things the video needed:
- **the phone drifted** up to ~67 px during the take → `perception/stabilizer.py`
  lines every frame up with the first one using the floor pattern
  (`--stabilize`);
- **heads are out of the picture and the camera looks down** → OpenCV's HOG
  (made for whole standing people) finds almost nobody. So we added a third
  detector: **YOLOv4-tiny** (pretrained on COCO, run by OpenCV DNN,
  `--detector yolo`, score threshold 0.5). It finds people without heads.
  HOG stays in as the honest "pretrained baseline that fails here".

⏳ **Your job now: label it.** Saidumar sends you `Husan_labeling_kit.zip`:
141 pictures (one every 0.5 s) + `labels.csv` + `HOW_TO_LABEL.md`. For each
picture write: people with their feet in the blue box, 1/0 for someone in the
black box, 1/0 for "someone is partly hidden". Your own counting, ~30–40 min.
Then the scores (MAE, occluded vs not, HOG vs YOLO), the graph and the report
text get done from your labels.

**Get the YOLO files** (once, ~24 MB, not in git — the submission has no
links, so here is what to look for): the two files `yolov4-tiny.cfg` and
`yolov4-tiny.weights` from the official AlexeyAB **darknet** repository's
"darknet_yolo_v4_pre" release, saved into `models/yolov4-tiny/`. Check them:

| file | size | SHA-256 |
|---|---|---|
| `yolov4-tiny.cfg` | 3,231 B | `f858e3724962eedf3ac44e3b6cb3f0c3d9ed067c306bb831f539c578b924c90e` |
| `yolov4-tiny.weights` | 24,251,276 B | `cf9fbfd0f6d4869b35762f56100f50ed05268084078805f0e7989efe5bb8ca87` |

**The exact commands** (calibration = blue box then black box corners on the
first frame):
```bash
docker/hub.sh run 'ros2 run smart_hub video_eval --video videos/lab/IMG_0572.MOV --detector yolo --stabilize --labels videos/lab/labeling/labels.csv --out output/video_eval_yolo --image-points 488 199 1224 207 1248 335 457 330 933 546 1118 550 1173 1018 943 1018 --map-points 2 7 6 7 6 5 2 5 4.5 4.75 5.5 4.75 5.5 1.25 4.5 1.25'
```
(same with `--detector hog --hog-upscale 0.5 --out output/video_eval_hog` for the baseline)

## still not done (not your fault)

- "video showing the inhibited motion" (§8.3): a bus/robot stopping because
  someone's in the crossing. Your detector already says "crossing busy"; the
  stopping needs the coordinator (M2-B) + bus/robot managers (M2-A, M4).

## defence cheat sheet 🧑‍🏫

- **why the feet?** the homography only works for points *on the floor*;
  heads float 1.7 m up, feet touch the floor.
- **why 4 points?** a homography has 8 unknowns; each point gives 2
  equations → 4 points.
- **why track ids?** so 5 frames of the same person ≠ 5 people.
- **why `valid=false` instead of 0?** 0 means "nobody's there". false means
  "I can't see". Mixing them up is dangerous.
- **where's the logic tested?** `test_m1a_units.py` (each piece),
  `test_m1a_pipeline.py` (whole S1 on fake video, no ROS),
  `test_s1_launch.py` (whole S1 live in ROS), `test_video_eval.py`.
