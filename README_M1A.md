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

## 🎥 what YOU still need to do: the real video

The teacher wants proof on a **real** video too (§1.3, §8.2, §8.3). Code is
done — you just need to film and label. Here's the vibe:

**1. set up (10 min)**
- phone or webcam up high (shelf, tripod, tape it to the wall), pointing down
  at the floor. **Don't move it after this.**
- tape a rectangle on the floor = queue Q, and a strip = crossing C. Small is
  fine (1:4 scale: Q = 1.0 m × 0.5 m, C = 0.25 m × 0.875 m).
- pick 4 tape corners and write down where they are on the map in metres
  (for Q at 1:4 scale: (2,5) (6,5) (6,7) (2,7) — the software keeps map units).

**2. film (2–3 min)**
- friends walk into the queue one by one, wait, leave. Get moments with 0,
  3, 5 and 4 people. Someone walks through the crossing. Two people stand
  so one blocks the other for a sec.
- **feet must be in the picture** (the feet are what we measure). Faces
  don't matter — filming from above/behind is perfect.

**3. calibrate**
- open one frame, read the pixel position of your 4 tape corners (any
  image viewer shows it). Those 8 numbers = `image_points`.

**4. label ≥ 100 frames (the boring part 😮‍💨)**
- get a draft first (the detector guesses, you fix):
```bash
docker/hub.sh run 'ros2 run smart_hub video_eval --video videos/lab.mp4 --detector hog --prefill videos/labels.csv --every 15 --image-points u1 v1 u2 v2 u3 v3 u4 v4 --map-points 2 5 6 5 6 7 2 7'
```
- open `videos/labels.csv` and **fix every row by hand** — look at that
  frame and write the true number of people in Q (and 1/0 for someone in C).
  This has to be YOUR counting; the assignment says *manually annotated*.

**5. score it**
```bash
docker/hub.sh run 'ros2 run smart_hub video_eval --video videos/lab.mp4 --labels videos/labels.csv --detector hog --out output/video_eval --image-points u1 v1 u2 v2 u3 v3 u4 v4 --map-points 2 5 6 5 6 7 2 7'
```
→ `output/video_eval/results.json` (error = MAE, target ≤ 1), a graph
`count_vs_truth.png`, and pictures with boxes drawn (`frame_*.png`). Want
precision/recall too? Also make `boxes.csv` (`frame,x,y,w,h` for each
person) and add `--boxes videos/boxes.csv`.

Put the video in a `videos/` folder in the repo (it's not uploaded to git
unless you choose to — big files). **Or just send the video + your 4
measured corners to Saidumar / Claude and the rest (calibration, running,
graphs, report text) gets done — you still check the labels yourself.**

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
