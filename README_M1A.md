# M1-A — Passenger Detection & Queue Counting (Husan) — S1

Branch `feature/m1a-passenger-detector` (kept as its own branch; also merged
into `integration/current-team-work`). Written for Husan with his agreement;
he owns it — read every file before the individual defence.

## What it does (assignment §8, role doc M1-A)

`passenger_detector` turns `/camera/image_raw` into `/hub/queue`
(`QueueState`, 5 Hz, R10):

```
image ─► detect people ─► foot point of each box ─► homography (pixels → metres)
      ─► track ids over time ─► count confirmed people inside queue Q
                              └► flag anyone inside crossing C (held 2 s)
```

| File | Role |
|---|---|
| `src/smart_hub/smart_hub/passenger_detector.py` | ROS node: image conversion, parameters, 5 Hz publisher, events |
| `perception/detectors.py` | `blob` baseline (dark regions = people) and `hog` (OpenCV pretrained people detector) |
| `perception/homography.py` | 4-point DLT homography, image → floor |
| `perception/tracker.py` | greedy nearest-neighbour tracking in metres; 0.4 s confirm, 0.4 s hold, 0.5 s occlusion memory |
| `perception/zones.py` | queue Q / crossing C polygons, ray-casting point-in-polygon |
| `perception/queue_logic.py` | count, confidence, crossing hold, freeze/stale → `valid=false` |
| `s1_check.py`, `launch/s1.launch.py` | S1 acceptance run against simulator truth |
| `video_publisher.py` | plays a recorded video as `/camera/image_raw` for the real-video demo |
| `config/passenger_detector.yaml` | parameters and the 4-point calibration |

The camera image itself comes from M5's simulator (`camera_model.py`): people
are drawn as figures on the floor and queue Q / crossing C are painted lines.

## Rules and where they come from

| Rule | Value | Source |
|---|---|---|
| Publish rate | 5 Hz, R10 | §5 |
| Appearance / disappearance debounce | 0.4 s | §8.2 step 4 |
| Identity kept through occlusion | 0.5 s (covers the 0.3 s T02 case) | §8.2 step 4; role doc says 0.3 s |
| Feed freeze → `valid=false` | after 1.0 s silence **or** 1.0 s without a newer source stamp | §8.2 step 5, §16.1 |
| Crossing held occupied | until C is clear for 2.0 s | §8.2 step 6 |
| Invalid state | `count 0, confidence 0, valid false, crossing_occupied true` (unknown crossing must deny entry) | §6.1, §16.1 |
| `header.stamp` | observation time of the frame used; publication time while invalid | §6 |
| Confidence | mean detector score of counted people (blob = 1.0) | §6.1 |

## Run it

```bash
docker/hub.sh s1
```
```bash
docker/hub.sh s1 hog
```
Inside `docker/hub.sh shell`: `ros2 launch smart_hub s1.launch.py detector:=blob`,
then `ros2 topic echo /hub/queue`.

Real video (needs a recording of the lab floor):
```bash
ros2 run smart_hub video_publisher --ros-args -p path:=/ws/videos/lab.mp4
```
```bash
ros2 run smart_hub passenger_detector --ros-args --params-file install/smart_hub/share/smart_hub/config/passenger_detector.yaml -p detector:=hog -p image_points:="[u1,v1,u2,v2,u3,v3,u4,v4]" -p map_points:="[x1,y1,x2,y2,x3,y3,x4,y4]"
```
Calibrate by clicking four floor points in one frame and measuring them on
the floor (metres, map frame).

## Verified (Docker, Apple Silicon, simulation)

| Test | Result |
|---|---|
| `test_m1a_pipeline.py` (offline full pipeline on the S1 script) | T01 counts 0, 3, 5, 4; no inflation; 0.3 s occlusion → no new id; crossing 2 s hold; freeze invalid ≤ 1.2 s |
| `test_m1a_units.py` | homography (exact on calibration points, ≤ 2 cm elsewhere vs. the camera model), zones, tracker, queue logic, frozen stamp |
| `test_passenger_detector_node.py` | image encodings, frames → count, parameter validation |
| `test_s1_launch.py` / `docker/hub.sh s1` (live ROS) | blob: MAE 0.000 over 126 settled samples, 6 tracks for 6 people, invalid 1.03 s after freeze, crossing clear 2.47 s after leaving |
| `docker/hub.sh s1 hog` | MAE 0.99 (target ≤ 1), recall 4/6 people on the synthetic figures — HOG is for real video; exact counts are only claimed for `blob` |

## Recording the real video (§1.3, §8.2, §8.3)

The simulator proves the logic; the assignment also wants **one real
camera/video demonstration** evaluated on **at least 100 hand-labelled
frames** (count MAE, precision/recall if boxes are labelled, behaviour under
partial occlusion). Yes, this means recording a real video.

1. **Camera:** a phone or webcam fixed high up (shelf, tripod, taped to a
   wall), looking down at the floor at an angle. It must not move during or
   after calibration.
2. **Floor:** tape a rectangle for queue Q and a strip for crossing C. A
   1:4 scale is fine (§2.1: a 3 m × 2 m board for the 12 m × 8 m map —
   then Q is 1.0 m × 0.5 m and C 0.25 m × 0.875 m on the floor; keep map
   coordinates in software). Mark 4 floor points far apart (the Q corners
   work) and write down their map coordinates in metres.
3. **Film 2–3 minutes**, steady light: people join the queue one by one,
   stand, leave; one walks through C; include 0, 3, 5 and 4 people in Q,
   two people overlapping (partial occlusion), and someone passing in front
   of the queue. Whole bodies including feet must be visible — the feet are
   what the homography maps.
4. **Calibrate:** open one frame, read the pixel coordinates of the 4 taped
   points (any image viewer shows them), put them in `image_points`, and the
   measured map coordinates in `map_points` (`config/passenger_detector.yaml`
   or `-p` overrides).
5. **Label ≥ 100 frames:** e.g. one frame per second. For each frame write
   `frame, count_in_Q, someone_in_C` in a CSV; for precision/recall also draw
   a box around each person (CVAT or LabelImg).
6. **Run and report:** `video_publisher` + `passenger_detector detector:=hog`
   (commands above), record `/hub/queue` with `ros2 bag record`, compare with
   the labels: MAE, precision/recall, and the failures (occlusion, people at
   the image edge). Report failures honestly (§8.3).

No faces are needed (§14.3); film from above or behind.

## Not done (needs the team / a real camera)

- §8.3 "at least 100 manually annotated video frames" with count MAE,
  precision/recall and partial occlusion: needs a recorded lab video and
  hand labels. Tools are ready (`video_publisher`, `detector:=hog`).
- Video evidence (annotated before/during/after frames, count-vs-truth plot).
- Robot/bus are not drawn in the synthetic camera (only people), so the
  detector never sees vehicles there.
