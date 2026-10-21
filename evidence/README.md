# Evidence

Measured results behind the report (`docs/report/`). Everything here is
**simulation**, produced in Docker on Apple Silicon at commit `af37efa`
(branch `integration/current-team-work`), and can be regenerated with the
commands in the root README §5. Each JSON records the commit it came from.

## M5 — simulator, safety supervisor (S9)

| File | What it shows | Command |
|---|---|---|
| `m5/s9/s9_check.json` | S9 recovery sequence, 13 checks incl. T16 (5 trials) and T17 controller loss | `docker/hub.sh s9` |
| `m5/s9/events.csv`, `summary.json` | every safety event; stop latencies, simulated stop times, motion while latched (0) | same run |
| `m5/s9/bag_S9_seed9/`, `bag_S9_info.txt` | rosbag (MCAP), 13.6 s, 1,784 messages, safety topics (§20.2) | same run |
| `m5/s9/params/*.yaml` | parameter dump of every M5 node (§17) | same run |
| `m5/s9_repeat/s9_repeat.{json,csv}` | 5 faults × 5 mission phases, 25/25 passed (§16.3) | `docker/hub.sh s9x` |
| `m5/dashboard/` | dashboard video and frames during S9 (`/hub/markers` rendered without RViz) | `docker/hub.sh s9` |

T17 process-kill (from `test_t17_process_kill.py`, same commit): controller
killed → supervisor zero after 0.522 s; supervisor killed → bridge zero after
0.499 s; bridge killed → simulator stops.

## M1-A — passenger detection (S1)

| File | What it shows | Command |
|---|---|---|
| `m1a/s1_blob/s1_check.json` | T01/T02/crossing checks, blob baseline: MAE 0.000, 6/6 tracks, freeze → invalid 1.034 s | `docker/hub.sh s1` |
| `m1a/s1_blob/s1_count_vs_truth.png`, `s1_timeseries.csv` | count-versus-truth plot and data (§8.3) | same run |
| `m1a/s1_hog/` | same with OpenCV HOG: MAE 0.992, recall 0.67 (made for real video) | `docker/hub.sh s1 hog` |
| `m1a/camera/*.png` | annotated synthetic camera frames before / during / with a person in C | rendered from the S1 script |

**Missing (TODO):** the §8.3 real-video evaluation (≥ 100 hand-labelled
frames) — see README_M1A.md.

## Tests

`tests/colcon_test_summary.txt`: 187 tests, 185 passed, 2 expected failures
(M1-B style debt), 0 errors.

M1-B's S2 evidence lives in `src/smart_hub/results/` (his branch layout).
