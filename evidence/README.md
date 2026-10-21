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

## M2-A — bus docking (S3)

Code and checker by M5 for M2-A (review pending). Commit `8235182` plus the
uncommitted M2-A work (`8235182-dirty`); the picture was redrawn from the
same CSV after a plotting fix.

| File | What it shows | Command |
|---|---|---|
| `m2a/s3/s3_check.json` | 12/12 checks + all 15 trials: T05 10/10 (≤ 0.040 m), door interlock, crossing grant/release, boarding 4/1, ETA error at 5 m (max 1.12 s), T06 cancel 0.043 s, pedestrian in C, blocked bay, stale pose 0.53 s, coordinator dispatch | `docker/hub.sh s3` |
| `m2a/s3/s3_trials.csv` | one row per trial (status, pose error, door time, boarding, ETA error) | same run |
| `m2a/s3/s3_trace.csv` | bus path trace: position, heading, measured speed at 10 Hz (§10.3) | same run |
| `m2a/s3/s3_events.csv` | action feedback and results, crossing grant/renew/release, door, docking, boarding events | same run |
| `m2a/s3/s3_paths.png` | paths and distance-to-B over time for every trial | same run |

## M2-B — dispatch (T04)

Checker by M5, node by M2-B (two small changes, see README §10).

| File | What it shows | Command |
|---|---|---|
| `m2b/t04/t04_check.json` | 6/6: threshold noise → 0 dispatches; rising demand → exactly 1 (PREDICTED, third sample), no second goal; invalid forecast → 0; feed gap → 0; observed queue 7 → FALLBACK | `docker/hub.sh t04` |
| `m2b/t04/t04_timeline.csv` | every forecast/queue value sent and every coordinator and bus event, per phase | same run |

## M3 — parking and traffic (S4, S5)

Scenario files and checkers by M5; nodes by M3. Commit `72e2fde` plus the
uncommitted S4/S5 work (recorded as `72e2fde-dirty` in the JSON).

| File | What it shows | Command |
|---|---|---|
| `m3/s4/s4_check.json` | 11/11: concurrent requests, idempotent retry, wrong-token release, arrival, offline → UNKNOWN, T08 expiry keeps sensed state, full lot, T07 20 pairs: 0 double allocations, p95 39 ms | `docker/hub.sh s4` |
| `m3/s5/s5_check.json` | 9/9: no false congestion, CONGESTED 0.29 s, clears 1.8 s, T10 UNKNOWN 0.995 s with −1.0 values, recovery, BLOCKED 0.33 s, unblocked 2.3 s | `docker/hub.sh s5` |
| `m3/s5/s5_labels.png`, `s5_timeseries.csv` | Cs-J label and car count over the whole run | same run |

## Tests

`tests/colcon_test_summary.txt`: 187 tests, 185 passed, 2 expected failures
(M1-B style debt), 0 errors.

M1-B's S2 evidence lives in `src/smart_hub/results/` (his branch layout).
