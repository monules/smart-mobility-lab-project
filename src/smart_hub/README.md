# S2 — AI Demand Prediction (`demand_predictor`, owner M1-B)

Drop-in code for the team's `smart_hub` ament_python package: a ROS 2 Jazzy
node that forecasts the passenger queue 30 s ahead, the training/evaluation
pipeline for test **T03**, a synthetic `/hub/queue` replayer so S2 works
before S1 exists, and 46 unit tests.

```
smart_hub/
  demand_predictor.py        ROS node  (/hub/queue -> /hub/demand, /hub/events)
  fake_queue_publisher.py    ROS node  synthetic /hub/queue replay (test tool)
  demand/                    pure-Python logic, no ROS imports
    features.py              feature vector + 1 Hz history with gap rules
    models.py                persistence baseline + exported ridge model
    estimator.py             freshness checks, warm-up, prediction tick
    dispatch_policy.py       S2 dispatch rule (reference for M2's coordinator)
    simulation.py            seeded episodes, detector model, closed-loop bus
scripts/train_demand_model.py  episodes -> split -> fit -> T03 -> dispatch study
models/demand_ridge_h30.json   shipped model (ID = hash of its coefficients)
config/demand_predictor.yaml   parameters (validated at startup)
launch/demand_demo.launch.py   fake publisher + predictor, standalone demo
results/                       metrics.json, CSVs, plots, split manifest, log
test/                          unittest/pytest suites (colcon test runs them)
```

## 1. Run it

* **On your Mac:** see `RUN_ON_MAC.md` at the workspace root (Docker:
  `./s2.sh build`, then `./s2.sh demo`).
* **On Ubuntu 24.04 with ROS 2 Jazzy installed:**

```bash
cd smart_hub_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch smart_hub demand_demo.launch.py seed:=1025 run_id:=s2-demo-01
# other terminal (after sourcing): ros2 topic echo /hub/demand
```

Expected output: 59 s of `valid: false` (warm-up), then a forecast every
second with `model_id: ridge-demand-h30-d31fd020ac`. A camera freeze makes
the next tick invalid; after recovery the forecast returns once 31 s of
gap-free history exist.

## 2. Merge into the team repository

1. Copy this `smart_hub/demand/`, `smart_hub/demand_predictor.py`,
   `smart_hub/fake_queue_publisher.py`, `models/`, `config/demand_predictor.yaml`,
   `launch/demand_demo.launch.py` and `test/test_demand_*.py` into the team's
   `src/smart_hub/`. Do not overwrite the team's `smart_hub/__init__.py`.
2. Merge `setup.py`: use `packages=find_packages(exclude=['test', 'scripts'])`
   (a plain `packages=[package_name]` silently skips `smart_hub.demand`), and
   add the `models`, `config`, `launch` data_files and the two
   console_scripts entries from this `setup.py`.
3. Merge the `exec_depend`/`test_depend` lines from this `package.xml`.
4. Replace the temporary `smart_hub_interfaces` with M5's full package.

The node itself needs **no numpy or scikit-learn**: the model is evaluated in
pure Python from its JSON. Those libraries (plus matplotlib for plots) are
only needed to retrain. In the integrated hub launch, start only
`demand_predictor` with `config/demand_predictor.yaml`; the real
`passenger_detector` provides `/hub/queue`. Never run `fake_queue_publisher`
alongside it.

## 3. Retrain and reproduce T03

```bash
python3 scripts/train_demand_model.py      # ~6 s; writes models/ and results/
python3 -m pytest test -q                   # or: colcon test --packages-select smart_hub
```

Same seeds give the same model ID. Outputs: `results/metrics.json`
(everything below), `per_episode_test.csv`, `dispatch_comparison.csv`,
`split_manifest.json`, three PNG figures, `train_log.txt`, and
`alt_ridge_queue_h30.json` (the rejected queue-target model, kept as evidence).

## 4. What `/hub/demand` means

| Field | Valid forecast | Invalid forecast |
|---|---|---|
| `valid` | `true` | `false` |
| `predicted_count` | ridge forecast, ≥ 0 | `-1.0` |
| `baseline_count` | persistence `q_t` | `q_t` if the current sample is fresh, else `-1.0` |
| `model_id` | e.g. `ridge-demand-h30-d31fd020ac` | same (or `persistence-h30`) |
| `horizon_sec` | `30` | `30` |
| `header.stamp` | observation time of the `/hub/queue` sample used | publication time |

`-1.0` is used instead of 0 because a stale zero is not an empty queue.
Consumers must check `valid` first. The reason for every validity change is
published on `/hub/events` (`entity_id: demand_predictor`, `correlation_id:
model_id`).

## 5. Validity rules

| Situation | Result |
|---|---|
| Fewer than 60 valid 1 Hz samples since start | `WARMUP` |
| No new valid message accepted since the last tick, or receipt age > 1.0 s, or source-stamp age > 1.0 s | `STALE_INPUT` (first tick after a freeze, i.e. < 1.2 s) |
| `valid=false`, confidence < `queue_confidence_min`, negative/NaN count, stamp > 0.5 s in the future | message rejected, counts as no observation |
| Same or older `header.stamp` re-sent (frozen source) | rejected as `FROZEN_STAMP` |
| A gap within the last 31 s | `GAP_RECOVERY` (features are never computed over filled-in values) |
| 60 consecutive missing seconds | history cleared, full warm-up again |
| Model raises or returns non-finite | `MODEL_ERROR` |
| Model file missing / edited / wrong features or horizon | node starts on `persistence-h30` and emits `MODEL_FALLBACK` (`require_model:=true` refuses to start instead) |

Parameters are range-checked at startup and locked at runtime: changing one
requires a restart, so each run's parameter dump (logged at startup) stays
true for the whole run.

## 6. Design decisions to know for the defence

**Features** (identical code offline and online): `q_t`, 10 s mean, 30 s
mean, 10 s trend `q_t − q_{t−10}`, 30 s arrivals, 30 s departures. This is
the assignment's vector plus the 30 s mean from the team plan. `QueueState`
has only a count, so arrivals/departures are *count-delta proxies*
(positive/negative 1 s changes). A join and a leave in the same second
cancel out. That is the documented proxy assignment 9.2 allows.

**Forecast target: unserved demand, not the raw future queue.** The
shipped model predicts how many people would be waiting at t+30 *if no bus
boarded in between*. A model trained on the raw future queue learns that
long queues tend to shrink because a bus usually comes. Used for dispatch,
that forecast delays the very bus it is predicting, and it made waits
longer than the purely reactive rule. Both models are trained and reported;
see `results/S2_report.md`. Confirm with the instructor that this target
definition is acceptable.

**Split:** 50 seeded six-minute episodes over 7 demand profiles, split
30/10/10 by whole episode before any windowing, stratified so every profile
is in every split. The scaler is fitted on train only and alpha is chosen on
validation only. Labels come from simulator ground truth and are never
features; a unit test replays observations up to *t* and checks the
features match, to show nothing after *t* is used.

## 7. Verified here vs. still to do on a ROS machine

Verified in this environment (no ROS installed): all pure-Python logic (46
tests, including an end-to-end T03 training run), and the two ROS nodes'
wiring against stub `rclpy`/message classes: field names and types, QoS
depths, stamp preservation, events, fallback, parameter rejection and the
fake publisher's 5 Hz replay.

Not verified here, do on Ubuntu 24.04 + Jazzy: `colcon build`, the real
interface package, launch file, `ros2 topic hz/info` output, and a bag of
`/hub/queue` + `/hub/demand` + `/hub/events` as T03/S2 evidence.

## 8. Handoffs

* **M5 (interfaces):** needs the three message types above, unchanged from
  section 6.
* **M1-A (S1):** `header.stamp` must be the observation time; publish
  `valid=false` on freeze (do not keep re-stamping old counts); `zone_id: Q`.
* **M2 (coordinator / T04):** use `valid` before `predicted_count`;
  `smart_hub/demand/dispatch_policy.py` implements the section 9.3 rule
  (3 × ≥ 6, one outstanding goal, 30 s cooldown, clear after 5 × < 4,
  labelled `FALLBACK` to the observed count) with unit tests, if useful.
