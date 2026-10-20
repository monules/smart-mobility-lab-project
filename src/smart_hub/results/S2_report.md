# S2 AI Demand Prediction — results (draft section for the technical report)

All numbers come from `python3 scripts/train_demand_model.py` (seeds
1000–1049, split seed 42) and are stored in `results/metrics.json`. Data are
**synthetic**: seeded simulator episodes with a noisy 5 Hz detector model, not
camera-derived tracks.

## Method

* **Data:** 50 independent six-minute episodes covering seven demand
  profiles (steady, ramp up, ramp down, peak, wave, burst arrivals, noisy
  around the dispatch threshold). Each episode has detector count flicker
  and 0–2 outages (invalid flag, silence, or frozen source stamp; 30 of 50
  episodes have at least one). In 30 episodes the bus runs on a timetable;
  in the other 20 it is dispatched by the reactive rule.
* **Split:** whole episodes, 30 train / 10 validation / 10 test, fixed
  before windowing, stratified so every profile is in every split.
* **Features** (computed by the same code as the ROS node, at 1 Hz):
  `q_t`, 10 s mean, 30 s mean, 10 s trend, 30 s arrivals and departures
  (count-delta proxies).
* **Model:** standardized ridge regression, prediction clipped at 0. The
  scaler is fitted on train only. Alpha is chosen from 8 values on
  validation MAE (selected: 30).
* **Baseline:** persistence, `q̂(t+30) = q_t`.
* **Target:** unserved demand at t+30, i.e. people who would still be
  waiting at t+30 if no bus boarded after t. A second model trained on the
  raw future queue length is reported for comparison.

## T03 — held-out test episodes (2,500 forecasts)

| Model (target) | MAE | RMSE | MAE diff vs persistence (95% CI, episode bootstrap) | Episodes better / worse |
|---|---|---|---|---|
| **Ridge, demand target (shipped)** | **1.215** | **1.636** | **−0.292 (−0.543 … −0.044)** | **7 / 3** |
| Persistence, same labels | 1.506 | 2.091 | — | — |
| Ridge, raw-queue target | 1.624 | 2.074 | −0.044 (−0.157 … +0.070) | 5 / 5 |
| Persistence, raw-queue labels | 1.668 | 2.183 | — | — |

Inference latency (feature extraction + model, pure Python):
p50 0.007 ms, **p95 0.013 ms** (n = 2,797; Linux x86_64, Python 3.13).
This is far below the 100 ms target. Re-measure on the team's reference
machine from the node's `latency ms:` log line.

**Pass:** shipped-model MAE is lower than persistence on unseen episodes
(confidence interval excludes zero), p95 < 100 ms, ≥ 30 episodes.

### Where the model is worse (required documented case)

* **Steady and ramp-down episodes** (seeds 1008, 1024, 1038): ridge MAE
  0.94 / 1.28 / 1.17 vs persistence 0.85 / 0.98 / 1.03. When demand is
  falling or flat, the model's built-in expectation of new arrivals adds a
  bias of up to about one passenger.
* **Fast acceleration** (seed 1044, ramp up): the largest single errors
  are forecasts of 7–8 when 15–16 people actually accumulated. A linear
  model of the last 30 s cannot anticipate a surge that has not started
  yet. It still beats persistence on this episode (2.03 vs 2.29), but
  both under-forecast badly.
* **Raw-queue target:** statistically indistinguishable from persistence.
  The section below shows it also made dispatch worse.

## Downstream effect: predictive vs reactive dispatch

The bus is dispatched by the section 9.3 rule (3 consecutive samples ≥ 6,
one outstanding mission, 30 s cooldown, escalation cleared after
5 samples < 4). Bus travel time is 22–40 s, boarding is up to 4 people over
8 s. 50 unseen seeds (10 test + 40 never used) were run. Every arm sees the
identical passenger arrival stream.

| Dispatch rule | Mean wait of boarded passengers | Boarded | Still waiting at end (censored) | Dispatches |
|---|---|---|---|---|
| Reactive (observed queue ≥ 6) | 73.7 s | 750 | 204 | 213 |
| **Predictive, demand-target model** | **63.7 s** | **817** | **159** | 234 |
| Predictive, raw-queue-target model | 75.1 s | 757 | 196 | 211 |

* Demand-target forecast vs reactive: **−10.0 s** paired mean difference
  (95% CI −14.9 … −6.0). Lower wait in 35 of 50 episodes.
* Raw-queue-target forecast vs reactive: **+1.4 s** (95% CI +0.4 … +2.7).
  Lower wait in only 4 of 50 episodes.

The raw-queue model has learned that long queues tend to shrink because a
bus usually arrives. Used for dispatch, that forecast postpones the bus it
is predicting. Forecasting demand without service avoids this feedback
loop. Part of the gain comes from earlier, slightly more frequent dispatch
(234 vs 213 missions); the run-level CSV allows weighing this against bus
operating cost. As the assignment states, forecast accuracy alone does not
establish transport benefit.

## Limitations

* All results are on synthetic data. Real detector noise (M1-A's
  `passenger_detector`) will differ, so re-evaluate on recorded `/hub/queue`
  bags once S1 runs.
* Arrival/departure features are proxies from count changes. Offsetting
  events within one second are invisible.
* The demand target ignores the 16-person queue capacity, and labels come
  from simulator ground truth that does not exist on real data. On real
  data the label would have to be rebuilt from track entries and boardings.
* The dispatch study is an offline closed-loop simulation using
  `dispatch_policy.py`, not the integrated ROS coordinator (T04/S8, M2).
* While the forecast is invalid (warm-up, outages) the predictive arm falls
  back to the observed-count rule: 41 of its 234 dispatches were labelled
  `FALLBACK`. The fallback's share of the benefit was not isolated.

## Figures

* `forecast_examples.png`: three held-out episodes. Shows the label, the
  ridge forecast and persistence; the warm-up and outage gaps appear as
  breaks in the lines.
* `per_episode_mae.png`: MAE per test episode, with the three
  model-worse episodes labelled.
* `dispatch_wait_paired.png`: per-episode mean wait, predictive vs reactive,
  for both targets.
