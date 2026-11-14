# What changed since the midterm (final term) — read your part 👀

The prof's review gave us **13.5 / 15** and a list of weak spots. For the final
we fixed them, added the **mandatory UI**, and re-ran everything.
Saidumar (M5) made the changes with Claude Code, **with your permission**. Now
each of you needs to **read your part below, try it, and say "confirmed" (or
"this is wrong") in the group chat.** That's the whole job. ~15 minutes.

How to try anything (same for everyone):

```bash
docker/hub.sh image     # once, if you never built the image
docker/hub.sh build
docker/hub.sh ui        # then open localhost:8080 in your browser
```

In the page: pick your task on the left (or type what you want in the **Ask**
box, like "robot escort with 2 trials"), press **Run**, watch the map, read the
green/red result. Your run's files land in `output/ui/<run>/`.

---

## 🖥️ New for everyone: the Hub Control Center (the mandatory UI)

- **What it is:** a web page that runs inside our Docker. No internet needed.
- **Tasks tab:** every scenario and test (S1–S9, T03, T04, T18, the real video
  check, unit tests per member) with its options (seed, trials, detector,
  dispatch mode…).
- **Ask box:** type normal words ("parking then congestion", "test the e-stop",
  "rush hour reactive seed 3"). It shows a plan first; nothing runs until you
  press Run. Several tasks in one sentence run one after another.
- **Live map + status tiles:** bus, robot, cars, people, crossing, parking,
  safety, both forecasts — moving in real time.
- **Result + History:** the checker's verdict (every check with ✓/✗) and all
  past runs with their logs.
- **How it works tab:** click a node to see who owns it and what it talks to.
- **It cannot control the hub.** It only starts the same commands you would
  type in a terminal and listens. The safety rule is untouched.
- Code: `src/smart_hub/smart_hub/control_center/` (catalog, intent, runs,
  live_state, node, web/).

---

## Husan (M1-A) — passenger detection

**The review said:** "97 % crossing accuracy can hide dangerous mistakes" and
"if the image alignment fails, don't keep using the old one".

**What changed:**
1. The real-video check now writes a full **crossing report**: how often it said
   "clear" when someone was really in the crossing (**false clear** = dangerous)
   vs "busy" when it was empty (**false busy** = safe, just slow).
2. Result on your 141 labels: **YOLO made 0 false clears.** All 4 of its
   mistakes were on the safe side. **HOG missed all 13 "person in crossing"
   frames** — its "90.8 % accuracy" is exactly what you get by always saying
   "clear". (So the prof's worry was right, and now we show it.)
3. If the image alignment is lost for more than 5 frames, the frame is marked
   **invalid** and the crossing is reported **not clear** (safe side).
4. Count numbers did not change: MAE 0.149 (YOLO, stabilised), 0.114 (not
   stabilised), 2.660 (HOG). We still say honestly that stabilising made it
   slightly worse on this video.
5. The raw video and the YOLO weights now go **inside the submission ZIP**, with
   their hashes, so anyone can re-run it.

**Files:** `perception/crossing_metrics.py` (new), `video_eval.py`,
`evidence/m1a/real_video/results_*.json` (look for the `"crossing"` block).
**Try:** UI → "Real lab video evaluation". **Honest limit to say in the
defence:** only ONE person walks into the crossing in our video, and only 6
frames have people hiding behind each other.

## Tuymurod (M1-B) — demand prediction ⚠️ biggest review point

**The review said:** the assignment asks for the **future queue** (how many
people will stand in Q in 30 s). Our model predicted something else: how many
would be waiting **if no bus came**. Same topic name, different question.

**What changed:**
1. Now there are **two topics**, each with an honest name:
   - `/hub/demand` → **future queue** (what the assignment asks; T03 is judged
     here). Model `ridge-queue-h30`.
   - `/hub/demand_unserved` → **unserved demand** (our extra idea). Model
     `ridge-demand-h30`. The coordinator uses THIS one to decide when to send
     the bus.
2. Honest numbers (10 held-out episodes):
   - future queue: ridge **1.6245** vs persistence **1.6684** → not worse, but
     the gain is tiny and **not statistically clear** (the 95 % interval
     includes 0; better in 5 of 10 episodes). We say exactly that.
   - unserved demand: ridge **1.2147** vs persistence **1.5064** (clear gain).
   - **Never compare 1.21 with 1.62** — they are answers to different questions.
3. Why the extra target is useful: when a bus is boarding, the *future queue*
   goes down — so that forecast says "few people soon" right when a bus is
   already there. As a "send a bus" trigger it's the wrong signal. In the
   offline 50-seed study, dispatching on the future-queue forecast was slightly
   **worse** than no forecast (+1.4 s wait), dispatching on unserved demand was
   **better** (−10.0 s).
4. The 120 s rosbag was re-recorded with both topics.

**Files:** `demand_predictor.py`, `models/queue_ridge_h30.json` (new),
`results/S2_report.md` (first section), `results/metrics.json`
(`published_topics`). **Try:** UI → "Demand prediction (live)" and watch both
forecast tiles; "Retrain and evaluate" shows both targets.

## Fazliddin (M2-A) — bus docking

**The review said:** stopping uses a fixed 0.5 s look-ahead; backing into the
crossing again must need a new permission.

**What changed:**
1. The safety supervisor now uses the **stopping-distance formula from the
   assignment**: reaction distance + braking distance + 0.20 m margin. The bus at
   0.40 m/s needs 0.26 m; slower = shorter. (Before: same 0.5 s for every speed.)
2. New test: the bus that left the crossing and reverses toward it is **held**
   until it gets a **new** grant.
3. S3 was re-run with these rules: still passes (numbers in `evidence/m2a/s3/`).

4. **A real bug in your area got fixed (know this one for the defence 🔥):**
   in 5 of 20 rush-hour runs the bus docked fine but its *drive to the exit*
   ended as "pose stale". Why: the simulator removes a bus standing within
   0.30 m of exit E, but the bus only said "departed" at 0.05 m. With tiny
   position noise the simulator sometimes won that race and the bus vanished
   before it could say "done". Fix: the bus now says "departed" at 0.25 m
   (`exit_tol` in `bus_core.py`). 20 new tests replay it with different noise.
   The rush-hour checker now also counts the departure, so it can't hide again.

**Files:** `safety_core.py`, `test/test_safety_core.py`
(`test_reversing_back_into_the_crossing_needs_a_new_grant`), `bus_core.py`
(`exit_tol`, plus explanation lines),
`test/test_bus_core.py` (`test_departure_ends_before_the_simulator_removes_a_noisy_bus`). **Try:** UI → "Bus docking".

## Arkar (M2-B) — coordinator and S8

**The review said:** waits only counted people who boarded; no paired
statistics; the crossing check missed "a person walks in while a vehicle is
already inside".

**What changed:**
1. S8 now reports **everyone it saw**: boarded, **still waiting at the end**
   (they waited *at least* that long) and **left the queue**. A Kaplan–Meier
   median uses the boarded + still-waiting people; the ones who walked away
   are reported separately with how long they stood there (100–132 s; they
   leave because the bus is full).
2. Predictive vs reactive is compared **per seed** (same seed, both modes) with
   a bootstrap interval over whole episodes.
3. The checker records **every moment a person and a vehicle share the
   crossing**, who came second, and it is a **violation** if our bus/robot keeps
   moving more than 0.5 s after the person stepped in.
4. The coordinator now dispatches on `/hub/demand_unserved` (parameter
   `dispatch_demand_topic`) — same behaviour as at the midterm, honest name.
5. Your files were restyled (no logic change) so the style check covers them.

**Files:** `s8_stats.py` (new), `s8_check.py`, `s8_summary.py`,
`hub_coordinator.py`, `coordinator_core.py`. **Try:** UI → "Rush hour" (one
episode, ~4 min) and "Bus dispatch from the forecast" (T04).

## Carla (M3) — parking and traffic

**The review said:** "skipped tests and expected failures are not passes" and
"style debt".

**What changed:**
1. Your files now pass the style checks like everyone else's (imports, long
   lines, docstring format). **No logic changed** — all 59 of your tests pass.
2. Your 9 "smoke tests" were being **skipped** whenever real ROS was installed.
   Now they run in a separate clean Python and must pass. Running them showed
   they had been **broken for weeks** (a missing fake class after the Ctrl+C
   fix) and nobody noticed because they were skipped. Fixed.
3. Result: **0 skipped, 0 expected failures** in the whole test suite.

**Files:** `parking_engine.py`, `parking_manager.py`, `traffic_engine.py`,
`hub_map.py`, `test/test_node_smoke.py`, `test/ros_stubs.py`.
**Try:** UI → "Parking reservations" and "Congestion detection".

## Aidil (M4) — routing and robot

**The review said:** "no live proof of cost-only replanning" and "the 10 escort
runs are identical copies".

**What changed:**
1. New live trial in S7: three cars crawl on road Cs-J. The road is **not
   blocked**, only slow. The planner's other route becomes much cheaper
   (**52 s instead of 220 s, 76 % cheaper; the rule needs 15 %**), so the robot
   switches to `Cn-B-U-E-J-D` and arrives. The road was labelled CONGESTED and
   never BLOCKED.
2. The robot's odometry now has **1 cm of noise** (it was perfect before, which
   is why the runs were identical). Escorts now differ a little each time
   (arrival error ~0.04–0.05 m instead of always 0.027 m; limit 0.20 m).
3. Bonus bug found on the way: if a checker **crashed** in the middle, it still
   printed PASSED for the checks done so far. Now a crash = FAILED.

**Files:** `s7_check.py` (`congestion_trial`), `scenarios/S7.yaml`,
`scenario_runner.py`. Your `route_core.py` / `robot_core.py` logic is unchanged
(explanation lines added). **Try:** UI → "Robot escort".

## Saidumar (M5) — safety, simulator, integration

1. **Reset in 1 s:** the reset service now answers within 0.8 s (the assignment's
   client timeout is 1 s; we had 2 s). Measured over 25 resets: mean about
   20 ms, worst about 0.2 s.
2. **Decision records:** the supervisor publishes `/hub/safety_decision` — one
   numbered message with the latch state AND the commands of that decision.
   The logger counts "moving while latched" from it with **no grace window**;
   lost messages show up as gaps. Fault repetition: about 1,800 records,
   0 lost, 0 violations.
3. **Stopping distance:** see Fazliddin's part.
4. **Release:** LICENSE, third-party notices, exact versions, SHA-256 manifest;
   one script (`tools/run_release.sh`) re-runs everything on one commit.
5. **A checker bug (good defence story):** one of 25 fault trials failed
   although the stop was on time. The freeze lasts 2 s of *simulator* time,
   which can end ~20 ms before 2 s of real time. The robot correctly moved
   again, but the checker was still in its "nothing may move" window and
   counted it. Now that window ends when the frozen data really comes back
   (`fault_window_end` in `s9_repeat_check.py`, 5 tests). Moving while the
   data is really missing still fails.
6. **Tests:** 419, 0 failures, 0 skipped, 0 expected failures.

---

## ✅ What you do now

1. Pull the branch `integration/current-team-work`.
2. Read your part above (and open the files it names).
3. Run your task from the UI once.
4. Write in the group chat: **"<name>: confirmed"** or tell Saidumar what's wrong.
5. For the presentation (Mon Nov 17, 18:00) be ready to explain, for YOUR part:
   **one data stream**, **one action/service from start to end**, and **one bug
   that was fixed** — that is exactly what the prof said he checks.
