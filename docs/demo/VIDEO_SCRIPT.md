# Team video script — ~26 min (Prof. MP's email: 20–30 min, all members,
# project premise + walk through the source code + results + demo videos)

Team Shenanigans · team leader: Egamberdiev Saidumar

## How to use this

- **Everyone records their own part** (screen + voice). A small webcam
  bubble in a corner is a good idea: the email asks for the *participation of
  all members*, and a face + name card makes it obvious who is talking.
- Read slowly, about **130 words per minute**. Each block says how long it
  should take and roughly how many words it has.
- **Code walkthrough** = record VS Code: font size 16 or bigger, the file
  tree visible on the left, open the file named in the script, scroll to the
  function named, *highlight* the lines you talk about.
- **Clips** are in `videos/demo_clips/` and `CLIP_TIMES.md` there says
  where each moment is. Put **"Simulation"** in a corner of every clip and
  **"N× speed"** when you speed one up.
- Every number below is from `evidence/`. Say it exactly.
- Put the final edited file in `videos/final/` and run
  `tools/make_submission.sh` — it names and checks all upload files.
- **No links anywhere** — not in the video either (no URLs on title cards).

| # | part | who | time |
|---|---|---|---|
| 0 | intro: premise, team, map | Saidumar | 2:00 |
| 1 | architecture + how to run it | Saidumar | 2:00 |
| 2 | M1-A passenger detection | Husan | 2:45 |
| 3 | M1-B demand prediction | Tuymurod | 2:45 |
| 4 | M2-B coordinator | Arkar | 2:45 |
| 5 | M2-A bus docking | Fazliddin | 2:45 |
| 6 | M3 parking + traffic | Carla | 2:45 |
| 7 | M4 routing + escort robot | Aidil | 2:45 |
| 8 | M5 simulator + safety | Saidumar | 2:45 |
| 9 | S8: everything together + all results | Arkar | 2:00 |
| 10 | limits, lessons, outro | Saidumar + everyone | 1:00 |
| | **total** | | **≈ 26:15** |

Every member part (2–8) has the same four beats, so the video feels like one
piece:

1. **What my part does** (≈ 20 s)
2. **Code walkthrough** (≈ 80 s)
3. **Demo clip + results** (≈ 40 s)
4. **One bug I fixed** (≈ 25 s)

---

## 0 · Intro — Saidumar — 2:00 (~260 words)

**Screen:** title card "Smart Mobility Hub · Team Shenanigans · Smart Mobility
Engineering Lab, Inha University" (5 s) → first frame of `S8_rush_hour.mp4`
held still (the empty map) → zoom/point at each area as you name it.

> Hi, we're Team Shenanigans — Husan, Tuymurod, Fazliddin, Arkar, Carla,
> Aidil and me, Saidumar. This is our midterm project: a smart mobility hub
> built in ROS 2.
>
> The idea: a small transport hub, like a bus stop with a parking lot and a
> pedestrian crossing, run by software that has to be *safe* first and smart
> second. A camera counts people waiting for the bus. A model predicts how
> many will be waiting in thirty seconds. A coordinator decides when to send
> the bus and who may use the crossing. The bus docks and opens its door. Cars
> reserve parking. A robot escorts a passenger from R to D. And a safety
> supervisor can stop everything.
>
> Here's the map: twelve by eight metres. Entry A on the left, bus bay B,
> the orange strip is crossing C — the one place everybody shares — queue Q
> at the top, three parking slots, the robot's start R and its destination D,
> and obstacle O, which we use to block the lower road.
>
> The handbook has nine scenarios, S1 to S9, and eighteen tests, T01 to T18.
> We implemented all of them, in simulation, and every result you'll see is
> saved as an evidence file. Each of us will show their own code, a demo,
> and their results. Let's start with how it all fits together.

## 1 · Architecture and how to run it — Saidumar — 2:00 (~260 words)

**Screen:**
1. the ROS graph picture `videos/demo_clips/pictures/ros_graph.png` (also
   `docs/figures/ros_graph.png`; drawn from README §3)
2. VS Code: *File → Open Folder → smart-mobility-lab-project*, then in the
   Explorer on the left open `src → smart_hub_interfaces → msg` (14 files),
   `srv` (6), `action` (3), then `src → smart_hub → smart_hub` (all nodes)
3. terminal in the project folder: `docker/hub.sh` with nothing after it —
   prints the command list
4. terminal: `docker/hub.sh test` — takes ~5 min: start the recording near
   the end, or speed it up, so the last line shows "Summary: 352 tests,
   0 errors, 0 failures".

> We have two ROS packages. `smart_hub_interfaces` defines every message,
> service and action exactly as the handbook specifies — fourteen messages,
> six services, three actions. `smart_hub` holds all our nodes.
>
> Each node is one person's job and talks only through those interfaces:
> topics for streams like the queue count or vehicle positions, services for
> quick questions like "reserve a parking slot" or "compute a route", and
> actions for long jobs you can cancel — "dock the bus", "escort a
> passenger".
>
> One rule shapes the whole design: the bus and the robot only *suggest* a
> velocity. Only the safety supervisor publishes the real motor command, and
> the actuator bridge underneath has its own independent watchdog. Two safety
> nets.
>
> And one coding pattern we all used: the logic lives in a plain Python
> "core" file with no ROS inside — you'll see `bus_core`, `robot_core`,
> `parking_engine`, `safety_core` — and the ROS node is a thin wrapper. That
> makes the logic testable without starting ROS.
>
> Everything runs in Docker with one command. `docker/hub.sh test` builds
> and runs all 352 tests, including style checks. `docker/hub.sh s1` up to
> `s9` run each scenario live and write the evidence files. The whole rush
> hour, S8, is `docker/hub.sh s8`.

## 2 · M1-A Passenger detection (S1, T01–T02) — Husan — 2:45 (~360 words)

**Screen — code:**
1. `src/smart_hub/smart_hub/passenger_detector.py` → `on_image()` (≈ line 114)
2. `perception/detectors.py` → `BlobDetector.detect`, `HogDetector.detect`, `foot()`
3. `perception/homography.py` → `fit_homography()`, `image_to_map()`
4. `perception/tracker.py` → `Tracker.update()`
5. `perception/queue_logic.py` → `on_frame()`, `validity()`, `state()`

**Screen — demo:** `pictures/camera_during_t13_five_waiting.png` →
`pictures/camera_crossing_t22_person_in_C.png` → `S1_detection.mp4` (2×) →
`pictures/s1_count_vs_truth.png`. *If the real lab video is processed, show
it here with the boxes, and use its numbers.*

> **What:** My part is the camera. It answers two questions, five times a
> second: how many people are waiting in queue Q, and is anyone standing on
> crossing C?
>
> **Code:** Every image arrives in `on_image`. First a detector finds the
> people — a simple blob detector for our simulated camera, and OpenCV's HOG
> people detector for real video. From each box I take the foot point, the
> bottom middle, because that's where the person touches the floor. Then the
> homography — fitted once from four floor points with known map positions —
> turns that pixel into metres on the hub map. The tracker gives every person
> one ID: a new person counts only after 0.4 seconds, and someone who
> disappears is kept for a moment, so a short occlusion doesn't create a new
> person. Finally `queue_logic` counts the feet inside Q, checks C, and keeps
> the crossing "occupied" for two seconds after it empties. If the camera
> freezes or repeats an old frame, the output becomes invalid — "I don't
> know" — never "zero people".
>
> **Results:** In S1 the count followed the script exactly: zero, three,
> five, four. Error zero over 127 samples. Six tracks for six people, so no
> double counting. The freeze was detected in 1.03 seconds — the limit is
> 1.2. *(Real video: say the MAE / recall here.)*
>
> **Bug:** The count-versus-truth plot showed one: right after a freeze the
> count dropped to zero while still saying "valid". Now counts are judged at
> the last frame actually processed, and a regression test guards it.

## 3 · M1-B Demand prediction (S2, T03) — Tuymurod — 2:45 (~360 words)

**Screen — code:**
1. `scripts/train_demand_model.py` (top: episode generation, the split)
2. `src/smart_hub/results/split_manifest.json` (the 30/10/10 seeds)
3. `smart_hub/demand/features.py` → `extract_features()`, `DemandHistory`
4. `smart_hub/demand/models.py` → `PersistenceModel`, `LinearDemandModel.predict`
5. `smart_hub/demand/estimator.py` → `DemandEstimator.tick()`
6. `smart_hub/demand_predictor.py` → `_tick()`

**Screen — demo:** `S8_rush_hour.mp4` around 55–65 s, zoom on the text under
the queue ("prediction unavailable" → "demand +30s: 7.0") → report table
T03 (screenshot from the PDF).

> **What:** I predict how many people will be waiting thirty seconds from
> now, so the bus can be sent *before* the crowd peaks.
>
> **Code:** The training script generates fifty six-minute demand episodes
> with different arrival patterns. I split them by whole episodes — thirty
> for training, ten for validation, ten for testing — *before* making any
> windows, so no test data can leak into training; the manifest records the
> seeds. The features are only past observations: the current queue, the
> last ten seconds, the recent change, and proxies for arrivals and
> departures. The model is a ridge regression; the baseline is persistence —
> "nothing changes". The estimator waits for sixty seconds of history before
> its first forecast, and if the queue input goes stale or invalid it
> publishes valid = false with −1, never a fake number.
>
> **Results:** On 2,500 held-out forecasts, ridge has an error of 1.22
> people versus 1.51 for persistence, and inference takes 0.013 milliseconds.
> In a closed-loop study over fifty seeds, predictive dispatch lowered the
> mean wait from 73.7 to 63.7 seconds. Live in the hub, the forecast becomes
> valid only after warm-up — you can see "prediction unavailable" switch to
> a number here.
>
> **Bug:** The node crashed on its very first valid forecast: one logging
> call switched between warning and info severity at the same call site. A
> node-level test now drives it through warm-up, valid, stale and recovery.

## 4 · M2-B Hub coordinator (T04, crossing) — Arkar — 2:45 (~360 words)

**Screen — code:**
1. `smart_hub/coordinator_core.py` → `CrossingArbiter.request`, `_acquire`
   (the list of deny reasons), `_renew`, `_release`, `tick` (lease → HOLD)
2. same file → `DispatchPolicy.step`
3. `smart_hub/hub_coordinator.py` → `_dispatch_tick`, `_send_dock`,
   `_on_dock_accept` (late answers ignored), `_on_hub_event` (escort trigger)

**Screen — demo:** `S3_bus.mp4` 56–106 s (pedestrian in C: bus waits) →
title card "T04 6/6" over `evidence/m2b/t04/t04_timeline.csv` (screenshot).

> **What:** I'm the hub's traffic officer. I decide who may use crossing C,
> and when to send the bus.
>
> **Code:** The crossing is a token. `_acquire` denies when a pedestrian is
> there, when the state is unknown or the safety stop is latched, when someone
> else owns it, when C hasn't been clear for two seconds, or when it's not
> your turn — first in, first out. The owner renews every second; if the lease
> expires while they're still inside, I don't free it — it goes to HOLD until
> C is seen empty. Release only works once your footprint has left C.
>
> Dispatch: three forecasts of six or more in a row start escalation; five
> below four end it. While escalated I send one DockBus goal — only if no
> mission is outstanding, the bus says IDLE, and thirty seconds have passed.
> No forecast? Fall back to the real queue count, marked FALLBACK. Every
> mission has its own correlation ID, so a late reply can never start a
> second one.
>
> **Results:** T04 passed six of six live: forecasts wobbling around six —
> no dispatch; a rising forecast — exactly one bus, on the third sample;
> invalid forecast or no forecast — no dispatch; queue of seven without a
> forecast — one bus, FALLBACK. And twenty-eight unit tests.
>
> **Bug:** Compared with the original policy I added one check: the old
> version could send a bus while demand was already falling. And in
> integration we found the forecast comes at 1 Hz while my freshness limit
> was 1 second — it would expire exactly when the next one was due. Now the
> forecast gets 2.5 seconds.

## 5 · M2-A Bus docking (S3, T05–T06) — Fazliddin — 2:45 (~360 words)

**Screen — code:**
1. `smart_hub/bus_core.py` → module docstring (the state machine), `Path`
   (`c_in`/`c_out` = where the footprint meets C), `step()` (the speed limits
   from stop points), `_crossing()`, the docking check, `_board()`
2. `smart_hub/bus_manager.py` → `on_goal` (rejection reasons), `execute`,
   `on_cancel` (zero at once), `_start_route` (planner: 1 s, 2 retries)

**Screen — demo:** `S3_bus.mp4` 0–41 s at 2× (normal docking + boarding) →
41–55 s (cancel) → `pictures/s3_paths.png` → title card "T05 10/10 ·
worst 3.4 cm · T06 cancel 0.001 s".

> **What:** I drive the bus: from entry A to bus bay B, open the door,
> let people in, close it, report back. It's a ROS action, so the
> coordinator gets progress updates and can cancel.
>
> **Code:** The state machine goes IDLE, APPROACH, WAIT, DOCK, BOARD,
> COMPLETE — or SAFE_STOP from any moving state. The route comes from Aidil's
> planner. I compute exactly where the bus footprint would touch crossing C,
> ask for permission 2.5 metres before the stop line, renew every second,
> and release only once the whole bus is out. If the bay is blocked I wait
> *before* C without holding it, so I don't block anyone. "Docked" means
> within twenty centimetres, straight, and nearly still for a full second —
> only then is the door requested, for eight seconds, and the bus only moves
> again once the door is *measured* closed. Cancel publishes zero
> immediately, even before the next tick.
>
> **Results:** Ten of ten dockings, worst position error 3.4 centimetres.
> Arrival-time estimate off by at most 0.94 seconds at five metres. Cancel:
> zero command one millisecond after acceptance. A pedestrian in C, a car in
> the bay, a frozen position — the bus waited or stopped and still docked.
>
> **Bug:** The referee answers BAD_TOKEN when a permission is already gone;
> my first version waited for a different word and would have retried
> RELEASE forever. Found by reading Arkar's code before the live run.

## 6 · M3 Parking and traffic (S4–S5, T07–T10) — Carla — 2:45 (~360 words)

**Screen — code:**
1. `smart_hub/parking_engine.py` → module docstring (the design rules),
   `observe()` (debounce + freshness), `_eff()` (effective state),
   `reserve()` (the lock, the request-id cache, nearest free slot),
   `release()`, `_expire_locked()`
2. `smart_hub/parking_manager.py` → the services in one callback group
3. `smart_hub/traffic_engine.py` → `ingest()` (stamp + corridor gates),
   `evaluate()` (UNKNOWN / BLOCKED / CONGESTED / NORMAL)

**Screen — demo:** `S8_rush_hour.mp4` around 65 s (P2, P3 turn RESERVED) →
`pictures/s5_labels.png` → title card "T07 20 pairs · 0 double ·
p95 39 ms".

> **What:** Parking reservations, and the traffic state of every road.
>
> **Code:** For parking I keep two things apart: what the sensor sees, and
> who has a reservation. The state everyone sees is computed from both: a
> slot that's stale or still being debounced is UNKNOWN — never FREE. Every
> reservation runs under one lock, so two cars can never get the same slot.
> The request ID works as an idempotency key: send the same request twice,
> get the same answer — not a second slot. Release needs the right token
> *and* the right car. And when a reservation expires, only the paperwork is
> dropped; an occupied slot stays occupied.
>
> For traffic, every vehicle snapshot is checked: the timestamp must
> advance, the car must really be inside the road's corridor. Each road then
> gets a label twice a second: UNKNOWN when data is stale, BLOCKED when an
> obstacle touches it, CONGESTED when it's seventy percent full or slow for
> three updates in a row, otherwise NORMAL.
>
> **Results:** S4: two cars asked for P2 at once — P2 and P3. Twenty
> simultaneous pairs, zero double allocations, 95th-percentile response 39
> milliseconds. S5: congestion detected in 0.3 seconds, every road UNKNOWN
> 0.995 seconds after the feed froze — limit 1.2 — and BLOCKED 0.33 seconds
> after the obstacle. Fifty-nine unit tests.
>
> **Bug:** In integration, our smoke test could swap the real ROS library
> for test stubs and silently test nothing; it now only uses stubs when ROS
> isn't installed.

## 7 · M4 Routing and the escort robot (S6–S7, T11–T14, T18) — Aidil — 2:45 (~360 words)

**Screen — code:**
1. `smart_hub/route_core.py` → module docstring (the cost formula),
   `EdgeView.edge_cost()` (the removal reasons), `plan()` (Dijkstra)
2. `smart_hub/robot_core.py` → module docstring (states), `confirm_ready()`,
   `_planner()` (timeout, retries, late answers), `_validate_route()`
3. `test/test_route_core.py` → `oracle()` and `test_t11_...`

**Screen — demo:** `pictures/s6_maps.png` (panels 1, 3, 4) →
`S7_robot.mp4` 85–163 s at 3× (obstacle → detour) → title card
"T13 10/10 · ≤ 2.7 cm · T18 5/5".

> **What:** The hub's map app, and the escort robot.
>
> **Code:** Every road gets a price in seconds: length over speed, times one
> plus twice the density, plus a wait if the crossing is busy. Roads that
> are blocked, forbidden for this vehicle, or have stale data are *removed*,
> not just made expensive. Dijkstra picks the cheapest route; if nothing
> connects, the answer is NO_ROUTE. To prove it's right, a test compares
> every answer with a separate program that tries every possible path —
> twenty random road situations, same cost every time.
>
> The robot waits at the pickup until the passenger presses "ready" — and the
> button is refused at any other time. Routes only come from the planner:
> one-second timeout, two retries, then a clear failure; late answers are
> thrown away. While walking, it re-checks the rest of its route every tick:
> if a road ahead is blocked, it stops at the next corner and replans.
>
> **Results:** On empty roads A to E costs 25 seconds. With slow cars the
> lower road costs 115, so the planner takes the longer 35-second road —
> cost, not distance. Both roads blocked: NO_ROUTE, the robot never moves.
> Ten of ten escorts, 2.7 centimetres off. With the obstacle: a detour, 77.9
> seconds instead of 56.8. Planner killed: robot and bus fail in about three
> seconds and never move.
>
> **Bug:** The bus waiting at A counted as a traffic jam on its *own* road,
> so the planner sent it the long way. Our first fix used the road label and
> didn't work; the second uses the position — the asker is never traffic
> for itself.

## 8 · M5 Simulator, safety and infrastructure (S9, T16–T18) — Saidumar — 2:45 (~360 words)

**Screen — code:**
1. `smart_hub/sim_core.py` → `HubSim.step()` (fixed 0.05 s step, seeded)
2. `smart_hub/safety_core.py` → `tick()`, `_agent_holds()`,
   `_crossing_reason()` (latch vs hold)
3. `smart_hub/bridge_core.py` → `tick()` (own watchdog + door interlock)
4. `smart_hub/scenario_runner.py` → `on_reset` / `_drive_handshake`
5. terminal: `videos/demo_clips/terminal_t18.mov` (green PASS lines)

**Screen — demo:** `S9_estop.mp4` (LATCHED) → title cards "T16 0.4 ms
(limit 200 ms)" and "S9 repeat 25/25".

> **What:** I built the world everyone else drives in, the safety layer,
> and the tools: interfaces, simulator, safety supervisor, actuator bridge,
> metrics, dashboard, Docker, and the checkers.
>
> **Code:** The simulator advances in fixed 0.05-second steps with a seed,
> so the same scenario and seed give the same run. It publishes noisy
> observations for the nodes and private ground truth only for evaluation,
> and it can freeze any sensor or insert an obstacle on schedule.
>
> The safety core has two kinds of stop. A *hold* — stale data, open door,
> not enough clearance, no crossing permission — releases by itself when the
> reason goes away. A *latch* — e-stop, collision — never releases by itself:
> clearing it is refused while the button is pressed, while a command is
> still nonzero, or while odometry is stale. The bridge underneath has its
> own half-second watchdog and refuses to open the bus door unless the bus
> has been still. A scenario reset waits until all eight nodes acknowledge.
>
> **Results:** E-stop to zero command: 0.4 milliseconds, limit two hundred.
> Five faults at five mission phases: twenty-five of twenty-five. Kill a
> controller: zero after half a second; kill the supervisor itself: the
> bridge still stops. And T18 — no planner — passes, as you see here.
>
> **Bug:** My reset service first ran on a multi-threaded executor, which
> slowed the simulator from 20 to 6 steps per second and broke S1 and S9.
> I rewrote it as an asynchronous service in its own callback group.

## 9 · S8 — everything together, and all results — Arkar — 2:00 (~260 words)

**Screen:** `S8_rush_hour.mp4` at 4× (whole run ≈ 60 s; point at: forecast,
bus sent, P2/P3, pedestrian, obstacle, detour, bus leaving at E) →
`pictures/s8_compare.png` → report section 11, the T01–T18 table
(screenshot).

> S8 is the real exam: every node at once, on the handbook's timeline. The
> queue grows to seven and the forecast sends the bus — once. Two cars ask
> for the same parking slot at the same moment. A passenger asks for an
> escort and presses ready. Someone steps onto the crossing — the bus and the
> robot both pause, even though they already had permission. An obstacle
> blocks the lower road; the robot reroutes. The bus boards four people and
> leaves through exit E.
>
> We ran ten different rush hours, in predictive and in reactive dispatch
> mode: twenty runs. Zero collisions, zero crossing conflicts, zero double
> parking, zero duplicate buses — and every mission completed. That's T15.
> The forecast sent the bus 1.4 seconds earlier on average; that's small,
> because our script's queue jumps up very fast, and we report it as it is.
>
> The hardest bug of the whole project showed up here: the first rush hour
> was perfectly safe — but nothing moved. Our S8 launch file had an argument
> called "mode", and so did the main hub launch, where "mode" means
> simulation or hardware. ROS shares launch arguments, so the actuator bridge
> received "mode: predictive" and refused to start. Safe, but useless. We
> renamed it, and everything ran.
>
> Here are all eighteen tests, T01 to T18 — all passed in simulation, with
> the evidence files in our submission.

## 10 · Limits, lessons, outro — Saidumar + everyone — 1:00 (~120 words)

**Screen:** limitations list (title card) → final card: team name + the
seven names with their roles, each person says one line over it (optional:
everyone's webcam tile).

> **Saidumar:** Honest limits: everything here is simulation. The vehicles
> have no inertia, the people are drawn figures, and several of our tests are
> very repeatable because the simulated sensors have little noise.
> *(If done: our camera was also tested on a real lab video, with error X.)*
> What we learned: write the logic so you can test it without ROS, check every
> interface against the real files, and re-run everything after every fix —
> half our best bugs were found by re-running.
>
> **Each member, one line:** "I'm ___, I built ___." (≈ 5 s each)
>
> **Saidumar:** Thanks for watching — Team Shenanigans.

---

## Recording checklist

- [ ] each member recorded their part (screen + voice, webcam optional)
- [ ] every part names the files shown, and the code is readable
- [ ] clips and pictures inserted where the script says
- [ ] "Simulation" and "N× speed" labels on the clips
- [ ] total between 20:00 and 30:00 (target ≈ 26)
- [ ] no URLs anywhere on screen
- [ ] exported as MP4, 1080p (it must stay under 1 GB)
- [ ] saved into `videos/final/`, then `tools/make_submission.sh`
