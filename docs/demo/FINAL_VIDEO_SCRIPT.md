# Final video script — about 27 min

Assignment: 20–30 minutes, all members take part, project premise, walk through
the entire source code, results, demo videos. Team Shenanigans · team leader:
Egamberdiev Saidumar.

The midterm script (`VIDEO_SCRIPT.md`) stays as it is. This one is for the
final video. It keeps the same order, adds the Hub Control Center, and gives
every member a new beat: **what changed since the midterm**.

## How to use this

- Everyone records their own part: screen + voice, with a small webcam bubble
  and your name on screen, so it is obvious that all members took part.
- Read slowly, about 130 words per minute. Each part says how long it should be.
- Code walkthrough = record VS Code, font size 16 or bigger, file tree visible,
  open the file named here, highlight the lines you talk about.
- Live demo = start the Control Center (`docker/hub.sh ui`), open the page on
  the same computer, and record the browser. If port 8080 is busy on your
  computer, use `docker/hub.sh ui 8090`.
- Hand in your recording as `videos/raw_final/NN_part_name.mov` (numbers as in
  the table). Saidumar assembles the final file into
  `videos/final/final_video.mp4` and runs `tools/make_submission.sh`.
- Put "Simulation" in a corner of every simulator clip, and "N× speed" when
  you speed one up.
- Every number below is from `evidence/`. Say it exactly as written.
- No links anywhere, not on title cards either.

| # | part | who | time |
|---|---|---|---|
| 0 | premise, team, what is new in the final | Saidumar | 1:45 |
| 1 | Hub Control Center: live demo | Saidumar | 3:00 |
| 2 | architecture and source tree | Saidumar | 1:45 |
| 3 | M1-A passenger detection | Husan | 2:30 |
| 4 | M1-B demand prediction | Tuymurod | 2:45 |
| 5 | M2-B coordinator | Arkar | 2:30 |
| 6 | M2-A bus docking | Fazliddin | 2:30 |
| 7 | M3 parking and traffic | Carla | 2:30 |
| 8 | M4 routing and escort robot | Aidil | 2:30 |
| 9 | M5 simulator and safety | Saidumar | 2:30 |
| 10 | S8 rush hour and all results | Arkar | 2:00 |
| 11 | limits, lessons, outro | Saidumar + everyone | 1:00 |
| | **total** | | **≈ 27:15** |

Member parts 3–9 all have the same four beats:

1. **What my part does** (about 20 s)
2. **Code walkthrough** (about 60 s)
3. **What changed since the midterm, and the result** (about 50 s)
4. **Run it from the Control Center** (about 20 s, screen only, you may talk over it)

---

## 0 · Premise and what is new — Saidumar — 1:45

**Screen:** title card "Smart Mobility Hub · Final · Team Shenanigans" (5 s) →
the Control Center page, idle → the table from report section 2.

> This is the Smart Mobility Hub: a small transport hub in ROS 2 where a bus,
> an escort robot, cars and people share one site. A camera counts the
> passenger queue, a model forecasts demand, a coordinator sends the bus, the
> bus docks, cars reserve parking, traffic is monitored, routes are replanned,
> and one safety supervisor is the only node allowed to send motor commands.
> Everything runs in simulation, plus one real lab video for the camera part.
>
> At the midterm all nine scenarios already ran. For the final we did three
> things. One: we built the mandatory user interface, the Hub Control Center.
> Two: we took the professor's review and fixed every point in it, one by one.
> Three: we ran every scenario again on one single commit, so all the evidence
> belongs to the same code. Along the way the stricter checks found three
> hidden bugs, and we will show them honestly.

## 1 · Hub Control Center, live — Saidumar — 3:00

**Screen:** the browser, recorded live. Follow these steps exactly.

1. The idle page: point at the three areas (ask box and task list, live map,
   status and last result).
2. Type `test the e-stop` → Plan → show the plan card → Run. Let S9 run
   (about 15 s). Show the safety tile turn to LATCHED with its reason, then
   the verdict panel: 13 of 13 checks, open two of them.
3. Type `robot escort with 2 trials then parking` → show that it became a
   queue of two tasks → Run. While S7 runs, point at the robot, crossing C
   and the route line. Speed this section up (mark "4× speed").
4. Click the task "Rush hour" in the list, choose seed 3 and "reactive" in
   its options, run it, and show 30 s of the live map at 4× speed.
5. History tab: open an old run and its log.

> This is the Hub Control Center. It is one page, and it is itself a ROS 2
> node inside our package. I can choose a task in three ways: click it in the
> list, where every scenario is grouped by subsystem and owner; filter by a
> name; or just type what I want in plain words. A small keyword parser,
> running offline inside the node, turns the sentence into a plan. It shows me
> the plan first and nothing starts until I confirm.
>
> When a task runs, the map in the middle is live: it is drawn from the hub's
> own topics. Here is the safety state, who owns the crossing, the queue, and
> both forecasts next to each other. When the run ends, the page reads the
> same checker file that the command line writes and lists every check with
> its measured value. So a run from this page is the same run, with the same
> evidence, as a run from the terminal.
>
> Two design decisions. The page has no control authority: it publishes
> nothing and calls no service, so it cannot move a vehicle or clear a stop.
> And because it can start processes, it listens on this computer only, and
> every option is checked against a fixed catalogue before a command is built.

## 2 · Architecture and source tree — Saidumar — 1:45

**Screen:** `docs/figures/ros_graph.png` → VS Code file tree of
`src/smart_hub/smart_hub/` (expand `perception/`, `demand/`, `control_center/`)
→ `src/smart_hub_interfaces/` → `docker/hub.sh` → `evidence/`.

> Two packages. `smart_hub_interfaces` holds the 15 messages, 6 services and
> 3 actions. `smart_hub` holds every node. Each node follows one pattern: the
> logic is a pure Python core with unit tests, and a thin ROS wrapper connects
> it to topics. That is why we can test a bus docking or a parking race
> without starting ROS.
>
> The red path in this graph is the safety rule of the whole project:
> managers only publish candidate velocities, and only the safety supervisor
> publishes final commands. `docker/hub.sh` runs everything in one container.
> `evidence/` holds the result of every scenario, and every file in it carries
> the same commit.

## 3 · M1-A Passenger detection — Husan — 2:30

**Screen — code:** `perception/` → `detectors.py`, `tracker.py`,
`stabilizer.py`, then **`crossing_metrics.py`** (`crossing_report`).
**Screen — result:** `evidence/m1a/real_video/count_vs_truth_yolo.png`, then
the `crossing` block of `results_yolo_stabilized.json` next to the HOG one.

> **What:** I turn camera frames into the queue count and the "someone is in
> the crossing" flag.
>
> **Code:** A detector finds people, the foot point is mapped to floor metres
> with a homography, a tracker keeps identities, and confirmed tracks inside
> Q are counted. In simulation the count error is zero. On my real lab video,
> 141 frames that I labelled by hand, the mean error is 0.149 people.
>
> **What changed:** The review said one accuracy number for the crossing flag
> hides the dangerous mistake. So now we report two mistakes separately. A
> false clear means a person is in the crossing and we say clear: dangerous.
> A false busy means it is empty and we say occupied: safe, only slower.
> Result on the lab video: our main detector made zero false clears and four
> false busy, and three of those four are the deliberate two-second hold.
> The old HOG detector scored 90.8 percent, which sounds good, but it never
> saw the person at all. 90.8 percent is exactly what you get by always
> answering "clear", because only 13 of 141 frames have a person there.
> One honest limit: the video has one single entry into the crossing.

**Control Center:** run "Real lab video evaluation" and show the verdict.

## 4 · M1-B Demand prediction — Tuymurod — 2:45

**Screen — code:** `demand/features.py`, `demand/ridge.py`,
`demand_predictor.py` (the two estimators and the two publishers),
`scripts/train_demand_model.py`. **Screen — result:** the first table of
`results/S2_report.md`.

> **What:** I forecast the passenger queue 30 seconds ahead, so the bus can be
> sent before the queue is long.
>
> **Code:** Six features from the last 30 seconds of the queue, a ridge
> regression, and a persistence baseline. 50 simulated episodes, split by
> whole episode into 30, 10 and 10, so no test data leaks into training.
>
> **What changed:** This was our biggest review point. At the midterm I
> published a forecast of unserved demand, meaning how many people would wait
> if no bus came, under the topic name the assignment uses for the future
> queue. That was the wrong name for it. Now there are two honest topics.
> `/hub/demand` is the real future queue. On that target my model has error
> 1.625 against 1.668 for the baseline: not worse, but the confidence interval
> includes zero, so I do not claim it is better. `/hub/demand_unserved` is our
> extra target: 1.215 against 1.506, and that gain is real. In a 50-seed
> study, dispatching from the unserved forecast cut the mean wait by 10
> seconds, and dispatching from the queue forecast gave no gain. So the
> coordinator uses the unserved one, and says so in every dispatch event.

**Control Center:** run "Demand prediction (live)"; point at the two forecast
tiles.

## 5 · M2-B Hub coordinator — Arkar — 2:30

**Screen — code:** `coordinator_core.py` (crossing arbiter, dispatch
hysteresis), `hub_coordinator.py` (`dispatch_demand_topic`), `s8_stats.py`.
**Screen — result:** `evidence/m2b/t04/t04_check.json`.

> **What:** I am the referee. I decide who may use crossing C, and when the
> bus is dispatched.
>
> **Code:** The crossing has one owner at a time, with a lease that must be
> renewed, and it is only free after two clear seconds. Dispatch needs three
> forecasts in a row at six or more, so noise around the threshold does not
> send a bus. T04 passes six of six: no dispatch from noise, exactly one
> dispatch per mission, none from an invalid forecast, and a fallback from
> the observed queue.
>
> **What changed:** My files were brought to the common code style, so the
> two "expected failure" style tests are gone. The coordinator now names the
> forecast topic it uses. And the rush-hour statistics are stricter, which I
> show in part 10.

**Control Center:** run "Bus dispatch from the forecast".

## 6 · M2-A Bus docking — Fazliddin — 2:30

**Screen — code:** `bus_core.py` (state machine in the module text, `step()`,
the docking check, then **the `exit_tol` lines in the depart branch**),
`bus_manager.py`. **Screen — result:** `evidence/m2a/s3/s3_paths.png`.

> **What:** I drive the bus from entry A to bay B, open the door, board
> people, and later drive to the exit.
>
> **Code:** States go IDLE, APPROACH, WAIT, DOCK, BOARD, COMPLETE, with
> SAFE_STOP from any moving state. I ask for the crossing before the stop
> line, renew it every second, and release it when the whole bus is out.
> Ten of ten dockings, worst position error 4.3 centimetres, now with
> position noise, so the ten runs are really different.
>
> **What changed:** Two things. The safety clearance no longer looks a fixed
> half second ahead. It uses stopping distance: reaction distance plus
> braking distance. For the bus at full speed that is 26 centimetres.
> And one real bug. In our first final run, in 5 of 20 rush-hour episodes,
> the bus docked fine but its trip to the exit failed. The simulator removes
> a bus standing within 30 centimetres of the exit, and my bus only said
> "departed" at 5 centimetres. With a little position noise the simulator
> sometimes won that race. Now the bus reports at 25 centimetres, 20 tests
> replay the fault, and the rush-hour checker counts the departure.

**Control Center:** run "Bus docking" with 3 trials.

## 7 · M3 Parking and traffic — Carla — 2:30

**Screen — code:** `parking_engine.py` (physical and administrative state,
the lock), `parking_manager.py`, `traffic_engine.py` (label rules),
`test_node_smoke.py`. **Screen — result:** `evidence/m3/s5/s5_labels.png`.

> **What:** I give out parking slots and I label every road edge as normal,
> congested, blocked or unknown.
>
> **Code:** Parking keeps what the sensor sees apart from who reserved it,
> and every change runs under one lock, so two cars can never get the same
> slot: 20 simultaneous pairs, zero double allocations. Traffic turns an edge
> UNKNOWN within one second when the data is stale, and never reports stale
> data as free.
>
> **What changed:** My files now pass the common style checks with no
> exceptions. And nine smoke tests that were marked "skipped" at the midterm
> now run on every build. When we turned them on, we found they had been
> broken for weeks without anyone seeing it, because a skipped test cannot
> fail. They are fixed. The whole suite is now 419 tests with zero skipped.

**Control Center:** type "parking then congestion".

## 8 · M4 Routing and escort robot — Aidil — 2:30

**Screen — code:** `route_core.py` (edge cost, removed edges, Dijkstra),
`robot_core.py` (states, the replan rule with the 15 percent gain),
`s7_check.py` (`congestion_trial`). **Screen — result:**
`evidence/m4/s7/s7_paths.png`.

> **What:** I plan routes and I drive the escort robot from pickup to
> destination.
>
> **Code:** Edge cost is travel time, raised by measured density. Forbidden,
> blocked or stale edges are removed, not just made expensive. The robot
> re-checks its route every tick.
>
> **What changed:** Two review points. First, replanning for a cheaper route
> was only unit-tested. Now there is a live trial: three slow cars make edge
> Cs-J congested, never blocked. The cost of my route becomes 220 seconds
> against 52 for the detour, a 76 percent gain where the rule needs 15, and
> the robot switches after 1.3 seconds with the reason CHEAPER. Second, my
> ten escort runs were identical copies. The reason was that the robot's
> odometry was the exact simulator truth. Now positions carry seeded noise:
> ten of ten still arrive, with errors up to 5.1 centimetres against a
> 20 centimetre limit.

**Control Center:** type "robot escort with 2 trials".

## 9 · M5 Simulator and safety — Saidumar — 2:30

**Screen — code:** `safety_core.py` (`stopping_distance`, `stop_horizon`),
`safety_supervisor.py` (`_publish_decision`), `msg/SafetyDecision.msg`,
`scenario_runner.py` (reset handshake), `tools/run_release.sh`.
**Screen — result:** `evidence/m5/s9_repeat/s9_repeat.json`.

> **What:** The simulator, the safety supervisor, and the infrastructure.
>
> **Code:** Every 50 milliseconds the supervisor checks the stop latch, data
> freshness, clearance, the door and the crossing, then clamps the speed.
> An e-stop gives a zero command in under 2 milliseconds, against a
> 200 millisecond limit.
>
> **What changed:** Three review points. The reset service could take 2
> seconds while its callers waited 1; it now answers within 0.8, and 25 of 25
> resets came back in time, the slowest in 17 milliseconds. The logger used to ignore a 0.1 second window
> around each stop, which could hide a real fault; now the supervisor
> publishes numbered decision records, latch and commands in one message:
> 1,813 records, zero gaps, zero with motion while latched. And the evidence
> came from mixed revisions; now one script re-runs everything on one commit
> and writes the versions and a hash of every file.
> One more lesson: a checker of ours crashed in the middle and still said
> PASSED. Every checker now turns a crash into a failed check.

**Control Center:** run "Fault repetition" and show 25 of 25.

## 10 · S8 rush hour and all results — Arkar — 2:00

**Screen:** `evidence/s8/dashboard_seed1_predictive/dashboard.mp4` at 4× →
`evidence/s8/s8_compare.png` → the test matrix in the report.

> This is everything together: camera, forecast, coordinator, bus, parking,
> traffic, planner, robot and safety, for 240 seconds, 10 seeds, in two
> dispatch modes. 20 episodes.
>
> Result: zero collisions, zero crossing conflicts, zero double allocations,
> zero duplicate dispatches, and every mission complete, now including the
> bus leaving through the exit.
>
> The statistics are stricter than at the midterm. We now report all 70
> passengers per mode, not only the 40 who boarded: 30 walked away because
> the bus was full, after standing in the queue for 100 to 132 seconds. We
> compare the two modes seed by seed with a bootstrap interval: predictive
> dispatch sends the bus 1.9 seconds earlier and lowers the median wait by
> 1.4 seconds, in all ten seeds. The interval does not include zero, but the
> effect is small and holds for this one rush-hour shape only. And the crossing check now looks at every pair of bodies:
> zero seconds of a vehicle and a pedestrian together in C.

## 11 · Limits, lessons, outro — Saidumar + everyone — 1:00

**Screen:** report section 15 → every member's webcam in a grid.

> Our limits: everything except one camera video is simulation. Vehicles
> have no inertia. The real video has one crossing entry. On the
> assignment's own forecast target we do not beat the baseline.
> Our lesson: every time we made a check stricter, it found something that
> had been passing quietly. So we would rather show a strict check and an
> honest number than a green tick.
> (everyone, one after another: name and role) Thank you.

## Recording checklist

- [ ] webcam bubble and name visible in every part
- [ ] "Simulation" label on simulator clips, speed label when sped up
- [ ] numbers read exactly as written here
- [ ] no links on screen (close browser tabs, hide bookmarks bar; the address
      bar of the Control Center shows only the local page)
- [ ] total length between 20 and 30 minutes
