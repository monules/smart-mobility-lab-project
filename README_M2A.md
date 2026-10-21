# M2-A — the bus driver 🚌 (Fazliddin, scenario S3)

Branch: `feature/m2a-bus-manager` (also merged into
`integration/current-team-work`). Written for Fazliddin by Saidumar (M5)
with Claude — so now it's yours: read it, run it, get it, own it before the
defence. 🫡

---

## tl;dr

A bus starts at the entrance A, drives to the bus stop B, opens its door so
people can get in, closes it, done. Sounds easy — the hard part is being
**safe**: never drive into the road crossing without permission, never move
with the door open, stop at once when someone says "cancel", and stop when
you can't see where you are.

Your program is `bus_manager`. Somebody (the coordinator, Arkar's node)
says "bus, go dock at B" → your program drives the bus there and reports
back. That "go do this long job and tell me how it's going" thing is a ROS
**action** called `/bus/dock` (type `DockBus`).

## the bus's moods (state machine) 🎭

```
IDLE ──goal──▶ APPROACH ──▶ (WAIT) ──▶ DOCK ──▶ BOARD ──▶ COMPLETE
                 └──────── anything goes wrong ──────▶ SAFE_STOP
```

| mood | what the bus does |
|---|---|
| `IDLE` | parked, nothing to do, sends zero speed |
| `APPROACH` | drives the route A → N → Cn → B, slows down for the corner at N |
| `WAIT` | stopped at the line in front of crossing C: no permission yet (someone is on the crossing) **or** the bus stop B is blocked by another car |
| `DOCK` | last metre: creeping into the stop |
| `BOARD` | door open 8 s, people get in (max 4, the rest wait for the next bus) |
| `COMPLETE` | door closed again, job done ✅ |
| `SAFE_STOP` | cancelled, timeout, safety stop, lost its position, door broke → zero speed |

## the rules (the stuff the teacher checks) 📏

| rule | in normal words | number |
|---|---|---|
| ask before the crossing | call `/crossing/request ACQUIRE` before the bus even touches C; no "yes" → stop at the line 0.25 m before C | ask from 2.5 m before the line, retry every 0.5 s |
| keep your permission alive | the permission runs out after 3 s, so say `RENEW` | every 1 s |
| give it back | `RELEASE` only once the whole bus is out of C | 0.05 m margin |
| blocked bus stop? don't hog the crossing | if another car is in the bay, wait **before** the crossing without asking for it, so others can still use it | car within 0.8 m of B |
| "docked" means really docked | close enough + straight enough + really stopped, and it has to **stay** like that | ≤ 0.20 m, ≤ 0.17 rad, < 0.02 m/s, for 1 s |
| door only when docked | ask to open only after "docked"; the bridge also refuses unless the bus was still for 0.5 s | — |
| no driving with the door open | zero speed the whole time; drive again only once the door is **measured** closed | — |
| cancel = stop now | the moment a cancel is accepted → zero speed | measured 0.05 s (limit 0.5 s) |
| can't see yourself = stop | bus position older than 0.5 s → zero speed (`SAFE_STOP`); continue when it's back, give up after 5 s | 0.5 s |
| ETA | time left = distance left on the road ÷ smoothed real speed + corner turn time + 1.5 s for the slow final metre; "I don't know" (`-1`, `eta_valid=false`) while waiting | — |

You only ever **suggest** a speed (`/bus/cmd_vel_candidate`). Saidumar's
`safety_supervisor` has the final word (`/bus/cmd_vel`) — it also stops the
bus by itself if it enters C without the crossing, if the door is open, if
data is stale. Two safety nets > one. 🥅🥅

## the files

| file | what it does |
|---|---|
| `src/smart_hub/smart_hub/bus_core.py` | **the brain**: route following, moods, crossing, docking, door, ETA. Plain Python, no ROS → easy to test |
| `src/smart_hub/smart_hub/bus_manager.py` | the ROS wrapper: the `/bus/dock` action server, topics, calls the crossing and route services |
| `src/smart_hub/test/test_bus_core.py` | 10 tests: the brain drives the simulator with no ROS (normal docking, crossing, pedestrian, blocked bay, cancel, lost position, ETA, timeout) |
| `src/smart_hub/smart_hub/s3_check.py`, `launch/s3.launch.py` | the full S3 exam on the real running hub (15 trials) |
| `src/smart_hub/scenarios/S3.yaml` | the story: bus at A, 5 people waiting in Q |

Saidumar's simulator got one new trick for you: when the door is open and
the bus stands still at B, the waiting people get in one by one (first after
1 s, then every 1.5 s, max 4) and each one is logged as `PASSENGER_BOARDED`.

## topics & services (who talks to whom) 🔌

| | name | type | with whom |
|---|---|---|---|
| serves (action) | `/bus/dock` | `DockBus` | Arkar's `hub_coordinator` (and `s3_check` in the test) |
| reads | `/hub/vehicles` | `VehicleStateArray` | simulator: where the bus is, other cars |
| reads | `/bus/door_open` | `Bool` | actuator bridge: is the door **really** open |
| reads | `/hub/safety` | `SafetyState` | safety supervisor: emergency stop latched? |
| writes | `/bus/cmd_vel_candidate` | `TwistStamped`, 20 Hz | safety supervisor |
| writes | `/bus/door_request` | `Bool` | actuator bridge |
| writes | `/hub/bus_arrival` | `BusArrival`, 2 Hz | coordinator (it only sends a bus that says `IDLE`), dashboard |
| writes | `/bus/route` | `Path` | dashboard draws the route |
| calls | `/crossing/request` | `RequestCrossing` | Arkar's crossing referee |
| calls | `/route/compute` | `ComputeRoute` | Aidil's route planner (M4): wait 1 s, retry max 2 times, then fail with `PLANNER_UNAVAILABLE` (T18); `NO_ROUTE` fails at once |

## run it 🚀

```bash
docker/hub.sh s3        # full exam, ~12 min (10 normal dockings + 5 special trials)
docker/hub.sh s3 1      # quick version: 1 normal docking + the 5 special ones (~5 min)
```
→ prints `S3 verdict: PASSED` and saves the proof in `output/`:
`s3_check.json` (all checks + every trial), `s3_trials.csv` (one row per
trial), `s3_trace.csv` (bus position + speed over time), `s3_events.csv`
(crossing grant/release, door, docking, boarding, action feedback),
`s3_paths.png` (the picture).

## what's proven ✅ (simulation)

Measured with `docker/hub.sh s3` (all 12 checks passed; proof in
[`evidence/m2a/s3/`](evidence/m2a/s3)):

- **10/10 normal dockings** (T05): position error 0.013–0.034 m (limit
  0.20), yaw error ≤ 0.006 rad (limit 0.17), ~40 s per trip incl. 8 s boarding
- **door:** open 8.1 s per stop, **0** moments where the bus moved with the
  door open
- **crossing:** permission granted before the bus touched C and given back
  after it left — 10/10; 0 collisions
- **boarding:** 4 people get in every time, 1 stays waiting
- **ETA at 5 m left:** off by 0.24 s on average, 0.94 s worst (limit 3 s);
  ETA "unknown" only for the first 0.2 s of each trip
- **cancel** (T06): zero speed on `/bus/cmd_vel` **0.001 s** after the cancel
  was accepted (limit 0.5 s), status `CANCELED`, crossing given back
- **person on the crossing:** referee said `PEDESTRIAN_PRESENT`, bus waited
  outside C (0 moments inside C with the person there), then docked
- **blocked bay:** waited with `BAY_BLOCKED` outside C, docked once the car left
- **lost position:** zero speed 0.535 s after the data froze (limit 0.65 s),
  `SAFE_STOP`, then continued and docked
- **Arkar's coordinator sent the bus by itself** (7 people in Q, `FALLBACK`
  on the observed queue because no forecast was running) → docked, 4 in, 3 waiting
- unit tests: `test_bus_core.py` (10) + boarding tests in `test_sim_core.py`
  + goal-rejection test in `test_reset_launch.py`

## the 15 trials explained like you're 5 🧸

1. **Trials 1–10, normal:** go dock. Must work 10 out of 10 (T05).
2. **Cancel (T06):** while driving, the boss says "never mind" → the bus must
   stop within 0.5 s and give the crossing permission back.
3. **Person on the crossing:** someone stands in C → the referee says no →
   the bus waits at the line. Person leaves → referee says yes (after 2 s of
   "all clear") → bus goes.
4. **Bus stop blocked:** a car parks in B → the bus waits before the crossing
   without asking for it. Car leaves → bus goes.
5. **Lost position:** the position data freezes for 1.5 s → the bus stops
   within 0.5 s (`SAFE_STOP`), then continues when the data is back.
6. **The real boss:** 7 people in the queue → Arkar's coordinator decides by
   itself to send the bus (no test code involved) → bus docks, 4 get in, 3
   wait.

## still not done / honest limits

- **Departure** after `COMPLETE` is not implemented: the assignment says it
  is a separate step ("a separate state transition or extension action").
  In the test the scenario is reset and the bus starts at A again.
- **Only bay B** (the baseline). A goal for another bay is rejected.
- **Route planner:** Aidil's planner now gives the route (A-N-Cn-B in all
  15 trips). If it doesn't answer, the bus waits 1 s, tries 2 more times,
  then fails with `PLANNER_UNAVAILABLE` and never moves (T18). The old
  built-in route is only an off-by-default option (`allow_route_fallback`).
- **Cancel inside C:** the bus stops at once (safety first) and keeps renewing
  the crossing until a new goal drives it out; it never releases while
  inside C. Not tested live (the T06 cancel happens before C).
- The simulated bus has **no inertia** (speed changes instantly), so the
  "stopped after cancel" time is shorter than a real bus would need.
- **Not reviewed by Fazliddin yet** — he must read the code and the results
  before the defence and fix anything he disagrees with.

## defence cheat sheet 🧑‍🏫

- **why an action and not a service?** docking takes ~40 s: you want
  progress (feedback: phase, distance left, ETA), a final result, and the
  ability to **cancel** halfway. A service can do none of that.
- **one pub/sub path to explain:** `/hub/vehicles` (sim) → `bus_manager`
  → `/bus/cmd_vel_candidate` → `safety_supervisor` → `/bus/cmd_vel` →
  `actuator_bridge` → `/bus/cmd_vel_applied` → simulator moves the bus.
- **one action lifecycle:** goal sent → accepted (checks bus id, bay,
  duration, not busy, position known, no safety stop) → executing with
  feedback 5 Hz → succeeded / aborted / canceled with `final_state` + `reason`.
- **why ask for the crossing *before* it?** the safety supervisor looks 0.5 s
  ahead and stops any bus whose footprint would enter C without being the
  owner — asking late = stopping on the line every time.
- **why distance along the road for ETA?** straight-line distance from A to
  B cuts through the crossing and corners; the bus can't. The assignment
  says so too (10.2 step 1).
- **problems found and fixed while building it** (good for "a failed test
  you corrected" — but make sure you understand them): (1) the referee
  answers `BAD_TOKEN` when a permission is already gone; the first version
  waited for a different word and would have retried RELEASE forever;
  (2) if the permission was lost while the bus was *inside* C, the first
  version would have braked in the middle of the crossing — now it never
  stops inside C on its own; (3) after a "position lost" stop it went back
  to `APPROACH` even when it had been boarding — now it resumes the mood it
  was in. All three were caught by reading the code against Arkar's arbiter
  and the closed-loop tests, before the live run.
