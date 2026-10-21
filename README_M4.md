# M4 — Google Maps + the escort robot 🗺️🤖 (Aidil, scenarios S6 + S7)

Branch: `feature/m4-routing-robot` (also merged into
`integration/current-team-work`). Written for Aidil by Saidumar (M5) with
Claude, with Aidil's OK — so now it's yours: read it, run it, get it, own it
before the defence. 🫡

---

## tl;dr

You own two programs:

1. **`route_planner`** = the hub's Google Maps. Anyone (bus, robot, car)
   asks "how do I get from A to E?" → it answers with the cheapest route, or
   says **`NO_ROUTE`** out loud when there is none. (S6)
2. **`robot_manager`** = the escort robot. It picks up a passenger at R,
   waits for them to press "ready", walks them to D through the crossing,
   and goes around obstacles. (S7)

## part 1: the route planner 🗺️

**"Cheapest" isn't "shortest".** Every road gets a price in seconds:

```
cost = length / max(speed, 0.05) × (1 + 2 × density) + crossing_wait
```

- `length`: metres of road
- `speed`: how fast you can go there (your top speed, or slower if the
  traffic on that road is slower — from Carla's `traffic_monitor`)
- `density`: how full the road is, 0 = empty, 1 = packed
- `crossing_wait`: +5 s if the road goes through crossing C and C is busy
  (someone on it, someone else owns it, or we don't know)

Then **Dijkstra** finds the cheapest chain of roads. (Tiny map → no fancy A*
needed, the assignment says so.)

**Some roads are deleted, not just made expensive** ❌ — "a little penalty"
could still get picked, a deleted road can't:

| deleted when… | why |
|---|---|
| not your road | robots don't drive in parking, buses don't park (map rules, 3.1) |
| an obstacle touches it | obstacle radius + your radius + 5 cm |
| traffic says BLOCKED or UNKNOWN | trust the traffic monitor |
| the info is old (> 1.5 s traffic, > 1 s obstacles) | "stale = blocked" (13.2) |
| the caller excluded it | `excluded_edges` in the request |

Nothing left between start and goal → `success=false, reason=NO_ROUTE`.

## part 2: the escort robot 🤖

```
IDLE → TO_PICKUP → WAIT_READY → ESCORT → ARRIVED ✅
   pauses (robot stops, mission keeps going): BLOCKED, WAIT_CROSSING
   bad endings: CANCELLED, FAILED, SAFE_STOP
```

| step | what happens | number |
|---|---|---|
| goal comes in | check ids, reject a 2nd goal while busy (`BUSY`), reject a reused request id (`DUPLICATE_REQUEST`) | — |
| at pickup R | stop, say "press ready" (`PASSENGER_PROMPT` event), wait for `/robot/confirm_ready` | max 20 s, then `FAILED READY_TIMEOUT`, never moved |
| ready button | only works while waiting at pickup — pressing it at any other time is refused and logged | — |
| ask for a route | `/route/compute`, wait max 1 s, try again max 2 times, then `FAILED PLANNER_UNAVAILABLE` (T18). Answers that come too late are thrown away | 1 s × 3 |
| route too long for the deadline | `FAILED DEADLINE_INFEASIBLE` | — |
| driving | follows the waypoints, turns on the spot when the heading is way off, max 0.22 m/s, max 0.9 rad/s turn | — |
| crossing C | asks before entering, stops 0.2 m before C if the answer is no (`WAIT_CROSSING`), renews every 1 s, gives it back only when fully out | — |
| obstacle on a road ahead | stops at the next corner (never on the blocked road), asks for a new route from there, continues only with the new, checked route | — |
| obstacle on the road it's ON | stops, goes back to the corner it came from, new route from there | — |
| obstacle just next to the path | pauses (`BLOCKED`) until it's gone | 0.15 m extra space |
| road got slow (no block) | may switch route, but max once every 2 s and only if ≥ 15 % cheaper | 2 s, 15 % |
| arrived | within 0.20 m of D for 1 s → `ARRIVED` | 0.20 m, 1 s |
| odometry frozen | zero speed after 0.5 s, `SAFE_STOP ODOM_STALE` after 2 more seconds | 0.5 s + 2 s |
| cancel | zero speed right away | — |

Progress = travelled ÷ (travelled + distance left). It can go **down** after a
detour, on purpose: the distance left always follows the new route (the
assignment says your definition has to say this — it does).

Like the bus, the robot only **suggests** speeds (`/robot/cmd_vel_candidate`);
Saidumar's `safety_supervisor` has the final word.

## the files

| file | what it does |
|---|---|
| `src/smart_hub/smart_hub/route_core.py` | **planner brain**: cost formula, deleting roads, Dijkstra (no ROS) |
| `src/smart_hub/smart_hub/route_planner.py` | ROS wrapper: the `/route/compute` service, `/route/edge_costs` (prices of every road, 1 Hz) |
| `src/smart_hub/smart_hub/robot_core.py` | **robot brain**: moods, route following, rerouting, crossing, planner retries (no ROS) |
| `src/smart_hub/smart_hub/robot_manager.py` | ROS wrapper: `/robot/escort`, `/robot/navigate_to`, `/robot/confirm_ready` |
| `test/test_route_core.py` | 28 tests incl. **T11**: 20 random road situations, each answer compared with a separate "try every possible path" checker |
| `test/test_robot_core.py` | 15 tests: the robot brain drives the simulator with no ROS |
| `s6_check.py`, `s7_check.py`, `t18_check.py` + launch files | the live exams |
| `scenarios/S6.yaml`, `S7.yaml` | the stories |

The robot also uses Saidumar's map (`geometry.py`) and the bus's path helper
(`bus_core.Path`), so robot and bus treat the crossing exactly the same way.

## topics & services 🔌

| | name | type | with whom |
|---|---|---|---|
| serves | `/route/compute` | `ComputeRoute` | bus_manager, robot_manager, anyone |
| serves (action) | `/robot/escort` | `EscortPassenger` | coordinator (later, S8) / `s7_check` |
| serves (action) | `/robot/navigate_to` | `NavigateTo` | reusable "just drive there" |
| serves | `/robot/confirm_ready` | `std_srvs/Trigger` | the passenger's button |
| reads | `/traffic/edges` | `EdgeStateArray` | Carla's traffic monitor |
| reads | `/hub/vehicles` | `VehicleStateArray` | simulator: obstacles |
| reads | `/crossing/state` | `CrossingState` | Arkar's coordinator |
| reads | `/robot/odom`, `/hub/safety` | | simulator, safety supervisor |
| writes | `/robot/cmd_vel_candidate` | `TwistStamped`, 20 Hz | safety supervisor |
| writes | `/robot/route` | `Path` | dashboard draws it |
| writes | `/route/edge_costs` | `String` (JSON) | anyone curious / evidence |
| calls | `/crossing/request` | `RequestCrossing` | Arkar's referee, as `robot_1` |

## run it 🚀

```bash
docker/hub.sh s6        # routing exam, ~1.5 min
docker/hub.sh s7        # escort exam: 10 escorts + 6 special trials, ~15 min
docker/hub.sh s7 1      # quick: 1 escort + the 6 special trials, ~5 min
docker/hub.sh t18       # planner missing / too slow, ~1 min
```

## what's proven ✅ (simulation)

Measured with `docker/hub.sh s6`, `s7`, `t18` — all checks passed; proof in
[`evidence/m4/`](evidence/m4):

**S6 routing (8/8)**
- empty roads: car A→E takes A-Cs-J-E = **25 s** (10 m ÷ 0.40 m/s); the upper
  detour costs **35 s** (14 m) ✅ (13.3 step 1)
- obstacle O on Cs-J → Cs-J deleted (`OBSTACLE`) → route A-N-Cn-B-U-E, no
  deleted road in it ✅
- 3 slow cars on Cs-J (jammed, not blocked) → lower road 115 s vs detour 35 s
  → detour chosen although it is 4 m longer ✅ ("cost, not distance")
- O + O2 (Cs-J and Cn-B) → car `NO_ROUTE`; robot NavigateTo E → `NO_ROUTE`
  after 0.09 s, **0 nonzero commands, moved 0.000 m** (T12) ✅
- roads back → new route checked first, then the robot drives (arrived at W,
  2.5 cm off) ✅
- **T11 live:** 5/5 answers = the "try every path" checker; + 20 random
  situations in the unit tests ✅
- answer time: **29 ms** p95 over 78 calls (limit 250 ms) ✅
- picture: `s6_maps.png` — price of every road before/after each change

**S7 escort (9/9)**
- **10/10 escorts** R→D, arrival error **0.027 m** (limit 0.20) (T13) ✅
- progress updates at 5 Hz (needs ≥ 2), crossing held whenever the robot
  was in C, 0 conflicts, 0 obstacle contacts ✅
- no "ready" → `FAILED READY_TIMEOUT` after 20.0 s, never moved; pressing
  ready while idle is refused ✅
- obstacle O while walking → detour `W-Cn-B-U-E-J-D`, distance left
  10.4 m → 14.3 m, arrived; **77.9 s vs 56.8 s** for the normal route ✅
- cancel → accepted in 0.022 s, zero speed 0.036 s later ✅
- 2nd goal while busy → `BUSY`; reused request id → `DUPLICATE_REQUEST` ✅
- destination P1 (parking = cars only) → `NO_ROUTE`, never moved ✅
- odometry frozen → zero speed after 0.49 s, `SAFE_STOP ODOM_STALE` after
  2.5 s ✅
- honest note: the S7 world has no sensor noise, so the 10 normal escorts
  are near-identical repeats (56.8–56.9 s), not 10 different situations

**T18 planner loss (5/5, with Saidumar)**
- planner gone → robot and bus each fail with `PLANNER_UNAVAILABLE` after
  ~3.1 s (1 s × 3 tries), **never moved** ✅
- planner answers after 1.5 s (too late) → same failure, all 3 late answers
  thrown away, never moved ✅
- planner answers after 0.3 s → robot arrives normally ✅

**unit tests:** `test_route_core.py` 28 (incl. T11 × 20 seeds),
`test_robot_core.py` 15

## still not done / honest limits

- **The coordinator doesn't send escort goals yet** — the assignment gives no
  trigger for "start the escort at t = 70 s" (Arkar's README, question 3).
  In S7 the checker sends them. Needed for S8.
- **Congestion replanning** for the robot (2 s / 15 % rule) is in the code
  and in the unit tests, but no live test makes a robot road congested.
- **Cancel inside C:** the robot stops at once and keeps the crossing until
  it is out — it never gives back a crossing it is standing in.
- **No inertia** in the simulated robot, so "stopped after cancel" is
  instant; a real robot needs time and distance.
- The robot is counted as traffic by the traffic monitor when it drives on
  a road (Cs-J etc.); standing still there can make that road look
  congested for a moment.
- **Not reviewed by Aidil yet** — read the code and results before the
  defence and fix anything you disagree with.

## defence cheat sheet 🧑‍🏫

- **why is routing a service but escorting an action?** computing a route
  takes milliseconds → ask, get answer. Escorting takes a minute, needs
  progress updates and a cancel button → action.
- **why delete blocked roads instead of a big penalty?** a penalty can still
  win when everything else is expensive; a deleted road never gets used.
  And if every road is gone, the planner must *say* `NO_ROUTE`.
- **how do you know Dijkstra is right?** T11: 20 random road situations ×
  8 trips, every answer compared with a separate program that tries every
  possible path. Same cost (±0.0001 %), never a forbidden road.
- **what if the planner is slow or dead?** wait 1 s, try 2 more times, then
  fail clearly; late answers are thrown away so the robot never follows an
  old route (T18).
- **one pub/sub path:** `/hub/vehicles` (obstacle O appears) →
  `robot_manager` sees Cs-J is blocked → asks `/route/compute` → new route
  → `/robot/cmd_vel_candidate` → `safety_supervisor` → `/robot/cmd_vel` →
  bridge → simulator.
- **bugs found while building it** (for "a failed test you corrected" —
  make sure you understand them): (1) "navigate to where you already are"
  never finished; (2) when the robot had to turn back into the crossing, the
  normal stop line was already behind it, so it would have driven in
  without asking — now it stops right where it is and asks first; (3) 🏆
  the best one: re-running the bus test with your planner, 3 of 15 bus
  trips went the long way round and parked backwards. Why? The bus waiting
  at A is itself "a car standing still" on its first road, so the traffic
  monitor called that road jammed and your planner priced it at 80 s
  instead of 5 s. Fix: when the bus/robot asks, the planner takes the asker
  itself out of every road it is standing on — you're not a traffic jam
  for yourself. (First try used the bus's "which road am I on" label: didn't
  work, because at A that label flips between two roads with sensor noise.
  Second try uses its position. Lesson: test again after every fix!); (4) the bus used to fall back to a fixed route when the planner was missing, which
  breaks T18 — now it fails clearly too (fallback is an off-by-default
  option).
