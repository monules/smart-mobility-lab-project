# Smart Mobility Hub — ROS 2 Simulation & System Architecture

Smart Mobility Engineering Lab (Prof. Mehdi Pirahandeh), Inha University.
A miniature Songdo transport station in ROS 2: detect passengers, predict
demand, dock a bus, manage parking, route traffic, escort a passenger with a
robot — and stop safely when anything goes wrong. Every decision must leave
observable, reproducible evidence (assignment §1).

> **Status (branch `integration/current-team-work`, local only).** M5
> (simulator, safety, infrastructure) and M1-B (demand prediction) are
> integrated, built and tested here. M2-B's coordinator is on `main` but not
> yet inside the ROS package; M3's work is on an open PR branch; **M1-A,
> M2-A and M4 have no code yet**. See [§10 Status](#10-current-status-and-next-steps).

Platform: **ROS 2 Jazzy, Ubuntu 24.04, Python `rclpy`**, run in Docker
(verified on Apple Silicon `aarch64`). Requirements and section numbers (§)
refer to `Smart_Mobility_Hub_Assignment.pdf`; roles come from the
7-person role document. Both PDFs are kept out of git.

**Contents:** [1 Overview](#1-project-overview) ·
[2 Repository map](#2-repository-map) ·
[3 Architecture](#3-architecture-and-data-flow) ·
[4 Setup](#4-environment-setup) ·
[5 Running](#5-how-to-run-the-project) ·
[6 Developing](#6-how-to-develop-a-new-component) ·
[7 Standards](#7-team-integration-standards) ·
[8 Tests](#8-tests-and-acceptance-criteria) ·
[9 Git workflow](#9-git-workflow-for-teammates) ·
[10 Status](#10-current-status-and-next-steps)

---

## 1. Project overview

Nine scenarios on one 12 m × 8 m map (§2): S1 passenger counting, S2 AI
demand prediction, S3 bus docking, S4 parking, S5 congestion, S6 route
replanning, S7 robot escort, S8 integrated rush hour, S9 faults and
emergency stop. Managers never drive motors: they publish *candidate*
velocities, and only `safety_supervisor` publishes final commands (§4.2).

| Member | Role | Nodes (§4, role doc) | Scenario | Code in this branch |
|---|---|---|---|---|
| M1-A Husan | Perception | `passenger_detector` | S1 | ❌ none yet |
| M1-B Tuymurod | Demand prediction | `demand_predictor` | S2 | ✅ merged |
| M2-A Fazliddin | Bus & docking | `bus_manager` | S3 | ❌ none yet |
| M2-B Arkar | Hub coordinator | `hub_coordinator` | S8 | ⚠️ root files from `main`, not in the package |
| M3 Carla | Parking & traffic | `parking_manager`, `traffic_monitor` | S4, S5 | ⚠️ open PR #3 branch, not merged |
| M4 Aidil | Routing & robot | `route_planner`, `robot_manager` | S6, S7 | ❌ none yet |
| M5 Saidumar | Simulator, safety, infra | `scenario_runner`, `safety_supervisor`, `actuator_bridge`, `metrics_logger`, `dashboard_marker_publisher` | S9 | ✅ complete for simulation |

## 2. Repository map

```
.
├── README.md                    ← this file (team entry point)
├── README_M5.md                 M5 design notes and evidence
├── README_M2B.md                M2-B coordinator notes (from main)
├── RUN_ON_MAC.md, Dockerfile, s2.sh, .dockerignore   M1-B's S2 Docker workflow
├── docker/
│   ├── Dockerfile               team dev image (ROS 2 Jazzy, pinned digest)
│   └── hub.sh                   build / test / run helper (repo mounted at /ws)
├── docs/
│   ├── INTEGRATION_CONTRACT.md  topics, services, IDs, safety and reset rules
│   └── TEAM_STANDARDS.md        short copyable conventions
├── coordinator_core.py, hub_coordinator.py, test_coordinator_core.py   M2-B (not built)
└── src/
    ├── smart_hub_interfaces/    SHARED: 14 msg, 6 srv, 3 action (§6–7)
    └── smart_hub/               SHARED ament_python package (all nodes)
        ├── smart_hub/
        │   ├── demand/                  M1-B pure logic (features, models, policy, simulation)
        │   ├── demand_predictor.py      M1-B node: /hub/queue -> /hub/demand
        │   ├── fake_queue_publisher.py  M1-B test tool: synthetic /hub/queue
        │   ├── scenario_runner.py       M5 node   + sim_core.py (20 Hz simulator)
        │   ├── safety_supervisor.py     M5 node   + safety_core.py
        │   ├── actuator_bridge.py       M5 node   + bridge_core.py
        │   ├── metrics_logger.py        M5 node   + metrics_core.py
        │   ├── dashboard_marker_publisher.py  M5 node + marker_core.py
        │   ├── geometry.py, ros_common.py      M5 shared helpers (map, QoS)
        │   ├── s9_fault_check.py        M5 test tool: S9 sequence, T16/T17
        │   └── mock_candidate_publisher.py     M5 test tool: stand-in for M2-A/M4
        ├── launch/  hub.launch.py, s9.launch.py (M5) · demand_demo.launch.py (M1-B)
        ├── config/  m5_safety.yaml (M5) · demand_predictor.yaml (M1-B)
        ├── scenarios/S9.yaml    (M5)    models/demand_ridge_h30.json (M1-B)
        ├── scripts/train_demand_model.py · results/ (M1-B T03 evidence)
        ├── test/    test_demand_*.py (M1-B) · all other test_*.py (M5)
        ├── setup.py, package.xml        SHARED (every member adds entries)
        └── README.md                    M1-B S2 notes
```

**Shared files** (change only with the affected owners' agreement):
`src/smart_hub_interfaces/**`, `src/smart_hub/setup.py`,
`src/smart_hub/package.xml`, `docker/`, `docs/`, this README.
Everything else belongs to the member named above.

## 3. Architecture and data flow

```mermaid
flowchart LR
  subgraph M5sim[M5 simulator]
    SR[scenario_runner]
  end
  subgraph Perception
    PD[passenger_detector<br/>M1-A ❌]
    DP[demand_predictor<br/>M1-B ✅]
  end
  subgraph Coordination
    HC[hub_coordinator<br/>M2-B ⚠️]
    PM[parking_manager<br/>M3 ⚠️]
    TM[traffic_monitor<br/>M3 ⚠️]
    RP[route_planner<br/>M4 ❌]
  end
  subgraph Managers
    BM[bus_manager<br/>M2-A ❌]
    RM[robot_manager<br/>M4 ❌]
  end
  subgraph M5safety[M5 safety]
    SS[safety_supervisor]
    AB[actuator_bridge]
  end
  ML[metrics_logger M5]
  DB[dashboard_marker_publisher M5]

  SR -- "/camera/image_raw ❌" --> PD
  PD -- "/hub/queue QueueState" --> DP
  DP -- "/hub/demand DemandPrediction" --> HC
  SR -- "/hub/vehicles VehicleStateArray" --> TM & HC & SS & BM
  SR -- "/robot/odom Odometry" --> SS & RM
  SR -- "/parking/observed SlotObservationArray" --> PM
  TM -- "/traffic/edges EdgeStateArray" --> RP & HC
  PM -- "/parking/state ParkingState" --> HC
  HC -- "/crossing/state CrossingState" --> SS & BM & RM
  HC -. "/bus/dock DockBus action" .-> BM
  HC -. "/robot/escort EscortPassenger action" .-> RM
  BM & RM -. "/route/compute srv" .-> RP
  BM & RM -. "/crossing/request srv" .-> HC
  BM -- "/bus/cmd_vel_candidate TwistStamped" --> SS
  RM -- "/robot/cmd_vel_candidate TwistStamped" --> SS
  SS -- "/robot/cmd_vel, /bus/cmd_vel Twist" --> AB
  AB -- "/robot/cmd_vel_applied, /bus/cmd_vel_applied Twist" --> SR
  AB -- "/bus/door_open Bool" --> SS
  SS -- "/hub/safety SafetyState" --> HC & BM & RM
  SS & SR & DP & HC & PM & TM -- "/hub/events HubEvent" --> ML
  SR -- "/sim/truth" --> ML
  SS & HC & PM & TM & DP --> DB
  DB -- "/hub/markers MarkerArray" --> RV[RViz]
```

Solid arrows are topics; dotted arrows are services/actions. The full
inventory with rates, QoS and owners is in
[docs/INTEGRATION_CONTRACT.md §3–4](docs/INTEGRATION_CONTRACT.md#3-topics).

**How the M5 stack works.** `scenario_runner` steps a deterministic 2D world
at a fixed 0.05 s and publishes odometry, observed vehicles, parking sensors
and the e-stop heartbeat. `safety_supervisor` checks every 50 ms (monotonic
clock) that the e-stop is released, inputs are fresh, there is clearance,
the bus door is shut and crossing C is granted, then clamps and forwards the
candidates. `actuator_bridge` independently zeroes any final command older
than 0.5 s, and the simulator stops any body without a fresh applied
command — three layers that each stop the robot alone (T17).
`metrics_logger` writes per-run evidence; `dashboard_marker_publisher`
draws the state for RViz and has no control authority.

## 4. Environment setup

Supported: **Docker** with the ROS 2 Jazzy image (`docker/`). Do not install
a native ROS on macOS; Jazzy does not support it.

Prerequisites: Docker Desktop running (Apple Silicon or x86 both work — the
base image is multi-arch), git, about 4 GB free for the image.

```bash
docker/hub.sh image
```
```bash
docker/hub.sh build
```

- The repository is **mounted** at `/ws`, not copied: edit on your machine,
  then `docker/hub.sh build` (needed after adding files or editing
  `setup.py`; `--symlink-install` picks up Python edits automatically).
- `build/ install/ log/ output/` appear in the repo root; all are git-ignored.
- Builds use one package at a time and `-j2` because Docker Desktop on an
  8 GB Mac gives Linux about 4 GB.
- The base image is pinned by digest so everyone builds on the same ROS.
- M1-B's older workflow (`./s2.sh build`, `./s2.sh test`, root `Dockerfile`)
  still works on this branch — it copies `src/` into its own image, so code
  changes need `./s2.sh build` again. Verified: 134 tests, 0 failures.

## 5. How to run the project

| Goal | Command | Result |
|---|---|---|
| Run all tests | `docker/hub.sh test` | colcon test + summary |
| S9 acceptance demo (~15 s) | `docker/hub.sh s9` | `S9 verdict: PASSED`, exit 0 only if every check passed; evidence in `output/` |
| Shell with workspace sourced | `docker/hub.sh shell` | interactive bash in `/ws` |
| One command | `docker/hub.sh run 'ros2 node list'` | |

Inside `docker/hub.sh shell`:

```bash
ros2 launch smart_hub hub.launch.py scenario:=S9 seed:=9 mode:=sim
```
Starts `scenario_runner`, `safety_supervisor`, `actuator_bridge`,
`metrics_logger`, `dashboard_marker_publisher`. Arguments: `scenario`
(file in `scenarios/`), `seed`, `mode` (only `sim`; `hardware` is refused),
`output_dir` (default `output`), `publish_estop` (`true`: the runner sends
the 10 Hz e-stop heartbeat).

```bash
ros2 launch smart_hub demand_demo.launch.py seed:=1025 run_id:=demo
```
M1-B's S2 demo (fake queue → predictor; forecast valid after ~60 s warm-up).

Useful inspections (all verified):
```bash
ros2 topic echo /hub/safety
```
```bash
ros2 topic pub --times 3 -r 10 -w 1 --qos-durability transient_local /hub/estop std_msgs/msg/Bool "{data: true}"
```
```bash
ros2 service call /safety/clear_stop smart_hub_interfaces/srv/ClearSafetyStop "{operator_id: me, acknowledgement: ok}"
```
```bash
ros2 service call /scenario/reset smart_hub_interfaces/srv/ResetScenario "{scenario_id: S9, seed: 42, preserve_logs: true}"
```
```bash
ros2 topic hz /hub/markers
```

**Outputs.** `output/s9_check.json` (S9 checks, T16 latencies),
`output/<run_id>/events.csv` and `summary.json` (stop latencies, physical
stop times, `motion_while_latched`, interventions; the commit is recorded,
with `-dirty` when `src/` or `docker/` has uncommitted changes).

**Dashboard / RViz.** `/hub/markers` (5 Hz, frame `map`) draws: site,
queue Q with forecast text (`demand +30s: …` or `prediction unavailable`),
graph edges plus red **blocked** / dark-grey **unknown** / orange
**density ≥ 0.70** overlays from `/traffic/edges`, crossing C coloured by
`/crossing/state`, parking slots by `/parking/state`, vehicles and obstacle
O as footprint cylinders, and safety/last-event text. Anything older than
1 s is drawn grey/UNKNOWN. **Not drawn:** planned routes (no interface
publishes them) and speed-based congestion (`EdgeState` has no label).
In RViz set Fixed Frame `map` and add a *MarkerArray* display on
`/hub/markers`. The dev image has no RViz/GUI; viewing it needs a desktop
ROS 2 Jazzy install on the same `ROS_DOMAIN_ID` — not set up or tested on
the Mac.

**Troubleshooting (all seen during development)**

| Symptom | Cause / fix |
|---|---|
| `Cannot connect to the Docker daemon` | start Docker Desktop |
| `Unknown package 'smart_hub_interfaces'` from the CLI | workspace not sourced: use `docker/hub.sh shell`/`run`, or `source install/setup.bash` |
| `ros2 topic pub --once` e-stop has no effect | it can exit before discovery; use `--times 3 -r 10 -w 1` as above |
| `ros2 node list` empty right after launch | CLI discovery delay; nodes still run — use `ros2 topic hz` |
| New node "executable not found" | add it to `setup.py` `console_scripts`, then `docker/hub.sh build` |
| Everything held, reason `ESTOP_SIGNAL_STALE` | nothing publishes `/hub/estop`; launch with `publish_estop:=true` (default) |

## 6. How to develop a new component

Conventions used by every node in this package:

1. **Pure logic module** (no ROS imports) — e.g. `smart_hub/my_core.py`,
   time passed in as an argument so tests need no sleeps.
2. **Thin ROS node** `smart_hub/my_node.py` with `main()`; import QoS
   profiles from `smart_hub.ros_common` (`R1, R10, R100, R1T`) and message
   types **only** from `smart_hub_interfaces` — never copy a `.msg`.
3. **Register** in `src/smart_hub/setup.py`:
   `'my_node = smart_hub.my_node:main',` (under your member comment), and
   add new ROS dependencies to `src/smart_hub/package.xml`.
4. **Launch**: own `launch/my_demo.launch.py`; add your node to
   `hub.launch.py` once it is ready for the integrated run.
5. **Tests** in `src/smart_hub/test/test_my_*.py` (pytest); ROS node tests
   skip cleanly when ROS is missing (see `test_scenario_runner_node.py`).
6. **Verify**: `docker/hub.sh build && docker/hub.sh test` — 0 failures,
   flake8 + pep257 clean (single quotes, imports sorted alphabetically,
   lines ≤ 99).

Need a new message or field? Do **not** add it alone: follow
[INTEGRATION_CONTRACT.md §10](docs/INTEGRATION_CONTRACT.md#10-changing-this-contract).

## 7. Team integration standards

Summary (details: [docs/INTEGRATION_CONTRACT.md](docs/INTEGRATION_CONTRACT.md),
copyable list: [docs/TEAM_STANDARDS.md](docs/TEAM_STANDARDS.md)):

| Topic | Rule | Kind |
|---|---|---|
| Final motor commands | only `safety_supervisor`; managers publish candidates | **Assignment** §4.2 |
| Speed caps, timeouts | 0.25 / 0.40 m/s; command 0.5 s; sensors 1.0 s; stop ≤ 0.20 s | **Assignment** §3.3 |
| E-stop | `true` latches, `false` never clears; reset via `/safety/clear_stop` | **Assignment** §5, §7.1 |
| Reset rejected while | e-stop on/stale, sensor stale, hazard, speed > 0.02 m/s, nonzero command | Assignment list + 0.02 m/s and "command" are **proposals** |
| Vehicle ids / kinds | `robot_1`, `bus_1`; kind `robot bus car obstacle` | **Proposal** (only spelling in use) |
| Edge ids | sorted node names (`E-J`) | **Proposal** (M3 + M5) |
| Stale `/hub/safety` (> 1 s) | treat as stop | **Proposal** (M2-B already does) |
| Per-agent safety reasons | in `SafetyState.reason` (`robot:COMMAND_STALE`) | **Proposal**; per-agent field is **open** |
| Reset acknowledgement, escort trigger, parking entrance topics | — | **Unresolved** (contract §9) |

## 8. Tests and acceptance criteria

Run everything: `docker/hub.sh test`.

| Group | Files | What it proves |
|---|---|---|
| M5 safety logic | `test_safety_core.py` (42) | caps, stale command/odometry/vehicles/heartbeat, latch, every reset rejection, no auto-restart, clearance, crossing, door |
| M5 bridge, sim, metrics, map, dashboard | `test_bridge_core.py`, `test_sim_core.py`, `test_metrics_core.py`, `test_geometry.py`, `test_marker_core.py` | watchdog, door interlock, determinism, evidence maths, marker content |
| M5 node tests (real rclpy) | `test_safety_supervisor_node.py`, `test_scenario_runner_node.py` | stamp handling, e-stop, reset success/failure |
| **T17** process kill | `test_t17_process_kill.py` | SIGKILL controller → supervisor zero; SIGKILL supervisor → bridge zero; kill bridge → sim stops |
| **S9 / T16** | `test_s9_launch.py` (launch_testing) | full §16.3 sequence on the real launch file |
| M1-B S2 | `test_demand_*.py` | features, models, dispatch rule, estimator, T03 run, real-ROS node |
| Linters | `test_flake8.py`, `test_pep257.py` | ament style; M1-B files tracked as xfail (owner restyle pending) |

**Acceptance tests owned by M5 (§21.1):**
- **S9 / T16** — e-stop while the robot moves: zero output ≤ 0.20 s after
  supervisor receipt; `false` keeps the latch; reset rejected while a sensor
  is stale; accepted when safe; robot stays stopped until a new goal.
- **T17** — "kill controller, then supervisor": safety and adapter stop
  motion independently.
- **T18** — planner absent/late reply: timeout, bounded retry, no stale
  path. **BLOCKED**: needs M4's `route_planner` and a manager that calls
  `/route/compute` (M2-A/M4).

**Verified results** (Docker on Apple Silicon, 2026-10-21, integration
commit `6ebf009`, simulation only):

| Check | Result |
|---|---|
| `docker/hub.sh test` (×2) | 134 tests: **132 passed, 2 xfailed** (colcon lists xfail as "skipped"), 0 failures |
| `docker/hub.sh s9` (×3) | **PASSED**, 13/13 checks each run |
| T16 e-stop → first zero `/robot/cmd_vel` (measured outside the supervisor) | 0.9–2.1 ms over 5 trials; supervisor receipt → zero ≤ 0.30 ms; limit 200 ms |
| Simulated robot speed < 0.02 m/s after e-stop | 0.09–0.14 s |
| T17 controller killed → supervisor zero | 0.50–0.53 s (×3) |
| T17 supervisor killed → bridge zero | 0.52–0.53 s (×3) |
| Nonzero output while latched | 0 |
| M1-B S2 runtime with M5 stack | `/hub/demand` 1 Hz, valid after warm-up; dashboard shows the forecast |
| M2-B / M3 against these interfaces | crossing granted, then denied with `SAFETY_STOP` after an e-stop; parking/edge states consumed (scratch workspace, not merged) |
| M2-B `test_coordinator_core.py`; M3 branch `tests/` | 28 passed; 59 passed, 9 skipped (run from clean exports) |

One S9 run once exited 0 without writing `s9_check.json` (not reproduced in
3 reruns); `docker/hub.sh s9` now fails if the file is missing.

## 9. Git workflow for teammates

| Branch | Purpose |
|---|---|
| `main` (origin) | assignment base; contains M2-B's coordinator (PR #2). Not modified by integration work |
| `m5-safety-supervisor` (local) | M5 work, one commit on top of `main` |
| `integration/current-team-work` (local) | this branch: M5 + M1-B merged, shared docs; **not pushed yet** |
| `origin/feature/m1b-demand-predictor` | M1-B's branch (merged here) |
| `origin/feature/m2b-coordinator`, `origin/ARKARKYAW2003-patch-1` | already merged into `main` (PR #2, #1) |
| `origin/m3-parking-traffic` | M3, open PR #3; not merged here (files at repo root, not in `src/smart_hub`) |

Once the team agrees to publish the integration branch (it is local today),
teammates would use:

```bash
git fetch origin
```
```bash
git switch -c feature/<you>-<topic> origin/integration/current-team-work
```
Work, then:
```bash
docker/hub.sh build && docker/hub.sh test
```
```bash
git add <your files only>
```
```bash
git commit -m "M4: route_planner with Dijkstra (T11, T12)"
```
and open a pull request **into `integration/current-team-work`**, not `main`.

Never: `git push --force`, committing `build/ install/ log/ output/`, or
`git add .` / `git add -A` (it would pick up the assignment PDFs, which
are deliberately untracked). Merge into `main` only as a team decision.

## 10. Current status and next steps

**Assignment requirements → implementation** (§21.1 test matrix):

| Item | Owner | State |
|---|---|---|
| Interfaces §6–7 (all) | M5 | ✅ |
| S1 / T01–T02 detection, occlusion | M1-A | ❌ not started in repo |
| S2 / T03 prediction (+ T04 reference policy) | M1-B | ✅ merged, tests pass |
| T04 dispatch hysteresis, S8 / T15 | M2-B | ⚠️ core logic + 28 tests on `main`; node not in package; T15 needs all nodes |
| S3 / T05–T06 docking, cancel | M2-A | ❌ |
| S4–S5 / T07–T10 parking, congestion | M3 | ⚠️ PR #3 branch, 59 tests pass; not merged |
| S6–S7 / T11–T14 routing, escort | M4 | ❌ |
| S9 / T16, T17 | M5 | ✅ verified in simulation |
| T18 planner loss | M4 + M5 | ⛔ blocked (no planner) |
| Simulator §3.2 (20 Hz, seeded, reset, fault injection) | M5 | ✅ — no camera images, no RViz routes |
| Dashboard §4 | M5 | ✅ — routes not drawn (no interface) |
| Physical extension §22 | team | not started (optional) |

**Next steps**
- **M2-B:** move `coordinator_core.py`/`hub_coordinator.py` into
  `src/smart_hub/smart_hub/` and the test into `src/smart_hub/test/` (as
  your README says), add the `hub_coordinator` console script. Verified to
  build and run against these interfaces in a scratch workspace.
- **M3:** move `smart_hub/*.py` and `tests/` into `src/smart_hub/` and
  rebase PR #3 on this branch; confirm `kind: obstacle`; note a parked bus
  at A is labelled `A-Cs: CONGESTED` (speed 0 counts as slow).
- **M1-B:** restyle for `ament_flake8` (mostly single quotes) and fix one
  docstring; then delete the `PENDING_OWNER_RESTYLE` entries.
- **M1-A, M2-A, M4:** start from §6 above; use `robot_1`/`bus_1`; publish
  stamped candidates only. M4's planner unblocks T18.
- **M5:** camera image stream for S1 and route markers once a route
  interface is agreed; reset acknowledgements (contract §9).
- **Team:** answer [contract §9](docs/INTEGRATION_CONTRACT.md#9-open-decisions-who-must-answer)
  and decide when to push `integration/current-team-work`.
