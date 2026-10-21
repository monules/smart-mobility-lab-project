# M5 — Simulator, Safety Supervisor & Infrastructure (Saidumar) — S9

Branch `m5-safety-supervisor`. ROS 2 **Jazzy**, Ubuntu 24.04, run in Docker
(verified natively on Apple Silicon, `aarch64`).

## 1. What this branch adds

| Path | What it is |
|---|---|
| `src/smart_hub_interfaces/` | **Full team interface package**: 14 msgs, 6 srvs, 3 actions, copied field-for-field from assignment §6–7. `QueueState`, `DemandPrediction`, `HubEvent` are byte-identical to M1-B's temporary copy, so M1-B's code runs unchanged. |
| `src/smart_hub/smart_hub/safety_supervisor.py` | ROS node: the only publisher of `/robot/cmd_vel` and `/bus/cmd_vel`. Logic in `safety_core.py`. |
| `src/smart_hub/smart_hub/actuator_bridge.py` | ROS node: final commands → simulated motors, with its **own** watchdog, speed caps and bus door interlock. Logic in `bridge_core.py`. |
| `src/smart_hub/smart_hub/scenario_runner.py` | ROS node: 20 Hz deterministic simulator, YAML scenarios, fault injection, `/scenario/reset`. Logic in `sim_core.py`. |
| `src/smart_hub/smart_hub/metrics_logger.py` | ROS node: per-run `events.csv` + `summary.json`. Logic in `metrics_core.py`. |
| `src/smart_hub/smart_hub/dashboard_marker_publisher.py` | ROS node: read-only RViz overlay on `/hub/markers` (§3.1). Logic in `marker_core.py`. |
| `src/smart_hub/smart_hub/camera_model.py` | Synthetic pinhole camera (640×480, 10 Hz `/camera/image_raw`, sensor QoS) that draws pedestrians for M1-A's detector; floor outlines of Q and C. |
| `src/smart_hub/scenarios/S1.yaml` | S1 script from §8.3 (counts 0→3→5→4, crossing pedestrian t=20–24 s, camera freeze t=30 s, 0.3 s occlusion at t=12 s). |
| `src/smart_hub/smart_hub/geometry.py`, `ros_common.py` | Map M1 geometry/graph; QoS profiles and stamp helpers. |
| `src/smart_hub/smart_hub/s9_fault_check.py` | Test tool: runs the §16.3 S9 recovery sequence (T16, T17) and writes `s9_check.json`. |
| `src/smart_hub/smart_hub/mock_candidate_publisher.py` | Test tool: stand-in for M4/M2-A candidate commands until their managers exist. |
| `src/smart_hub/launch/hub.launch.py`, `s9.launch.py` | M5 stack launch (`scenario`, `seed`, `mode` args) and the S9 demo. |
| `src/smart_hub/config/m5_safety.yaml`, `scenarios/S9.yaml` | Parameters (assignment values marked `(A)`), S9 scenario. |
| `src/smart_hub/test/` | M5 tests (see §6); team-wide results are in the root `README.md` §8. |
| `docker/Dockerfile`, `docker/hub.sh` | Dev image (derived from M1-B's) + build/test/run helper. |

## 2. Commands (run from the repository root on the Mac)

```bash
docker/hub.sh image      # once: build the ROS 2 Jazzy image (~3 min)
docker/hub.sh build      # colcon build (both packages, 1 worker, -j2 for 8 GB Macs)
docker/hub.sh test       # colcon test + colcon test-result --verbose
docker/hub.sh s9         # S9 demo/acceptance run (~15 s) -> output/
docker/hub.sh shell      # interactive shell with the workspace sourced
```

The repo is mounted at `/ws` (not copied), so edits need no image rebuild —
only `docker/hub.sh build` after adding files or changing `setup.py`.
Evidence lands in `output/` (git-ignored): `output/s9_check.json`,
`output/<run_id>/{events.csv,summary.json}`, a rosbag
`output/bag_S9_seed<seed>/` with `bag_S9_info.txt` (§20.2), and a parameter
dump per M5 node in `output/params/` (§17); each records the commit
(`-dirty` if `src/` or `docker/` has uncommitted changes).

Simulator pedestrians (`pedestrian`, `occlude` events) are seen only by the
camera; their true positions are on `/sim/truth` with kind `pedestrian`.
Camera faults: `freeze` with topic `camera` (silence) or `camera_repeat`
(same frame, old stamp — a frozen source, §16.1).

Inside `docker/hub.sh shell`:
```bash
ros2 launch smart_hub hub.launch.py scenario:=S9 seed:=9 mode:=sim
ros2 topic echo /hub/safety
ros2 topic pub --times 3 -r 10 -w 1 --qos-durability transient_local /hub/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /safety/clear_stop smart_hub_interfaces/srv/ClearSafetyStop "{operator_id: me, acknowledgement: ok}"
ros2 service call /scenario/reset smart_hub_interfaces/srv/ResetScenario "{scenario_id: S9, seed: 42, preserve_logs: true}"
```

## 3. How the pieces connect

```
 M4 robot_manager ──/robot/cmd_vel_candidate──┐            ┌─/robot/cmd_vel──┐
 M2-A bus_manager ──/bus/cmd_vel_candidate────┤            ├─/bus/cmd_vel────┤
 operator/runner ───/hub/estop────────────────┤  safety_   │                 ▼
 scenario_runner ───/robot/odom, /hub/vehicles┼► supervisor┤          actuator_bridge
 M2-B coordinator ──/crossing/state───────────┤            │       (own 0.5 s watchdog,
 actuator_bridge ───/bus/door_open────────────┘            │        caps, door interlock)
                                                           │                 │ /*/cmd_vel_applied
             /hub/safety (SafetyState) ◄───────────────────┘                 ▼
             ► M2-B coordinator, managers                             scenario_runner
                                                                 (20 Hz unicycle sim, own
   every node ──/hub/events──► metrics_logger ◄── /sim/truth ───  drive timeout) ──► /robot/odom,
                                                                  /hub/vehicles, /parking/observed
```

Each node = a small pure-Python *core* (all decisions, unit-tested with an
injected clock) + a thin ROS wrapper (messages, QoS, timers). Each node uses
one single-threaded executor, so its callbacks never run concurrently and
need no locks. Freshness uses `time.monotonic()` (never jumps) **and** the
message header stamp; unstamped or future-stamped data is never fresh.

### 3.1 Dashboard markers (`dashboard_marker_publisher`)

Read-only, **no control authority**: its only publisher is `/hub/markers`
(`visualization_msgs/MarkerArray`, 5 Hz, R10, frame `map`, lifetime 1 s);
it never publishes any `cmd_vel`, request or service call (§4.2).

| Input | Drawn as |
|---|---|
| `/hub/vehicles` (R10) | circle per agent/obstacle, diameter from `RADIUS_BY_KIND`; invalid = grey |
| `/crossing/state` (R1T) | box C coloured by state: FREE green, GRANTED blue, HOLD yellow, OCCUPIED purple, PEDESTRIAN orange, UNKNOWN grey; text shows owner |
| `/parking/state` (R1T) | 1.0 × 0.8 m box at P1–P3: FREE green, RESERVED yellow, OCCUPIED red, UNKNOWN grey |
| `/hub/safety` (R1T), `/hub/events` (R100) | text above the map: `SAFETY: OK / STOP LATCHED (reason)`, last event |
| `/hub/demand` (R10, M1-B) | text in queue Q: `demand +30s: X (now Y)`, or `prediction unavailable` if invalid/stale (§9.3) |
| `/traffic/edges` (R10, M3) | edge overlays: red blocked, dark grey invalid, orange density ≥ 0.70 (no speed-based congestion: `EdgeState` has no label) |
| — | **not drawn: planned routes** (no interface publishes them) |
| static (`geometry.py`) | site 12 × 8 m, queue Q x∈[2,6] y∈[5,7], graph edges as one LINE_LIST |

Any state not received in the last 1 s (`stale_after`) is drawn as
UNKNOWN/grey. In RViz: Fixed Frame `map`, add *MarkerArray* on `/hub/markers`.

## 4. Safety rules and where they come from

| Rule | Source | Implemented in |
|---|---|---|
| Safety tick 20 Hz, monotonic clock | §16.2 | `safety_supervisor.tick` |
| Robot ≤ 0.25 m/s, bus ≤ 0.40 m/s | §3.3 | `SafetyCore._clamp`; again in `BridgeCore` |
| Candidate older than 0.5 s → zero | §3.3, §16.1 | `COMMAND_STALE` hold |
| Odometry/pose older than 1.0 s → zero | §3.3, §16.1 | `ODOM_STALE` / `POSE_STALE` hold |
| Reject future stamps beyond a declared tolerance (0.2 s) | §5 | `SafetyCore.observed_at` |
| E-stop true latches; false never clears it | §5, §7.1, §16.3 | `SafetyCore.on_estop` |
| Clear rejected while stop asserted, hazard, stale sensor, motion | §7.1, §18.7 | `SafetyCore.request_clear` |
| Clear does not restart missions | §7.1, §16.3 | clear also requires zero/stale candidates |
| Clearance + motion footprint, boundary, door, crossing | §16.2, §18.7 | holds `CLEARANCE`, `BOUNDARY_AHEAD`, `DOOR_*`, `CROSSING_*` |
| Final command expiry in the adapter (supervisor crash) | §16.2, T17 | `BridgeCore.tick` |
| Every intervention has a reason + correlation id | §21.3 | `/hub/events`, `SafetyState.reason` |

**Latched stop vs hold** (M5 decision — the assignment only says "latch when
required"):

| Kind | Triggers | Effect | Ends |
|---|---|---|---|
| **Latch** | e-stop true, footprint outside the site (`BOUNDARY`), footprint overlap (`COLLISION`) | every agent zero, `stop_latched=true` | only `/safety/clear_stop` when every reset rule passes |
| **Hold** | stale command/odometry/vehicles/e-stop heartbeat, door open/unknown, clearance, crossing not granted/unknown, unknown body | affected agent(s) zero | automatically, when the condition is gone |

**Reset rules** (`/safety/clear_stop`): rejected with every reason listed if
the e-stop input is true or its heartbeat is stale, `/hub/vehicles` or an
agent's pose is stale, an agent's measured speed > 0.02 m/s, a latching hazard
is still present, or a manager still requests nonzero motion. "Zero velocity"
therefore means **both measured and commanded** zero.

**Three independent stop layers** (T17): supervisor (0.5 s candidate timeout)
→ actuator_bridge (0.5 s final-command timeout, own caps) → simulator (0.5 s
drive timeout). Any one dying cannot leave a nonzero motor command.

## 5. Decisions needing team confirmation

1. **One shared `/hub/safety`.** `SafetyState` has no agent field, so per-agent
   holds are in `reason` (e.g. `robot:COMMAND_STALE; bus:DOOR_OPEN`).
   `motion_permitted` is the *system* flag (false on latch, stale e-stop
   heartbeat or stale `/hub/vehicles`); idle agents do not clear it.
   M2-B's coordinator already uses only `stop_latched` + freshness — verified.
2. **E-stop heartbeat required** (`require_estop_heartbeat: true`): no fresh
   `/hub/estop` within 1 s → everything held. `scenario_runner` publishes it at
   10 Hz (`publish_estop:=true`); in S9 the operator tool does.
3. **IDs:** `robot_1`, `bus_1` (as in M2-B's README); obstacles are
   `VehicleState` entries with `kind: obstacle` in `/hub/vehicles` (M3's
   assumption, now implemented). `edge_id` uses M3's sorted form (`E-J`).
4. **Bus freshness** comes from its `/hub/vehicles` entry (no `/bus/odom` exists).
5. **Bridge ↔ simulator topics** `/robot/cmd_vel_applied`, `/bus/cmd_vel_applied`
   are M5-internal (both ends are M5's).
6. **`/scenario/inject`** (`std_msgs/String`, JSON event) is test-only.
7. **ResetScenario** resets the simulator and publishes `SCENARIO_RESET` with the
   new run id; there is no interface for other nodes to *acknowledge* a reset
   (§7.1 asks for it) — needs a team decision.
8. **Turn-rate caps** (robot 1.0, bus 0.5 rad/s) are M5 choices, not assignment values.

## 6. Verification (all run in the Docker container, 2026-10-21)

`docker/hub.sh test` → **74 tests, 0 failures, 0 skipped**, three runs in a row
on `m5-safety-supervisor` (81 with `test_marker_core.py`, added later). On
`integration/current-team-work` (M5 + M1-B): 134 tests, 132 passed,
2 xfailed — see the root `README.md` §8.

| Test file | Kind | Covers |
|---|---|---|
| `test_safety_core.py` (42) | unit, injected clock | pass-through, clamps, stale command/odom/vehicles/heartbeat, future/NaN rejection, latch, every reset rejection, accepted reset, no auto-restart, boundary/collision latch, clearance, crossing, door, events |
| `test_bridge_core.py` (7) | unit | watchdog stop (T17 core), caps, door interlock |
| `test_sim_core.py` (6) | unit | unicycle, same seed ⇒ same run, reset, drive timeout, events |
| `test_metrics_core.py` (4), `test_geometry.py` (4) | unit | latency, physical stop, violations, files; map geometry |
| `test_marker_core.py` (9) | unit | marker shapes/radii, unique ids, crossing/parking colours, stale → UNKNOWN, safety text, demand text, edge overlays |
| `test_safety_supervisor_node.py` (4), `test_scenario_runner_node.py` (3) | real rclpy node | unstamped/future stamps, e-stop, params; reset success/failure |
| `test_t17_process_kill.py` | **ROS integration, real processes** | SIGKILL controller → supervisor zero; SIGKILL supervisor → bridge zero; kill bridge → sim stops |
| `test_s9_launch.py` | **launch_testing** | full S9 sequence on `hub.launch.py`; checker exits 0 |
| `test_flake8.py`, `test_pep257.py` | ament linters (§21.1) | D213 off: summary-on-first-line, as in all teammates' code |

Measured (simulation only, not hardware):
- **T16** (e-stop → first zero `/robot/cmd_vel`, measured outside the
  supervisor): max 1.4 ms over 5 trials (limit 200 ms); supervisor receipt →
  zero ≤ 0.31 ms; simulated speed < 0.02 m/s after 0.10–0.15 s.
- **T17**: controller killed → supervisor zero after 0.45–0.50 s; supervisor
  killed → bridge zero after 0.50–0.54 s (timeout 0.5 s).
- **Dashboard**: with `hub.launch.py` running, `ros2 topic hz /hub/markers`
  = 4.99–5.00 Hz (std dev 4 ms); 20 markers in frame `map`; RELIABLE/VOLATILE.

Integration (scratch workspace, nothing merged): M2-B `hub_coordinator`,
M3 `parking_manager` + `traffic_monitor` and M1-B `demand_predictor` were
built against this interface package and run with the M5 stack. Verified:
crossing ACQUIRE granted with a fresh unlatched `/hub/safety` and denied with
`SAFETY_STOP` after an e-stop; parking P1 OCCUPIED / P2 FREE / P3 UNKNOWN
from `/parking/observed`; edge labels from `/hub/vehicles`; forecast
`valid:false` during warm-up.

## 7. Not done / limits

- Merging with M1-B: done on `integration/current-team-work` (union of
  `setup.py` / `package.xml` entries; M5's interface package kept).
- The synthetic camera draws only people (no robot/bus), so S1 images never
  contain vehicles. `dashboard_marker_publisher` draws the demand forecast
  and `/traffic/edges` overlays but not routes (no route topic exists) and
  not speed-based congestion (`EdgeState` has no label).
- `/scenario/reset` has no way to wait for other nodes' acknowledgements
  (§7.1; needs a team interface decision).
- T18 (planner loss) needs M4's planner; `mode:=hardware` is refused on purpose.
- Observation for M3: a vehicle parked at a node (bus at A) is labelled
  `A-Cs: CONGESTED` because speed 0 counts as slow.
