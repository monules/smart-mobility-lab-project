# Smart Mobility Hub — Integration Contract

The single reference for how components talk to each other. Every row says
where it comes from:

- **(A)** required by the assignment (`Smart_Mobility_Hub_Assignment.pdf`,
  section numbers given as §n);
- **(Impl)** implemented and tested on branch `integration/current-team-work`;
- **(Prop)** proposed convention, adopted by default, may be changed by team agreement;
- **(Open)** undecided — needs the named owner(s).

The `.msg/.srv/.action` files in `src/smart_hub_interfaces/` are the binding
form of §6–7 (copied field for field). If this document and those files ever
disagree, the files win and this document must be fixed.

## 1. Packages and ownership

| Package / location | Type | Owner | Status on this branch |
|---|---|---|---|
| `src/smart_hub_interfaces` | ament_cmake, rosidl | M5 (shared contract) | (Impl) all 14 msg, 6 srv, 3 action |
| `src/smart_hub` | ament_python | everyone (one package, §19.1) | M1-A, M1-B, M2-B, M3 and M5 nodes present; M2-B/M3 moved in from the repository root (`git mv`, history kept) |

Ownership per node follows the role document (7-person split):
M1-A `passenger_detector`, M1-B `demand_predictor`, M2-A `bus_manager`,
M2-B `hub_coordinator`, M3 `parking_manager` + `traffic_monitor`,
M4 `route_planner` + `robot_manager`, M5 `scenario_runner`,
`safety_supervisor`, `actuator_bridge`, `metrics_logger`,
`dashboard_marker_publisher` (§4).

## 2. Identifiers

| Item | Value | Source |
|---|---|---|
| Robot id | `robot_1` | (Prop) used by M5 and M2-B; no other spelling exists in any branch |
| Bus id | `bus_1` | (Prop) used by M5, M2-B, M3; no other spelling exists |
| Car ids | unique strings, e.g. `car_1` | (Open) M3/M2-A; only uniqueness matters to M5 |
| `VehicleState.kind` | `robot`, `bus`, `car`, `obstacle` | `string` field (§6), no enum exists. `robot`/`bus`/`car` used by M2-B; `obstacle` is M3's assumption, implemented by M5 (Impl) |
| `RequestCrossing.requester_id` | must equal `VehicleState.vehicle_id` | (Prop) M2-B README Q1; the coordinator and supervisor match on it |
| Queue zone | `zone_id: Q` | M1-B README (Impl) |
| Graph nodes | `A Cs J E N Cn B U R W D P1 P2 P3` | (A) §2.1, §3.1 |
| Edge ids | node names sorted, joined by `-`: `Cs-J`, `E-J`, `B-Cn` | (Prop) M3's `hub_map.edge_id`; M5 `geometry.edge_id` identical (Impl) |
| Frames | `map`; `robot/odom`, `robot/base_link`; bus equivalents | (A) §5; map→odom is identity in the simulator |
| Units | metres, seconds, radians, m/s | (A) §2.1, integration rule 1 |

## 3. Topics

Rates and QoS are (A) §5 unless noted. R*n* = reliable, keep-last depth *n*,
volatile; T = transient-local. "Status" is for this branch.

| Topic | Type | Rate / QoS | Publisher (owner) | Subscribers | Status |
|---|---|---|---|---|---|
| `/camera/image_raw` | sensor_msgs/Image | 10 Hz, sensor | scenario_runner synthetic camera (M5) or `video_publisher` (M1-A) | passenger_detector | (Impl) bgr8 640×480; freeze modes `camera`, `camera_repeat` |
| `/hub/queue` | QueueState | 5 Hz, R10 | passenger_detector (M1-A) | demand_predictor, coordinator, safety | (Impl) stamp = frame time; invalid ⇒ count 0, `crossing_occupied` true; M1-B `fake_queue_publisher` remains a test stand-in |
| `/hub/demand` | DemandPrediction | 1 Hz, R10 | demand_predictor (M1-B) | coordinator, dashboard | (Impl) |
| `/hub/vehicles` | VehicleStateArray | 10 Hz, R10 | scenario_runner (M5) | bus_manager, traffic_monitor, safety, coordinator, dashboard | (Impl) observed state; `valid` per entry |
| `/sim/truth` | VehicleStateArray | 10 Hz, R10 | scenario_runner (M5) | metrics_logger, tests | (Impl) M5-internal, private ground truth (§3.2) |
| `/robot/odom` | nav_msgs/Odometry | 20 Hz, R10 | scenario_runner (M5) | robot_manager, safety | (Impl) |
| `/hub/bus_arrival` | BusArrival | 2 Hz, R10 | bus_manager (M2-A) | coordinator | (Impl) `state: IDLE` when free (M2-B README Q4) |
| `/parking/observed` | SlotObservationArray | 5 Hz, R10 | scenario_runner (M5) | parking_manager | (Impl) offline slot → `valid: false` |
| `/parking/state` | ParkingState | 2 Hz + change, R1T | parking_manager (M3) | coordinator, dashboard | (Impl) runs in `hub.launch.py` |
| `/traffic/edges` | EdgeStateArray | 2 Hz, R10 | traffic_monitor (M3) | route_planner, coordinator, dashboard | (Impl) runs in `hub.launch.py`; invalid ⇒ speed/density −1.0 |
| `/crossing/state` | CrossingState | 10 Hz, R1T | hub_coordinator (M2-B) | managers, safety, dashboard | (Impl) runs in `hub.launch.py`; states `FREE GRANTED HOLD OCCUPIED PEDESTRIAN UNKNOWN` |
| `/robot/cmd_vel_candidate` | geometry_msgs/TwistStamped | 20 Hz, R1 | robot_manager (M4) | safety_supervisor **only** | (Impl) zero while idle |
| `/bus/cmd_vel_candidate` | geometry_msgs/TwistStamped | 20 Hz, R1 | bus_manager (M2-A) | safety_supervisor **only** | (Impl) zero while idle |
| `/robot/cmd_vel`, `/bus/cmd_vel` | geometry_msgs/Twist | 20 Hz, R1 | **safety_supervisor only** (M5) | actuator_bridge | (Impl) |
| `/robot/cmd_vel_applied`, `/bus/cmd_vel_applied` | geometry_msgs/Twist | 20 Hz, R1 | actuator_bridge (M5) | scenario_runner | (Impl) M5-internal (Prop) |
| `/bus/door_request` | std_msgs/Bool | R1 | bus_manager (M2-A) | actuator_bridge | (Impl) 20 Hz, true only while docked/boarding |
| `/bus/door_open` | std_msgs/Bool | 10 Hz, R1 | actuator_bridge (M5) | bus_manager, safety | (Impl) measured state |
| `/parking/barrier_request` / `_open` | std_msgs/Bool | R1 | parking_manager (M3) / actuator_bridge (M5) | bridge / parking_manager | (A) §11.3; bridge mirrors request (sim) |
| `/hub/estop` | std_msgs/Bool | event + 10 Hz, R1T | operator / scenario_runner | safety_supervisor, metrics | (Impl) see §5 |
| `/hub/safety` | SafetyState | 10 Hz, R1T | **safety_supervisor** (M5) | every manager, coordinator, dashboard | (Impl) see §4 |
| `/hub/events` | HubEvent | on change, R100 | every decision node | metrics_logger, dashboard | (Impl) for M5, M1-B, M2-B, M3 |
| `/hub/markers` | visualization_msgs/MarkerArray | 5 Hz, R10 | dashboard_marker_publisher (M5) | RViz | (Impl) read-only |
| `/robot/route`, `/bus/route` | nav_msgs/Path | on every new/changed plan, R1T | robot_manager (M4) / bus_manager (M2-A) | dashboard_marker_publisher, RViz | **Decided by M5** (no assignment topic exists): frame `map`; poses = remaining waypoints current → goal; empty Path = no route. Dashboard side (Impl) |
| `/scenario/inject` | std_msgs/String (JSON) | R10 | tests only | scenario_runner | (Impl) **test-only**, not part of the hub |
| `/traffic/edge_labels` | std_msgs/String (JSON) | 2 Hz, R10 | traffic_monitor (M3) | dashboard (future) | M3 additive topic, not in §5 |
| `/parking/entry_claim`, `/parking/entrance_clear` | String, Bool | — | (Open) | parking_manager | M3 additive inputs; no publisher decided |

## 4. Services and actions (A) §7

| Name | Type | Server (owner) | Clients | Status |
|---|---|---|---|---|
| `/parking/reserve_slot` | ReserveParking | parking_manager (M3) | coordinator / car client | (Impl) |
| `/parking/release_slot` | ReleaseParking | parking_manager (M3) | same | (Impl) |
| `/route/compute` | ComputeRoute | route_planner (M4) | bus_manager, robot_manager | (Impl); clients: 1 s timeout, ≤ 2 retries, then explicit failure (T18); `NO_ROUTE` when no permitted path |
| `/crossing/request` | RequestCrossing | hub_coordinator (M2-B) | bus_manager, robot_manager | (Impl); `requester_id` `bus_1` / `robot_1` |
| `/scenario/reset` | ResetScenario | scenario_runner (M5) | test orchestration | (Impl) see §7 |
| `/safety/clear_stop` | ClearSafetyStop | safety_supervisor (M5) | operator | (Impl) see §6 |
| `/robot/confirm_ready` | std_srvs/Trigger | robot_manager (M4) | button / dashboard | (Impl) accepted only in WAIT_READY |
| `/bus/dock` | DockBus action | bus_manager (M2-A) | coordinator | (Impl); bay B only; `final_state` COMPLETE or SAFE_STOP |
| `/robot/escort` | EscortPassenger action | robot_manager (M4) | coordinator | (Impl); trigger at t=70 s is still (Open) (M2-B Q3) |
| `/robot/navigate_to` | NavigateTo action | robot_manager (M4) | internal, tests | (Impl) |

Service rules (A) §7.1: normal response ≤ 250 ms, client timeout 1 s,
retry with the same request id; services never wait for physical completion.

## 5. Safety state: who decides and how others must react

**Authority.** `safety_supervisor` is the only node that publishes
`/robot/cmd_vel` and `/bus/cmd_vel` (A, §4.2 "Only safety_supervisor
publishes final motor velocity"), and the only publisher of `/hub/safety`.

**`SafetyState` meaning (Impl).**

| Field | Meaning |
|---|---|
| `header.stamp` | publication time |
| `stop_latched` | a latched stop is active; only `/safety/clear_stop` removes it |
| `motion_permitted` | **system-wide** flag: false while latched, while the e-stop signal is stale, or while `/hub/vehicles` is stale. Per-agent holds do not clear it |
| `reason` | `OK`, or every active reason separated by `; `, e.g. `LATCHED:ESTOP; robot:COMMAND_STALE; bus:DOOR_OPEN` |

**Per-vehicle hazards.** `SafetyState` has no agent field (A, §6), so
vehicle-specific holds appear in `reason` as `<agent>:<REASON>[/<REASON>]`
with `agent` = `robot` or `bus`. A per-agent field would change the
interface lock — (Open) for the whole team; nothing needs it yet.

**Rate, QoS, freshness.** 10 Hz plus immediately on every latch change,
reliable + transient-local depth 1 (A §5). **Consumers must treat a
`/hub/safety` older than 1.0 s, never received, or with an unset stamp as
"stop latched"** — never as permission (Prop; M2-B's coordinator already
does exactly this with `freshness_sec = 1.0`). Transient-local means a
restarted node immediately receives the last state; the 1 s age check
stops it from trusting that state if the supervisor has died.

**How others react to a stop (A §7.2, §15.2):**
- managers (M2-A, M4): stop publishing nonzero candidates, abort the running
  action (`abort()` "for a latched safety stop"), and do not restart until
  `stop_latched` is false **and** a new goal arrives;
- coordinator (M2-B): no new crossing grants (Impl: `SAFETY_STOP` denial verified);
- the supervisor zeroes outputs regardless of what managers do.

## 6. Emergency stop, timeouts and limits

| Rule | Value | Source |
|---|---|---|
| Safety loop | 20 Hz, monotonic clock | (A) §16.2 |
| Speed caps robot / bus | 0.25 / 0.40 m/s | (A) §3.3 |
| Candidate command stale | > 0.5 s → zero output (hold) | (A) §3.3, §16.1 |
| Odometry / perception stale | > 1.0 s → zero output (hold) | (A) §3.3, §16.1 |
| Future-stamp tolerance | reject stamps > 0.2 s ahead | (A) §5 "declared tolerance"; value (Prop) |
| Unset header stamp | never fresh | (Prop) same rule as M2-B |
| Stop-command latency | ≤ 0.20 s, receipt → first zero | (A) §3.3, §21.3, T16 |
| `/hub/estop` | `true` latches; `false` never clears | (A) §5, §7.1 |
| E-stop heartbeat | published at 10 Hz; none for 1.0 s → everything held (`ESTOP_SIGNAL_STALE`) | 10 Hz (A) §5; the 1.0 s hold (Prop) |
| Latching faults | e-stop, footprint outside the site, footprint overlap | (A) "latch when required" §18.7; the list is (Prop) |
| Non-latching holds | stale command/odometry/vehicles, door open/unknown, clearance, crossing not granted/unknown, unknown body | (Prop) |
| Bridge watchdog | final command > 0.5 s old → zero, independently | (A) §16.2, T17; value (Prop) |
| Simulator drive timeout | applied command > 0.5 s old → body stops | (Prop), third layer |
| Clearance | 0.20 m outside footprints, 0.5 s look-ahead | 0.20 m (A) §3.3; look-ahead (Prop) |
| Turn-rate caps robot / bus | 1.0 / 0.5 rad/s | (Prop), not in assignment |

## 7. Reset rules

**`/safety/clear_stop` (ClearSafetyStop)** (A §7.1, §16.3, §18.7 + Impl).
Rejected (`success: false`, `reason` lists *every* failed condition,
`;`-separated) while any of these holds:

| Condition | Reason token |
|---|---|
| e-stop input still true | `ESTOP_ASSERTED` |
| e-stop heartbeat stale | `ESTOP_SIGNAL_STALE` |
| `/hub/vehicles` stale | `VEHICLES_STALE` |
| robot odometry / bus pose stale or invalid | `robot:ODOM_STALE`, `bus:POSE_STALE` |
| measured speed > 0.02 m/s | `<agent>:MOVING` |
| a latching hazard is still present | `BOUNDARY:<agent>`, `COLLISION:<agent>-<id>` |
| a manager still commands nonzero motion | `<agent>:COMMAND_NONZERO` |

Success returns `CLEARED:<latch reasons>`; with nothing latched it returns
`success: true, reason: NO_LATCHED_STOP`. Clearing only re-enables
readiness: the service publishes no motion, and the robot moves again only
when a manager sends a new nonzero candidate.

*Zero-motion tolerance:* the assignment gives no reset tolerance. 0.02 m/s is
its "stopped" threshold for docking (§10.2, §18.6), reused here (Prop).

**`/scenario/reset` (ResetScenario)** (A §7.1 + Impl): reloads
`scenarios/<scenario_id>.yaml` with the seed, restores the initial world with
every body at rest, and returns `success`, a new `run_id`
(`<scenario>-s<seed>-r<n>`) and a reason. A failed reset (unknown scenario)
returns `success: false` and leaves the running world unchanged. It does
**not** clear the safety latch (that is `/safety/clear_stop`'s job).
**Reset handshake — decided by M5** (A §7.1 "returns a new run identifier
after all nodes acknowledge reset"; no new message type, it reuses
`HubEvent` on `/hub/events`):

1. `scenario_runner` loads and installs the new world (bodies at rest), then
   publishes `event: SCENARIO_RESET_REQUEST`, `correlation_id: <new run id>`.
2. Every node in the runner's `reset_participants` parameter resets its own
   state **once per run id** and publishes `event: RESET_ACK`,
   `entity_id: <its node name>`, `correlation_id: <same run id>`
   (`smart_hub.ros_common.reset_ack()`). A repeated request for the same run
   id is only re-acknowledged.
3. `/hub/events` is volatile, so the runner re-sends the request every
   0.25 s until all acks arrive or `reset_ack_timeout` (2.0 s) passes.
4. The service answers `success: true`, `reason: RESET … acks=<n>`; or
   `success: false`, `reason: MISSING_ACK: <names>` (the new world is
   running, the named nodes did not confirm); acks for another run id never
   count. Then it publishes `SCENARIO_RESET`.

Participants today (Impl): `safety_supervisor` (forgets all commands and
samples so every agent is held until fresh data, **keeps the stop latch** —
a reset is not a way around `/safety/clear_stop`), `actuator_bridge` (drops
commands, closes the door), `metrics_logger` (flushes the old run),
`dashboard_marker_publisher` (clears its picture), and with
`perception:=true` `passenger_detector` (forgets tracks). The service runs
as a coroutine in its own callback group, so the 20 Hz tick keeps running
while it waits. Other members join by following README §6.1.

## 8. Freshness rule for everyone (A §5, §6)

Stamp state messages with the **observation** time, not publication time;
judge freshness by both the stamp and local receipt time; an invalid or
low-confidence observation is UNKNOWN, never zero; a retained
(transient-local) sample is not necessarily fresh.

## 9. Open decisions (who must answer)

| # | Decision | Owners |
|---|---|---|
| 1 | Who starts the escort at t = 70 s in S8 | M2-B, M4, M5 |
| 2 | ~~Acknowledgement mechanism for `/scenario/reset`~~ | **decided by M5**, see §7 |
| 2b | ~~Topic for planned routes~~ | **decided by M5**: `/robot/route`, `/bus/route` (`nav_msgs/Path`), §3 |
| 3 | Per-agent field in `SafetyState` (currently in `reason` only) | all (interface change) |
| 4 | Publishers of `/parking/entry_claim`, `/parking/entrance_clear` | M3, M5 |
| 5 | Car id format | M3, M2-A |
| 6 | ~~Moving M2-B's and M3's files into `src/smart_hub`~~ | **done** (git mv, owners informed) |
| 7 | ~~M1-B restyle for `ament_flake8`~~ | **done** (`feature/m1b-style-reset`) |
| 8 | Grading split (7 roles vs the assignment's 5) | team + instructor (M2-B README Q7) |

## 10. Changing this contract

1. Propose the change in the team chat, naming every affected owner.
2. Change `.msg/.srv/.action` files only after all affected owners agree
   (integration rule 1), in one commit that also updates this file.
3. Rebuild and run `docker/hub.sh test` before merging into the integration branch.
