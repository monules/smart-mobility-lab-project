# M2-B Hub Coordinator (Arkar) - S8

> **Update (integration branch):** the files below now live in
> `src/smart_hub/smart_hub/` and `src/smart_hub/test/` (moved with `git mv`,
> code unchanged), `hub_coordinator` is a console script, and
> `hub.launch.py` starts it. The table below is kept for history.

> **Update 2 (branch `feature/m2b-t04-dispatch`, by Saidumar/M5):**
> - **T04 passes live, 6/6** — `docker/hub.sh t04`, proof in
>   `evidence/m2b/t04/`. The checker (`t04_check.py`) plays the forecaster
>   and the queue sensor; your real coordinator and the real bus_manager do
>   the rest.
> - Two changes in `hub_coordinator.py`: (1) new parameter
>   `demand_freshness_sec` = 2.5 s, because `/hub/demand` comes at 1 Hz and
>   with the shared 1.0 s limit a forecast goes stale exactly when the next one is due (found by review);
>   (2) `main()` no longer prints `RCLError: rcl_shutdown already called` on
>   Ctrl+C (catches the shutdown and calls `rclpy.shutdown()` only if
>   `rclpy.ok()`).
> - Q4 is answered: bus_manager publishes `state: IDLE` at 2 Hz when free.
> - Still yours: reset handshake, restyle.
>
> **Update 3 (branch `feature/s8-integration`, by Saidumar/M5):**
> - **Question 3 answered — the escort trigger:** the kiosk (in simulation:
>   `scenario_runner`) publishes `ESCORT_REQUEST` on `/hub/events` with JSON
>   `{request_id, passenger_id, pickup, destination}`; your coordinator now
>   listens (`_on_hub_event`) and calls your `send_escort()`.
> - **Reactive mode added** (your "still missing" list): parameter
>   `dispatch_mode` = `predictive` (forecast, else FALLBACK on the observed
>   queue) or `reactive` (observed queue only). S8/T15 compares both on the
>   same ten seeds — `docker/hub.sh s8`.
> - **Result: T15 PASSED** — 20/20 episodes with 0 collisions, 0 crossing
>   conflicts, 0 double allocations, 0 duplicate dispatches; every mission
>   completed; predictive dispatch on average 1.4 s earlier than reactive
>   (`evidence/s8/`). Read `s8_check.py` and the S8 section of the report.

## Files and where they go in the repo
| File | Goes to |
|---|---|
| `coordinator_core.py` | `smart_hub/smart_hub/coordinator_core.py` |
| `hub_coordinator.py` | `smart_hub/smart_hub/hub_coordinator.py` |
| `test_coordinator_core.py` | `smart_hub/tests/` (change the import to `from smart_hub.coordinator_core import ...`) |

Add the console script in `setup.py`: `'hub_coordinator = smart_hub.hub_coordinator:main'`.
Branch: `feature/m2b-coordinator`. Don't push to `main` until `colcon build` passes on your laptop.

## What it does
- **`/crossing/request`** (RequestCrossing): ACQUIRE / RENEW / RELEASE, one owner token at a time.
  - Denies on: pedestrian in C, unknown/stale state, safety stop, someone else owns it, C not clear for 2.0 s, not first in FIFO.
  - Lease expiry never frees an occupied crossing: the owner goes to HOLD and is only reclaimed after C is observed clear for 2.0 s.
  - RELEASE only works once the owner's footprint is outside C.
- **`/crossing/state`** at 10 Hz (R1 transient-local) and **`/hub/events`** (R100) for every grant, release, hold, denial change, dispatch and result.
- **Bus dispatch** at 1 Hz: 3 consecutive forecasts >= 6, bus IDLE, no outstanding goal, 30 s cooldown, escalation clears after 5 forecasts < 4. If the forecast is invalid it falls back to the fresh observed count (reason `FALLBACK`). One `DockBus` goal at a time, and late replies are ignored by correlation id.
- **Escort client**: `send_escort()` exists, but nothing calls it yet (see question 3).

## Questions to settle with the team (interface lock)
1. `requester_id` in `RequestCrossing` must equal `VehicleState.vehicle_id` (e.g. `bus_1`, `robot_1`), otherwise I can't tell who is inside C. `VehicleState.kind` must be `bus`, `car` or `robot`. (M2-A, M4, M5)
2. Lease is 3 s, so bus and robot must send RENEW about every 1 s while crossing. (M2-A, M4)
3. Who triggers the escort at t = 70 s? The assignment gives no interface for it. (M5 / scenario_runner)
4. `/hub/bus_arrival.state` must be exactly `IDLE` when the bus is free, published at 2 Hz. (M2-A)
5. If any vehicle in `/hub/vehicles` is `valid=false`, I treat the crossing as UNKNOWN and deny entry. (M5, M3)
6. Dispatch rule: escalation starts after 3 samples >= 6 and clears after 5 samples < 4. While escalated, a new bus is sent only if the current signal is still >= 6, no goal is outstanding, the bus is IDLE and 30 s have passed. This is Tuymurod's rule (`demand/dispatch_policy.py`) plus one check: his version could send a bus on a falling queue before escalation cleared. Tell him, because his paired predictive-vs-reactive comparison should use the same rule.
7. The doc splits the project across 7 roles but the assignment assumes 5 members. Confirm the grading split with the instructor.

## Verified vs not verified
- **Verified:** `coordinator_core.py`, 28 unit tests passing (plain `unittest`; pytest runs them too).
- **Not verified:** `hub_coordinator.py` has not been run. ROS 2 was not available when it was written, so check message field names against the real interface files, then build and run it.
- **Not done yet (your evidence):** T04 and T15 results, `ros2 bag` captures, the concurrent-request test over real services, and the S8 timeline run. Those need the other nodes.

## Repo state (checked 18 Oct)
- `smart_hub_interfaces` on `feature/m1b-demand-predictor` is TEMPORARY (only QueueState, DemandPrediction, HubEvent). Its comment says M5 owns the full package. Don't push a second version; agree who writes it. I still need the other messages/services/actions from it: CrossingState, SafetyState, BusArrival, VehicleStateArray, RequestCrossing, DockBus, EscortPassenger.
- The package layout is `src/smart_hub/` (ament_python) and `src/smart_hub_interfaces/` (ament_cmake). Put my files in `src/smart_hub/smart_hub/` and add the `hub_coordinator` console script to its `setup.py`.
- Still missing in the coordinator: a reactive mode (needed for the S8 predictive-vs-reactive comparison; use observed count only).

## Mock testing without teammates
Publish fake `/hub/queue`, `/hub/vehicles`, `/hub/safety` (stop_latched=false), `/hub/bus_arrival` (state IDLE) and `/hub/demand` (predicted_count 7, valid true) with correct header stamps, then call:
```
ros2 service call /crossing/request smart_hub_interfaces/srv/RequestCrossing "{requester_id: 'bus_1', operation: 'ACQUIRE', token: ''}"
```

## For your defense
Be ready to explain: why the crossing is a service and docking an action; what happens on a service timeout (the same-token ACQUIRE retry); why lease expiry goes to HOLD and not FREE; how the dispatch counters reset on invalid input; and which callback group everything runs in (default mutually exclusive, so no locks needed).
