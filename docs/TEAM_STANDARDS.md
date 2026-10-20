# Team standards (short version)

Copy this into the team chat. Full details and sources:
[INTEGRATION_CONTRACT.md](INTEGRATION_CONTRACT.md).
**(A)** = required by the assignment; everything else is a default the
team can change by agreement.

```text
SMART MOBILITY HUB — INTEGRATION DEFAULTS
1. IDs: robot_1, bus_1. VehicleState.kind is one of robot | bus | car | obstacle.
   RequestCrossing.requester_id = your VehicleState.vehicle_id.
2. Edge ids: node names sorted + "-" (Cs-J, E-J, B-Cn). Frames: map, robot/odom, robot/base_link.
3. Interfaces: use ONLY smart_hub_interfaces. Never add a second copy of a .msg/.srv/.action.
   Changing a field needs every affected owner's OK first. (A: interface lock)
4. Motion: managers publish /robot|bus/cmd_vel_candidate (TwistStamped, 20 Hz, stamped NOW,
   frame = base_link). Only safety_supervisor publishes /robot|bus/cmd_vel. (A)
5. Safety: read /hub/safety (SafetyState, 10 Hz, reliable + transient-local).
   Older than 1.0 s, never received or unstamped  ==> behave as STOP LATCHED.
   stop_latched=true ==> stop candidates, abort your action, wait for a NEW goal. (A for abort)
6. E-stop: /hub/estop true latches (A). false never clears it (A). Only /safety/clear_stop
   clears, and only when: e-stop false, sensors fresh, no hazard, robot+bus stopped
   (<= 0.02 m/s) and no manager still commanding motion.
7. Freshness: stamp = observation time; invalid/low-confidence = UNKNOWN, never 0 (A).
   Command stale after 0.5 s, sensors after 1.0 s (A).
8. Limits: robot 0.25 m/s, bus 0.40 m/s (A).
9. Events: publish /hub/events (HubEvent) on transitions only, with a correlation_id (A).
10. Code: your node in src/smart_hub/smart_hub/<name>.py, logic in a pure-Python module with
   unit tests, console script in setup.py, tests in src/smart_hub/test/.
   Build + test in Docker: docker/hub.sh build && docker/hub.sh test. All green before merging.
11. Git: own feature branch; never force-push; never commit build/ install/ log/ output/ or
   the assignment PDFs; merge into integration/current-team-work, not main.
```

## Why each rule exists

| Rule | Reason |
|---|---|
| Fixed ids and kinds | The coordinator finds who is inside crossing C and the supervisor finds the bus pose by matching `vehicle_id`; a different spelling silently means "unknown agent" |
| Sorted edge ids | M3's traffic monitor and M5's simulator produce `edge_id` independently; both must spell `E-J` the same way |
| One interface package | Two copies of a message produce incompatible types at runtime |
| Candidates only | §4.2: a dashboard or manager must never bypass the safety arbitration |
| Stale safety = stop | A transient-local sample from a crashed supervisor would otherwise look valid forever |
| E-stop latch | §7.1: "A false stop message never automatically clears the latch" |
| UNKNOWN ≠ 0 | §6.1: a stale zero is not an empty queue/road |
| Docker build/test before merge | §Git rule 4: "If colcon build fails on your machine, it will break everyone else's laptop too" |

## Needs a team decision (not decided by these defaults)

See [INTEGRATION_CONTRACT.md §9](INTEGRATION_CONTRACT.md#9-open-decisions-who-must-answer):
escort trigger at t = 70 s, reset acknowledgements, a per-agent field in
`SafetyState`, the parking entrance topics, car id format, moving M2-B/M3
files into the package, and M1-B's lint restyle.
