# Running S2 on a Mac (Apple Silicon)

ROS 2 Jazzy does not install natively on macOS, so it runs inside Docker
(a small Ubuntu 24.04 + ROS 2 Jazzy container). Option B needs no ROS at all.

## Option A — full ROS 2 run in Docker (recommended)

**One-time setup**

1. Install **Docker Desktop for Mac (Apple chip)** from docker.com and open it
   once (wait until the whale icon says "running").
2. Unzip `smart_hub_ws.zip`, e.g. into `~/workspaces/`.
3. In Terminal:

```bash
cd ~/workspaces/smart_hub_ws
chmod +x s2.sh
./s2.sh build          # downloads ROS 2 Jazzy + builds both packages (~5 min first time)
```

**Run the demo**

```bash
./s2.sh demo           # synthetic /hub/queue -> demand_predictor
```

You will see about 60 s of `forecast INVALID (WARMUP)` and then
`forecast valid (OK)`. Open a **second Terminal tab** to watch the topic:

```bash
cd ~/workspaces/smart_hub_ws
./s2.sh shell
ros2 topic echo /hub/demand       # predicted_count, baseline_count, model_id, valid
ros2 topic hz /hub/demand         # ~1.0 Hz
ros2 topic echo /hub/events       # MODEL_LOADED, DEMAND_VALID / DEMAND_INVALID
```

Press **Ctrl+C** in the first tab to stop. `./s2.sh demo 1044` replays a
different episode (any number works; 1044 is a fast ramp-up, 1004 has a
frozen-camera outage).

**Tests, T03 and evidence**

```bash
./s2.sh test           # colcon test: 46 tests incl. an end-to-end T03 training run
./s2.sh train          # retrain; metrics.json, CSVs, plots -> ./output/
./s2.sh evidence       # 2-minute bag of /hub/queue,/hub/demand,/hub/events
                       # + topic QoS info + measured rate -> ./output/
```

## Option B — Python only, no ROS (training, T03 numbers, tests)

```bash
cd ~/workspaces/smart_hub_ws/src/smart_hub
python3 -m venv .venv && source .venv/bin/activate
pip install numpy scikit-learn matplotlib pytest
python3 scripts/train_demand_model.py    # T03 table + plots in results/
python3 -m pytest test -q                # 46 passed
```

Use `python3 -m pytest` (not bare `pytest`) so the package is importable.
This does not run the ROS node.

## If something goes wrong

| Symptom | Fix |
|---|---|
| `Cannot connect to the Docker daemon` | Docker Desktop is not running: open it and wait |
| `./s2.sh: Permission denied` | `chmod +x s2.sh` |
| `No such container: s2` (from `./s2.sh shell`) | start `./s2.sh demo` in the other tab first |
| `Conflict. The container name "/s2" is already in use` | `docker rm -f s2` |
| Changed code doesn't show up | rerun `./s2.sh build` (code is copied into the image) |
| Forecast stays `STALE_INPUT` | `/hub/queue` isn't arriving: check `ros2 topic hz /hub/queue` |

## Temporary interfaces package

`src/smart_hub_interfaces/` holds **only** `QueueState`, `DemandPrediction`
and `HubEvent`, copied field-for-field from assignment section 6, so S2 can
run before the team package exists. When M5's full `smart_hub_interfaces` is
ready, delete this folder and put theirs in `src/`; no S2 code changes.
