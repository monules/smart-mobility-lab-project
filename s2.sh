#!/usr/bin/env bash
# Helper for running the S2 demand predictor in Docker (macOS / Linux / WSL).
#
#   ./s2.sh build            build the ROS 2 Jazzy image (first time: ~5 min)
#   ./s2.sh demo [seed]      run fake /hub/queue + demand_predictor (Ctrl+C stops)
#   ./s2.sh shell            open a 2nd terminal inside the running demo
#   ./s2.sh test             colcon test (all 46 unit tests incl. T03 run)
#   ./s2.sh train            retrain + T03 metrics/plots -> ./output/
#   ./s2.sh evidence [seed]  120 s run: bag + topic rate/QoS -> ./output/
set -euo pipefail

IMAGE=s2-demand
NAME=s2
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/output"
ROS="source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash"

cmd="${1:-help}"
seed="${2:-1025}"

case "$cmd" in
  build)
    docker build -t "$IMAGE" "$HERE"
    ;;
  demo)
    docker run --rm -it --name "$NAME" "$IMAGE" bash -c \
      "$ROS && ros2 launch smart_hub demand_demo.launch.py seed:=$seed run_id:=demo-$seed"
    ;;
  shell)
    docker exec -it "$NAME" bash
    ;;
  test)
    docker run --rm "$IMAGE" bash -c \
      "$ROS && cd /ws && colcon test --packages-select smart_hub \
         --event-handlers console_direct+ && colcon test-result --verbose"
    ;;
  train)
    mkdir -p "$OUT"
    docker run --rm -v "$OUT:/out" "$IMAGE" bash -c \
      "cd /ws/src/smart_hub && python3 scripts/train_demand_model.py \
         --out-dir /out --model-out /out/demand_ridge_h30.json"
    echo "Results in $OUT"
    ;;
  evidence)
    mkdir -p "$OUT"
    rm -rf "$OUT/s2_bag_$seed"
    docker run --rm -v "$OUT:/out" "$IMAGE" bash -c "
      $ROS
      ros2 launch smart_hub demand_demo.launch.py seed:=$seed \
        run_id:=evidence-$seed > /out/s2_launch_$seed.log 2>&1 &
      LPID=\$!
      sleep 3
      ros2 topic info /hub/demand --verbose > /out/s2_topic_info.txt
      (timeout -s INT 20 ros2 topic hz /hub/demand > /out/s2_demand_hz.txt 2>&1 || true) &
      timeout -s INT 120 ros2 bag record -o /out/s2_bag_$seed \
        /hub/queue /hub/demand /hub/events || true
      ros2 bag info /out/s2_bag_$seed > /out/s2_bag_info.txt
      kill -INT \$LPID 2>/dev/null || true
      wait \$LPID 2>/dev/null || true"
    echo "Evidence in $OUT (bag, launch log, topic info, rate)"
    ;;
  *)
    sed -n '2,9p' "$0"
    ;;
esac
