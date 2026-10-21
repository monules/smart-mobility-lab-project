#!/usr/bin/env bash
# Smart Mobility Hub: build, test and run inside the ROS 2 Jazzy container.
#
#   docker/hub.sh image          build the dev image (once, ~3 min)
#   docker/hub.sh build          colcon build every package under src/
#   docker/hub.sh test           colcon test + result summary
#   docker/hub.sh s9 [seed]      run the S9 fault demo (launch, ~20 s)
#   docker/hub.sh s1 [blob|hog]  run the S1 passenger-counting check (~45 s)
#   docker/hub.sh shell          interactive shell, workspace sourced
#   docker/hub.sh run '<cmd>'    run one command with the workspace sourced
#
# The repository is mounted at /ws (no copy), so edits on the Mac are seen
# immediately. build/ install/ log/ appear in the repo root; they are in
# .gitignore. Builds run one package and two compile jobs at a time because
# Docker Desktop on an 8 GB Mac has about 4 GB for Linux.
set -euo pipefail

IMAGE=smart-hub-dev:jazzy
REPO="$(cd "$(dirname "$0")/.." && pwd)"
COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
# Evidence must not claim a clean commit when the code differs from it
# (untracked files count: new nodes are untracked until committed).
DIRTY="$(git -C "$REPO" status --porcelain -- src docker 2>/dev/null | head -c1)"
[ -n "$DIRTY" ] && COMMIT="$COMMIT-dirty"

SETUP="source /opt/ros/jazzy/setup.bash && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; }"
BUILD="MAKEFLAGS=-j2 colcon build --symlink-install --base-paths src --parallel-workers 1"

in_container() {
  # $1 = extra docker flags (e.g. -it), $2 = bash command
  docker run --rm $1 -v "$REPO:/ws" -w /ws \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-37}" -e GIT_COMMIT="$COMMIT" \
    "$IMAGE" bash -c "$SETUP || exit 1
$2"   # newline, not &&: a trailing '&' in $2 must not background the setup
}

case "${1:-help}" in
  image) docker build -t "$IMAGE" "$REPO/docker" ;;
  build) in_container "" "$BUILD" ;;
  test)
    in_container "" "colcon test --base-paths src --parallel-workers 1 \
      --event-handlers console_direct+ ; colcon test-result --verbose" ;;
  s9)
    # Evidence (assignment 17, 20.2): rosbag of the safety topics, parameter
    # dump of every M5 node, metrics and s9_check.json, all under output/.
    # ros2 launch exits 0 even when the check fails, so the verdict is read
    # from the evidence file: no file or passed=false -> nonzero exit.
    SEED="${2:-9}"
    in_container "" "set -m
      mkdir -p output/params && rm -rf output/s9_check.json output/bag_S9_seed$SEED
      ros2 bag record -o output/bag_S9_seed$SEED --topics /hub/estop /hub/safety \
        /hub/events /robot/cmd_vel_candidate /robot/cmd_vel /robot/cmd_vel_applied \
        /bus/cmd_vel_candidate /bus/cmd_vel /robot/odom /hub/vehicles /sim/truth \
        > output/bag_record.log 2>&1 &
      BAG=\$!
      sleep 2
      ros2 launch smart_hub s9.launch.py seed:=$SEED output_dir:=/ws/output &
      LAUNCH=\$!
      sleep 5
      for n in scenario_runner safety_supervisor actuator_bridge metrics_logger \
          dashboard_marker_publisher; do
        ros2 param dump /\$n > output/params/\$n.yaml 2>/dev/null
      done
      wait \$LAUNCH
      kill -INT \$BAG; wait \$BAG
      ros2 bag info output/bag_S9_seed$SEED > output/bag_S9_info.txt 2>&1
      python3 -c \"import json, sys; r = json.load(open('output/s9_check.json')); \
print('S9 verdict:', 'PASSED' if r['passed'] else 'FAILED'); sys.exit(0 if r['passed'] else 1)\"" ;;
  s1)
    # S1 passenger counting (M1-A): detector blob (default) or hog.
    in_container "" "mkdir -p output && rm -f output/s1_check.json && \
      ros2 launch smart_hub s1.launch.py detector:=${2:-blob} output_dir:=/ws/output; \
      python3 -c \"import json, sys; r = json.load(open('output/s1_check.json')); \
print('S1 verdict:', 'PASSED' if r['passed'] else 'FAILED'); sys.exit(0 if r['passed'] else 1)\"" ;;
  shell) in_container "-it" "exec bash" ;;
  run) in_container "" "${2:?usage: docker/hub.sh run '<command>'}" ;;
  *) sed -n '2,10p' "$0" ;;
esac
