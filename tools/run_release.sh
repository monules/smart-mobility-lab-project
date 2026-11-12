#!/usr/bin/env bash
# Re-run every scenario and test on ONE commit and file the results in evidence/.
#
#   tools/run_release.sh            everything (about 2.5 hours)
#   tools/run_release.sh quick      the same with 2 S8 seeds, 2 dockings, 2 escorts
#
# Why: the midterm evidence was collected over several commits. The review
# asked for one release whose results all come from the same source tree.
# docker/hub.sh stamps each result with the commit (and "-dirty" if src/ or
# docker/ differ from it), so do not edit or commit code while this runs.
# Each step's console output is kept in output/release/<step>.log, and the
# script stops at the first failing step: a failed check is investigated,
# never skipped. Old S8 episode and dashboard folders are moved to
# output/release/replaced/ (not deleted) so that no stale file stays in
# evidence/.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HUB="$ROOT/docker/hub.sh"
EV="$ROOT/evidence"
OUT="$ROOT/output"
LOG="$OUT/release"
QUICK="${1:-}"
SEEDS=10; DOCKS=10; ESCORTS=10
[ "$QUICK" = quick ] && { SEEDS=2; DOCKS=2; ESCORTS=2; }
mkdir -p "$LOG/replaced"

step() {   # step <name> <command...>: run, log, stop on failure
  local name="$1"; shift
  printf '%s  %-14s start\n' "$(date +%H:%M:%S)" "$name"
  if "$@" > "$LOG/$name.log" 2>&1; then
    printf '%s  %-14s ok\n' "$(date +%H:%M:%S)" "$name"
  else
    printf '%s  %-14s FAILED (see output/release/%s.log)\n' "$(date +%H:%M:%S)" "$name" "$name"
    exit 1
  fi
}
put() {    # put <evidence subdir> <files under output/...>
  local dest="$EV/$1"; shift
  mkdir -p "$dest"
  for f in "$@"; do cp -Rf "$OUT/$f" "$dest/"; done
}

git -C "$ROOT" diff --quiet HEAD -- src docker || { echo "src/ or docker/ has uncommitted changes"; exit 1; }
echo "release commit: $(git -C "$ROOT" rev-parse --short HEAD)  (seeds $SEEDS, dockings $DOCKS, escorts $ESCORTS)"

step build "$HUB" build
step tests "$HUB" test
grep -E '^test/|^=+ .*(passed|failed)|^Summary' "$LOG/tests.log" > "$EV/tests/colcon_test_summary.txt"

step s1_blob "$HUB" s1 blob
put m1a/s1_blob s1_check.json s1_count_vs_truth.png s1_timeseries.csv
step s1_hog "$HUB" s1 hog
put m1a/s1_hog s1_check.json s1_count_vs_truth.png

step s3 "$HUB" s3 "$DOCKS"
put m2a/s3 s3_check.json s3_events.csv s3_paths.png s3_trace.csv s3_trials.csv
step t04 "$HUB" t04
put m2b/t04 t04_check.json t04_timeline.csv
step s4 "$HUB" s4
put m3/s4 s4_check.json
step s5 "$HUB" s5
put m3/s5 s5_check.json s5_labels.png s5_timeseries.csv
step s6 "$HUB" s6
put m4/s6 s6_check.json s6_maps.png s6_routes.csv
step s7 "$HUB" s7 "$ESCORTS"
put m4/s7 s7_check.json s7_events.csv s7_paths.png s7_trace.csv s7_trials.csv
step t18 "$HUB" t18
put m4/t18 t18_check.json

step s9 "$HUB" s9
put m5/s9 s9_check.json params bag_S9_seed9 bag_S9_info.txt
cp -f "$OUT"/S9-s9-r1/events.csv "$OUT"/S9-s9-r1/summary.json "$EV/m5/s9/"
step s9_repeat "$HUB" s9x
put m5/s9_repeat repeat/s9_repeat.json repeat/s9_repeat.csv

step s8 "$HUB" s8 "$SEEDS"
for d in episodes dashboard_seed1_predictive; do
  [ -e "$EV/s8/$d" ] && mv "$EV/s8/$d" "$LOG/replaced/s8_${d}_$(date +%H%M%S)"
done
put s8 s8/s8_summary.json s8/s8_runs.csv s8/s8_compare.png s8/bag_seed1_predictive
for mode in predictive reactive; do
  for s in $(seq 1 "$SEEDS"); do
    mkdir -p "$EV/s8/episodes/$mode/seed$s"
    cp -f "$OUT/s8/$mode/seed$s/s8_episode.json" "$OUT/s8/$mode/seed$s/s8_timeline.csv" \
      "$EV/s8/episodes/$mode/seed$s/"
  done
done
DASH="$OUT/s8/predictive/seed1/dashboard"
mkdir -p "$EV/s8/dashboard_seed1_predictive"
cp -f "$DASH/dashboard.mp4" "$EV/s8/dashboard_seed1_predictive/"
ls "$DASH"/frame_*.png | awk 'NR % 25 == 1' | head -8 | while read -r f; do
  cp -f "$f" "$EV/s8/dashboard_seed1_predictive/"
done

# Real lab video: needs the raw video and the detector weights (not in git).
if [ -f "$ROOT/videos/lab/IMG_0572.MOV" ] && [ -f "$ROOT/models/yolov4-tiny/yolov4-tiny.weights" ]; then
  P="--image-points 488 199 1224 207 1248 335 457 330 933 546 1118 550 1173 1018 943 1018 \
--map-points 2 7 6 7 6 5 2 5 4.5 4.75 5.5 4.75 5.5 1.25 4.5 1.25"
  V="--video videos/lab/IMG_0572.MOV --labels evidence/m1a/real_video/labels_husan.csv"
  video() {  # video <name> <plot name or -> <detector args...>
    local name="$1" plot="$2"; shift 2
    step "video_$name" "$HUB" run "ros2 run smart_hub video_eval $V $* --out output/release_video/$name $P"
    cp -f "$OUT/release_video/$name/results.json" "$EV/m1a/real_video/results_$name.json"
    cp -f "$OUT/release_video/$name/per_frame.csv" "$EV/m1a/real_video/per_frame_$name.csv"
    [ "$plot" = - ] || cp -f "$OUT/release_video/$name/count_vs_truth.png" \
      "$EV/m1a/real_video/count_vs_truth_$plot.png"
  }
  video yolo_stabilized yolo --detector yolo --stabilize
  video yolo_not_stabilized - --detector yolo
  video hog_stabilized hog --detector hog --hog-upscale 0.5 --stabilize
else
  echo "real video skipped: videos/lab/IMG_0572.MOV or the YOLO weights are missing"
fi

step release_info "$HUB" run "python3 tools/release_info.py"
echo "ALL STEPS PASSED on $(git -C "$ROOT" rev-parse --short HEAD); evidence/ is updated"
