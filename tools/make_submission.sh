#!/usr/bin/env bash
# Build the FINAL-TERM upload set for Team Shenanigans and check the rules of
# Prof. MP's announcement (Nov 2026):
#   1. main report named VIP_<TEAMNAME>_PROJECT_Finalterm.pdf
#   2. a 20-30 minute video with all members
#   3. source code, datasets and supporting files in ONE zip
#   4. the finalized Team Plan Excel file
#   * keep the midterm files and add the new and revised parts
#   * at most 10 files, each at most 1 GB, NO external links anywhere
#
#   tools/make_submission.sh            build into submission/final/ and check
#
# Inputs (the script says what is missing):
#   docs/report/Smart_Mobility_Hub_Report.pdf   docker/hub.sh report
#   docs/plan/Team_Plan.xlsx                    python3 tools/make_team_plan.py
#   videos/final/final_video.mp4                tools/assemble_video.py
#   videos/demo_clips/                          scenario and UI clips
#   docs/midterm/*.pdf, videos/midterm/*.mp4    the midterm files as submitted
#   videos/lab/IMG_0572.MOV, models/yolov4-tiny the dataset and the detector
# The source zip is `git archive` of the committed HEAD (tracked files only)
# plus the lab video and the detector files, which are too large for git but
# are needed to regenerate the real-video result (midterm review).
set -euo pipefail

TEAM="TeamShenanigans"
BASE="VIP_${TEAM}_PROJECT_Finalterm"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/submission/final"
LIMIT=$((1024 * 1024 * 1024))
problems=0

say() { printf '%s\n' "$*"; }
bad() { printf '  ✗ %s\n' "$*"; problems=$((problems + 1)); }
ok() { printf '  ✓ %s\n' "$*"; }

mkdir -p "$OUT"
# start from an empty folder without deleting anything: old builds are moved aside
if [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then
  mkdir -p "$ROOT/submission/previous"
  mv "$OUT" "$ROOT/submission/previous/final_$(date +%m%d_%H%M%S)"
  mkdir -p "$OUT"
fi

say "1. report"
if [ -f "$ROOT/docs/report/Smart_Mobility_Hub_Report.pdf" ]; then
  cp "$ROOT/docs/report/Smart_Mobility_Hub_Report.pdf" "$OUT/$BASE.pdf"
  ok "$BASE.pdf"
else
  bad "docs/report/Smart_Mobility_Hub_Report.pdf missing: run docker/hub.sh report"
fi

say "2. source + datasets zip (git HEAD + lab video + detector files)"
if ! git -C "$ROOT" diff --quiet HEAD -- src docker docs evidence tools '*.md' LICENSE; then
  bad "uncommitted changes in tracked files: commit first (the zip uses HEAD)"
fi
STAGE="$(mktemp -d)"
git -C "$ROOT" archive --format=tar --prefix="${BASE}_Source/" HEAD | tar -x -C "$STAGE"
for f in videos/lab/IMG_0572.MOV models/yolov4-tiny/yolov4-tiny.cfg \
         models/yolov4-tiny/yolov4-tiny.weights; do
  if [ -f "$ROOT/$f" ]; then
    mkdir -p "$STAGE/${BASE}_Source/$(dirname "$f")"
    cp "$ROOT/$f" "$STAGE/${BASE}_Source/$f"
  else
    bad "$f missing: the real-video result could not be regenerated from the zip"
  fi
done
(cd "$STAGE" && zip -qr "$OUT/${BASE}_Source.zip" "${BASE}_Source" -x '*.DS_Store')
ok "${BASE}_Source.zip from $(git -C "$ROOT" rev-parse --short HEAD) on $(git -C "$ROOT" branch --show-current)"
# the manifest in the zip must match the dataset files that were added
if [ -f "$ROOT/evidence/release/MANIFEST.sha256" ]; then
  want="$(grep 'videos/lab/IMG_0572.MOV' "$ROOT/evidence/release/MANIFEST.sha256" | cut -d' ' -f1)"
  have="$(shasum -a 256 "$ROOT/videos/lab/IMG_0572.MOV" 2>/dev/null | cut -d' ' -f1)"
  [ -n "$want" ] && [ "$want" = "$have" ] && ok "lab video matches the manifest hash" \
    || bad "lab video hash differs from evidence/release/MANIFEST.sha256"
fi

say "3. final video (20-30 minutes)"
if [ -f "$ROOT/videos/final/final_video.mp4" ]; then
  cp "$ROOT/videos/final/final_video.mp4" "$OUT/${BASE}_Video.mp4"
  ok "${BASE}_Video.mp4"
else
  bad "videos/final/final_video.mp4 missing (record the parts, then tools/assemble_video.py)"
fi

say "4. team plan"
if [ -f "$ROOT/docs/plan/Team_Plan.xlsx" ]; then
  cp "$ROOT/docs/plan/Team_Plan.xlsx" "$OUT/${BASE}_TeamPlan.xlsx"
  ok "${BASE}_TeamPlan.xlsx"
else
  bad "docs/plan/Team_Plan.xlsx missing: run tools/make_team_plan.py"
fi

say "5. demo clips"
if [ -d "$ROOT/videos/demo_clips" ]; then
  (cd "$ROOT/videos" && zip -qr "$OUT/${BASE}_DemoClips.zip" demo_clips -x '*.DS_Store')
  ok "${BASE}_DemoClips.zip"
else
  bad "videos/demo_clips missing"
fi

say "6. midterm files, kept as submitted"
for f in "$ROOT"/docs/midterm/*.pdf "$ROOT"/videos/midterm/*.mp4; do
  if [ -f "$f" ]; then cp "$f" "$OUT/"; ok "$(basename "$f")"; else bad "midterm file missing: $f"; fi
done

say "checks"
n=$(find "$OUT" -maxdepth 1 -type f | wc -l | tr -d ' ')
[ "$n" -le 10 ] && ok "$n files (limit 10)" || bad "$n files (limit 10)"
while IFS= read -r -d '' f; do
  size=$(stat -f %z "$f" 2>/dev/null || stat -c %s "$f")
  [ "$size" -le "$LIMIT" ] || bad "$(basename "$f") is $((size / 1048576)) MB (limit 1024 MB)"
done < <(find "$OUT" -maxdepth 1 -type f -print0)
ok "every file at most 1 GB (sizes below)"

# no external links: the report text and every text file of the source zip
if [ -f "$OUT/$BASE.pdf" ]; then
  report=$(python3 - "$OUT/$BASE.pdf" <<'PY'
import re, sys
try:
    from pypdf import PdfReader
except ImportError:
    print('?pypdf'); sys.exit()
text = ''.join(p.extract_text() or '' for p in PdfReader(sys.argv[1]).pages)
links = sorted(set(re.findall(r'(?:https?://|www\.)\S+|\b\S+\.(?:com|io|org|net)/\S*', text)))
links = [x for x in links if not re.match(r'https?://(localhost|127\.0\.0\.1)', x)]
print('links=%s todo=%d draft=%s pages=%d' % (','.join(links), text.count('TODO'),
                                            'DRAFT' in text, len(PdfReader(sys.argv[1]).pages)))
PY
)
  case "$report" in
    '?pypdf') bad "pip install pypdf to check the report" ;;
    links=\ *) ok "report: no external links ($report)" ;;
    *) bad "report check: $report" ;;
  esac
  case "$report" in *"todo=0 draft=False"*) ok "report: no TODO or DRAFT marker" ;;
    '?pypdf') ;; *) bad "report still has a TODO or DRAFT marker" ;; esac
fi
ziplinks=$(cd "$ROOT" && git grep -nIE '(https?://|www\.)' HEAD -- . ':!*.json' ':!*.csv' \
  ':!*.mcap' ':!*.yaml' ':!*.pdf' 2>/dev/null \
  | grep -vE 'w3\.org/2000/svg|https?://(localhost|127\.0\.0\.1)' || true)
# (an XML namespace name and the UI's own address on this machine are not external links)
if [ -z "$ziplinks" ]; then ok "source zip: no external links in text files"
else bad "source zip contains links (remove or reword):"; say "$ziplinks" | head -20 | sed 's/^/      /'; fi

say ""
say "submission/final/:"
find "$OUT" -maxdepth 1 -type f -exec ls -lh {} \; | awk '{print "  " $5 "\t" $9}' | sed "s#$OUT/##"
if [ "$problems" -eq 0 ]; then
  say "READY: upload these files (team leader), then every member logs in and checks them."
else
  say "NOT READY: $problems problem(s) above."
  exit 1
fi
