#!/usr/bin/env bash
# Build the midterm upload set for Team Shenanigans and check the rules from
# Prof. MP's email (Oct 2026):
#   * main report named SME_<TEAMNAME>_PROJECT_Midterm.pdf
#   * source code, datasets and supporting files in ONE zip
#   * the 20-30 min team video uploaded as a file
#   * at most 10 files, each at most 1 GB, NO external links anywhere
#
#   tools/make_submission.sh            build into submission/ and check
#
# Inputs it expects (it says what is missing):
#   docs/report/Smart_Mobility_Hub_Report.pdf   (docker/hub.sh report)
#   videos/final/*.mp4                          the edited team video (exactly one)
#   videos/demo_clips/                          dashboard clips + pictures
# The source zip is made with `git archive` from the committed HEAD, so only
# tracked files go in (no build/, install/, log/, output/, videos/).
set -euo pipefail

TEAM="TeamShenanigans"
BASE="SME_${TEAM}_PROJECT_Midterm"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/submission"
LIMIT=$((1024 * 1024 * 1024))
problems=0

say() { printf '%s\n' "$*"; }
bad() { printf '  ✗ %s\n' "$*"; problems=$((problems + 1)); }
ok() { printf '  ✓ %s\n' "$*"; }

mkdir -p "$OUT"
rm -f "$OUT"/"$BASE"*

say "1. report"
if [ -f "$ROOT/docs/report/Smart_Mobility_Hub_Report.pdf" ]; then
  cp "$ROOT/docs/report/Smart_Mobility_Hub_Report.pdf" "$OUT/$BASE.pdf"
  ok "$BASE.pdf"
else
  bad "docs/report/Smart_Mobility_Hub_Report.pdf missing: run docker/hub.sh report"
fi

say "2. source zip (git HEAD)"
if ! git -C "$ROOT" diff --quiet HEAD -- src docker docs evidence tools README*.md; then
  bad "uncommitted changes in src/docker/docs/evidence/tools: commit first (the zip uses HEAD)"
fi
git -C "$ROOT" archive --format=zip --prefix="${BASE}_Source/" \
  -o "$OUT/${BASE}_Source.zip" HEAD
ok "${BASE}_Source.zip from $(git -C "$ROOT" rev-parse --short HEAD) on $(git -C "$ROOT" branch --show-current)"

say "3. team video"
videos=()
while IFS= read -r -d '' f; do videos+=("$f"); done \
  < <(find "$ROOT/videos/final" -maxdepth 1 -type f -iname '*.mp4' -print0 2>/dev/null)
if [ "${#videos[@]}" -eq 1 ]; then
  cp "${videos[0]}" "$OUT/${BASE}_Video.mp4"
  ok "${BASE}_Video.mp4"
else
  bad "put exactly one edited .mp4 in videos/final/ (found ${#videos[@]})"
fi

say "4. demo clips"
if [ -d "$ROOT/videos/demo_clips" ]; then
  (cd "$ROOT/videos" && zip -qr "$OUT/${BASE}_DemoClips.zip" demo_clips -x '*.DS_Store')
  ok "${BASE}_DemoClips.zip"
else
  bad "videos/demo_clips missing"
fi

say "checks"
n=$(find "$OUT" -maxdepth 1 -type f -name "$BASE*" | wc -l | tr -d ' ')
[ "$n" -le 10 ] && ok "$n files (limit 10)" || bad "$n files (limit 10)"
while IFS= read -r -d '' f; do
  size=$(stat -f %z "$f" 2>/dev/null || stat -c %s "$f")
  [ "$size" -le "$LIMIT" ] || bad "$(basename "$f") is $((size / 1048576)) MB (limit 1024 MB)"
done < <(find "$OUT" -maxdepth 1 -type f -name "$BASE*" -print0)
ok "every file at most 1 GB (sizes below)"

# no external links: report text and every text file in the source zip
if [ -f "$OUT/$BASE.pdf" ]; then
  links=$(python3 - "$OUT/$BASE.pdf" <<'EOF'
import re, sys
try:
    from pypdf import PdfReader
except ImportError:
    print('?pypdf missing'); sys.exit()
text = ''.join(p.extract_text() or '' for p in PdfReader(sys.argv[1]).pages)
found = sorted(set(re.findall(r'(?:https?://|www\.)\S+|\b\S+\.(?:com|io|org|net)/\S*', text)))
print('\n'.join(found))
EOF
)
  if [ -z "$links" ]; then ok "report: no external links"
  elif [ "$links" = "?pypdf missing" ]; then bad "pip install pypdf to check the report for links"
  else bad "report contains links:"; say "$links" | sed 's/^/      /'; fi
  todo=$(python3 - "$OUT/$BASE.pdf" <<'EOF' 2>/dev/null || true
import sys
from pypdf import PdfReader
t = ''.join(p.extract_text() or '' for p in PdfReader(sys.argv[1]).pages)
print(t.count('TODO'), 'DRAFT' in t)
EOF
)
  say "  • report still says TODO $(echo "$todo" | cut -d' ' -f1) times; draft marker: $(echo "$todo" | cut -d' ' -f2) (fine before the final version, fix before uploading)"
fi
# (PDFs are checked by their text above; their internal bytes name the PDF library)
ziplinks=$(cd "$ROOT" && git grep -nIE '(https?://|www\.)' HEAD -- . ':!*.json' ':!*.csv' ':!*.mcap' \
  ':!*.yaml' ':!*.pdf' 2>/dev/null | grep -v 'w3.org/2000/svg' || true)
if [ -z "$ziplinks" ]; then ok "source zip: no external links in text files"
else bad "source zip contains links (remove or reword):"; say "$ziplinks" | head -20 | sed 's/^/      /'; fi

say ""
say "submission/:"
find "$OUT" -maxdepth 1 -type f -name "$BASE*" -exec ls -lh {} \; | awk '{print "  " $5 "\t" $9}' \
  | sed "s#$OUT/##"
if [ "$problems" -eq 0 ]; then
  say "READY: upload these files (team leader), then every member logs in and checks them."
else
  say "NOT READY: $problems problem(s) above."
  exit 1
fi
