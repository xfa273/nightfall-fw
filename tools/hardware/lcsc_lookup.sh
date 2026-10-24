#!/bin/bash
# LCSC part-number lookup helper (uses jina reader because LCSC search is JS-only).
#
#   ./tools/hardware/lcsc_lookup.sh "0402WGF1001TCE" "CL05A105KA5NNNC" ...
#
# Prints candidate LCSC codes (Cxxxxx), manufacturer part name and rough stock
# for every query, spaced to stay under the reader rate limit.
set -u
OUTDIR="${LCSC_LOOKUP_CACHE:-/tmp/lcsc}"
mkdir -p "$OUTDIR"
LOG="$OUTDIR/lookup_$(date +%Y%m%d_%H%M%S).log"
: > "$LOG"
for q in "$@"; do
  slug=$(printf '%s' "$q" | tr -c 'A-Za-z0-9' '_')
  page="$OUTDIR/raw_${slug}.md"
  curl -sL --max-time 60 -H 'User-Agent: Mozilla/5.0' \
    "https://r.jina.ai/https://www.lcsc.com/search?q=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$q")" -o "$page"
  {
    echo "### QUERY: $q"
    python3 - "$page" <<'PYEOF'
import re,sys
data=open(sys.argv[1],encoding='utf-8',errors='replace').read()
seen=set()
for line in data.splitlines():
    codes=re.findall(r'product-detail/(C\d+)\.html', line)
    if not codes:
        continue
    mp=re.search(r'\[([^\]\|\[]+)\]\(https://www\.lcsc\.com/product-detail/'+re.escape(codes[0])+r'\.html', line)
    stock=re.search(r'([\d,]+)\s+In Stock', line)
    basic='BASIC' if 'Basic Parts' in line.replace('False','') else ''
    price=re.search(r'\d+\+\$([\d.]+)', line)
    for c in codes:
        if c in seen:
            continue
        seen.add(c)
        print(f"  {c} | {mp.group(1) if mp else '?'} | stock={stock.group(1) if stock else '?'} | {basic} | {('$'+price.group(1)) if price else ''}")
    if len(seen)>=8:
        break
if not seen:
    print('  (no result rows parsed)')
PYEOF
  } >> "$LOG"
  sleep 12
done
cat "$LOG"
