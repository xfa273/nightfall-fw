#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
board="$project_dir/WeAct-Adapter-5V.kicad_pcb"
output_dir="$project_dir/fabrication/gerbers"
zip_file="$project_dir/fabrication/WeAct-Adapter-5V-gerbers.zip"

if command -v kicad-cli >/dev/null 2>&1; then
    kicad_cli=kicad-cli
elif [ -x /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli ]; then
    kicad_cli=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
else
    echo "kicad-cli not found" >&2
    exit 1
fi

rm -rf "$output_dir"
mkdir -p "$output_dir"

"$kicad_cli" pcb export gerbers \
    --output "$output_dir/" \
    --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,Edge.Cuts \
    --subtract-soldermask \
    "$board"

"$kicad_cli" pcb export drill \
    --output "$output_dir/" \
    --format excellon \
    --excellon-units mm \
    --generate-map \
    --map-format pdf \
    --generate-report \
    --report-path "$output_dir/WeAct-Adapter-5V-drill-report.txt" \
    "$board"

"$kicad_cli" pcb export ipcd356 \
    --output "$output_dir/WeAct-Adapter-5V.ipc" \
    "$board"

"$kicad_cli" pcb export stats \
    --output "$output_dir/WeAct-Adapter-5V-board-stats.txt" \
    --format report \
    --units mm \
    "$board"

rm -f "$zip_file"
(
    cd "$output_dir"
    zip -q "$zip_file" ./*
)

echo "$zip_file"
