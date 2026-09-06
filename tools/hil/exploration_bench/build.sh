#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
kind=${1:-smoke}
out=${2:-build/exploration_mcu/$kind.elf}
mkdir -p "$(dirname "$out")"
sources=(tools/hil/exploration_bench/boot.c)
flags=(-Itools/hil/exploration_bench -Icommon/exploration -Icommon/route)
case "$kind" in
  smoke) sources+=(tools/hil/exploration_bench/smoke.c) ;;
  policy) sources+=(tools/hil/exploration_bench/policy_bench.c common/exploration/exploration_policy.c) ;;
  planner)
    : "${3:?planner requires a generated benchmark case header as argument 3}"
    fixture=$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$3")
    flags+=("-DNF_BENCH_CASES_HEADER=\"$fixture\"")
    if [[ -n "${4:-}" ]]; then flags+=("-DNF_BENCH_CASE_INDEX=$4"); fi
    sources+=(tools/hil/exploration_bench/planner_bench.c common/route/mcu_slalom_time_planner.c common/route/mcu_slalom_tables.c)
    ;;
  *) echo "Usage: $0 {smoke|policy|planner} [output.elf] [fixture.h] [case-index]" >&2; exit 2 ;;
esac
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -O2 -g -ffreestanding -fno-builtin -nostdlib -Wall -Wextra -Werror \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -T tools/hil/exploration_bench/ram.ld "${flags[@]}" "${sources[@]}" -o "$out" -lgcc
arm-none-eabi-size "$out"
