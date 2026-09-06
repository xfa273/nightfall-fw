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
  adapter|trace)
    # Compile the actual adapter source (included by the test translation
    # unit), using only header declarations from the production HAL/CMSIS.
    # No dependency on a pre-existing CMake build or compile_commands.json.
    f413_root=platform/stm32f413/HM_Nightfall_f413_preorder
    flags+=(-DSTM32F413xx -DUSE_HAL_DRIVER -DNIGHTFALL_F413_EXPLORATION_ENABLED=1
      # The SRAM image also holds normally-Flash code/tables. This fixture is
      # 16x16; a 192 KiB lease is ample and avoids overlapping the fixed mailbox.
      '-DNF_BENCH_WORKSPACE_BYTES=(NF_MCU_SLALOM_WORKSPACE_BYTES-8U*1024U)'
      -Invm -Iparams/f413_preorder -Iplatform/trace -Iplatform/stm32f405/Core/Inc
      "-I$f413_root/Core/Inc"
      "-I$f413_root/Drivers/STM32F4xx_HAL_Driver/Inc"
      "-I$f413_root/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
      "-I$f413_root/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
      "-I$f413_root/Drivers/CMSIS/Include")
    if [[ "$kind" == trace ]]; then
      sources+=(tools/hil/exploration_bench/trace_lease_bench.c)
    else
      sources+=(tools/hil/exploration_bench/adapter_bench.c
        common/exploration/exploration_policy.c
        common/route/mcu_slalom_time_planner.c common/route/mcu_slalom_tables.c)
    fi
    ;;
  planner)
    : "${3:?planner requires a generated benchmark case header as argument 3}"
    fixture=$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$3")
    flags+=("-DNF_BENCH_CASES_HEADER=\"$fixture\"")
    if [[ -n "${4:-}" ]]; then flags+=("-DNF_BENCH_CASE_INDEX=$4"); fi
    sources+=(tools/hil/exploration_bench/planner_bench.c common/route/mcu_slalom_time_planner.c common/route/mcu_slalom_tables.c)
    ;;
  *) echo "Usage: $0 {smoke|policy|adapter|trace|planner} [output.elf] [fixture.h] [case-index]" >&2; exit 2 ;;
esac
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -O2 -g -ffreestanding -fno-builtin -nostdlib -Wall -Wextra -Werror \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -T tools/hil/exploration_bench/ram.ld "${flags[@]}" "${sources[@]}" -o "$out" \
  --specs=nano.specs -lc -lgcc
arm-none-eabi-size "$out"
