#!/usr/bin/env python3
"""Fail the optional ARM preview build if static memory eats the stack margin."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--size-tool', required=True)
p.add_argument('--elf', type=Path, required=True)
p.add_argument('--bin', type=Path, required=True)
a = p.parse_args()
report = subprocess.check_output([a.size_tool, '-A', str(a.elf)], text=True)
ram = ccm = 0
for line in report.splitlines():
    columns = line.split()
    if len(columns) != 3 or not columns[0].startswith('.'):
        continue
    size, address = map(int, columns[1:])
    if 0x20000000 <= address < 0x20020000:
        ram += size
    elif 0x10000000 <= address < 0x10010000:
        ccm += size
image_size = a.bin.stat().st_size
# Existing linker reservation includes a 1 KiB minimum stack. Keep another
# 8 KiB available; .su files document the planner's static call frames.
if not 0 < ram <= 128 * 1024 - 8 * 1024:
    raise SystemExit(f'FAIL SRAM stack margin: {ram} bytes used')
if ccm > 64 * 1024:
    raise SystemExit(f'FAIL CCMRAM: {ccm} bytes used')
if image_size > 0xA0000:
    raise SystemExit(f'FAIL F405 sector 9 calibration boundary: {image_size} bytes')
print(f'PASS memory: RAM={ram}/131072 CCM={ccm}/65536 '
      f'extra_stack_margin={131072-ram} image={image_size}/655360 bytes')
