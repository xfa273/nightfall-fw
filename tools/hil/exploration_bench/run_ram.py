#!/usr/bin/env python3
"""Run the reviewed non-motor ELF in F413 SRAM; never write internal Flash.

The application is restarted afterwards. Volatile application state is lost.
Internal Flash (including identity) is read and hashed before/after. No UART,
FRAM, erase, option-byte or application-programming commands are issued.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import time

CLI = Path('/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/'
           'STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
RAM = 0x20000000
MAILBOX = 0x2004C000
FLASH = 0x08000000
FLASH_BYTES = 1536 * 1024
HEADER_FIELDS = 'magic version status row_count cpu_hz rcc_cr rcc_pllcfgr rcc_cfgr cfsr hfsr fault_pc reserved'.split()
ROW_FIELDS = 'case_id status cycles_low cycles_high max_slice_cycles slices workspace_bytes goal_entry_us stop_us expanded required_edges checksum'.split()


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--elf', type=Path, required=True)
    p.add_argument('--serial', required=True, help='exact ST-LINK serial')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--timeout', type=float, default=180)
    p.add_argument('--run', action='store_true', help='execute non-motor RAM benchmark and reset application')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    elf = a.elf.resolve()
    nm = subprocess.check_output(['arm-none-eabi-nm', '--defined-only', str(elf)], text=True)
    symbols = {m[2]: int(m[0], 16) for line in nm.splitlines()
               if len(m := line.split()) == 3}
    if symbols.get('nf_bench_output') != MAILBOX or not RAM <= symbols.get('bench_entry', 0) < MAILBOX:
        raise ValueError('ELF must be the reviewed exploration benchmark with SRAM entry and fixed mailbox')
    binary = a.output / 'benchmark.bin'
    subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(elf), str(binary)], check=True)
    if not 64 <= binary.stat().st_size < MAILBOX - RAM:
        raise ValueError('load image exceeds SRAM region')
    here = Path(__file__).resolve().parent
    prepare_elf = a.output / 'prepare.elf'
    prepare_bin = a.output / 'prepare.bin'
    subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m4', '-mthumb', '-Os',
                    '-ffreestanding', '-nostdlib', '-T', str(here / 'prepare.ld'),
                    str(here / 'prepare.c'), '-o', str(prepare_elf)], check=True)
    subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(prepare_elf), str(prepare_bin)], check=True)
    report = {'elf': str(elf), 'elf_sha256': sha(elf), 'serial': a.serial,
              'entry': symbols['bench_entry'], 'binary_bytes': binary.stat().st_size,
              'motors_allowed': False, 'flash_write': False, 'fram_access': False}
    if not a.run:
        print(json.dumps(report | {'dry_run': True}, indent=2))
        return
    log = a.output / 'cubeprogrammer.log'

    def cube(*args):
        cmd = [str(CLI), '-c', 'port=SWD', 'mode=HOTPLUG', 'sn=' + a.serial, *map(str, args)]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        out = ANSI.sub('', result.stdout + result.stderr)
        with log.open('a') as f:
            f.write('\n' + ' '.join(cmd) + '\n' + out)
        if result.returncode or re.search(r'\bError:', out):
            raise RuntimeError(out)
        return out

    def read32(address):
        out = cube('-r32', hex(address), 4)
        m = re.search(rf'0x{address:08X}\s*:\s*([0-9a-f]+)', out, re.I)
        if not m:
            raise RuntimeError('Cannot parse SWD read: ' + out)
        return int(m[1], 16)

    # Refuse to interrupt a driving machine, even if its application differs.
    if read32(0xE0042000) & 0xFFF != 0x463:
        raise RuntimeError('Only the verified STM32F413 target is supported')
    for address in [0x40000034, 0x4000003C, 0x40014434]:
        if read32(address):
            raise RuntimeError('Motor/fan compare is nonzero; leave target untouched')
    if read32(0x40020414) & (1 << 2):
        raise RuntimeError('Motor standby is enabled; leave target untouched')
    pll = read32(0x40023804)
    cfgr = read32(0x40023808)
    if (pll & 0x007F7FFF) != 0x3210 or (cfgr & 0xFC) != 8:
        raise RuntimeError('Expected HSI PLL M16/N200/P2, AHB /1 (100 MHz)')
    before = a.output / 'flash-before.bin'
    after = a.output / 'flash-after.bin'
    cube('-u', hex(FLASH), FLASH_BYTES, before.resolve())
    report['flash_before_sha256'] = sha(before)
    altered = False
    try:
        altered = True
        cube('-halt')
        # This small first stage lies above all production DMA buffers. It
        # disables peripherals before the larger image replaces their RAM.
        cube('-w', prepare_bin.resolve(), '0x2004E000', '-v')
        cube('-coreReg', 'MSP=0x20050000', 'PC=0x2004E000',
             'XPSR=0x01000000', 'PRIMASK=1', 'CONTROL=0', '-run')
        if read32(MAILBOX + 8) != 0x50524550:
            raise RuntimeError('Peripheral-quiesce first stage did not complete')
        cube('-halt')
        cube('-w', binary.resolve(), hex(RAM), '-v')
        cube('-coreReg', 'MSP=' + hex(MAILBOX), 'PC=' + hex(symbols['bench_entry']),
             'XPSR=0x01000000', 'PRIMASK=1', 'CONTROL=0', '-run')
        deadline = time.monotonic() + a.timeout
        while time.monotonic() < deadline:
            status = read32(MAILBOX + 8)
            if status in (2, 0xFFFFFFFF):
                break
            time.sleep(0.25)
        else:
            raise TimeoutError('RAM benchmark deadline expired')
        result_bin = a.output / 'mailbox.bin'
        cube('-u', hex(MAILBOX), 48 + 64 * 48, result_bin.resolve())
        data = result_bin.read_bytes()
        header = dict(zip(HEADER_FIELDS, struct.unpack_from('<12I', data)))
        if header['magic'] != 0x4E464542 or header['version'] != 1 or header['row_count'] > 64:
            raise RuntimeError('Invalid benchmark mailbox')
        report['header'] = header
        report['rows'] = []
        for i in range(header['row_count']):
            row = dict(zip(ROW_FIELDS, struct.unpack_from('<12I', data, 48 + i * 48)))
            row['cycles'] = (row['cycles_high'] << 32) | row['cycles_low']
            row['elapsed_ms'] = row['cycles'] * 1000 / header['cpu_hz']
            row['max_slice_ms'] = row['max_slice_cycles'] * 1000 / header['cpu_hz']
            report['rows'].append(row)
        if status != 2:
            raise RuntimeError('Benchmark fault: ' + json.dumps(header))
    finally:
        if altered:
            cube('-rst')
            report['application_reset'] = True
        cube('-u', hex(FLASH), FLASH_BYTES, after.resolve())
        report['flash_after_sha256'] = sha(after)
        report['flash_unchanged'] = report['flash_before_sha256'] == report['flash_after_sha256']
        (a.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        if not report['flash_unchanged']:
            raise RuntimeError('Flash digest changed unexpectedly')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
