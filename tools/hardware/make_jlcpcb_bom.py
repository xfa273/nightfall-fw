#!/usr/bin/env python3
"""Build JLCPCB PCBA BOM + placement (CPL) files for the mini_r2_0 boards.

Usage:
    tools/hardware/make_jlcpcb_bom.py MAP_CSV PCB_KICAD_PCB OUT_PREFIX

Parses the kicad_pcb footprints (reference, value, footprint, position,
rotation, layer, SMT/through-hole) and maps them through the LCSC mapping
table (hardware/mini_r2_0/bom/lcsc_map.csv) into:

    OUT_PREFIX_bom.csv   JLCPCB upload BOM: Comment, Designator(s), Quantity, LCSC Part #
    OUT_PREFIX_cpl.csv   JLCPCB upload CPL (SMT parts only): Designator, Mid X,
                         Mid Y, Rotation, Layer
    OUT_PREFIX_tht.csv   Through-hole parts excluded from the CPL (hand/pin
                         insertion): same columns

Test points (refs starting with TP or EC) are skipped.  Rows without an LCSC
code get an empty part number, i.e. JLCPCB treats them as customer-supplied
parts; those lines are also printed on stderr as a reminder.
"""
import csv
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kicad_pcb_extract import parse_sexp, get_prop, find


def fp_base(name):
    """Drop the trailing '_<digits>' export suffix added by the Eagle->KiCad converter."""
    return re.sub(r'_\d+$', '', name)


def load_map(path):
    entries = []
    with open(path, encoding='utf-8') as fh:
        for row in csv.reader(l for l in fh if not l.lstrip().startswith('#')):
            if not row or row[0] == 'ref_pattern':
                continue
            row = (row + [''] * 7)[:6] + [','.join(row[6:])]
            entries.append(row)
    return entries


def match_entry(entries, ref, value, footprint):
    for ref_pat, value_pat, fp_pat, mpn, lcsc, status, note in entries:
        if ref_pat and not re.search(ref_pat, ref):
            continue
        if value_pat and value_pat != value:
            continue
        if fp_pat and fp_pat not in footprint:
            continue
        return dict(mpn=mpn, lcsc=lcsc, status=status, note=note)
    return None


def collect(pcb_path, entries):
    tree = parse_sexp(open(pcb_path, encoding='utf-8').read())
    parts, unmapped = [], []
    for fp in find(tree, 'footprint'):
        ref = get_prop(fp, 'Reference') or ''
        value = (get_prop(fp, 'Value') or '').strip()
        footprint = fp[1]
        if re.match(r'^(TP\d|EC_)', ref):
            continue
        at = next((e for e in fp if isinstance(e, list) and e[0] == 'at'), ['at', '0', '0', '0'])
        x, y, rot = float(at[1]), float(at[2]), float(at[3] if len(at) > 3 else 0)
        layer = next((e[1] for e in fp if isinstance(e, list) and e[0] == 'layer'), 'F.Cu')
        side = 'Front' if layer == 'F.Cu' else 'Bottom'
        th = any(len(p) > 2 and p[2] == 'thru_hole' for p in find(fp, 'pad'))
        entry = match_entry(entries, ref, value, fp_base(footprint))
        if entry is None:
            unmapped.append((ref, value, footprint))
            continue
        parts.append(dict(ref=ref, mpn=entry['mpn'] or value or ref, code=entry['lcsc'],
                          status=entry['status'], note=entry['note'], x=x, y=y,
                          rot=rot % 360.0, side=side, th=th))
    parts.sort(key=lambda p: p['ref'])
    return parts, unmapped


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    map_path, pcb_path, out_prefix = sys.argv[1:]
    parts, unmapped = collect(pcb_path, load_map(map_path))

    groups = {}
    for p in parts:
        groups.setdefault((p['mpn'], p['code']), []).append(p)

    with open(out_prefix + '_bom.csv', 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['Comment', 'Designator(s)', 'Quantity', 'LCSC Part #'])
        for (mpn, code), rows in sorted(groups.items()):
            w.writerow([mpn, ','.join(sorted(r['ref'] for r in rows)), len(rows), code])

    for suffix, rows in (('_cpl.csv', [p for p in parts if not p['th']]),
                         ('_tht.csv', [p for p in parts if p['th']])):
        with open(out_prefix + suffix, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['Designator', 'Mid X', 'Mid Y', 'Rotation', 'Layer'])
            for p in rows:
                w.writerow([p['ref'], f"{p['x']:.4f}", f"{p['y']:.4f}", f"{p['rot']:.2f}", p['side']])

    print(f'{len(parts)} parts -> {len(groups)} BOM lines ({sum(1 for g in groups if not g[1])} without LCSC code)')
    for p in parts:
        if not p['code']:
            print(f'  NO-CODE {p["ref"]:12} {p["mpn"]:22} [{p["status"]}] {p["note"]}')
    for u in sorted(unmapped):
        print(f'  UNMAPPED {u}')


if __name__ == '__main__':
    main()
