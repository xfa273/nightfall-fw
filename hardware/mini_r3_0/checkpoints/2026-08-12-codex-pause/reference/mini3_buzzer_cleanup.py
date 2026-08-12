#!/usr/bin/env python3
import sys
import pcbnew


def mm(v):
    return round(pcbnew.ToMM(v), 6)


b = pcbnew.LoadBoard(sys.argv[1])
for item in list(b.GetTracks()):
    if item.GetNetname() != '/BUZZER_PWM':
        continue
    if isinstance(item, pcbnew.PCB_VIA):
        p = item.GetPosition()
        if abs(mm(p.x) - 149.05) < 0.001 and abs(mm(p.y) - 111.45) < 0.001:
            b.Remove(item)
    elif mm(item.GetWidth()) < 0.19:
        b.Remove(item)
if not pcbnew.SaveBoard(sys.argv[2], b):
    raise SystemExit('save failed')
