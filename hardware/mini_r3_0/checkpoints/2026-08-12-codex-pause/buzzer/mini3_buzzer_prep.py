#!/usr/bin/env python3
import sys
import pcbnew


def mm(value):
    return pcbnew.FromMM(value)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def xy(point):
    return (round(pcbnew.ToMM(point.x), 6), round(pcbnew.ToMM(point.y), 6))


def add_track(board, net, layer, start, end, width):
    track = pcbnew.PCB_TRACK(board)
    track.SetNet(net)
    track.SetLayer(layer)
    track.SetStart(pt(*start))
    track.SetEnd(pt(*end))
    track.SetWidth(mm(width))
    board.Add(track)


source, output = sys.argv[1:3]
via_x = float(sys.argv[3]) if len(sys.argv) > 3 else 149.25
via_y = float(sys.argv[4]) if len(sys.argv) > 4 else 111.50
board = pcbnew.LoadBoard(source)
net = board.FindNet('/SPI2_MOSI')
if net is None:
    raise SystemExit('missing /SPI2_MOSI')

# Move only the local MOSI bottom-side spine to In2.  The far rear segment
# remains on B.Cu and the two existing through vias join the sections.
for item in list(board.GetTracks()):
    if isinstance(item, pcbnew.PCB_VIA) or item.GetNetname() != '/SPI2_MOSI':
        continue
    if item.GetLayer() != pcbnew.B_Cu:
        continue
    a, b = xy(item.GetStart()), xy(item.GetEnd())
    if abs(a[1] - 106.9751) < 0.0002 or abs(b[1] - 106.9751) < 0.0002:
        continue
    board.Remove(item)

mosi = [
    ((149.65, 117.30), (149.65, 116.00)),
    ((149.65, 116.00), (149.65, 114.50)),
    ((149.65, 114.50), (149.65, 113.00)),
    ((149.65, 113.00), (149.65, 112.70)),
    ((149.65, 112.70), (149.65, 111.00)),
    ((149.65, 111.00), (149.65, 109.70)),
    ((149.65, 109.70), (149.30, 109.30)),
    ((149.30, 109.30), (149.00, 108.80)),
    ((149.00, 108.80), (148.7254, 107.9960)),
]
for start, end in mosi:
    add_track(board, net, pcbnew.In2_Cu, start, end, 0.20)

# Only the QFN toe transition is allowed to use the 0.40/0.20 via.
buzzer = board.FindNet('/BUZZER_PWM')
via = pcbnew.PCB_VIA(board)
via.SetNet(buzzer)
via.SetPosition(pt(via_x, via_y))
via.SetWidth(mm(0.40))
via.SetDrill(mm(0.20))
via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
board.Add(via)
add_track(board, buzzer, pcbnew.F_Cu,
          (149.739949, 110.986275),
          ((149.739949 + via_x) / 2.0, (110.986275 + via_y) / 2.0), 0.16)
add_track(board, buzzer, pcbnew.F_Cu,
          ((149.739949 + via_x) / 2.0, (110.986275 + via_y) / 2.0),
          (via_x, via_y), 0.16)

if not pcbnew.SaveBoard(output, board):
    raise SystemExit('save failed')
