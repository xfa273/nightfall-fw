#!/usr/bin/env python3
import argparse
import json
import wx

_app = wx.AppConsole()
import pcbnew


def key(board, track):
    if isinstance(track, pcbnew.PCB_VIA):
        position = track.GetPosition()
        return (
            "v", track.GetNetname(), int(position.x), int(position.y),
            int(track.GetWidth(track.GetLayer())), int(track.GetDrillValue()),
            board.GetLayerName(track.TopLayer()), board.GetLayerName(track.BottomLayer()),
        )
    start = (int(track.GetStartX()), int(track.GetStartY()))
    end = (int(track.GetEndX()), int(track.GetEndY()))
    if end < start:
        start, end = end, start
    return (
        "t", track.GetNetname(), board.GetLayerName(track.GetLayer()), int(track.GetWidth()),
        start[0], start[1], end[0], end[1],
    )


parser = argparse.ArgumentParser()
parser.add_argument("source")
parser.add_argument("output")
parser.add_argument("delta")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.source)
delta = json.load(open(args.delta))
if delta.get("coordinate_unit") != "nm":
    raise RuntimeError("delta is not integer nanometer geometry")
remaining = [tuple(item) for item in delta["removed"]]
matches = []
for track in board.GetTracks():
    item = key(board, track)
    if item in remaining:
        remaining.remove(item)
        matches.append(track)
if remaining:
    raise RuntimeError(f"missing removals ({len(remaining)}): {remaining[:4]}")
for track in matches:
    board.Remove(track)

for item in delta["added"]:
    if item[0] == "v":
        _, net, x, y, diameter, drill, top, bottom = item
        via = pcbnew.PCB_VIA(board)
        via.SetNet(board.FindNet(net))
        via.SetPosition(pcbnew.VECTOR2I(x, y))
        via.SetWidth(diameter)
        via.SetDrill(drill)
        via.SetLayerPair(board.GetLayerID(top), board.GetLayerID(bottom))
        board.Add(via)
    else:
        _, net, layer, width, x1, y1, x2, y2 = item
        track = pcbnew.PCB_TRACK(board)
        track.SetNet(board.FindNet(net))
        track.SetLayer(board.GetLayerID(layer))
        track.SetWidth(width)
        track.SetStart(pcbnew.VECTOR2I(x1, y1))
        track.SetEnd(pcbnew.VECTOR2I(x2, y2))
        board.Add(track)
pcbnew.SaveBoard(args.output, board)
