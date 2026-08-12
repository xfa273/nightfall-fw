"""KiCad Action Plugin that draws footprints by component type."""

from __future__ import annotations

import re

import pcbnew

try:
    import wx
except ImportError:  # Allows host-side testing with only the pcbnew module.
    wx = None


PLUGIN_NAME = "Component Type Outlines"
GROUP_NAME = "__component_type_outlines__"
LINE_WIDTH_MM = 0.10
PADDING_MM = 0.15

LAYER_NAMES = {
    pcbnew.User_1: "R / Resistor",
    pcbnew.User_2: "C / Capacitor",
    pcbnew.User_3: "LED",
    pcbnew.User_4: "Other / IC",
}

CATEGORY_LAYERS = {
    "resistor": pcbnew.User_1,
    "capacitor": pcbnew.User_2,
    "led": pcbnew.User_3,
    "other": pcbnew.User_4,
}


def classify_footprint(footprint: pcbnew.FOOTPRINT) -> str:
    """Return a visual category using reference first, then LED metadata."""

    reference = str(footprint.GetReference()).strip().upper()
    value = str(footprint.GetValue()).strip().upper()
    footprint_name = str(footprint.GetFPID().GetLibItemName()).strip().upper()

    if re.match(r"^LED(?=\d|[_?])", reference):
        return "led"

    # Some schematics use D1-style references for LEDs.  Do not color ordinary
    # diodes as LEDs unless their value or footprint explicitly says LED.
    if re.match(r"^D(?=\d|[_?])", reference) and "LED" in (
        value + " " + footprint_name
    ):
        return "led"

    if re.match(r"^R(?=\d|[_?])", reference):
        return "resistor"

    if re.match(r"^C(?=\d|[_?])", reference):
        return "capacitor"

    return "other"


def _footprint_box(footprint: pcbnew.FOOTPRINT) -> pcbnew.BOX2I:
    courtyard_layer = pcbnew.B_CrtYd if footprint.IsFlipped() else pcbnew.F_CrtYd
    courtyard = footprint.GetCourtyard(courtyard_layer)

    if courtyard.OutlineCount() > 0:
        box = courtyard.BBox()
    else:
        box = footprint.GetBoundingBox(False)

    box.Inflate(pcbnew.FromMM(PADDING_MM))
    return box


def _remove_previous_outlines(board: pcbnew.BOARD) -> int:
    removed = 0

    for group in list(board.Groups()):
        if str(group.GetName()) != GROUP_NAME:
            continue

        for item in list(group.GetItems()):
            group.RemoveItem(item)
            board.Delete(item)
            removed += 1

        board.Delete(group)

    return removed


def regenerate_outlines(board: pcbnew.BOARD) -> dict[str, int]:
    """Replace this plugin's generated rectangles and return category counts."""

    _remove_previous_outlines(board)

    for layer, name in LAYER_NAMES.items():
        board.SetLayerName(layer, name)

    group = pcbnew.PCB_GROUP(board)
    group.SetName(GROUP_NAME)
    group.SetLocked(True)
    board.Add(group)

    counts = {category: 0 for category in CATEGORY_LAYERS}
    line_width = pcbnew.FromMM(LINE_WIDTH_MM)

    for footprint in board.GetFootprints():
        category = classify_footprint(footprint)
        layer = CATEGORY_LAYERS[category]
        box = _footprint_box(footprint)

        if box.GetWidth() <= 0 or box.GetHeight() <= 0:
            continue

        outline = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_RECT)
        outline.SetLayer(layer)
        outline.SetStart(pcbnew.VECTOR2I(box.GetLeft(), box.GetTop()))
        outline.SetEnd(pcbnew.VECTOR2I(box.GetRight(), box.GetBottom()))
        outline.SetWidth(line_width)
        outline.SetLocked(True)

        board.Add(outline)
        group.AddItem(outline)
        counts[category] += 1

    return counts


class ComponentTypeOutlines(pcbnew.ActionPlugin):
    def defaults(self) -> None:
        self.name = PLUGIN_NAME
        self.category = "Layout helpers"
        self.description = (
            "Draw footprint outlines on User.1-User.4, classified as "
            "resistor, capacitor, LED, or other."
        )
        self.show_toolbar_button = True

    def Run(self) -> None:  # noqa: N802 - KiCad ActionPlugin API name
        board = pcbnew.GetBoard()

        if board is None:
            self._show_message("PCBが開かれていません。", error=True)
            return

        try:
            counts = regenerate_outlines(board)
            pcbnew.Refresh()
        except Exception as exc:  # Keep failures visible inside KiCad.
            self._show_message(f"アウトライン生成に失敗しました。\n\n{exc}", error=True)
            raise

        self._show_message(
            "部品種類別アウトラインを更新しました。\n\n"
            f"抵抗: {counts['resistor']}\n"
            f"コンデンサ: {counts['capacitor']}\n"
            f"LED: {counts['led']}\n"
            f"その他: {counts['other']}\n\n"
            "再実行すると既存の生成枠を置き換えます。"
        )

    @staticmethod
    def _show_message(message: str, error: bool = False) -> None:
        if wx is None:
            return

        style = wx.OK | (wx.ICON_ERROR if error else wx.ICON_INFORMATION)
        wx.MessageBox(message, PLUGIN_NAME, style)
