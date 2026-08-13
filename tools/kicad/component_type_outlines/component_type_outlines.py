"""KiCad Action Plugin that draws footprints by component type."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import pcbnew

try:
    import wx
except ImportError:  # Allows host-side testing with only the pcbnew module.
    wx = None


PLUGIN_NAME = "Component Type Outlines"
GROUP_NAME = "__component_type_outlines__"
METADATA_FIELD_NAME = "__component_type_outline__"
METADATA_VERSION = "2"
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
    # FP_SHAPE coordinates are stored in the footprint's local coordinate
    # system.  Calculate the box on an orientation-zero copy so a rotated part
    # is not rotated a second time after saving and reopening the board.
    source_items = list(footprint.GraphicalItems())
    _, source_outline = _find_generated_outline(footprint)
    excluded_indices = set()

    if source_outline is not None:
        source_uuid = str(source_outline.m_Uuid.AsString())
        excluded_indices.update(
            index
            for index, item in enumerate(source_items)
            if str(item.m_Uuid.AsString()) == source_uuid
        )

    for group in footprint.Groups():
        if str(group.GetName()) == GROUP_NAME:
            group_uuids = {
                str(item.m_Uuid.AsString()) for item in group.GetItems()
            }
            excluded_indices.update(
                index
                for index, item in enumerate(source_items)
                if str(item.m_Uuid.AsString()) in group_uuids
            )

    normalized = pcbnew.Cast_to_FOOTPRINT(footprint.Duplicate(False))
    normalized_items = list(normalized.GraphicalItems())
    normalized.SetOrientation(pcbnew.EDA_ANGLE(0, pcbnew.DEGREES_T))
    courtyard_layer = pcbnew.B_CrtYd if normalized.IsFlipped() else pcbnew.F_CrtYd
    courtyard = normalized.GetCourtyard(courtyard_layer)

    if courtyard.OutlineCount() > 0:
        box = courtyard.BBox()
    else:
        # GetBoundingBox() would include a previously generated outline and
        # grow on every run.  Merge the real pad/graphic boxes explicitly.
        excluded_uuids = {
            str(field.m_Uuid.AsString()) for field in normalized.GetFields()
        }
        excluded_uuids.update(
            str(normalized_items[index].m_Uuid.AsString())
            for index in excluded_indices
            if index < len(normalized_items)
        )

        body_layers = {
            pcbnew.F_SilkS,
            pcbnew.B_SilkS,
            pcbnew.F_Fab,
            pcbnew.B_Fab,
            pcbnew.F_CrtYd,
            pcbnew.B_CrtYd,
        }
        content_boxes = [pad.GetBoundingBox() for pad in normalized.Pads()]
        content_boxes.extend(
            item.GetBoundingBox()
            for item in normalized.GraphicalItems()
            if str(item.m_Uuid.AsString()) not in excluded_uuids
            and item.GetLayer() in body_layers
        )

        if not content_boxes:
            return pcbnew.BOX2I()

        first = content_boxes[0]
        box = pcbnew.BOX2I(first.GetPosition(), first.GetSize())

        for content_box in content_boxes[1:]:
            box.Merge(content_box)

    box.Inflate(pcbnew.FromMM(PADDING_MM))
    position = normalized.GetPosition()
    box.Move(pcbnew.VECTOR2I(-position.x, -position.y))
    return box


def _delete_generated_group(container, group: pcbnew.PCB_GROUP) -> int:
    removed = 0

    for item in list(group.GetItems()):
        group.RemoveItem(item)
        container.Delete(item)
        removed += 1

    container.Delete(group)
    return removed


def _remove_previous_outlines(board: pcbnew.BOARD) -> int:
    """Remove generated frames during a closed-board file migration."""

    removed = 0

    # Version 1 placed every rectangle directly on the board.  Remove that
    # legacy group once when migrating a board to footprint-owned graphics.
    for group in list(board.Groups()):
        if str(group.GetName()) == GROUP_NAME:
            removed += _delete_generated_group(board, group)

    for footprint in board.GetFootprints():
        # Version 2 stores the generated shape UUID in a hidden footprint
        # property.  This lets us replace only our own graphic without using
        # footprint-contained groups, which KiCad 10 cannot safely refresh.
        if footprint.HasField(METADATA_FIELD_NAME):
            metadata = footprint.GetField(METADATA_FIELD_NAME)
            parts = str(metadata.GetText()).split("|")

            if len(parts) >= 2:
                outline_uuid = parts[1]

                for item in list(footprint.GraphicalItems()):
                    if str(item.m_Uuid.AsString()) == outline_uuid:
                        footprint.Remove(item)
                        removed += 1
                        break

            footprint.Remove(metadata)

        # Remove the short-lived group-based format as well.  It may exist in
        # a board saved by a development build of this plugin.
        for group in list(footprint.Groups()):
            if str(group.GetName()) == GROUP_NAME:
                removed += _delete_generated_group(footprint, group)

    return removed


def _find_generated_outline(footprint: pcbnew.FOOTPRINT):
    if not footprint.HasField(METADATA_FIELD_NAME):
        return None, None

    metadata = footprint.GetField(METADATA_FIELD_NAME)
    parts = str(metadata.GetText()).split("|")

    if len(parts) < 2:
        return metadata, None

    outline_uuid = parts[1]

    for item in footprint.GraphicalItems():
        if str(item.m_Uuid.AsString()) == outline_uuid:
            return metadata, item

    return metadata, None


def _configure_outline(
    footprint: pcbnew.FOOTPRINT,
    outline: pcbnew.PCB_SHAPE,
    layer: pcbnew.PCB_LAYER_ID,
    box: pcbnew.BOX2I,
    line_width: int,
) -> None:
    outline.SetLayer(layer)

    # Let KiCad transform each local corner to board coordinates.  A temporary
    # child item applies orientation and flipping without changing the actual
    # component (important for arbitrary-angle footprints).
    probe = pcbnew.PCB_SHAPE(footprint, pcbnew.SHAPE_T_SEGMENT)
    points = pcbnew.VECTOR_VECTOR2I()

    for x, y in (
        (box.GetLeft(), box.GetTop()),
        (box.GetRight(), box.GetTop()),
        (box.GetRight(), box.GetBottom()),
        (box.GetLeft(), box.GetBottom()),
    ):
        probe.SetFPRelativePosition(pcbnew.VECTOR2I(x, y))
        world = probe.GetPosition()
        points.append(pcbnew.VECTOR2I(world.x, world.y))

    outline.SetShape(pcbnew.SHAPE_T_POLY)
    outline.SetPolyPoints(points)

    outline.SetFilled(False)
    outline.SetWidth(line_width)
    # A locked footprint child intercepts selection in PCB Editor and makes
    # the otherwise-unlocked component appear immovable.  Child graphics move
    # with their parent without being locked, so keep the generated frame free.
    outline.SetLocked(False)


def has_legacy_board_group(board: pcbnew.BOARD) -> bool:
    return any(str(group.GetName()) == GROUP_NAME for group in board.Groups())


def regenerate_outlines(board: pcbnew.BOARD) -> dict[str, int]:
    """Create or update footprint-owned rectangles and return category counts.

    Existing shapes are updated in place.  KiCad 10 does not safely update its
    live view when an Action Plugin deletes footprint graphics or groups, so
    destructive cleanup is reserved for ``migrate_outlines`` while the board is
    closed.
    """

    for layer, name in LAYER_NAMES.items():
        board.SetLayerName(layer, name)

    counts = {category: 0 for category in CATEGORY_LAYERS}
    line_width = pcbnew.FromMM(LINE_WIDTH_MM)

    for footprint in board.GetFootprints():
        category = classify_footprint(footprint)
        layer = CATEGORY_LAYERS[category]
        box = _footprint_box(footprint)

        if box.GetWidth() <= 0 or box.GetHeight() <= 0:
            continue

        metadata, outline = _find_generated_outline(footprint)

        if outline is None:
            # Parenting the shape to the footprint makes the colored frame
            # follow position, rotation, and side changes without rerunning.
            outline = pcbnew.PCB_SHAPE(footprint, pcbnew.SHAPE_T_POLY)
            footprint.Add(outline)

        _configure_outline(footprint, outline, layer, box, line_width)

        # Record the exact generated item in a hidden custom property on the
        # component.  A later run can therefore remove it without touching
        # unrelated user graphics on the same User layer.
        if metadata is None:
            metadata = pcbnew.PCB_FIELD(
                footprint, pcbnew.FIELD_T_USER, METADATA_FIELD_NAME
            )
            metadata.SetVisible(False)
            footprint.Add(metadata)

        metadata.SetText(
            f"{METADATA_VERSION}|{outline.m_Uuid.AsString()}|{category}"
        )
        counts[category] += 1

    return counts


def migrate_outlines(board: pcbnew.BOARD) -> dict[str, int]:
    """Perform one-time destructive cleanup on a board not open in KiCad."""

    _remove_previous_outlines(board)
    return regenerate_outlines(board)


class ComponentTypeOutlines(pcbnew.ActionPlugin):
    def defaults(self) -> None:
        self.name = PLUGIN_NAME
        self.category = "Layout helpers"
        self.description = (
            "Add footprint-following outlines on User.1-User.4, classified "
            "as resistor, capacitor, LED, or other."
        )
        self.show_toolbar_button = True

    def Run(self) -> None:  # noqa: N802 - KiCad ActionPlugin API name
        board = pcbnew.GetBoard()

        if board is None:
            self._show_message("PCBが開かれていません。", error=True)
            return

        if has_legacy_board_group(board):
            self._show_message(
                "旧版の基板直下アウトラインを検出しました。\n\n"
                "KiCad 10では開いた基板からグループを削除すると異常終了するため、"
                "PCBエディターを閉じて一度だけファイル移行を実行してください。\n"
                "詳しくはプラグインのREADMEを参照してください。",
                error=True,
            )
            return

        try:
            existing = sum(
                _find_generated_outline(footprint)[1] is not None
                for footprint in board.GetFootprints()
            )
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
            "枠は部品の移動・回転に追従します。\n"
            "再実行すると既存の生成枠を更新します。"
            + (
                "\n\n新しい枠は保存後に基板を開き直すと表示されます。"
                if existing < sum(counts.values())
                else ""
            )
        )

    @staticmethod
    def _show_message(message: str, error: bool = False) -> None:
        if wx is None:
            return

        style = wx.OK | (wx.ICON_ERROR if error else wx.ICON_INFORMATION)
        wx.MessageBox(message, PLUGIN_NAME, style)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Migrate a closed KiCad PCB to footprint-following outlines."
    )
    parser.add_argument("board", type=Path, help="Path to a .kicad_pcb file")
    args = parser.parse_args()

    board_path = args.board.expanduser().resolve()
    board = pcbnew.LoadBoard(str(board_path))
    counts = migrate_outlines(board)
    pcbnew.SaveBoard(str(board_path), board)

    print(
        "Updated component outlines: "
        + ", ".join(f"{category}={count}" for category, count in counts.items())
    )


if __name__ == "__main__":
    main()
