#!/usr/bin/env python3
"""Generate print-ready fixed ArUco markers for maze-plane rectification."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import cv2
from reportlab.lib.colors import Color, black, white
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas


DICTIONARY_NAME = "DICT_4X4_50"
MARKER_LAYOUT = (
    (5, "TOP LEFT"),
    (7, "TOP RIGHT"),
    (6, "BOTTOM LEFT"),
    (4, "BOTTOM RIGHT"),
)
DEFAULT_SIZES_MM = (60.0, 40.0)
MODULES_PER_SIDE = 6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an A4 PDF and individual SVG files for Nightfall's fixed "
            "DICT_4X4_50 markers."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/pdf/nightfall_fixed_aruco_markers"),
        help="output directory",
    )
    parser.add_argument(
        "--sizes-mm",
        type=float,
        nargs="+",
        default=list(DEFAULT_SIZES_MM),
        help="black marker side lengths, one A4 page per size",
    )
    return parser.parse_args()


def marker_modules(marker_id: int) -> list[list[int]]:
    if marker_id < 0 or marker_id >= 50:
        raise ValueError("DICT_4X4_50 marker ID must be in 0..49")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = cv2.aruco.generateImageMarker(
        dictionary,
        marker_id,
        MODULES_PER_SIDE,
        borderBits=1,
    )
    if image.shape != (MODULES_PER_SIDE, MODULES_PER_SIDE):
        raise RuntimeError(f"unexpected marker shape: {image.shape}")
    values = set(int(value) for value in image.reshape(-1))
    if not values.issubset({0, 255}):
        raise RuntimeError(f"unexpected marker values: {sorted(values)}")
    return [
        [1 if int(image[row, column]) == 0 else 0 for column in range(image.shape[1])]
        for row in range(image.shape[0])
    ]


def quiet_zone_mm(side_mm: float) -> float:
    return max(10.0, side_mm / MODULES_PER_SIDE)


def validate_sizes(sizes_mm: Iterable[float]) -> tuple[float, ...]:
    result = tuple(float(value) for value in sizes_mm)
    if not result:
        raise ValueError("at least one marker size is required")
    for value in result:
        if not math.isfinite(value) or value <= 0:
            raise ValueError("marker sizes must be positive finite values")
        card_width = value + 2.0 * quiet_zone_mm(value)
        if card_width * 2.0 + 20.0 > 210.0:
            raise ValueError(
                f"{value:g} mm markers do not fit two-up on A4; "
                "use 70 mm or smaller"
            )
    return result


def draw_marker(
    canvas: Canvas,
    marker_id: int,
    marker_x_mm: float,
    marker_y_mm: float,
    side_mm: float,
) -> None:
    modules = marker_modules(marker_id)
    cell_mm = side_mm / MODULES_PER_SIDE
    canvas.setFillColor(white)
    canvas.rect(
        marker_x_mm * mm,
        marker_y_mm * mm,
        side_mm * mm,
        side_mm * mm,
        stroke=0,
        fill=1,
    )
    canvas.setFillColor(black)
    for row, values in enumerate(modules):
        for column, is_black in enumerate(values):
            if not is_black:
                continue
            x_mm = marker_x_mm + column * cell_mm
            y_mm = marker_y_mm + (MODULES_PER_SIDE - row - 1) * cell_mm
            canvas.rect(
                x_mm * mm,
                y_mm * mm,
                cell_mm * mm,
                cell_mm * mm,
                stroke=0,
                fill=1,
            )


def draw_card(
    canvas: Canvas,
    marker_id: int,
    role: str,
    x_mm: float,
    y_mm: float,
    side_mm: float,
) -> None:
    quiet_mm = quiet_zone_mm(side_mm)
    label_height_mm = 10.0
    card_width_mm = side_mm + 2.0 * quiet_mm
    card_height_mm = side_mm + 2.0 * quiet_mm + label_height_mm

    canvas.setFillColor(white)
    canvas.setStrokeColor(Color(0.7, 0.7, 0.7))
    canvas.setLineWidth(0.25 * mm)
    canvas.setDash(1.5 * mm, 1.5 * mm)
    canvas.rect(
        x_mm * mm,
        y_mm * mm,
        card_width_mm * mm,
        card_height_mm * mm,
        stroke=1,
        fill=1,
    )
    canvas.setDash()

    marker_x_mm = x_mm + quiet_mm
    marker_y_mm = y_mm + label_height_mm + quiet_mm
    draw_marker(canvas, marker_id, marker_x_mm, marker_y_mm, side_mm)

    canvas.setFillColor(black)
    canvas.setFont("Helvetica-Bold", 9)
    canvas.drawCentredString(
        (x_mm + card_width_mm / 2.0) * mm,
        (y_mm + 4.0) * mm,
        f"ID {marker_id} - {role}",
    )


def draw_page(canvas: Canvas, side_mm: float, page_number: int) -> None:
    quiet_mm = quiet_zone_mm(side_mm)
    card_width_mm = side_mm + 2.0 * quiet_mm
    card_height_mm = side_mm + 2.0 * quiet_mm + 10.0

    canvas.setTitle("Nightfall fixed ArUco marker print pack")
    canvas.setAuthor("Nightfall vision tools")
    canvas.setFillColor(black)
    canvas.setFont("Helvetica-Bold", 14)
    canvas.drawString(
        15.0 * mm,
        282.0 * mm,
        (
            f"Nightfall fixed ArUco markers - {DICTIONARY_NAME} - "
            f"{side_mm:g} mm black side"
        ),
    )
    canvas.setFont("Helvetica", 8.5)
    canvas.drawString(
        15.0 * mm,
        275.5 * mm,
        "Place all four on the maze plane. Keep the printed top edges facing maze +Y.",
    )
    canvas.drawString(
        15.0 * mm,
        270.5 * mm,
        "Use this page alone; do not mix marker sizes. Cut only on the light dashed lines.",
    )

    left_x_mm = 15.0
    right_x_mm = 210.0 - 15.0 - card_width_mm
    top_y_mm = 166.0
    bottom_y_mm = top_y_mm - card_height_mm - 10.0
    positions = (
        (left_x_mm, top_y_mm),
        (right_x_mm, top_y_mm),
        (left_x_mm, bottom_y_mm),
        (right_x_mm, bottom_y_mm),
    )
    for (marker_id, role), (x_mm, y_mm) in zip(MARKER_LAYOUT, positions):
        draw_card(canvas, marker_id, role, x_mm, y_mm, side_mm)

    ruler_start_x_mm = 55.0
    ruler_y_mm = 25.0
    canvas.setStrokeColor(black)
    canvas.setLineWidth(0.35 * mm)
    canvas.line(
        ruler_start_x_mm * mm,
        ruler_y_mm * mm,
        (ruler_start_x_mm + 100.0) * mm,
        ruler_y_mm * mm,
    )
    for x_mm in (ruler_start_x_mm, ruler_start_x_mm + 100.0):
        canvas.line(
            x_mm * mm,
            (ruler_y_mm - 2.0) * mm,
            x_mm * mm,
            (ruler_y_mm + 2.0) * mm,
        )
    canvas.setFont("Helvetica-Bold", 9)
    canvas.drawCentredString(
        105.0 * mm,
        30.0 * mm,
        "PRINT SCALE CHECK: 100.0 mm",
    )
    canvas.setFont("Helvetica", 8)
    canvas.drawCentredString(
        105.0 * mm,
        15.0 * mm,
        (
            f"Print at 100% / Actual Size. Verify the line is 100.0 mm and "
            f"each black square is {side_mm:g}.0 mm."
        ),
    )
    canvas.setFont("Helvetica", 7)
    canvas.drawRightString(
        195.0 * mm,
        7.0 * mm,
        f"Page {page_number}",
    )
    canvas.showPage()


def make_svg(marker_id: int, side_mm: float) -> str:
    modules = marker_modules(marker_id)
    quiet_mm = quiet_zone_mm(side_mm)
    canvas_mm = side_mm + 2.0 * quiet_mm
    cell_mm = side_mm / MODULES_PER_SIDE
    rectangles = [
        (
            f'  <rect x="{quiet_mm + column * cell_mm:.8f}" '
            f'y="{quiet_mm + row * cell_mm:.8f}" '
            f'width="{cell_mm:.8f}" height="{cell_mm:.8f}" />'
        )
        for row, values in enumerate(modules)
        for column, is_black in enumerate(values)
        if is_black
    ]
    return "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            (
                f'<svg xmlns="http://www.w3.org/2000/svg" '
                f'width="{canvas_mm:g}mm" height="{canvas_mm:g}mm" '
                f'viewBox="0 0 {canvas_mm:g} {canvas_mm:g}" '
                f'shape-rendering="crispEdges">'
            ),
            (
                f'  <title>{DICTIONARY_NAME} ID {marker_id}, '
                f'{side_mm:g} mm black side</title>'
            ),
            f'  <rect width="{canvas_mm:g}" height="{canvas_mm:g}" fill="white" />',
            '  <g fill="black">',
            *rectangles,
            "  </g>",
            "</svg>",
            "",
        ]
    )


def generate(output_dir: Path, sizes_mm: Iterable[float]) -> dict[str, object]:
    sizes = validate_sizes(sizes_mm)
    output_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = output_dir / "nightfall_fixed_aruco_markers_a4.pdf"
    canvas = Canvas(str(pdf_path), pagesize=A4, pageCompression=1)
    for page_number, side_mm in enumerate(sizes, start=1):
        draw_page(canvas, side_mm, page_number)
    canvas.save()

    svg_files: list[dict[str, object]] = []
    for side_mm in sizes:
        for marker_id, role in MARKER_LAYOUT:
            path = output_dir / (
                f"aruco_4x4_50_id{marker_id}_{side_mm:g}mm_black_side.svg"
            )
            path.write_text(make_svg(marker_id, side_mm), encoding="utf-8")
            svg_files.append(
                {
                    "path": path.name,
                    "marker_id": marker_id,
                    "role": role.lower().replace(" ", "_"),
                    "black_side_mm": side_mm,
                    "quiet_zone_mm": quiet_zone_mm(side_mm),
                }
            )

    manifest: dict[str, object] = {
        "schema": "nightfall_fixed_aruco_print_pack_v1",
        "dictionary": DICTIONARY_NAME,
        "pdf": pdf_path.name,
        "recommended_black_side_mm": sizes[0],
        "marker_layout": {
            "top_left": 5,
            "top_right": 7,
            "bottom_right": 4,
            "bottom_left": 6,
        },
        "pages": [
            {
                "page": index,
                "black_side_mm": side_mm,
                "module_mm": side_mm / MODULES_PER_SIDE,
                "quiet_zone_mm": quiet_zone_mm(side_mm),
            }
            for index, side_mm in enumerate(sizes, start=1)
        ],
        "svg_files": svg_files,
        "print": {
            "paper": "A4",
            "scale": "100% / Actual Size",
            "scale_check_mm": 100.0,
            "measure": "black outer square, not the white card",
        },
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> int:
    args = parse_args()
    manifest = generate(args.output_dir, args.sizes_mm)
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
