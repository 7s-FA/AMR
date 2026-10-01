"""Generate BURGER 1/2 parking boards using the existing docking geometry.

Run from the workspace root:
  PYTHONPATH=host_ws/src/docking_vision python3 -m docking_vision.generate_station_boards
Requires the normal vision dependencies plus reportlab for vector PDF output.
"""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .board import DockingBoard
from .vision import MarkerFinder, make_marker


ROOT = Path(__file__).resolve().parents[4]
STATIONS = [('burger1', 'BURGER 1', [4, 5, 6, 7]),
            ('burger2', 'BURGER 2', [8, 9, 10, 11])]
PX_PER_MM = 10
MARGIN_MM = 10


def create_pdf(path, title, config, marker_images):
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont

    regular, bold = 'Helvetica', 'Helvetica-Bold'
    fonts = Path('/usr/share/fonts/truetype/dejavu')
    if (fonts / 'DejaVuSans.ttf').exists() and (fonts / 'DejaVuSans-Bold.ttf').exists():
        regular, bold = 'StationSans', 'StationSans-Bold'
        pdfmetrics.registerFont(TTFont(regular, str(fonts / 'DejaVuSans.ttf')))
        pdfmetrics.registerFont(TTFont(bold, str(fonts / 'DejaVuSans-Bold.ttf')))
    pdf = canvas.Canvas(str(path), pagesize=A4)
    pdf.setTitle(title + ' - 30 mm ArUco board')
    pdf.setAuthor('docking_vision')
    pdf.setFont(regular, 10)
    pdf.drawString(16 * mm, 272 * mm,
                   f'SMALL 30 mm BOARD - {config["dictionary"]} - {title}')
    layout = config['layout']
    pdf.drawString(16 * mm, 265 * mm,
                   f'IDs: {layout[0][0]}  {layout[0][1]} / {layout[1][0]}  {layout[1][1]}'
                   '   |   Marker: 30 mm   |   Gap: 10 mm')
    # Match the original A4 layout: 90 mm board at x=60 mm, top=55 mm.
    board_left, board_bottom = 60 * mm, 152 * mm
    outer_h_mm = config['board_outer_size_m'][1] * 1000
    length_mm = config['marker_length_m'] * 1000
    for marker, raster in zip(config['markers'], marker_images):
        # Extract the 6 x 6 binary cells produced by the project generator.
        cells = raster[::raster.shape[0] // 6, ::raster.shape[1] // 6]
        assert cells.shape == (6, 6)
        left, top, _ = marker['corners_m'][0]
        x = board_left + (MARGIN_MM + left * 1000) * mm
        y_top = board_bottom + (MARGIN_MM + outer_h_mm - top * 1000) * mm
        cell = length_mm * mm / 6
        for row in range(6):
            for col in range(6):
                if cells[row, col] == 0:
                    pdf.rect(x + col * cell, y_top - (row + 1) * cell,
                             cell, cell, stroke=0, fill=1)
    pdf.setFont(regular, 10)
    pdf.drawString(16 * mm, 55 * mm, 'Print A4 at 100% / Actual size. Do not fit to page.')
    pdf.drawString(16 * mm, 49 * mm, 'Verify each black marker square measures 30 x 30 mm.')
    pdf.setLineWidth(0.5)
    pdf.line(16 * mm, 32.5 * mm, 116 * mm, 32.5 * mm)
    for x in (16, 116):
        pdf.line(x * mm, 30.5 * mm, x * mm, 34.5 * mm)
    pdf.drawString(16 * mm, 24 * mm, 'Scale check: 100 mm')
    pdf.showPage()
    pdf.save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'docs/markers/charging_stations')
    parser.add_argument('--pdf-output', type=Path, default=ROOT / 'output/pdf')
    args = parser.parse_args(argv)
    source = json.loads((ROOT / 'docs/markers/burger2/small_30mm/board_config.json').read_text())
    runtime = yaml.safe_load((ROOT / 'host_ws/src/docking_vision/config/docking_board.yaml').read_text())
    if (source['dictionary'] != 'DICT_4X4_50' or source['marker_length_m'] != .03
            or source['marker_separation_m'] != .01 or source['board_outer_size_m'] != [.07, .07]):
        raise ValueError('Expected the existing 30 mm, 10 mm gap, DICT_4X4_50 reference board')
    used_ids = {m['id'] for m in runtime['markers']} | {m['id'] for m in source['markers']}
    finder = MarkerFinder(source['dictionary'])
    args.pdf_output.mkdir(parents=True, exist_ok=True)
    for name, title, ids in STATIONS:
        if used_ids.intersection(ids):
            raise ValueError(f'ID collision: {ids}')
        used_ids.update(ids)
        out = args.output / name
        out.mkdir(parents=True, exist_ok=True)
        config = copy.deepcopy(source)
        config['layout'] = [ids[:2], ids[2:]]
        config['origin'] = f'top-left outer corner of marker {ids[0]}; x right, y down, z=0 on board'
        config['station'] = name
        config['white_margin_m'] = MARGIN_MM / 1000
        pixels = round(config['marker_length_m'] * 1000 * PX_PER_MM)
        board = np.full((90 * PX_PER_MM, 90 * PX_PER_MM), 255, np.uint8)
        marker_images = []
        station_runtime = copy.deepcopy(runtime)
        for index, (marker, marker_id) in enumerate(zip(config['markers'], ids)):
            marker['id'] = marker_id
            station_runtime['markers'][index]['id'] = marker_id
            padded = make_marker(config['dictionary'], marker_id, pixels, margin=1)
            raster = padded[1:-1, 1:-1]
            marker_images.append(raster)
            left, top, _ = marker['corners_m'][0]
            x = round((MARGIN_MM + left * 1000) * PX_PER_MM)
            y = round((MARGIN_MM + top * 1000) * PX_PER_MM)
            board[y:y + pixels, x:x + pixels] = raster
            # Match the reference 500 px PNG: 300 px marker + 100 px margins.
            margin = MARGIN_MM * PX_PER_MM
            individual = cv2.copyMakeBorder(raster, margin, margin, margin, margin,
                                           cv2.BORDER_CONSTANT, value=255)
            if not cv2.imwrite(str(out / f'marker_{marker_id}.png'), individual):
                raise RuntimeError('Failed to write marker PNG')
        _, detected, _ = finder.find(cv2.cvtColor(board, cv2.COLOR_GRAY2BGR))
        if detected is None or sorted(detected.ravel().tolist()) != ids:
            raise RuntimeError(f'Generated board failed detection: {name}')
        DockingBoard(station_runtime)
        if not cv2.imwrite(str(out / 'docking_board.png'), board):
            raise RuntimeError('Failed to write board PNG')
        (out / 'board_config.json').write_text(json.dumps(config, indent=2) + '\n')
        (out / 'docking_board.yaml').write_text(yaml.safe_dump(station_runtime, sort_keys=False))
        pdf = args.pdf_output / f'charging_station_{name}_A4.pdf'
        create_pdf(pdf, title, config, marker_images)
        print(f'{name}: verified IDs {ids}; {pdf}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
