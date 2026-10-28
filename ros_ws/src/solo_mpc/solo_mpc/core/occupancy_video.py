"""Readable, world-aligned views of the controller's measured occupancy map."""

import math

import cv2
import numpy as np

from .visual_style import vehicle_color


OCCUPANCY_TOPIC = '/mpc/occupancy_grid'
# OpenCV uses BGR. Unknown space stays visibly distinct from observed free space.
UNKNOWN = (62, 53, 46)
FREE = (188, 220, 201)
OCCUPIED = (88, 115, 247)
PANEL = (28, 24, 21)
TEXT = (242, 242, 242)
MUTED = (179, 175, 168)


def fuse_occupancy(maps):
    """Union equal, world-aligned maps; an obstacle wins over a free report.

    Keep unobserved cells unknown even when the planner's unknown-space policy
    allows traversal. Do not display the inflated planning clearance as a wall.
    """
    occupied = np.logical_or.reduce([m.occupied for m in maps])
    known = np.logical_or.reduce([~m.unknown for m in maps])
    cells = np.full(occupied.shape, -1, dtype=np.int8)
    cells[known] = 0
    cells[occupied] = 100
    return cells


def render_occupancy(cells, resolution, origin, positions, names, stamp_ns):
    """Return a labelled panel; grid row zero is world minimum Y, at the bottom.

    No world geometry or target truth is used: only classified LiDAR cells and
    measured UAV positions at or before the map's simulation timestamp.
    """
    height, width = cells.shape
    panel_width, padding = 560, 20
    scale = min((panel_width - 2 * padding) / width, 390 / height)
    map_width, map_height = max(1, round(width * scale)), max(1, round(height * scale))
    left, top = (panel_width - map_width) // 2, 82
    bottom = top + map_height
    fleet_rows = math.ceil(len(names) / 3)
    panel = np.full((bottom + 104 + 25 * fleet_rows, panel_width, 3), PANEL, np.uint8)

    def text(label, xy, size=0.48, color=TEXT, thickness=1):
        cv2.putText(panel, label, xy, cv2.FONT_HERSHEY_SIMPLEX, size, color,
                    thickness, cv2.LINE_AA)

    text('OCCUPANCY MAP', (padding, 30), 0.72, thickness=2)
    text('LiDAR only | combined fleet scans', (padding, 54), color=MUTED)
    text('+Y', (left, top - 7), 0.36, MUTED)
    raster = np.full((*cells.shape, 3), UNKNOWN, np.uint8)
    raster[cells == 0] = FREE
    raster[cells > 0] = OCCUPIED
    panel[top:bottom, left:left + map_width] = cv2.resize(
        raster[::-1], (map_width, map_height), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(panel, (left, top), (left + map_width - 1, bottom - 1), MUTED, 1)
    text('+X', (left + map_width - 23, bottom + 15), 0.36, MUTED)

    for i, name in enumerate(names):
        color = vehicle_color(i)
        if name in positions:
            x, y = positions[name][:2]
            gx, gy = (x - origin[0]) / resolution, (y - origin[1]) / resolution
            if 0 <= gx < width and 0 <= gy < height:
                px = left + min(map_width - 1, int(gx * map_width / width))
                py = bottom - 1 - min(map_height - 1, int(gy * map_height / height))
                cv2.circle(panel, (px, py), 10, PANEL, -1, cv2.LINE_AA)
                cv2.circle(panel, (px, py), 9, color, 2, cv2.LINE_AA)
                label = str(i + 1)
                tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)[0][0]
                text(label, (px - tw // 2, py + 4), 0.36)
        x, y = padding + (i % 3) * 176, bottom + 111 + (i // 3) * 25
        cv2.circle(panel, (x + 5, y - 4), 5, color, -1, cv2.LINE_AA)
        label = f'{i + 1} {name}'
        size = min(0.43, 151 / max(1, cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0]))
        text(label, (x + 17, y), size)

    for i, (label, color) in enumerate((('Unknown', UNKNOWN), ('Scanned free', FREE),
                                       ('Obstacle', OCCUPIED))):
        x, y = padding + i * 176, bottom + 33
        cv2.rectangle(panel, (x, y - 11), (x + 13, y + 2), color, -1)
        cv2.rectangle(panel, (x, y - 11), (x + 13, y + 2), MUTED, 1)
        text(label, (x + 21, y), 0.44)

    # A metric scale and explicit world axes avoid implying geographic north.
    span = width * resolution
    bar_m = 10 ** math.floor(math.log10(span / 4))
    if bar_m * 5 <= span / 4:
        bar_m *= 5
    elif bar_m * 2 <= span / 4:
        bar_m *= 2
    bar_px = max(1, round(bar_m / span * map_width))
    y = bottom + 62
    cv2.line(panel, (padding, y), (padding + bar_px, y), TEXT, 2)
    for x in (padding, padding + bar_px):
        cv2.line(panel, (x, y - 4), (x, y + 4), TEXT, 1)
    text(f'{bar_m:g} m', (padding + bar_px + 9, y + 5), 0.43)
    observed = 100 * np.count_nonzero(cells >= 0) / cells.size
    text(f'Scanned {observed:.1f}% | t={stamp_ns / 1e9:.1f}s',
         (padding, bottom + 88), 0.45, MUTED)
    return panel


def overlay_occupancy(frame, panel, *, max_height=1.0):
    """Place a bounded map panel on the left, keeping its aspect ratio."""
    height, width = frame.shape[:2]
    gap = max(1, min(12, height // 20))
    if panel is None:
        panel = np.full((220, 560, 3), PANEL, np.uint8)
        cv2.putText(panel, 'OCCUPANCY MAP', (20, 36), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, TEXT, 2, cv2.LINE_AA)
        cv2.putText(panel, 'Waiting for LiDAR mapping...', (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.53, MUTED, 1, cv2.LINE_AA)
    ih, iw = panel.shape[:2]
    scale = min((width * 0.30 - gap) / iw, (height * max_height - 2 * gap) / ih)
    w, h = max(1, round(iw * scale)), max(1, round(ih * scale))
    frame[gap:gap + h, gap:gap + w] = cv2.resize(panel, (w, h), interpolation=cv2.INTER_AREA)
    return frame
