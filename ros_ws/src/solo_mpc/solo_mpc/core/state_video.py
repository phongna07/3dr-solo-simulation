"""Timestamped measured flight state and a world-aligned video instrument panel."""

from collections import deque
import math

import cv2
import numpy as np

from .occupancy_video import MUTED, PANEL, TEXT, VEHICLE_COLORS


VELOCITY = (255, 210, 91)
ACCELERATION = (124, 230, 164)
COMMAND = (102, 180, 255)
MAX_AGE_NS = 1_000_000_000


class FlightStateHistory:
    """Bound each stream and sample only data at or before the video tick.

    Acceleration is a low-pass estimate of dv/dt, in world axes, excluding
    gravity. A gap over one second resets the derivative rather than joining
    unrelated samples. Missing/stale values stay unavailable, never fake zero.
    """

    def __init__(self):
        self.samples = {field: deque(maxlen=256) for field in ('pose', 'velocity', 'command')}

    def write(self, field, values, stamp_ns):
        history = self.samples[field]
        values = np.asarray(values, dtype=float)
        if (values.shape != ((7,) if field == 'pose' else (3,))
                or not np.isfinite(values).all()
                or (history and stamp_ns <= history[-1][0])):
            return
        if field == 'pose':
            norm = np.linalg.norm(values[3:])
            if norm < 0.9:
                return
            x, y, z, w = values[3:] / norm
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            value = (values[:3].copy(), yaw)
        elif field == 'velocity':
            acceleration = None
            if history and stamp_ns - history[-1][0] <= MAX_AGE_NS:
                old_stamp, (old_velocity, old_acceleration) = history[-1]
                derivative = (values - old_velocity) / ((stamp_ns - old_stamp) / 1e9)
                acceleration = (derivative if old_acceleration is None
                                else 0.3 * derivative + 0.7 * old_acceleration)
            value = (values.copy(), acceleration)
        else:
            value = values.copy()
        history.append((stamp_ns, value))

    def sample(self, stamp_ns):
        state = dict.fromkeys(('position', 'yaw', 'velocity', 'acceleration', 'command'))
        seen = False
        for field, history in self.samples.items():
            for at, value in reversed(history):
                if at <= stamp_ns:
                    seen = True
                    if stamp_ns - at <= MAX_AGE_NS:
                        if field == 'pose':
                            state['position'], state['yaw'] = value
                        elif field == 'velocity':
                            state['velocity'], state['acceleration'] = value
                        else:
                            state['command'] = value
                    break
        state['status'] = ('LIVE' if state['position'] is not None
                           or state['velocity'] is not None else 'STALE' if seen else 'WAITING')
        return state


def vector_tip(center, vector, full_scale, radius):
    """Project XY to screen (+Y up); use a fixed scale and cap at the dial rim."""
    xy = np.asarray(vector, dtype=float)[:2]
    length = np.linalg.norm(xy)
    offset = xy * radius / max(full_scale, length)
    return round(center[0] + offset[0]), round(center[1] - offset[1])


def render_states(states, stamp_ns):
    """One row per UAV: measured nose direction, XY vectors and XYZ values."""
    row_height, header, footer = 124, 72, 48
    panel = np.full((header + row_height * len(states) + footer, 560, 3), PANEL, np.uint8)

    def text(label, xy, size=0.43, color=TEXT, thickness=1, max_width=None):
        if max_width:
            width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, size, thickness)[0][0]
            size *= min(1, max_width / max(1, width))
        cv2.putText(panel, label, xy, cv2.FONT_HERSHEY_SIMPLEX, size, color,
                    thickness, cv2.LINE_AA)

    text('DRONE STATE', (20, 29), 0.70, thickness=2)
    text(f't={stamp_ns / 1e9:.1f}s', (420, 28), 0.43, MUTED, max_width=122)
    text('World axes | +Y up, +X right | nose = heading', (20, 53), color=MUTED)
    for i, (name, state) in enumerate(states.items()):
        top = header + i * row_height
        color = VEHICLE_COLORS[i % len(VEHICLE_COLORS)]
        cv2.rectangle(panel, (12, top), (547, top + row_height - 7), (39, 35, 31), -1)
        cv2.line(panel, (12, top), (12, top + row_height - 7), color, 3)
        label = f'{i + 1} {name}'
        text(label, (22, top + 19), 0.46, color, max_width=120)
        if state['status'] != 'LIVE':
            text(state['status'], (170, top + 18), 0.37, MUTED)
        for axis, x in zip('XYZ', (282, 382, 482)):
            text(axis, (x, top + 18), 0.35, MUTED)

        center, radius = (78, top + 65), 32
        cv2.circle(panel, center, radius, (88, 79, 69), 1, cv2.LINE_AA)
        cv2.line(panel, (center[0] - radius, center[1]),
                 (center[0] + radius, center[1]), (62, 55, 48), 1)
        cv2.line(panel, (center[0], center[1] - radius),
                 (center[0], center[1] + radius), (62, 55, 48), 1)
        text('Y', (center[0] - 3, center[1] - radius - 3), 0.27, MUTED)
        text('X', (center[0] + radius + 5, center[1] + 4), 0.27, MUTED)
        # Different units have separate, fixed scales, never per-frame normalization.
        # Draw the thick command first so measured acceleration remains visible.
        for field, ink, scale, thickness in (
            ('command', COMMAND, 2.0, 3), ('acceleration', ACCELERATION, 2.0, 2),
            ('velocity', VELOCITY, 2.0, 1),
        ):
            vector = state[field]
            if vector is not None:
                tip = vector_tip(center, vector, scale, radius)
                if tip != center:
                    cv2.arrowedLine(panel, center, tip, ink, thickness,
                                    cv2.LINE_AA, tipLength=0.28)
                else:
                    cv2.circle(panel, center, 2, ink, -1, cv2.LINE_AA)

        yaw = state['yaw']
        if yaw is not None:
            # A small quadrotor silhouette and a white nose show actual attitude,
            # independently of the direction in which the aircraft is travelling.
            forward = np.array([math.cos(yaw), -math.sin(yaw)])
            right = np.array([-forward[1], forward[0]])
            c = np.array(center)
            for side in (-1, 1):
                start = tuple(np.rint(c - forward * 6 + right * side * 6).astype(int))
                end = tuple(np.rint(c + forward * 6 - right * side * 6).astype(int))
                cv2.line(panel, start, end, color, 2, cv2.LINE_AA)
                for point in (start, end):
                    cv2.circle(panel, point, 2, color, 1, cv2.LINE_AA)
            nose = np.rint([c + forward * 17, c + forward * 8 + right * 4,
                            c + forward * 8 - right * 4]).astype(np.int32)
            cv2.fillConvexPoly(panel, nose, TEXT, cv2.LINE_AA)
        heading = '--' if yaw is None else f'{math.degrees(yaw) % 360:.0f} deg'
        text(f'yaw {heading}', (22, top + 111), 0.36, MUTED)

        for j, (field, label, ink) in enumerate((
            ('position', 'p  m', TEXT), ('velocity', 'v  m/s', VELOCITY),
            ('acceleration', 'a est', ACCELERATION), ('command', 'cmd a', COMMAND),
        )):
            y = top + 39 + 22 * j
            text(label, (164, y), color=ink)
            vector = state[field]
            for axis, x in enumerate((268, 368, 468)):
                value = '--' if vector is None else f'{vector[axis]:+.2f}'
                text(value, (x, y), color=ink, max_width=72)

    y = panel.shape[0] - 31
    for x, label, color in ((20, 'v', VELOCITY), (103, 'a est', ACCELERATION),
                             (225, 'MPC cmd a', COMMAND)):
        cv2.arrowedLine(panel, (x, y - 4), (x + 20, y - 4), color, 2,
                        cv2.LINE_AA, tipLength=0.35)
        text(label, (x + 27, y), 0.38, color)
    text('Rim: 2 m/s or 2 m/s^2 | a, cmd a: m/s^2 | -- unavailable',
         (20, y + 22), 0.37, MUTED)
    return panel


def overlay_states(frame, panel):
    """Anchor below the map; limit height for arbitrary fleet sizes."""
    height, width = frame.shape[:2]
    gap = max(1, min(12, height // 20))
    ih, iw = panel.shape[:2]
    scale = min((width * 0.30 - gap) / iw, (height * 0.46 - gap) / ih)
    w, h = max(1, round(iw * scale)), max(1, round(ih * scale))
    top = height - gap - h
    frame[top:top + h, gap:gap + w] = cv2.resize(panel, (w, h), interpolation=cv2.INTER_AREA)
    return frame
