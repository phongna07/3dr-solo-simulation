"""Compose camera feeds on one constant-rate simulation-time timeline."""

from collections import deque
import time

import cv2

from .video import FrameVideo
from .occupancy_video import overlay_occupancy
from .state_video import FlightStateHistory, overlay_states, render_states


def overlay_views(background, views):
    """Fit labelled, aspect-preserving FOV panels down the right-hand edge."""
    frame = background.copy()
    height, width = frame.shape[:2]
    gap = max(1, min(12, height // (3 * len(views) + 1)))
    side = min(width // 4 - gap, (height - gap * (len(views) + 1)) // len(views))
    if side < 8:
        raise ValueError("Overhead resolution is too small for the fleet's FOV panels")
    label_height = max(4, min(28, side // 8))
    x = width - gap - side
    for i, (name, image) in enumerate(views.items()):
        y = gap + i * (side + gap)
        frame[y:y + side, x:x + side] = (24, 24, 24)
        if image is not None:
            ih, iw = image.shape[:2]
            scale = min((side - 4) / iw, (side - label_height - 2) / ih)
            w, h = max(1, round(iw * scale)), max(1, round(ih * scale))
            image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
            left, top = x + (side - w) // 2, y + label_height
            top += (side - label_height - h) // 2
            frame[top:top + h, left:left + w] = image
        label = name if image is not None else f"{name} (waiting)"
        scale = min(0.6, label_height / 30)
        text_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
        scale *= min(1.0, (side - 6) / max(1, text_width))
        cv2.putText(frame, label, (x + 3, y + label_height - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(frame, (x, y), (x + side - 1, y + side - 1), (220, 220, 220), 1)
    return frame


class CompositeVideo:
    """Buffer delivery jitter and sample every feed by its Gazebo timestamp.

    Output begins at the first overhead frame. At each output tick, use only
    frames at or before that tick; slower or interrupted feeds hold their last
    image. Wait up to 0.5 simulation seconds or 2 wall seconds for FOV delivery.
    No camera can block recording indefinitely, including during shutdown.
    """

    def __init__(self, path, fps, vehicles, *, occupancy=False, telemetry=False,
                 timestamps_path=None):
        self.vehicles = tuple(vehicles)
        if not self.vehicles or len(set(self.vehicles)) != len(self.vehicles):
            raise ValueError("A composite video needs unique vehicle names")
        if {'overhead', 'occupancy'} & set(self.vehicles):
            raise ValueError("The vehicle names 'overhead' and 'occupancy' are reserved for combined recording")
        self.occupancy = occupancy
        self.telemetry = {name: FlightStateHistory() for name in self.vehicles} if telemetry else {}
        self.video = FrameVideo(path, fps, timestamps_path=timestamps_path)
        # FOV queues are also bounded if the overhead feed never arrives/stalls.
        self.pending = {name: deque(maxlen=256) for name in ('overhead', *self.vehicles)}
        if occupancy:
            self.pending['occupancy'] = deque(maxlen=64)
        self.latest = {}
        self.current = {name: None for name in self.pending}
        self.background_received = None
        self.start_ns = None
        self.buffer_ns = 500_000_000
        self.max_wait = 2.0

    def write_state(self, name, field, values, stamp_ns):
        if self.video.closed:
            raise RuntimeError("Video is closed")
        self.telemetry[name].write(field, values, stamp_ns)

    def write(self, name, frame, stamp_ns):
        if self.video.closed:
            raise RuntimeError("Video is closed")
        if name not in self.pending:
            raise ValueError(f"Unknown camera: {name}")
        if stamp_ns <= self.latest.get(name, -1):
            return  # Repeated/out-of-order samples cannot move playback backwards.
        now = time.monotonic()
        self.latest[name] = stamp_ns
        self.pending[name].append((stamp_ns, frame, now))
        if name == 'overhead' and self.start_ns is None:
            self.start_ns = stamp_ns
        self.flush(now=now)

    def flush(self, *, now=None, final=False):
        if self.start_ns is None or self.video.closed:
            return
        now = time.monotonic() if now is None else now
        while True:
            stamp = self.start_ns + round(self.video.frames * 1_000_000_000 / self.video.fps)
            if stamp > self.latest['overhead']:
                return
            for name, queue in self.pending.items():
                while queue and queue[0][0] <= stamp:
                    _, self.current[name], received = queue.popleft()
                    if name == 'overhead':
                        self.background_received = received
            ready = all(self.latest.get(name, -1) >= stamp for name in self.vehicles)
            expired = (self.latest['overhead'] - stamp >= self.buffer_ns
                       or now - self.background_received >= self.max_wait)
            if not (ready or expired or final):
                return
            views = {name: self.current[name] for name in self.vehicles}
            frame = overlay_views(self.current['overhead'], views)
            if self.occupancy:
                frame = overlay_occupancy(frame, self.current['occupancy'],
                                          max_height=0.50 if self.telemetry else 1.0)
            if self.telemetry:
                states = {name: history.sample(stamp) for name, history in self.telemetry.items()}
                frame = overlay_states(frame, render_states(states, stamp))
            self.video.write(frame, stamp)

    def close(self):
        try:
            self.flush(final=True)
        finally:
            self.video.close()
            for queue in self.pending.values():
                queue.clear()
            self.current.clear()
