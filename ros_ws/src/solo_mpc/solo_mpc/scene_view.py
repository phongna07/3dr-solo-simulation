"""Shared elevated target view for the GUI and optional scene recording."""

TARGET = "tracking_target"
FOLLOW_OFFSET = (-10.0, -10.0, 24.0)
OVERHEAD_TOPIC = "/overhead/camera/image_raw"
OVERHEAD_WIDTH = 1920
OVERHEAD_HEIGHT = 1080
OVERHEAD_FPS = 20.0


def follow_offset(settings):
    """Keep the elevated view above the flight envelope at either map scale."""
    scale = max(1.0, settings.get('flight_altitude', 3.0) / 3.0,
                settings.get('obstacle_height', 5.0) / 5.0)
    return tuple(value * scale for value in FOLLOW_OFFSET)
