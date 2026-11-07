"""Shared fleet palette and target camera framing for Gazebo and recording."""

import colorsys
import math


# BGR for OpenCV. Preserve the existing occupancy map and state panel colors.
VEHICLE_COLORS = ((255, 202, 91), (105, 225, 255), (222, 155, 239),
                  (134, 237, 167), (190, 175, 255), (246, 217, 155))


def vehicle_color(index):
    if index < len(VEHICLE_COLORS):
        return VEHICLE_COLORS[index]
    rgb = colorsys.hsv_to_rgb((0.57 + index * 0.61803398875) % 1, 0.65, 1)
    return tuple(round(c * 255) for c in reversed(rgb))


TARGET = "tracking_target"
FOLLOW_OFFSET = (-10.0, -10.0, 24.0)
OVERHEAD_TOPIC = "/overhead/camera/image_raw"
OVERHEAD_WIDTH = 1920
OVERHEAD_HEIGHT = 1080
OVERHEAD_FPS = 20.0


def follow_offset(settings):
    """A close target view above the obstacles, independent of map dimensions.

    Scaling every offset by obstacle height put the large-map view at 288 m;
    one-metre aircraft became only a few pixels even in Full HD. Keep the camera
    near vertical and frame the local action. Dispersed SEARCH UAVs can be
    outside this view until they approach the target.
    """
    altitude = settings.get('flight_altitude', 3.0)
    if 'overhead_camera_offset' in settings:
        offset = tuple(float(v) for v in settings['overhead_camera_offset'])
        if (len(offset) != 3 or not all(math.isfinite(v) for v in offset)
                or offset[2] <= max(altitude + 1, settings.get('obstacle_height', 5.0) + 10)):
            raise ValueError('overhead_camera_offset needs finite XYZ above the fleet and scenery')
        return offset
    height = max(FOLLOW_OFFSET[2], settings.get('obstacle_height', 5.0) + 10,
                 altitude + 70 if altitude > 3 else FOLLOW_OFFSET[2])
    return FOLLOW_OFFSET[:2] + (height,)
