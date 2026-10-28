"""Fleet colors shared by Gazebo coverage guides and recording panels."""

import colorsys


# BGR for OpenCV. Preserve the existing occupancy map and state panel colors.
VEHICLE_COLORS = ((255, 202, 91), (105, 225, 255), (222, 155, 239),
                  (134, 237, 167), (190, 175, 255), (246, 217, 155))


def vehicle_color(index):
    if index < len(VEHICLE_COLORS):
        return VEHICLE_COLORS[index]
    rgb = colorsys.hsv_to_rgb((0.57 + index * 0.61803398875) % 1, 0.65, 1)
    return tuple(round(c * 255) for c in reversed(rgb))
