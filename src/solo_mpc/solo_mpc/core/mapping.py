"""Log-odds integration from explicit rays, without inferring missing rays."""

import numpy as np

from .planner_grid import IncrementalLogOddsGrid, njit


@njit(cache=False)
def ray_cells(origin, endpoints, hits, ox, oy, resolution, width, height):
    """Rasterize observed rays once per cell, protecting simultaneous hits."""
    free = np.zeros(width * height, dtype=np.bool_)
    occupied = np.zeros(width * height, dtype=np.bool_)
    step = resolution * 0.5
    for i in range(len(endpoints)):
        dx, dy = endpoints[i, 0] - origin[0], endpoints[i, 1] - origin[1]
        length = np.sqrt(dx * dx + dy * dy)
        if length < resolution:
            continue
        samples = max(1, int((length - resolution) / step))
        for j in range(samples):
            x = origin[0] + j * step / length * dx
            y = origin[1] + j * step / length * dy
            if x < ox or y < oy:
                continue
            ix, iy = int((x - ox) / resolution), int((y - oy) / resolution)
            if ix < width and iy < height:
                free[iy * width + ix] = True
        if hits[i] and endpoints[i, 0] >= ox and endpoints[i, 1] >= oy:
            ix = int((endpoints[i, 0] - ox) / resolution)
            iy = int((endpoints[i, 1] - oy) / resolution)
            if ix < width and iy < height:
                occupied[iy * width + ix] = True
    return free, occupied


def warmup_scan_integration():
    ray_cells(
        np.zeros(2), np.empty((0, 2)), np.empty(0, dtype=np.bool_), 0.0, 0.0, 0.15, 2, 2
    )


class SensorGrid(IncrementalLogOddsGrid):
    def integrate(self, scan):
        # A fresh but empty scan contributes no evidence, including free space.
        if not len(scan.endpoints):
            return
        bounds = np.vstack((scan.origin[:2], scan.endpoints[:, :2]))
        lower = np.floor((bounds.min(axis=0) - [self.ox, self.oy]) / self.resolution).astype(int)
        upper = np.floor((bounds.max(axis=0) - [self.ox, self.oy]) / self.resolution).astype(int) + 1
        x0, y0 = np.maximum(lower, 0)
        x1, y1 = np.minimum(upper, [self.size_x, self.size_y])
        if x0 >= x1 or y0 >= y1:
            return
        free, occupied = ray_cells(
            np.ascontiguousarray(scan.origin[:2], dtype=float),
            np.ascontiguousarray(scan.endpoints[:, :2], dtype=float),
            scan.hits,
            self.ox + x0 * self.resolution,
            self.oy + y0 * self.resolution,
            self.resolution,
            x1 - x0,
            y1 - y0,
        )
        local = self.L[y0:y1, x0:x1]
        free = free.reshape(local.shape)
        occupied = occupied.reshape(local.shape)
        # Each cell receives one update per scan; grazing rays cannot clear hits.
        local[free & ~occupied] += self.l_free
        local[occupied] += self.l_occ
        np.clip(local, self.l_min, self.l_max, out=local)
        box = [y0, y1, x0, x1]
        if self._dirty_box is None:
            self._dirty_box = box
        else:
            previous = self._dirty_box
            self._dirty_box = [min(previous[0], y0), max(previous[1], y1),
                               min(previous[2], x0), max(previous[3], x1)]
        self._dirty = True

    def surface_points(self, center, radius):
        lower = np.floor((center[:2] - radius - [self.ox, self.oy]) / self.resolution).astype(int)
        upper = np.floor((center[:2] + radius - [self.ox, self.oy]) / self.resolution).astype(int) + 1
        x0, y0 = np.clip(lower, 0, [self.size_x, self.size_y])
        x1, y1 = np.clip(upper, 0, [self.size_x, self.size_y])
        ys, xs = np.nonzero(self.L[y0:y1, x0:x1] >= self.occ_logit)
        points = np.column_stack(
            [
                self.ox + (xs + x0 + 0.5) * self.resolution,
                self.oy + (ys + y0 + 0.5) * self.resolution,
            ]
        )
        return points[np.linalg.norm(points - center[:2], axis=1) <= radius]
