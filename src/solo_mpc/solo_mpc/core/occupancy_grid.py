"""Persistent occupancy-grid primitives shared by the local planner."""

import numpy as np

# Defaults are explicit; ROS scenario loading passes every planning parameter.
_RES_DEFAULT = 0.15
_INFLATE_DEFAULT = 0.51
_SENSING_DEFAULT = 3.0

def _shift(a, dy, dx):
    """Dịch mảng theo (dy,dx), phần trống = 0 (không wrap-around)."""
    out = np.zeros_like(a)
    H, W = a.shape
    y0, y1 = max(0, -dy), H - max(0, dy)
    x0, x1 = max(0, -dx), W - max(0, dx)
    out[y0 + dy:y1 + dy, x0 + dx:x1 + dx] = a[y0:y1, x0:x1]
    return out


def _dilate_disk(mask, r_cells):
    """Nở (dilation) mask nhị phân bằng đĩa bán kính r_cells."""
    if r_cells <= 0:
        return mask.copy()
    out = mask.copy()
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy <= r_cells * r_cells and (dx or dy):
                out |= _shift(mask, dy, dx)
    return out


# ============================================================
# Persistent log-odds occupancy grid (toàn world, bền vững)
# ============================================================
class PersistentLogOddsGrid:
    """
    Grid log-odds cố định trong khung world (không recenter mỗi frame).
    Trạng thái suy ra từ log-odds L:
        L <= free_logit  -> FREE      (đi được)
        L >= occ_logit   -> OCCUPIED  (vật cản, sẽ được inflate)
        ở giữa / chưa quan sát (L=0) -> UNKNOWN (mặc định CẤM đi)
    """

    def __init__(self, world_bounds, resolution=_RES_DEFAULT,
                 inflate_radius=_INFLATE_DEFAULT, sensing_radius=_SENSING_DEFAULT,
                 n_ray_bins=360,
                 l_occ=0.9, l_free=-0.7, l_min=-2.0, l_max=3.5,
                 occ_thresh=0.65, free_thresh=0.35,
                 treat_unknown_as_blocked=True):
        xmin, ymin, xmax, ymax = world_bounds
        self.ox, self.oy = float(xmin), float(ymin)
        self.resolution = float(resolution)
        self.sensing_radius = float(sensing_radius)
        self.n_bins = int(n_ray_bins)

        self.size_x = int(np.ceil((xmax - xmin) / self.resolution)) + 1
        self.size_y = int(np.ceil((ymax - ymin) / self.resolution)) + 1

        self.L = np.zeros((self.size_y, self.size_x), dtype=np.float32)

        self.l_occ, self.l_free = float(l_occ), float(l_free)
        self.l_min, self.l_max = float(l_min), float(l_max)
        # ngưỡng prob -> logit
        self.occ_logit  = float(np.log(occ_thresh / (1 - occ_thresh)))
        self.free_logit = float(np.log(free_thresh / (1 - free_thresh)))

        self.treat_unknown_as_blocked = bool(treat_unknown_as_blocked)
        self.set_inflate(inflate_radius)

        self.blocked = np.zeros_like(self.L, dtype=bool)
        self._dirty = True

    # ---- inflate ----
    def set_inflate(self, radius):
        self.inflate_radius = float(radius)
        self.inflate_cells = int(np.ceil(self.inflate_radius / self.resolution))

    # ---- toạ độ ----
    def world_to_grid(self, wx, wy):
        ix = int((wx - self.ox) / self.resolution)
        iy = int((wy - self.oy) / self.resolution)
        return ix, iy

    def grid_to_world(self, ix, iy):
        wx = self.ox + (ix + 0.5) * self.resolution
        wy = self.oy + (iy + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.size_x and 0 <= iy < self.size_y

    def is_within_world(self, wx, wy):
        return (self.ox <= wx < self.ox + self.size_x * self.resolution and
                self.oy <= wy < self.oy + self.size_y * self.resolution)

    # ---- nạp linear index hợp lệ từ mảng toạ độ world ----
    def _lin(self, xs, ys):
        ix = ((xs - self.ox) / self.resolution).astype(np.int64)
        iy = ((ys - self.oy) / self.resolution).astype(np.int64)
        m = (ix >= 0) & (ix < self.size_x) & (iy >= 0) & (iy < self.size_y)
        return iy[m] * self.size_x + ix[m]

    # ==========================================================
    # OBSERVE: cập nhật map từ 1 lần quét LiDAR
    #   robot_pos  : (x, y) gốc tia
    #   hit_points : Nx2(3) điểm LiDAR đập vào bề mặt vật cản
    # A bin without a hit is carved free over the whole sensing range, so every
    # bin must hold a LiDAR ray: bin bi is centred on the angle -pi + bi*2pi/n,
    # the angles of a LiDAR with n rays per turn starting at -pi, and n_bins must
    # equal that ray count (config derives GRID_RAY_BINS from LIDAR_ANGULAR_RES).
    # With 360 bins for 180 rays every other bin was empty and was carved free
    # straight through the obstacles, so their insides read as observed free.
    # ==========================================================
    def observe(self, robot_pos, hit_points):
        rx, ry = float(robot_pos[0]), float(robot_pos[1])
        res = self.resolution
        step = res * 0.5

        # gom min-range theo bin góc
        bin_range = np.full(self.n_bins, np.inf, dtype=np.float64)
        hp = None
        if hit_points is not None and len(hit_points) > 0:
            hp = np.asarray(hit_points, dtype=float)[:, :2]
            d = hp - np.array([rx, ry])
            rng = np.hypot(d[:, 0], d[:, 1])
            ang = np.arctan2(d[:, 1], d[:, 0])               # [-pi, pi]
            b = np.round((ang + np.pi) / (2 * np.pi) * self.n_bins).astype(int) % self.n_bins
            for bi, ri in zip(b, rng):
                if ri < bin_range[bi]:
                    bin_range[bi] = ri

        free_idx = []
        occ_idx = []

        two_pi = 2 * np.pi
        for bi in range(self.n_bins):
            ang = -np.pi + bi * (two_pi / self.n_bins)
            ca, sa = np.cos(ang), np.sin(ang)
            r_hit = bin_range[bi]
            seen = np.isfinite(r_hit) and r_hit <= self.sensing_radius
            free_len = min(r_hit, self.sensing_radius) if np.isfinite(r_hit) \
                else self.sensing_radius

            # FREE dọc tia tới ngay trước bề mặt
            n = int(max((free_len - res) / step, 0))
            if n > 0:
                rr = np.arange(n) * step
                xs = rx + rr * ca
                ys = ry + rr * sa
                lin = self._lin(xs, ys)
                if lin.size:
                    free_idx.append(np.unique(lin))   # unique trong cùng tia

            # OCCUPIED tại điểm hit
            if seen:
                hx, hy = rx + r_hit * ca, ry + r_hit * sa
                lin = self._lin(np.array([hx]), np.array([hy]))
                if lin.size:
                    occ_idx.append(lin)

        # mark thẳng các điểm hit thật (giữ độ nét bề mặt)
        if hp is not None:
            lin = self._lin(hp[:, 0], hp[:, 1])
            if lin.size:
                occ_idx.append(lin)

        Lflat = self.L.reshape(-1)
        occ = np.concatenate(occ_idx) if occ_idx else np.zeros(0, np.int64)
        if free_idx:
            # a cell hit in this scan is not cleared by other rays grazing it
            # (otherwise flying close along a wall erases the wall from the map)
            free = np.concatenate(free_idx)
            # nor a cell already known to be occupied (static obstacles: the LiDAR
            # does not see other UAVs; revisit if moving obstacles are added)
            keep = np.isin(free, occ) | (Lflat[free] >= self.occ_logit)
            np.add.at(Lflat, free[~keep], self.l_free)
        if occ.size:
            np.add.at(Lflat, occ, self.l_occ)

        np.clip(self.L, self.l_min, self.l_max, out=self.L)
        self._dirty = True

    # ==========================================================
    # REBUILD: dựng lớp planning (blocked) từ log-odds
    # ==========================================================
    def rebuild(self):
        occ = self.L >= self.occ_logit
        blocked = _dilate_disk(occ, self.inflate_cells)
        if self.treat_unknown_as_blocked:
            free = self.L <= self.free_logit
            unknown = (~free) & (~occ)
            blocked |= unknown
        self.blocked = blocked
        self._dirty = False

    def _ensure(self):
        if self._dirty:
            self.rebuild()

    # ---- truy vấn cho planner ----
    def is_blocked(self, ix, iy):
        if not self.in_bounds(ix, iy):
            return True
        self._ensure()
        return bool(self.blocked[iy, ix])

    def is_free(self, ix, iy):
        return self.in_bounds(ix, iy) and not self.is_blocked(ix, iy)

    def to_matrix(self):
        """matrix cho pathfind: -1 = obstacle, 1 = walkable. row=iy, col=ix."""
        self._ensure()
        return np.where(self.blocked, -1, 1).tolist()

    # ---- nearest free (xoáy ốc) ----
    def nearest_free(self, ix, iy, max_radius=20, known=False):
        """Nearest free cell in growing square rings. known=True also requires
        the cell to have been observed free: the inside of an obstacle is never
        observed and stays unknown, which the optimistic policy treats as free,
        so a UAV that ends up in an obstacle's inflated border could otherwise
        be snapped INTO the obstacle and plan straight through it."""
        self._ensure()

        def ok(x, y):
            return self.is_free(x, y) and not (known and self.L[y, x] > self.free_logit)

        if ok(ix, iy):
            return ix, iy
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if ok(nx, ny):
                        return nx, ny
        return None, None

    # ---- line-of-sight: đoạn p0->p1 có sạch không (cùng grid!) ----
    def segment_clear(self, wp0, wp1):
        self._ensure()
        p0 = np.asarray(wp0[:2], float)
        p1 = np.asarray(wp1[:2], float)
        d = p1 - p0
        dist = float(np.hypot(d[0], d[1]))
        if dist < 1e-9:
            ix, iy = self.world_to_grid(p0[0], p0[1])
            return self.is_free(ix, iy)
        n = int(dist / (self.resolution * 0.5)) + 1
        ts = np.linspace(0.0, 1.0, n)
        xs = p0[0] + ts * d[0]
        ys = p0[1] + ts * d[1]
        for wx, wy in zip(xs, ys):
            ix, iy = self.world_to_grid(wx, wy)
            if self.is_blocked(ix, iy):
                return False
        return True




# ============================================================
# Smoothing: string-pulling dùng line-of-sight trên CÙNG grid
# ============================================================
def _string_pull(path_world, grid: PersistentLogOddsGrid):
    if len(path_world) <= 2:
        return [list(p) for p in path_world]
    pts = [np.asarray(p[:2], float) for p in path_world]
    out = [pts[0]]
    anchor = 0
    i = 1
    while i < len(pts):
        # cố nối anchor -> i+? xa nhất còn line-of-sight
        if i == len(pts) - 1 or not grid.segment_clear(pts[anchor], pts[i + 1]):
            if not grid.segment_clear(pts[anchor], pts[i]):
                # đoạn anchor->i cũng kẹt: giữ điểm i-1 làm mốc an toàn
                out.append(pts[i - 1])
                anchor = i - 1
            else:
                out.append(pts[i])
                anchor = i
        i += 1
    if not np.allclose(out[-1], pts[-1]):
        out.append(pts[-1])
    return [list(p) for p in out]
