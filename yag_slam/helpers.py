# Copyright 2019 Jariullah Safi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import math
import time
from numba import njit, prange
import numpy as np
from yag_slam_cpp import ScanMatcherConfig
from collections import namedtuple


@njit(nogil=True)
def occupancy_grid_map_to_correlation_grid(map_im, res, smear_deviation=0.05, occupied_value=0):
    V, U = np.where(map_im == occupied_value)
    kernel = calculate_kernel(res, smear_deviation)
    cgrid = np.zeros(map_im.shape)

    for u, v in zip(U, V):
        cgrid[v, u] = 1.0
        smear_point(u, v, cgrid, kernel)

    return cgrid

@njit(nogil=True)
def _project_2d_scan(ranges_, xx, yy, rr, min_angle, max_angle, range_threshold=12):
    ranges = ranges_.copy()
    ranges[np.abs(ranges) > range_threshold] = 0
    ranges[np.isnan(ranges)] = 0
    angle_min = min_angle
    angle_max = max_angle
    angles = np.linspace(angle_min, angle_max, len(ranges))

    angles = angles[ranges > 0]
    ranges = ranges[ranges > 0]

    x, y, r = xx, yy, rr

    _x = ranges * np.cos(angles)
    _y = ranges * np.sin(angles)

    xvals = x + _x * np.cos(r) - _y * np.sin(r)
    yvals = y + _y * np.cos(r) + _x * np.sin(r)
    return xvals, yvals


@njit(nogil=True)
def _get_point_readings(ranges_, xx, yy, rr, min_angle, max_angle, angle_increment, range_threshold=12):
    xvals = []
    yvals = []
    for ii, r_ in enumerate(ranges_):
        if r_ > range_threshold or np.isnan(r_):
            continue
        angle = rr + min_angle + ii * angle_increment
        xvals.append(xx + r_ * np.cos(angle))
        yvals.append(yy + r_ * np.sin(angle))
    return np.array(xvals), np.array(yvals)


def _transform_points(ptsx, ptsy, x, y, t):
    xx, yy = _rotate_points(ptsx, ptsy, t)
    return xx + x, yy + y


@njit(nogil=True)
def _rotate_points(ptsx, ptsy, angle):
    return (ptsx * np.cos(angle) - ptsy * np.sin(angle), ptsy * np.cos(angle) + ptsx * np.sin(angle))


@njit(nogil=True)
def world_to_grid(xy, ox, oy, res):
    return np.round((xy[0] - ox) / res, 0, np.empty_like(xy[0])), np.round((xy[1] - oy) / res, 0, np.empty_like(xy[0]))


@njit(nogil=True)
def calculate_kernel(res, smear_deviation):
    size = int(4 * np.round(smear_deviation / res) + 1)
    kernel = np.zeros((size, size))
    half_size = int(size / 2)
    for i_ in range(size):
        i = i_ - half_size
        for j_ in range(size):
            j = j_ - half_size
            sqdist = (i * res)**2 + (j * res)**2
            kernel[i_, j_] = np.exp(-0.5 * sqdist / (smear_deviation**2))
    return kernel


@njit(nogil=True)
def in_grid_bounds(x, y, w, h):
    return (0 <= x < w) and (0 <= y < h)


@njit(nogil=True)
def smear_point(gx, gy, cgrid, kernel):
    h, w = cgrid.shape
    size = kernel.shape[0]
    half_size = int(size / 2)

    for sx in range(size):
        for sy in range(size):
            x = gx + (sx - half_size)
            y = gy + (sy - half_size)
            if in_grid_bounds(x, y, w, h):
                candidate = kernel[sy, sx]
                curr = cgrid[y, x]
                if candidate > curr:
                    cgrid[y, x] = candidate


@njit(nogil=True)
def add_scan_to_grid(gx, gy, cgrid, kernel):
    h, w = cgrid.shape
    for i in range(len(gx)):
        x_ = gx[i]
        y_ = gy[i]
        if not in_grid_bounds(x_, y_, w, h):
            continue
        cgrid[y_, x_] = 1.0
        smear_point(x_, y_, cgrid, kernel)


@njit(nogil=True)
def score_grid_points_on_grid(cgrid, gx, gy, scaling_factor=100, intify=True):
    h, w = cgrid.shape
    res = 0.0
    for l in range(len(gx)):
        _x = int(gx[l])
        _y = int(gy[l])
        if _x >= 0 and _x < w and _y >= 0 and _y < h:
            additive = scaling_factor * float(cgrid[_y, _x])
            if intify:
                additive = int(additive)
            res += additive
    return res


@njit(nogil=True)
def score_world_points_on_grid(cgrid, ptsx, ptsy, ox, oy, grid_resolution, scaling_factor=100, intify=True):
    x, y = ptsx, ptsy
    gx, gy = world_to_grid((x, y), ox, oy, grid_resolution)
    return score_grid_points_on_grid(cgrid, gx, gy, scaling_factor, intify)


@njit(parallel=True, nogil=True)
def find_best_pose(cgrid, local_frame_points, cx, cy, ct, ox, oy, xy_search_size, xy_resolution, angle_search_size,
                   angle_resolution, grid_resolution, penalize_distance_from_center):
    """
    cgrid is the correlation grid, essentially an image where occupied space is 1.0 and the rest are 0 (or transitions between those numbers)
    local_frame_points is a tuple of (pts_y, pts_y), these are lidar points where the robot pose is 0, 0, 0
    cx, cy, ct are the 2D pose of the center of the search area (also center of cgrid)
    ox, oy are offsets that describe that lower left (upper left?) corner of cgrid in robot/world space
    xy_search_size is the vertical and horizontal offset from the center of the grid that poses will be searched
    xy_resolution is the physical resolution of this search
    angle_search_size and resolution are the same things as xy... but for angles
    grid_resolution is the actual resolution of cgrid
    penalize_distance_from_center is a boolean that controls if estimates far away from center should be higher cost
    """
    ptsx, ptsy = local_frame_points

    # Center of grid in real coordinates
    sx_ = ox + cgrid.shape[0] * grid_resolution / 2
    sy_ = oy + cgrid.shape[1] * grid_resolution / 2

    # Pose search arrays
    xvals = np.arange(-xy_search_size + cx, xy_search_size + cx, xy_resolution)
    yvals = np.arange(-xy_search_size + cy, xy_search_size + cy, xy_resolution)
    tvals = np.arange(-angle_search_size + ct, angle_search_size + ct, angle_resolution)

    dist_var_penalty = 0.5
    ang_var_penalty = 1.0
    min_ang_penalty = 0.9
    min_dist_penalty = 1.0

    h, w = cgrid.shape

    # 3D array to hold pose scores
    out = np.ones((len(xvals), len(yvals), len(tvals))) * -1

    for k in prange(len(tvals)):
        xx, yy = _rotate_points(ptsx, ptsy, tvals[k])
        for i in range(len(xvals)):
            for j in range(len(yvals)):
                x = xvals[i] + xx
                y = yvals[j] + yy

                res = score_world_points_on_grid(cgrid, x, y, ox, oy, grid_resolution)

                penalty_val = 1.0
                if penalize_distance_from_center:
                    squared_dist = (xvals[i] - sx_)**2 + (yvals[j] - sy_)**2
                    dist_penalty = 1.0 - 0.2 * squared_dist / (dist_var_penalty * grid_resolution)
                    # dist_penalty = max(dist_penalty, min_dist_penalty)

                    squared_ang_dist = (tvals[k] - ct)**2
                    ang_penalty = 1.0 - 0.2 * squared_ang_dist / (ang_var_penalty * grid_resolution)
                    # ang_penalty = max(ang_penalty, min_ang_penalty)

                    penalty_val = (dist_penalty * ang_penalty)

                out[i, j, k] = res / len(ptsx) * penalty_val / 100.0

    m = np.argmax(out)

    o, t, th = out.shape

    ii = m // (t * th)
    jj = (m % (t * th)) // th
    kk = (m % (t * th)) % th

    response = out[ii, jj, kk]

    bx = 0
    by = 0
    bt = 0
    # total_poses = 0

    out_ = np.where(out >= response - 0.00000001)

    norm_ = 0.0

    for idx in range(len(out_[0])):
        i = out_[0][idx]
        j = out_[1][idx]
        k = out_[2][idx]
        bx += xvals[i]
        by += yvals[j]
        bt += tvals[k]
        norm_ += 1.0

    bx /= norm_
    by /= norm_
    bt /= norm_

    # how to compute variance
    """
    find dx and dy (bestpose - centerpose)
    go over some x, y, theta points
    for each, get response (maybe only keep it if it's close to bestresponse)
    add response to norm
    accumulate XX by adding (x - dx)**2*response, same for YY
    accumulate XY by adding (y-dy)*(x-dx)*response

    divide everyone by norm

    maybe divide everyone by the best response
    """

    XX = 0
    YY = 0
    XY = 0
    TH = 0
    norm = 0.0

    xs = max(0, ii - 5)
    ys = max(0, jj - 5)
    xe = min(len(xvals) - 1, ii + 6)
    ye = min(len(yvals) - 1, jj + 6)

    for ii_ in range(xs, xe):
        for jj_ in range(ys, ye):
            res_ = out[ii_, jj_, kk]
            # if res_ < response - 0.1:
            #     continue
            x_ = xvals[ii_]
            y_ = yvals[jj_]

            norm += res_
            XX += res_ * (x_ - bx)**2
            YY += res_ * (y_ - by)**2
            XY += (x_ - bx) * (y_ - by) * res_

    th_norm = 0.0
    ts = max(0, kk - 5)
    te = min(len(tvals) - 1, kk + 6)
    for kk_ in range(ts, te):
        res_ = out[ii, jj, kk_]
        # if res_ < response - 0.1:
        #     continue
        t_ = tvals[kk_]
        th_norm += res_
        TH += res_ * (t_ - bt)**2

    return [response, bx, by, bt, XX / norm / response, YY / norm / response, XY / norm / response, TH / th_norm]


@njit
def validate_points(ptsx, ptsy, vpx, vpy):
    # pts are known to not be NAN
    msd = 0.2**2
    retx = []
    rety = []
    fpx = ptsx[0]
    fpy = ptsy[0]

    tmpx = [0.0]
    tmpy = [0.0]

    for i in range(1, len(ptsx)):
        cpx = ptsx[i]
        cpy = ptsy[i]

        tmpx.append(cpx)
        tmpy.append(cpy)

        if (fpx - cpx)**2 + (fpy - cpy)**2 > msd:
            a = vpy - fpy
            b = fpx - vpx
            c = fpy * vpx - fpx * vpy
            fpx = cpx
            fpy = cpy
            ss = cpx * a + cpy * b + c
            if ss > 0.0:
                retx.extend(tmpx[1:])
                rety.extend(tmpy[1:])
            tmpx = [0.0]
            tmpy = [0.0]
    return retx, rety


def print_config(config):
    for name in dir(config):
        if '_' == name[0]:
            continue
        print("{}: {}".format(name, config.__getattribute__(name)))

default_config = {
    "angle_variance_penalty": 0.349,
    "distance_variance_penalty": 0.3,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "fine_search_angle_resolution": 0.00349,
    "use_response_expansion": True,
    "range_threshold": 20,
    "minimum_angle_penalty": 0.9,
    "search_size": 0.3,
    "resolution": 0.01,
    "smear_deviation": 0.03,
}

default_config_loop = {
    "angle_variance_penalty": 0.349,
    "distance_variance_penalty": 0.3,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "fine_search_angle_resolution": 0.00349,
    "use_response_expansion": True,
    "range_threshold": 20,
    "minimum_angle_penalty": 0.9,
    "resolution": 0.05,
    "search_size": 8.0,
    "smear_deviation": 0.03,
}


def make_config(d):
    config = ScanMatcherConfig()
    config_params = default_config.copy()
    if d:
        config_params.update(d)

    for key, value in config_params.items():
        config.__setattr__(key, value)

    return config


def poses_dist_squared(p1, p2):
    return (p1.x - p2.x)**2 + (p1.y - p2.y)**2


def scans_dist_squared(scan1, scan2):
    p1 = scan1.corrected_pose
    p2 = scan2.corrected_pose
    return (p1.x - p2.x)**2 + (p1.y - p2.y)**2


def scans_dist(scan1, scan2):
    return math.sqrt(scans_dist_squared(scan1, scan2))


Pose2 = namedtuple('Pose2', ['x', 'y'])


class RadiusHashSearch(object):
    def __init__(self, elements, accessor=lambda v: v.obj.corrected_pose, res=1.0):
        self.res = res
        self.hmap = {}
        self.accessor = accessor

        for el in elements:
            self.add_new_element(el)

    def pose_to_key(self, p):
        # return f"{int(p.x/self.res)}_{int(p.y/self.res)}"
        return (int(p.x / self.res), int(p.y / self.res))

    def key_to_pose(self, key):
        # x, y = key.split('_')
        x, y = key
        return Pose2(float(x) * self.res, float(y) * self.res)

    def add_new_element(self, element):
        key = self.pose_to_key(self.accessor(element))
        if key not in self.hmap:
            self.hmap[key] = []
        self.hmap[key].append(element)

    def crude_radius_search(self, start_pose, radius):
        """
        returns all boxes around the start_pose that are within radius + 2*res
        """
        r2 = (radius + self.res)**2
        all_elements = []
        for key, elements in self.hmap.items():
            pose = self.key_to_pose(key)
            if poses_dist_squared(pose, start_pose) < r2:
                all_elements.extend(elements)

        return all_elements


@njit(parallel=False, nogil=True)
def find_best_pose_non_symmetric(cgrid, local_frame_points, cx, cy, ct, ox, oy, xy_search_size, xy_resolution, angle_search_size,
                                 angle_resolution, grid_resolution, penalize_distance_from_center):
    """
    cgrid is the correlation grid, essentially an image where occupied space is 1.0 and the rest are 0 (or transitions between those numbers)
    local_frame_points is a tuple of (pts_y, pts_y), these are lidar points where the robot pose is 0, 0, 0
    cx, cy, ct are the 2D pose of the center of the search area (also center of cgrid)
    ox, oy are offsets that describe that lower left (upper left?) corner of cgrid in robot/world space
    xy_search_size is the vertical and horizontal offset from the center of the grid that poses will be searched
    xy_resolution is the physical resolution of this search
    angle_search_size and resolution are the same things as xy... but for angles
    grid_resolution is the actual resolution of cgrid
    penalize_distance_from_center is a boolean that controls if estimates far away from center should be higher cost
    """
    ptsx, ptsy = local_frame_points

    # Center of grid in real coordinates
    sx_ = cx
    sy_ = cy

    # Pose search arrays
    xvals = np.arange(-xy_search_size + cx, xy_search_size + cx, xy_resolution)
    yvals = np.arange(-xy_search_size + cy, xy_search_size + cy, xy_resolution)
    tvals = np.arange(-angle_search_size + ct, angle_search_size + ct, angle_resolution)

    dist_var_penalty = 0.5
    ang_var_penalty = 1.0
    min_ang_penalty = 0.9
    min_dist_penalty = 1.0

    h, w = cgrid.shape

    # 3D array to hold pose scores
    out = np.ones((len(xvals), len(yvals), len(tvals))) * -1

    for k in prange(len(tvals)):
        xx, yy = _rotate_points(ptsx, ptsy, tvals[k])
        for i in range(len(xvals)):
            for j in range(len(yvals)):
                x = xvals[i] + xx
                y = yvals[j] + yy

                res = score_world_points_on_grid(cgrid, x, y, ox, oy, grid_resolution)

                penalty_val = 1.0
                if penalize_distance_from_center:
                    squared_dist = (xvals[i] - sx_)**2 + (yvals[j] - sy_)**2
                    dist_penalty = 1.0 - 0.2 * squared_dist / (dist_var_penalty * grid_resolution)
                    # dist_penalty = max(dist_penalty, min_dist_penalty)

                    squared_ang_dist = (tvals[k] - ct)**2
                    ang_penalty = 1.0 - 0.2 * squared_ang_dist / (ang_var_penalty * grid_resolution)
                    # ang_penalty = max(ang_penalty, min_ang_penalty)

                    penalty_val = (dist_penalty * ang_penalty)

                out[i, j, k] = res / len(ptsx) * penalty_val / 100.0

    m = np.argmax(out)

    o, t, th = out.shape

    ii = m // (t * th)
    jj = (m % (t * th)) // th
    kk = (m % (t * th)) % th

    response = out[ii, jj, kk]

    bx = 0
    by = 0
    bt = 0
    # total_poses = 0

    out_ = np.where(out >= response - 0.00000001)

    norm_ = 0.0

    for idx in range(len(out_[0])):
        i = out_[0][idx]
        j = out_[1][idx]
        k = out_[2][idx]
        bx += xvals[i]
        by += yvals[j]
        bt += tvals[k]
        norm_ += 1.0

    bx /= norm_
    by /= norm_
    bt /= norm_

    # how to compute variance
    """
    find dx and dy (bestpose - centerpose)
    go over some x, y, theta points
    for each, get response (maybe only keep it if it's close to bestresponse)
    add response to norm
    accumulate XX by adding (x - dx)**2*response, same for YY
    accumulate XY by adding (y-dy)*(x-dx)*response

    divide everyone by norm

    maybe divide everyone by the best response
    """

    XX = 0
    YY = 0
    XY = 0
    TH = 0
    norm = 0.0

    xs = max(0, ii - 5)
    ys = max(0, jj - 5)
    xe = min(len(xvals) - 1, ii + 6)
    ye = min(len(yvals) - 1, jj + 6)

    for ii_ in range(xs, xe):
        for jj_ in range(ys, ye):
            res_ = out[ii_, jj_, kk]
            # if res_ < response - 0.1:
            #     continue
            x_ = xvals[ii_]
            y_ = yvals[jj_]

            norm += res_
            XX += res_ * (x_ - bx)**2
            YY += res_ * (y_ - by)**2
            XY += (x_ - bx) * (y_ - by) * res_

    th_norm = 0.0
    ts = max(0, kk - 5)
    te = min(len(tvals) - 1, kk + 6)
    for kk_ in range(ts, te):
        res_ = out[ii, jj, kk_]
        # if res_ < response - 0.1:
        #     continue
        t_ = tvals[kk_]
        th_norm += res_
        TH += res_ * (t_ - bt)**2

    return [response, bx, by, bt, XX / norm / response, YY / norm / response, XY / norm / response, TH / th_norm]


def visualize_slam_threeviz(slam, color="red", prefix=""):
    from threeviz.api import plot_3d, plot_pose, plot_polygon
    for ii, v in enumerate(slam.graph.vertices):
        lrs = v.obj
        _ = plot_3d(lrs.points()[0], lrs.points()[1], lrs.points()[0]*0, prefix+"scan"+str(ii), size=0.02, color=color)
        plot_pose(lrs.corrected_pose, prefix+"pose" + str(ii), size=0.1)
        time.sleep(0.001)

    for ii, e in enumerate(slam.graph.edges):
        s, t = e.source.obj, e.target.obj
        plot_polygon([[s.corrected_pose.x, s.corrected_pose.y, 0], 
                      [t.corrected_pose.x, t.corrected_pose.y, 0]], prefix+"edge" + str(ii), color=color)