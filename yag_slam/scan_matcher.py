from numba import njit, prange
from tiny_tf.tf import Transform
import numpy as np
import cv2
import time
from collections import defaultdict, namedtuple
from yag_slam_cpp import LocalizedRangeScan as LocalizedRangeScanCpp
from yag_slam_cpp import Pose2 as Pose2Cpp
from yag_slam_cpp import Wrapper, LaserScanConfig
from yag_slam import make_config, default_config, default_config_loop


ScanMatcherResult = namedtuple('ScanMatcherResult',
                               ['response', 'covariance', 'best_pose', 'meta'])


class LocalizedRangeScan:
    def __init__(self, ranges, min_angle, max_angle, angle_increment, min_range, max_range, range_threshold, x, y, t):
        self.ranges = np.array(ranges).copy()
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle_increment = angle_increment
        self.min_range = min_range
        self.max_range = max_range
        self.range_threshold = range_threshold

        self._odom_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)
        self._corrected_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)
        self._id = 0
        self._scan = LocalizedRangeScanCpp(
            LaserScanConfig(min_angle, max_angle, angle_increment, min_range, max_range, range_threshold, ""),
            ranges,
            Pose2Cpp(x, y, t),
            Pose2Cpp(x, y, t),
            0, 0.0)

    @property
    def num(self):
        return self._id

    @num.setter
    def num(self, val):
        self._id = val
        self._scan.num = val

    def _get_pose(self, odom=False):
        return self._odom_pose if odom else self._corrected_pose

    def _set_pose(self, val, odom=False):
        # val should be of type Transform
        cpp_pose = Pose2Cpp(val.x, val.y, val.euler[-1])
        if odom:
            self._odom_pose = val
            self._scan.odom_pose = cpp_pose
        else:
            self._corrected_pose = val
            self._scan.corrected_pose = cpp_pose

    @property
    def odom_pose(self):
        return self._get_pose(True)

    @odom_pose.setter
    def odom_pose(self, val):
        self._set_pose(val, True)

    @property
    def corrected_pose(self):
        return self._get_pose(False)

    @corrected_pose.setter
    def corrected_pose(self, val):
        self._set_pose(val, False)

    def points(self, odom=False):
        p = self.corrected_pose if not odom else self.odom_pose
        return self.points_for_pose2d(p.x, p.y, p.euler[-1])

    def points_local(self):
        return self.points_for_pose2d(0, 0, 0)

    def points_for_pose2d(self, x, y, t):
        return _get_point_readings(self.ranges, x, y, t, self.min_angle, 0.0, self.angle_increment, self.range_threshold)

    def copy(self):
        p = self.corrected_pose
        return LocalizedRangeScan(self.ranges.copy(), self.min_angle, self.max_angle, self.angle_increment, self.min_range, self.max_range, self.range_threshold, p.x, p.y, p.euler[-1])

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


class Scan2DMatcherCpp(object):
    def __init__(self, config_dict={}, loop=False):
        cfg = default_config if not loop else default_config_loop
        cfg = cfg.copy()
        cfg.update(config_dict)
        self._config = make_config(cfg)
        self._matcher = Wrapper(self._config)

    def match_scan(self, query, base_scans, penalty=True, do_fine=False):
        res = self._matcher.match_scan(query._scan, [b._scan for b in base_scans], penalty, do_fine)
        return ScanMatcherResult(res.response, res.covariance, Transform.from_pose2d(res.best_pose), None)


class Scan2DMatcher(object):
    def __init__(self, search_size=0.5, resolution=0.01, angle_size=0.349, angle_res=0.0349, range_threshold=12, smear_deviation=0.1):
        self.search_size = search_size
        self.resolution = resolution
        self.angle_size = angle_size
        self.angle_res = angle_res
        self.range_threshold = range_threshold
        self.smear_deviation = smear_deviation

    def match_scan(self, query, base_scans, penalty=True, do_fine=True):
        search_size = self.search_size
        resolution = self.resolution
        smear_deviation = self.smear_deviation
        angle_size = self.angle_size
        angle_res = self.angle_res
        range_threshold = self.range_threshold

        grid_size = int(search_size/resolution + 1 + 2 * range_threshold/resolution)
        cgrid = np.zeros((grid_size, grid_size))
        hh, ww = cgrid.shape

        ox = query.corrected_pose.x - 0.5*(ww-1)*resolution
        oy = query.corrected_pose.y - 0.5*(hh-1)*resolution

        kernel = calculate_kernel(resolution, smear_deviation)
        for scan in base_scans:
            ptsx, ptsy = scan.points()

            ptsx, ptsy = validate_points(ptsx, ptsy, query.corrected_pose.x, query.corrected_pose.y)

            gx, gy = world_to_grid((np.array(ptsx), np.array(ptsy)), ox, oy, resolution)
            gx = gx.astype('int32')
            gy = gy.astype('int32')

            add_scan_to_grid(gx, gy, cgrid, kernel)

        pts_local = query.points_local()

        res, x, y, t, xx, yy, xy, th = find_best_pose(
            cgrid, pts_local, query.corrected_pose.x, query.corrected_pose.y,
            query.corrected_pose.euler[-1], ox, oy, search_size*0.5,
            resolution*2, angle_size*0.5, angle_res, resolution, penalty)

        if do_fine:
            res, x, y, t, xx_, yy_, xy_, th = find_best_pose(
                cgrid, pts_local, x, y, t, ox, oy, resolution*2, resolution,
                0.0349*0.5, 0.00349, resolution, penalty)
        else:
            th = 4 * angle_res

        meta = None
        meta = {'grid': cgrid, 'kernel': kernel}
        covar = np.array([[xx, xy, 0], [xy, yy, 0], [0, 0, th]])

        return ScanMatcherResult(res, covar,
                                 Transform.from_position_euler(x, y, 0, 0, 0, t),
                                 meta)


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

@njit(nogil=True)
def _rotate_points(ptsx, ptsy, angle):
    return (ptsx * np.cos(angle) - ptsy * np.sin(angle),
            ptsy * np.cos(angle) + ptsx * np.sin(angle))

@njit(nogil=True)
def world_to_grid(xy, ox, oy, res):
    return np.round((xy[0]-ox)/res, 0, np.empty_like(xy[0])), np.round((xy[1]-oy)/res, 0, np.empty_like(xy[0]))

@njit(nogil=True)
def calculate_kernel(res, smear_deviation):
    size = int(4 * np.round(smear_deviation/res) + 1)
    kernel = np.zeros((size, size))
    half_size = int(size/2)
    for i_ in range(size):
        i = i_ - half_size
        for j_ in range(size):
            j = j_ - half_size
            sqdist = (i*res)**2 + (j*res)**2
            kernel[i_, j_] = np.exp(-0.5 * sqdist / (smear_deviation**2))
    return kernel

@njit(nogil=True)
def smear_point(gx, gy, cgrid, kernel):
    h, w = cgrid.shape
    size = kernel.shape[0]
    half_size = int(size/2)

    for sx in range(size):
        for sy in range(size):
            x = gx + (sx - half_size)
            y = gy + (sy - half_size)
            if x >= 0 and x < w and y >=0 and y < h:
                candidate = kernel[sy, sx]
                curr = cgrid[y, x]
                if candidate > curr:
                    cgrid[y, x] = candidate

@njit(nogil=True)
def add_scan_to_grid(gx, gy, cgrid, kernel):
    for i in range(len(gx)):
        x_ = gx[i]
        y_ = gy[i]
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
            additive = scaling_factor*float(cgrid[_y, _x])
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
def find_best_pose(cgrid, local_frame_points, cx, cy, ct, ox, oy, xy_search_size, xy_resolution, angle_search_size, angle_resolution, grid_resolution, penalize_distance_from_center):
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
    sx_ = ox + cgrid.shape[0]*grid_resolution/2
    sy_ = oy + cgrid.shape[1]*grid_resolution/2

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
    out = np.ones((len(xvals), len(yvals), len(tvals)))*-1

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
                    dist_penalty = 1.0 - 0.2*squared_dist/(dist_var_penalty*grid_resolution)
                    # dist_penalty = max(dist_penalty, min_dist_penalty)

                    squared_ang_dist = (tvals[k] - ct)**2
                    ang_penalty = 1.0 - 0.2 * squared_ang_dist / (ang_var_penalty*grid_resolution)
                    # ang_penalty = max(ang_penalty, min_ang_penalty)

                    penalty_val = (dist_penalty * ang_penalty)

                out[i, j, k] = res/len(ptsx)*penalty_val/100.0

    m = np.argmax(out)

    o, t, th = out.shape

    ii = m//(t*th)
    jj = (m%(t*th))//th
    kk = (m%(t*th))%th

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
            XX += res_*(x_ - bx)**2
            YY += res_*(y_ - by)**2
            XY += (x_ - bx) * (y_ - by)*res_

    th_norm = 0.0
    ts = max(0, kk - 5)
    te = min(len(tvals) - 1, kk + 6)
    for kk_ in range(ts, te):
        res_ = out[ii, jj, kk_]
        # if res_ < response - 0.1:
        #     continue
        t_ = tvals[kk_]
        th_norm += res_
        TH += res_*(t_ - bt)**2

    return [response, bx, by, bt, XX/norm/response, YY/norm/response, XY/norm/response, TH/th_norm]
