from numba import njit, prange
from tiny_tf.tf import Transform
import numpy as np
import cv2
import time
from collections import defaultdict, namedtuple


ScanMatcherResult = namedtuple('ScanMatcherResult',
                               ['response', 'covariance', 'best_pose', 'meta'])


class LocalizedRangeScan:
    def __init__(self, ranges, min_angle, max_angle, x, y, t):
        self.ranges = np.array(ranges)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.odom_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)
        self.corrected_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)

    def points(self, odom=False):
        p = self.corrected_pose if not odom else self.odom_pose
        xx, yy = _project_2d_scan(self.ranges, p.x, p.y, p.euler[-1], self.min_angle, self.max_angle)
        return (xx, yy)

    def copy(self):
        p = self.corrected_pose
        return LocalizedRangeScan(self.ranges.copy(), self.min_angle, self.max_angle, p.x, p.y, p.euler[-1])

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

class Scan2DMatcher(object):
    def __init__(self, search_size=0.5, resolution=0.01, angle_size=0.349, angle_res=0.0349, range_threshold=12):
        self.search_size = search_size
        self.resolution = resolution
        self.angle_size = angle_size
        self.angle_res = angle_res
        self.range_threshold = range_threshold

    def match_scan(self, query, base_scans, penalty=True, do_fine=True):
        search_size = self.search_size
        resolution = self.resolution
        smear_deviation = 0.1
        angle_size = self.angle_size
        angle_res = self.angle_res
        range_threshold = self.range_threshold

        grid_size = int(search_size/resolution + 1 + 2 * range_threshold/resolution)
        cgrid = np.zeros((grid_size, grid_size))
        hh, ww = cgrid.shape

        ox = query.corrected_pose.x - 0.5*(ww - 1)*resolution
        oy = query.corrected_pose.y - 0.5*(hh - 1)*resolution

        for scan in base_scans:
            ptsx, ptsy = scan.points()

            ptsx, ptsy = validate_points(ptsx, ptsy, query.corrected_pose.x, query.corrected_pose.y)

            gx, gy = world_to_grid((np.array(ptsx), np.array(ptsy)), ox, oy, resolution)
            gx = np.round(gx).astype('int32')
            gy = np.round(gy).astype('int32')
            kernel = add_scan_to_grid(gx, gy, cgrid, resolution, smear_deviation)

        # cgrid = cv2.dilate(cgrid, np.ones((3, 3)))
        # cgrid = cv2.GaussianBlur(cgrid, (15, 15), 15)

        res, x, y, t, xx, yy, xy, th = find_best_pose(
            cgrid, query.ranges, query.min_angle, query.max_angle,
            query.corrected_pose.x, query.corrected_pose.y,
            query.corrected_pose.euler[-1], ox, oy, search_size*0.5,
            resolution, angle_size * 0.5, angle_res, resolution,
            self.range_threshold, penalty)

        # if do_fine and False:
        #     res, x_, y_, t, xx_, yy_, xy_, th = find_best_pose(
        #         cgrid, query.ranges, query.min_angle, query.max_angle, x,
        #         y, t, ox, oy, resolution, resolution,
        #         angle_res * 0.5, angle_res * 0.1, resolution, self.range_threshold, penalty)

        meta = None
        meta = {'grid': cgrid, 'kernel': kernel}
        covar = np.array([[xx, xy, 0], [xy, yy, 0], [0, 0, th]])
        # covar = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


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
def world_to_grid(xy, ox, oy, res):
    return (xy[0]-ox)/res, (xy[1]-oy)/res

@njit(nogil=True)
def calculate_kernel(res, smear_deviation):
    size = int(2 * np.round(smear_deviation/res) + 1)
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
def add_scan_to_grid(gx, gy, cgrid, res, smear_deviation):
    h, w = cgrid.shape
    kernel = calculate_kernel(res, smear_deviation)
    size = kernel.shape[0]
    half_size = int(size/2)
    for i in range(len(gx)):
        x_ = gx[i]
        y_ = gy[i]
        for sx in range(size):
            for sy in range(size):
                x = x_ + (sx - half_size)
                y = y_ + (sy - half_size)
                if x >= 0 and x < w and y >=0 and y < h:
                    candidate = kernel[sy, sx]
                    curr = cgrid[y, x]
                    if candidate > curr:
                        cgrid[y, x] = candidate
    return kernel


@njit(parallel=True, nogil=True)
def find_best_pose(cgrid, ranges, min_angle, max_angle, cx, cy, ct, ox, oy, search_size, reso, angle_size, angle_res, projection_res, range_threshold, do_penalize):
    xvals = np.arange(-search_size + cx, search_size + cx, reso)
    yvals = np.arange(-search_size + cy, search_size + cy, reso)
    tvals = np.arange(-angle_size + ct, angle_size + ct, angle_res)

    dist_var_penalty = 0.5
    ang_var_penalty = 1.0
    min_ang_penalty = 0.9
    min_dist_penalty = 1.0

    h, w = cgrid.shape

    out = np.zeros((len(xvals), len(yvals), len(tvals)))
    penalty = np.zeros((len(xvals), len(yvals), len(tvals)))

    for k in prange(len(tvals)):
        xx, yy = _project_2d_scan(ranges, 0, 0, tvals[k], min_angle, max_angle, range_threshold)
        for j in range(len(yvals)):
            for i in range(len(xvals)):
                x = xx.copy() + xvals[i]
                y = yy.copy() + yvals[j]
                gx, gy = world_to_grid((x, y), ox, oy, projection_res)
                res = 0.0
                for l in range(len(gx)):
                    _x = np.round(gx[l])
                    _y = np.round(gy[l])
                    if _x >= 0 and _x < w and _y >=0 and _y < h:
                        res += float(cgrid[int(_y), int(_x)])

                penalty_val = 0.5
                if do_penalize:
                    squared_dist = (xvals[i] - cx)**2 + (yvals[j] - cy)**2
                    dist_penalty = 1.0 - 0.2*squared_dist/dist_var_penalty
                    dist_penalty = max(dist_penalty, min_dist_penalty)

                    squared_ang_dist = (tvals[k] - ct)**2
                    ang_penalty = 1.0 - 0.2 * squared_ang_dist / ang_var_penalty
                    ang_penalty = max(ang_penalty, min_ang_penalty)

                    penalty_val = (dist_penalty * ang_penalty)

                penalty[i, j, k] = penalty_val
                out[i, j, k] = res/len(gx)

    m = np.argmax(out)

    o, t, th = out.shape

    ii = m//(t*th)
    jj = (m%(t*th))//th
    kk = (m%(t*th))%th

    response = out[ii, jj, kk]
    # print(response, penalty[ii, jj, kk])
    response = response * penalty[ii, jj, kk]

    bx = 0
    by = 0
    bt = 0
    # total_poses = 0

    out_ = np.where(out >= response - 0.1)

    norm_ = 0.0

    for idx in range(len(out_[0])):
        i = out_[0][idx]
        j = out_[1][idx]
        k = out_[2][idx]
        res = out[i, j, k]
        p = penalty[i, j, k]
        bx += xvals[i]*res*p
        by += yvals[j]*res*p
        bt += tvals[k]*res*p
        norm_ += (res*p)

    # print(total_poses)
    bx /= norm_
    by /= norm_
    bt /= norm_

    # print(bx, by, bt)
    # print(xvals[ii], yvals[jj], tvals[kk])

    # bx = xvals[ii]
    # by = yvals[jj]
    # bt = tvals[kk]

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

    for ii_ in range(len(xvals)):
        for jj_ in range(len(yvals)):
            res_ = out[ii_, jj_, kk]
            if res_ < response - 0.2:
                continue
            x_ = xvals[ii_]
            y_ = yvals[jj_]

            norm += res_
            XX += res_*(x_ - bx)**2
            YY += res_*(y_ - by)**2
            XY += (x_ - bx) * (y_ - by)*res_

    th_norm = 0.0
    for kk_ in range(len(tvals)):
        res_ = out[ii, jj, kk_]
        if res_ < response - 0.3:
            continue
        t_ = tvals[kk_]
        th_norm += res_
        TH += res_*(t_ - bt)**2

    return [response, bx, by, bt, max(XX/norm/response, 0.1*projection_res), max(YY/norm/response, 0.1*projection_res), XY/norm/response, max(angle_res**2, TH/th_norm/response)]
