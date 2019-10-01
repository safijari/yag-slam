from numba import njit, prange
from tiny_tf.tf import Transform
import numpy as np
import cv2
import time


class lrs:
    def __init__(self, ranges, min_angle, max_angle, incr, x, y, t):
        self.ranges = np.array(ranges)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.odom_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)
        self.corrected_pose = Transform.from_position_euler(x, y, 0, 0, 0, t)

    def points(self, odom=False):
        xx, yy = _project(self.ranges, 0, 0, self.corrected_pose.euler[-1], self.min_angle, self.max_angle)
        return (xx + self.corrected_pose.x, yy + self.corrected_pose.y)


@njit(nogil=True)
def _project(ranges_, xx, yy, rr, min_angle, max_angle):
    ranges = ranges_.copy()
    ranges[np.abs(ranges) > 12] = 0
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
def add_scan_to_grid(gx, gy, cgrid):
    h, w = cgrid.shape
    for i in range(len(gx)):
        x = gx[i]
        y = gy[i]
        if x >= 0 and x < w and y >=0 and y < h:
            cgrid[y, x] = 1.0


@njit(parallel=True, nogil=True)
def find_best_pose(cgrid, ranges, min_angle, max_angle, cx, cy, ct, ox, oy, search_size, reso, angle_size, angle_res, projection_res):
    covariance_perturbation = 2 # in either side
    cp = covariance_perturbation
    
    xvals = np.arange(-search_size/2 + cx, search_size/2 + cx, reso)
    yvals = np.arange(-search_size/2 + cy, search_size/2 + cy, reso)
    tvals = np.arange(-angle_size/2 + ct, angle_size/2 + ct, angle_res)

    h, w = cgrid.shape
    
    out = np.zeros((len(xvals), len(yvals), len(tvals)))

    for k in prange(len(tvals)):
        xx, yy = _project(ranges, 0, 0, tvals[k], min_angle, max_angle)
        for j in range(len(yvals)):
            for i in range(len(xvals)):
                x = xx.copy() + xvals[i]
                y = yy.copy() + yvals[j]
                gx, gy = world_to_grid((x, y), ox, oy, projection_res)
                res = 0.0
                for l in range(len(gx)):
                    _x = gx[l]
                    _y = gy[l]
                    if _x >= 0 and _x < w and _y >=0 and _y < h:
                        res += float(cgrid[int(_y), int(_x)])
                out[i, j, k] = res/len(gx)

    m = np.argmax(out)
    
    o, t, th = out.shape

    ii = m//(t*th)
    jj = (m%(t*th))//th
    kk = (m%(t*th))%th
    
    response = out[ii, jj, kk]
    bx = xvals[ii]
    by = yvals[jj]
    bt = tvals[kk]
    
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
    
    # how to actually do this
    iistart = max(ii - cp, 0)
    iiend = min(ii+cp, len(xvals))
    jjstart = max(jj - cp, 0)
    jjend = min(jj+cp, len(yvals))
    kkstart = max(kk - cp, 0)
    kkend = min(kk+cp, len(tvals))

    XX = 0
    YY = 0
    XY = 0
    TH = 0
    norm = 0.0

    for ii in range(len(xvals)):
        for jj in range(len(yvals)):
            for kk in range(len(tvals)):
                res_ = out[ii, jj, kk]
                if res_ < response - 1:
                    continue
                x_ = xvals[ii]
                y_ = yvals[jj]
                t_ = tvals[kk]

                norm += res_
                XX += res_*(x_ - cx)**2
                YY += res_*(y_ - cy)**2
                XY += (x_- cx)*(y_- cy)*res_
                TH += res_*(t_ - ct)**2

    return [response, bx, by, bt, XX/norm/response, YY/norm/response, XY/norm/response, TH/norm/response]

def match_scans(query, base_scans, search_size=0.5, resolution=0.01, do_fine=True):
    angle_size = 0.349
    angle_res = 0.0349
    range_threshold = 12
    grid_size = int(search_size/resolution + 1 + 2 * range_threshold/resolution)
    cgrid = np.zeros((grid_size, grid_size))
    hh, ww = cgrid.shape

    ox = query.corrected_pose.x - 0.5*(ww - 1)*resolution
    oy = query.corrected_pose.y - 0.5*(hh - 1)*resolution

    for scan in base_scans:
        gx, gy = world_to_grid(scan.points(), ox, oy, resolution)
        gx = gx.astype('int32')
        gy = gy.astype('int32')
        add_scan_to_grid(gx, gy, cgrid)

    #cgrid = cv2.dilate(cgrid, np.ones((3, 3)))
    #cgrid = cv2.GaussianBlur(cgrid, (15, 15), 15)

    res, x, y, t, xx, yy, xy, th = find_best_pose(cgrid, query.ranges, query.min_angle, query.max_angle, 
                          query.corrected_pose.x, query.corrected_pose.y, query.corrected_pose.euler[-1], 
                          ox, oy, search_size, resolution*2, angle_size, angle_res*2, resolution)

    print(res, x, y, t, xx, yy, xy, th)
    if do_fine:
        res, x, y, t, xx, yy, xy, th = find_best_pose(cgrid, query.ranges, query.min_angle, query.max_angle, 
                            x, y, t,
                            ox, oy, search_size*0.2, resolution, angle_size*0.2, angle_res, resolution)

        print(res, x, y, t, xx, yy, xy, th)

    return res, x, y, t, cgrid, xx, yy, xy
