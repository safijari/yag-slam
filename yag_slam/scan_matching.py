from numba import njit, prange
from tiny_tf.tf import Transform
import numpy as np
import cv2
import time
from collections import defaultdict, namedtuple
from yag_slam_cpp import Wrapper, LaserScanConfig
from yag_slam.helpers import make_config, default_config, default_config_loop
from yag_slam.helpers import (_get_point_readings, _project_2d_scan,
                              _rotate_points, calculate_kernel, validate_points,
                              find_best_pose, add_scan_to_grid, world_to_grid, _transform_points)


ScanMatcherResult = namedtuple('ScanMatcherResult',
                               ['response', 'covariance', 'best_pose', 'meta'])

class Scan2DMatcherCpp(object):
    def __init__(self, config_dict={}, loop=False):
        cfg = default_config if not loop else default_config_loop
        cfg = cfg.copy()
        cfg.update(config_dict)
        self.config = make_config(cfg)
        self._matcher = Wrapper(self.config)

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


    def match_scan_sets(self, query_scans, base_scans, penalty=True, do_fine=True):
        search_size = self.search_size
        resolution = self.resolution
        smear_deviation = self.smear_deviation
        angle_size = self.angle_size
        angle_res = self.angle_res
        range_threshold = self.range_threshold

        grid_size = int(search_size/resolution + 1 + 2 * range_threshold/resolution)
        cgrid = np.zeros((grid_size, grid_size))
        hh, ww = cgrid.shape

        xvals, yvals = zip(*([(q.corrected_pose.x, q.corrected_pose.y) for q in query_scans]))
        ox_real = sum(xvals)/float(len(xvals))
        oy_real = sum(yvals)/float(len(yvals))
        oxy = Transform.from_position_euler(ox_real, oy_real, 0, 0, 0, 0)
        for query in query_scans:
            ox = ox_real - 0.5*(ww-1)*resolution
            oy = oy_real - 0.5*(hh-1)*resolution

        kernel = calculate_kernel(resolution, smear_deviation)
        for scan in base_scans:
            ptsx, ptsy = scan.points()

            ptsx, ptsy = validate_points(ptsx, ptsy, query.corrected_pose.x, query.corrected_pose.y)

            gx, gy = world_to_grid((np.array(ptsx), np.array(ptsy)), ox, oy, resolution)
            gx = gx.astype('int32')
            gy = gy.astype('int32')

            add_scan_to_grid(gx, gy, cgrid, kernel)

        xlocal = None
        ylocal = None
        for q in query_scans:
            _x, _y = q.points()
            if isinstance(xlocal, type(None)):
                xlocal, ylocal = _x, _y
            else:
                xlocal = np.hstack((xlocal, _x))
                ylocal = np.hstack((ylocal, _y))
        xlocal, ylocal = _transform_points(xlocal, ylocal, -ox_real, -oy_real, 0)
        pts_local = (xlocal, ylocal)

        res, x, y, t, xx, yy, xy, th = find_best_pose(
            cgrid, pts_local, ox_real, oy_real,
            0, ox, oy, search_size*0.5,
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

        oxy_corrected = Transform.from_position_euler(x, y, 0, 0, 0, t)

        diff = oxy_corrected - oxy

        return ScanMatcherResult(res, covar,
                                 [diff + q.corrected_pose for q in query_scans],
                                 meta)

    def match_scan_sets_with_map(self, cgrid, ox, oy, query_scans, penalty=True, do_fine=True):
        search_size = self.search_size
        resolution = self.resolution
        smear_deviation = self.smear_deviation
        angle_size = self.angle_size
        angle_res = self.angle_res
        range_threshold = self.range_threshold

        grid_size = int(search_size/resolution + 1 + 2 * range_threshold/resolution)
        # cgrid = np.zeros((grid_size, grid_size))
        hh, ww = cgrid.shape

        xvals, yvals = zip(*([(q.corrected_pose.x, q.corrected_pose.y) for q in query_scans]))
        ox_real = sum(xvals)/float(len(xvals))
        oy_real = sum(yvals)/float(len(yvals))
        oxy = Transform.from_position_euler(ox_real, oy_real, 0, 0, 0, 0)
        kernel = calculate_kernel(resolution, smear_deviation)
        
        xlocal = None
        ylocal = None
        for q in query_scans:
            _x, _y = q.points()
            if isinstance(xlocal, type(None)):
                xlocal, ylocal = _x, _y
            else:
                xlocal = np.hstack((xlocal, _x))
                ylocal = np.hstack((ylocal, _y))
        xlocal, ylocal = _transform_points(xlocal, ylocal, -ox_real, -oy_real, 0)
        pts_local = (xlocal, ylocal)

        res, x, y, t, xx, yy, xy, th = find_best_pose_non_symmetric(cgrid, pts_local, ox_real, 
            oy_real, 0 , ox, oy, 0.25, 0.01, 0.1, 0.01, 0.05, False )
        if do_fine:
            res, x, y, t, xx_, yy_, xy_, th = find_best_pose_non_symmetric(
                cgrid, pts_local, x, y, t, ox, oy, resolution*2, resolution,
                0.0349*0.5, 0.00349, resolution, penalty)
        else:
            th = 4 * angle_res

        meta = None
        meta = {'grid': cgrid, 'kernel': kernel}
        covar = np.array([[xx, xy, 0], [xy, yy, 0], [0, 0, th]])

        oxy_corrected = Transform.from_position_euler(x, y, 0, 0, 0, t)

        diff = oxy_corrected - oxy

        return ScanMatcherResult(res, covar,
                                 [q.corrected_pose + diff for q in query_scans],
                                 meta)

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
