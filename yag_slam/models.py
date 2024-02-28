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

from yag_slam_cpp import LocalizedRangeScan as LocalizedRangeScanCpp
from yag_slam_cpp import Pose2 as Pose2Cpp
from yag_slam_cpp import LaserScanConfig
from tiny_tf.tf import Transform
import numpy as np
from yag_slam.helpers import _get_point_readings


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
            LaserScanConfig(min_angle, max_angle, angle_increment, min_range, max_range, range_threshold, ""), ranges,
            Pose2Cpp(x, y, t), Pose2Cpp(x, y, t), 0, 0.0)

    @classmethod
    def deserialize(cls, args):
        return cls._deserialize(**args)

    @classmethod
    def _deserialize(cls, ranges, min_angle, max_angle, angle_increment, min_range, max_range, range_threshold, odom_pose, corrected_pose, num):
        out = cls(ranges, min_angle, max_angle, angle_increment, min_range, max_range, range_threshold, 0, 0, 0)
        del odom_pose["___name"]
        del corrected_pose["___name"]
        out.odom_pose = Transform(**odom_pose)
        out.corrected_pose = Transform(**corrected_pose)
        out.num = num
        return out

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
        return _get_point_readings(self.ranges, x, y, t, self.min_angle, 0.0, self.angle_increment,
                                   self.range_threshold)

    def copy(self):
        p = self.corrected_pose
        return LocalizedRangeScan(self.ranges.copy(), self.min_angle, self.max_angle, self.angle_increment,
                                  self.min_range, self.max_range, self.range_threshold, p.x, p.y, p.euler[-1])


    @classmethod
    def from_json(cls, d, x, y, t, invert=True):
        ranges = d['ranges']
        if invert:
            ranges = ranges[::-1]
        return cls(ranges, d['angle_min'], d['angle_max'], d['angle_increment'], d['range_min'], d['range_max'],
                   d['range_max']*0.9, x, y, t)
