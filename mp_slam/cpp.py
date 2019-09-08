from mp_slam_cpp import Pose2 as Pose2cpp
from mp_slam_cpp import LocalizedRangeScan as LocalizedRangeScancpp
from mp_slam_cpp import LaserScanConfig as LaserScanConfigcpp

class DumbSerialize(object):
    svars = None
    def to_dict(self):
        return {v: self.__getattribute__(v) for v in self.__class__.svars}
    @classmethod
    def from_dict(cls, d):
        return cls(*[d[v] for v in cls.svars])

class Pose2(Pose2cpp, DumbSerialize):
    svars = ['x', 'y', 'yaw']

    def __init__(self, x, y, yaw):
        super(Pose2, self).__init__(x, y, yaw)


# class LocalizedRangeScan(LocalizedRangeScancpp, DumbSerialize):
#     svars = ['num', 'odom_pose', 'corrected_pose', 'ranges']

class LaserScanConfig(LaserScanConfigcpp, DumbSerialize):
    def __init__(self, min_angle, max_angle, angular_resolution, min_range, max_range, range_threshold=None, sensor_name=""):
        if not range_threshold:
            range_threshold = max_range
        super(LaserScanConfig, self).__init__(min_angle, max_angle, angular_resolution, min_range, max_range, range_threshold, sensor_name)
