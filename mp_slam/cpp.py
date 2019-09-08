from mp_slam_cpp import Pose2 as Pose2cpp
from mp_slam_cpp import LocalizedRangeScan as LocalizedRangeScancpp

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


class LocalizedRangeScan(LocalizedRangeScancpp, DumbSerialize):
    svars = ['num', 'odom_pose', 'corrected_pose', 'ranges']
