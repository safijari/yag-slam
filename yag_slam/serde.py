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

from collections import namedtuple
import json
from yag_slam.helpers import make_config
from yag_slam_cpp import ScanMatcherConfig, Pose2, LaserScanConfig, Wrapper
from yag_slam.models import LocalizedRangeScan
from yag_slam.graph import LinkLabel
from tiny_tf.tf import Transform
import numpy

SerdeConfig = namedtuple('SerdeConfig', ['cls', 'variables', 'factory'])
NAME = '___name'


def _class_name(obj):
    return str(obj.__class__).split('.')[-1].split("'")[0]


def _serialize(obj):
    n = _class_name(obj)
    if n in _configs:
        d = {v: _serialize(obj.__getattribute__(v)) for v in _configs[n].variables}
        d[NAME] = n
        return d
    elif isinstance(obj, numpy.ndarray):
        return obj.tolist()
    else:
        return obj


def _deserialize(d):
    if isinstance(d, dict) and NAME in d:
        cfg = _configs[d[NAME]]
        if cfg.factory:
            dd = d.copy()
            del dd[NAME]
            return cfg.factory(dd)
        return cfg.cls(*[_deserialize(d[v]) for v in cfg.variables])
    return d


_configs = {
    'LocalizedRangeScan':
    SerdeConfig(LocalizedRangeScan, ["ranges", "min_angle", "max_angle", "angle_increment", "min_range", "max_range", "range_threshold", 'odom_pose', 'corrected_pose', 'num'], LocalizedRangeScan.deserialize),
    'Pose2':
    SerdeConfig(Pose2, ['x', 'y', 'yaw'], None),
    'LaserScanConfig':
    SerdeConfig(
        LaserScanConfig,
        ['min_angle', 'max_angle', 'angular_resolution', 'min_range', 'max_range', 'range_threshold', 'sensor_name'],
        None),
    'Wrapper':
    SerdeConfig(Wrapper, ['config'], None),
    'ScanMatcherConfig':
    SerdeConfig(ScanMatcherConfig, [v for v in dir(ScanMatcherConfig()) if v[0] != '_'], make_config),
    'LinkLabel': SerdeConfig(LinkLabel, ['mean', 'covariance'], None),
    'Transform': SerdeConfig(Transform, ["x", "y", "z", "qx", "qy", "qz", "qw"], None)
}