from collections import namedtuple
import json
from mp_slam import make_config
from mp_slam_cpp import ScanMatcherConfig, Pose2, LocalizedRangeScan, LaserScanConfig, Wrapper
from mp_slam.graph import LinkLabel

SerdeConfig = namedtuple('SerdeConfig', ['cls', 'variables', 'factory'])


def _class_name(obj):
    return str(obj.__class__).split('.')[-1].split("'")[0]


def _serialize(obj):
    n = _class_name(obj)
    if n in _configs:
        d = {v: _serialize(obj.__getattribute__(v)) for v in _configs[n].variables}
        d['___name'] = n
        return d
    else:
        return obj


def _deserialize(d):
    if isinstance(d, dict) and '___name' in d:
        cfg = _configs[d['___name']]
        if cfg.factory:
            return cfg.factory(d)
        return cfg.cls(*[_deserialize(d[v]) for v in cfg.variables])
    return d


_configs = {
    'LocalizedRangeScan':
    SerdeConfig(LocalizedRangeScan, ['config', 'ranges', 'odom_pose', 'corrected_pose', 'num', 'time'], None),
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
    'LinkLabel': SerdeConfig(LinkLabel, ['mean', 'covariance'], None)
}
