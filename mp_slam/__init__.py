from mp_slam_cpp import ScanMatcherConfig
import math
from collections import namedtuple


def print_config(config):
    for name in dir(config):
        if '_' == name[0]:
            continue
        print(f"{name}: {config.__getattribute__(name)}")


default_config = {
    "angle_variance_penalty": 1.0,
    "distance_variance_penalty": 0.5,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "fine_search_angle_offset": 0.00349,
    "use_response_expansion": True,
    "range_threshold": 55,
    "minimum_angle_penalty": 0.9,
    "search_size": 0.5,
    "resolution": 0.01,
    "smear_deviation": 0.1,
}

default_config_loop = {
    "angle_variance_penalty": 1.0,
    "distance_variance_penalty": 0.5,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "fine_search_angle_offset": 0.00349,
    "use_response_expansion": True,
    "range_threshold": 55,
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
        return (int(p.x/self.res), int(p.y/self.res))

    def key_to_pose(self, key):
        # x, y = key.split('_')
        x, y = key
        return Pose2(float(x)*self.res, float(y)*self.res)

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
