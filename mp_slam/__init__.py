from mp_slam_cpp import Wrapper, Pose2, ScanMatcherConfig
from uuid import uuid4
from tiny_tf.tf import Transform


def print_config(config):
    for name in dir(config):
        if '_' == name[0]:
            continue
        print(f"{name}: {config.__getattribute__(name)}")


default_config = {
    "angle_variance_penalty": 1.0,
    "distance_variance_penalty": 0.5,
    "coarse_search_angle_offset": 0.5,
    "use_response_expansion": True,
    "range_threshold": 20,
    "minimum_angle_penalty": 1.5,
    "search_size": 0.5,
    "smear_deviation": 0.1,
}


class MPScanMatcher(object):
    def __init__(self,
                 angular_res,
                 angle_min,
                 angle_max,
                 scan_buffer_len=20,
                 scan_buffer_distance=None,
                 sensor_name=None,
                 config_dict=None):
        self.config = ScanMatcherConfig()
        config_params = default_config.copy()
        if config_dict:
            config_params.update(config_dict)

        for key, value in config_params.items():
            self.config.__setattr__(key, value)

        sensor_name = str(uuid4()) if not sensor_name else sensor_name
        self.matcher = Wrapper(self.sensor_name, angular_res, angle_min, angle_max, self.config)

        self.scan_buffer_len = scan_buffer_len
        self.scan_buffer_distance = scan_buffer_distance

        self.current_shift = Transform(0, 0, 0, 0, 0, 0, 1).matrix

        self.recent_scans = []

    def _print_config(self):
        print_config(self.config)

    def _ranges_from_scan(self, scan, flip_ranges):
        ranges = scan['ranges'] if isinstance(scan, dict) else scan.ranges

        if flip_ranges:
            ranges = ranges[::-1]

        return ranges

    def process_scan(self, scan, x, y, yaw, flip_ranges=True):
        query = self.matcher.make_scan(self._ranges_from_scan(scan, flip_ranges), x, y, yaw)
        if len(self.recent_scans) == 0:
            self.recent_scans.append(query)
            return

        last_scan = self.recent_scans[-1]

        odom_diff = (Transform.from_pose2d(query.get_odometric_pose()) -
                     Transform.from_pose2d(last_scan.get_odometric_pose()))

        sm_correction = Transform.from_pose2d(last_scan.get_corrected_pose()) + odom_diff

        query.set_corrected_pose(Pose2(sm_correction.x, sm_correction.y, sm_correction.euler[-1]))

        # res contains res.response (0 to 1, 1 being best),
        # res.covariance (3x3 matrix, 1,1 is x cov, 2,2 is ycov, 3,3 is theta cov),
        # and res.best_pose (x, y, yaw) which is pose with largest response
        res = self.matcher.match_scan(query, self.recent_scans)

        # This could maybe not be done if the response is too low
        query.set_corrected_pose(res.best_pose)

        self.recent_scans.append(query)
        self.recent_scans = self.recent_scans[-self.scan_buffer_len:]

        return res
