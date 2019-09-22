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

from .__init__ import print_config, default_config

from yag_slam_cpp import Wrapper, ScanMatcherConfig, Pose2
from uuid import uuid4
from tiny_tf.tf import Transform


class ScanMatcher(object):
    def __init__(self,
                 angular_res,
                 angle_min,
                 angle_max,
                 scan_buffer_len=5,
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
        self.matcher = Wrapper(sensor_name, angular_res, angle_min, angle_max, self.config)

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
        import time
        start = time.time()
        query = self.matcher.make_scan(self._ranges_from_scan(scan, flip_ranges), x, y, yaw)
        print("made scan in {}".format(time.time()-start))

        if len(self.recent_scans) == 0:
            self.recent_scans.append(query)
            return

        last_scan = self.recent_scans[-1]

        start = time.time()
        odom_diff = (
            Transform.from_pose2d(query.odom_pose) - Transform.from_pose2d(last_scan.odom_pose))

        start = time.time()
        sm_correction = Transform.from_pose2d(last_scan.corrected_pose) + odom_diff
        print("applied diff in {}".format(time.time()-start))

        start = time.time()
        query.corrected_pose = (Pose2(sm_correction.x, sm_correction.y, sm_correction.euler[-1]))

        # res contains res.response (0 to 1, 1 being best),
        # res.covariance (3x3 matrix, 1,1 is x cov, 2,2 is ycov, 3,3 is theta cov),
        # and res.best_pose (x, y, yaw) which is pose with largest response

        start = time.time()
        res = self.matcher.match_scan(query, self.recent_scans)

        start = time.time()
        # This could maybe not be done if the response is too low
        query.corrected_pose = (res.best_pose)

        self.recent_scans.append(query)
        self.recent_scans = self.recent_scans[-self.scan_buffer_len:]

        return res
