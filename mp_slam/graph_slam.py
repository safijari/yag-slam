from .__init__ import print_config, default_config, make_config

from mp_slam_cpp import Wrapper, Pose2, ScanMatcherConfig
from uuid import uuid4
from tiny_tf.tf import Transform
from graph import Graph, Vertex, Edge, LinkLabel, do_breadth_first_traversal


class MPGraphSlam(object):
    def __init__(self,
                 angular_res,
                 angle_min,
                 angle_max,
                 scan_buffer_len=5,
                 scan_buffer_distance=5,
                 sensor_name=None,
                 config_dict_seq=None,
                 config_dict_loop=None):

        sensor_name = str(uuid4()) if not sensor_name else sensor_name
        self.seq_matcher = Wrapper(sensor_name, angular_res, angle_min, angle_max, make_config(config_dict_seq))
        self.loop_matcher = Wrapper(sensor_name, angular_res, angle_min, angle_max, make_config(config_dict_loop))

        self.scan_buffer_len = scan_buffer_len
        self.scan_buffer_distance = scan_buffer_distance
        self.graph = Graph()

        self.running_scans = []

    def _print_config(self):
        print_config(self.config)

    def _ranges_from_scan(self, scan, flip_ranges):
        ranges = scan['ranges'] if isinstance(scan, dict) else scan.ranges

        if flip_ranges:
            ranges = ranges[::-1]

        return ranges

    def add_vertex(self, scan):
        vertex = Vertex(scan)
        self.graph.add_vertex(vertex)
        # add to optimizer?

    def add_edges(self, scan):
        pass

    def process_scan(self, scan, x, y, yaw, flip_ranges=True):
        query = self.matcher.make_scan(self._ranges_from_scan(scan, flip_ranges), x, y, yaw)
        query.num = len(self.running_scans)

        if len(self.running_scans) == 0:
            self.running_scans.append(query)
            self.add_vertex(query)
            return

        last_scan = self.running_scans[-1]

        # Initialize starting location for matching

        odom_diff = (
            Transform.from_pose2d(query.get_odometric_pose()) - Transform.from_pose2d(last_scan.get_odometric_pose()))

        sm_correction = Transform.from_pose2d(last_scan.get_corrected_pose()) + odom_diff

        query.set_corrected_pose(Pose2(sm_correction.x, sm_correction.y, sm_correction.euler[-1]))

        res = self.matcher.match_scan(query, self.running_scans)
        query.set_corrected_pose(res.best_pose)

        # add to graph
        self.add_vertex(query)
        self.add_edges(query)

        self.running_scans.append(query)
        self.running_scans = self.running_scans[-self.scan_buffer_len:]

        return res
