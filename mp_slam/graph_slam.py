from .__init__ import print_config, default_config, make_config

from mp_slam_cpp import Wrapper, Pose2, ScanMatcherConfig
from uuid import uuid4
from tiny_tf.tf import Transform
from mp_slam.graph import Graph, Vertex, Edge, LinkLabel, do_breadth_first_traversal


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
        self.seq_matcher = Wrapper(sensor_name + "seq", angular_res, angle_min, angle_max, make_config(config_dict_seq))
        # self.loop_matcher = Wrapper(sensor_name + "loop", angular_res, angle_min, angle_max, make_config(config_dict_loop))

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

    def add_edges(self, scan, covariance):
        """
        - Link previous scan (done)
        - Link to "running scans" (later)
        - Link to "near chains" (later)
        """
        last_scan = self.running_scans[-1]
        self.link_scans(last_scan, scan, scan.corrected_pose, covariance)

    def link_scans(self, from_scan, to_scan, mean, covariance):
        to_vert = self.graph.vertices[to_scan.num]
        from_vert = self.graph.vertices[from_scan.num]
        for edge in from_vert.edges:
            if edge.target == to_vert:
                print("{} to {} already exists".format(from_scan.num, to_scan.num))
                # Edge already exists, quit
                return
        new_edge = Edge(from_vert, to_vert, LinkLabel(from_scan.corrected_pose, mean, covariance))
        self.graph.edges.append(new_edge)

    def link_to_closest_scan(self, others, this):
        """
        find closest scan
        link to it
        """
        raise NotImplementedError("might be needed for a more cohesive graph")

    def link_to_near_chains(self, ):
        raise NotImplementedError("might be needed for a more cohesive graph")

    def process_scan(self, scan, x, y, yaw, flip_ranges=True):


        query = self.seq_matcher.make_scan(self._ranges_from_scan(scan, flip_ranges), x, y, yaw)

        if len(self.running_scans) == 0:
            query.num = 0
            self.running_scans.append(query)
            self.add_vertex(query)
            return

        last_scan = self.running_scans[-1]
        query.num = last_scan.num + 1
        # Initialize starting location for matching

        odom_diff = (
            Transform.from_pose2d(query.get_odometric_pose()) - Transform.from_pose2d(last_scan.get_odometric_pose()))

        sm_correction = Transform.from_pose2d(last_scan.get_corrected_pose()) + odom_diff

        query.set_corrected_pose(Pose2(sm_correction.x, sm_correction.y, sm_correction.euler[-1]))

        res = self.seq_matcher.match_scan(query, self.running_scans)
        query.set_corrected_pose(res.best_pose)

        # add to graph
        self.add_vertex(query)
        self.add_edges(query, res.covariance)

        self.running_scans.append(query)
        self.running_scans = self.running_scans[-self.scan_buffer_len:]

        return res
