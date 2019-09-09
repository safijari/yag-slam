from .__init__ import print_config, default_config, make_config, scans_dist_squared, default_config_loop, RadiusHashSearch

from mp_slam_cpp import Wrapper, ScanMatcherConfig, LocalizedRangeScan, Pose2
from uuid import uuid4
from tiny_tf.tf import Transform
from mp_slam.graph import Graph, Vertex, Edge, LinkLabel, do_breadth_first_traversal
from sba_cpp import SPA2d
import numpy as np
import time

# The below violates encapsulation in the worst possible way
def make_near_scan_visitor(distance):
    distsq = distance**2
    def near_scan_visitor(first_node, current_node):
        ret = scans_dist_squared(first_node.obj, current_node.obj)
        return ret < distsq
    return near_scan_visitor


class MPGraphSlam(object):
    def __init__(self,
                 config,
                 scan_buffer_len=10,
                 scan_buffer_distance=None,  # TODO
                 sensor_name=None,
                 config_dict_seq=default_config,
                 config_dict_loop=default_config_loop,
                 loop_search_dist=3,
                 loop_search_min_chain_size=10,
                 min_response_coarse=0.35,
                 min_response_fine=0.45):

        sensor_name = str(uuid4()) if not sensor_name else sensor_name
        self.scan_config = config
        self.seq_matcher = Wrapper(make_config(config_dict_seq))
        self.loop_matcher = Wrapper(make_config(config_dict_loop))

        self.scan_buffer_len = scan_buffer_len
        self.scan_buffer_distance = scan_buffer_distance
        self.graph = Graph()

        self.loop_search_dist = loop_search_dist
        self.loop_search_min_chain_size = loop_search_min_chain_size
        self.near_scan_visitor = make_near_scan_visitor(loop_search_dist)

        self.running_scans = []
        self.opt = SPA2d()

        self.search = RadiusHashSearch([], res=self.loop_search_dist)
        self.min_response_coarse = min_response_coarse
        self.min_response_fine = min_response_fine

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
        p = vertex.obj.corrected_pose
        self.opt.add_node(p.x, p.y, p.yaw, vertex.obj.num)
        self.search.add_new_element(vertex)

    def add_edges(self, scan, covariance):
        """
        - Link previous scan (done)
        - Link to "running scans" (later)
        - Link to "near chains" (later)
        """
        last_scan = self.running_scans[-1]
        self.link_scans(last_scan, scan, scan.corrected_pose, covariance)
        self.link_to_closest_scan_in_chain(scan, self.running_scans, scan.corrected_pose, covariance)

    def link_scans(self, from_scan, to_scan, mean, covariance, supl=None):
        to_vert = self.graph.vertices[to_scan.num]
        from_vert = self.graph.vertices[from_scan.num]
        for edge in from_vert.edges:
            if edge.target == to_vert:
                # print("{} to {} already exists".format(from_scan.num, to_scan.num))
                # Edge already exists, quit
                return
        diff = Transform.from_pose2d(to_scan.corrected_pose) - Transform.from_pose2d(from_scan.corrected_pose)
        new_edge = Edge(from_vert, to_vert, LinkLabel(from_scan.corrected_pose, Pose2(diff.x, diff.y, diff.euler[-1]), covariance, supl))
        self.graph.edges.append(new_edge)

        src = from_vert.obj
        tgt = to_vert.obj
        diff = new_edge.info.mean
        self.opt.add_constraint(src.num, tgt.num, diff.x, diff.y, diff.yaw, np.linalg.inv(np.array(new_edge.info.covariance)).tolist())

    def link_to_closest_scan_in_chain(self, scan, chain, mean, covariance, supl=None):
        """
        find closest scan
        link to it
        """
        tmp_chain = [c for c in chain]
        tmp_chain.sort(key=lambda x: scans_dist_squared(x, scan))
        closest_scan = tmp_chain[0]
        # TODO check distance first?
        self.link_scans(closest_scan, scan, mean, covariance, supl)

    def link_to_near_chains(self, ):
        raise NotImplementedError("might be needed for a more cohesive graph")

    def try_to_close_loop(self, scan):
        """
        chains = FindPossibleLoopClosureChains
        for chain in chains:
          Do loop scan match and quit if coarse response too low or covar too high

          make temp scan with corrected pose from loop scan match

          Do seq match of temp scan against chain, quit if res....

          set query scan's corrected pose, link to chain
        """
        chains = self.find_possible_loop_closure_chains(scan)

        if len(chains) > 0:
            print("ooh chains")

        closed = False

        for chain in chains:
            # coarse
            res_coarse = self.loop_matcher.match_scan(scan, chain, False, False)
            # resp 0.35 for coarse, 0.4 for fine?
            # covar 3.0 for coarse?????
            if res_coarse.response < self.min_response_coarse:
                print("not good coarse response {}".format(res_coarse.response))
                continue

            if res_coarse.covariance[0][0] > 3.0 or res_coarse.covariance[1][1] > 3.0:
                print("WARN: covariance too high for coarse")

            p = res_coarse.best_pose
            tmpscan = self.seq_matcher.make_scan(self.scan_config, scan.ranges, p.x, p.y, p.yaw)

            res = self.seq_matcher.match_scan(tmpscan, chain, False, True)

            if res.response < self.min_response_fine:
                print("not good fine response {}".format(res.response))
                continue

            scan.corrected_pose = res.best_pose

            self.link_to_closest_scan_in_chain(scan, chain, res.best_pose, res.covariance, supl={'coarse': res_coarse, 'fine': res})

            closed = True

        if closed:
            print("successful loop closure")
            begin = time.time()
            self.opt.compute(100, 1.0e-4, True, 1.0e-9, 50)
            print("opt took {} seconds".format(time.time()-begin))

            for node, vtx in zip(self.opt.nodes, self.graph.vertices):
                vtx.obj.corrected_pose = Pose2(node.x, node.y, node.yaw)

            self.search = RadiusHashSearch(self.graph.vertices, res=self.loop_search_dist)

        return closed

    def find_possible_loop_closure_chains(self, scan):
        vert = self.graph.vertices[scan.num]
        near_linked_verts = set(do_breadth_first_traversal(vert, self.near_scan_visitor))
        # The below is a reimplementation of the dumb Karto method for finding chains, needs to be rewritten
        chains = []

        vertices = self.search.crude_radius_search(scan.corrected_pose, self.loop_search_dist)
        vertices.sort(key=lambda v: v.obj.num)

        current_chain = []
        for v1, v2 in zip(vertices, vertices[1:]):
            other_scan = v1.obj
            if other_scan == scan or other_scan in near_linked_verts:
                current_chain = []
                continue

            if scans_dist_squared(scan, other_scan) <= self.loop_search_dist:
                current_chain.append(other_scan)

            if len(current_chain) >= self.loop_search_min_chain_size:
                chains.append(current_chain)
                current_chain = []

            if (v2.obj.num - v1.obj.num) > 1:
                current_chain = []

        if current_chain:
            chains.append(current_chain)

        return chains

    def process_scan(self, scan, x, y, yaw, flip_ranges=True):
        query = self.seq_matcher.make_scan(self.scan_config, self._ranges_from_scan(scan, flip_ranges), x, y, yaw)

        if len(self.running_scans) == 0:
            query.num = 0
            self.running_scans.append(query)
            self.add_vertex(query)
            return None, None

        last_scan = self.running_scans[-1]
        query.num = last_scan.num + 1
        # Initialize starting location for matching

        odom_diff = (
            Transform.from_pose2d(query.get_odometric_pose()) - Transform.from_pose2d(last_scan.get_odometric_pose()))

        sm_correction = Transform.from_pose2d(last_scan.get_corrected_pose()) + odom_diff

        query.set_corrected_pose(Pose2(sm_correction.x, sm_correction.y, sm_correction.euler[-1]))

        res = self.seq_matcher.match_scan(query, self.running_scans, True, True)
        query.set_corrected_pose(res.best_pose)

        # add to graph
        self.add_vertex(query)
        self.add_edges(query, res.covariance)

        closed = self.try_to_close_loop(query)
        # closed = False

        self.running_scans.append(query)
        self.running_scans = self.running_scans[-self.scan_buffer_len:]

        return res, closed
