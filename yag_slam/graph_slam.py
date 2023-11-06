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


from yag_slam.helpers import print_config, default_config, make_config, scans_dist_squared, default_config_loop, RadiusHashSearch
from yag_slam_cpp import Wrapper, ScanMatcherConfig, LocalizedRangeScan, Pose2, create_occupancy_grid
from yag_slam.scan_matching import Scan2DMatcherCpp
from uuid import uuid4
from tiny_tf.tf import Transform
from yag_slam.graph import Graph, Vertex, Edge, LinkLabel, do_breadth_first_traversal
from sba_cpp import SPA2d
import numpy as np
import time
from yag_slam.serde import _serialize, _deserialize
import zlib
import msgpack


# The below violates encapsulation in the worst possible way
def make_near_scan_visitor(distance):
    distsq = distance**2

    def near_scan_visitor(first_node, current_node):
        ret = scans_dist_squared(first_node.obj, current_node.obj)
        return ret < distsq

    return near_scan_visitor


class GraphSlam(object):
    def __init__(
            self,
            seq_matcher,
            loop_matcher,
            scan_buffer_len=10,
            loop_search_dist=3,
            loop_search_min_chain_size=10,
            min_response_coarse=0.35,
            min_response_fine=0.45):

        self.seq_matcher = seq_matcher
        self.loop_matcher = loop_matcher

        self.scan_buffer_len = scan_buffer_len
        self.graph = Graph()

        self.loop_search_dist = loop_search_dist
        self.loop_search_min_chain_size = loop_search_min_chain_size
        self.near_scan_visitor = make_near_scan_visitor(loop_search_dist)

        self.running_scans = []
        self.opt = SPA2d()

        self.search = RadiusHashSearch([], res=self.loop_search_dist)
        self.min_response_coarse = min_response_coarse
        self.min_response_fine = min_response_fine

    @classmethod
    def default(cls):
        return cls(default_config, default_config_loop)

    def serialize(self):
        out = {}
        out['scans'] = [_serialize(v.obj) for v in self.graph.vertices]
        out['edges'] = [[e.source.obj.num, e.target.obj.num, _serialize(e.info)] for e in self.graph.edges]
        out['running_scans'] = [s.num for s in self.running_scans]
        out['seq_matcher_config'] = _serialize(self.seq_matcher.config)
        out['loop_matcher_config'] = _serialize(self.loop_matcher.config)
        out['scan_buffer_len'] = self.scan_buffer_len
        out['loop_search_dist'] = self.loop_search_dist
        out['loop_search_min_chain_size'] = self.loop_search_min_chain_size
        out['min_response_coarse'] = self.min_response_coarse
        out['min_response_fine'] = self.min_response_fine
        return out

    def binarize(self):
        return zlib.compress(msgpack.packb(self.serialize()))

    @classmethod
    def unbinarize(cls, d):
        return cls.deserialize(msgpack.unpackb(zlib.decompress(d)))

    def to_file(self, path):
        with open(path, "wb") as ff:
            ff.write(self.binarize())

    @classmethod
    def from_file(cls, path):
        with open(path, "rb") as ff:
            return cls.unbinarize(ff.read())


    @classmethod
    def deserialize(cls, d):
        obj = cls(
                  Scan2DMatcherCpp({k: v for k, v in d['seq_matcher_config'].items() if k != '___name'}),
                  Scan2DMatcherCpp({k: v for k, v in d['loop_matcher_config'].items() if k != '___name'}),
                  d['scan_buffer_len'],
                  d['loop_search_dist'], d['loop_search_min_chain_size'],
                  d['min_response_coarse'], d['min_response_fine'])
        for s in d['scans']:
            obj.add_vertex(_deserialize(s))

        vs = obj.graph.vertices
        for from_num, to_num, info in d['edges']:
            new_edge = Edge(vs[from_num], vs[to_num], _deserialize(info))
            obj.graph.add_edge(new_edge)
            diff = new_edge.info.mean
            obj.opt.add_constraint(from_num, to_num, diff.x, diff.y, diff.euler[-1],
                                    np.linalg.inv(np.array(new_edge.info.covariance)).tolist())

        obj.running_scans = [vs[i].obj for i in d['running_scans']]

        return obj

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
        self.opt.add_node(p.x, p.y, p.euler[-1], vertex.obj.num)
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
        diff = to_scan.corrected_pose - from_scan.corrected_pose
        new_edge = Edge(from_vert, to_vert,
                        LinkLabel(diff, covariance))
        self.graph.edges.append(new_edge)

        src = from_vert.obj
        tgt = to_vert.obj
        diff = new_edge.info.mean
        self.opt.add_constraint(src.num, tgt.num, diff.x, diff.y, diff.euler[-1],
                                np.linalg.inv(np.array(new_edge.info.covariance)).tolist())

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
        closed = False

        if not self.loop_matcher:
            return closed

        chains = self.find_possible_loop_closure_chains(scan)

        if len(chains) > 0:
            print("Found {} chains for loop closure".format(len(chains)))


        for chain in chains:
            # TODO Need to pick the best chain for loop closure and quit once we have a reasonable closure
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

            tmpscan = scan.copy()
            tmpscan.corrected_pose = p

            res = self.seq_matcher.match_scan(tmpscan, chain, False, True)

            if res.response < self.min_response_fine:
                print("not good fine response {}".format(res.response))
                continue

            scan.corrected_pose = res.best_pose

            self.link_to_closest_scan_in_chain(scan,
                                               chain,
                                               res.best_pose,
                                               res.covariance,
                                               supl={
                                                   'coarse': res_coarse,
                                                   'fine': res
                                               })

            closed = True
            break

        if closed:
            print("successful loop closure")
            self.run_opt()

        return closed

    def run_opt(self):
        begin = time.time()
        self.opt.compute(100, 1.0e-4, True, 1.0e-9, 50)
        print("opt took {} seconds".format(time.time() - begin))

        for node, vtx in zip(self.opt.nodes, self.graph.vertices):
            vtx.obj.corrected_pose = Transform.from_pose2d(Pose2(node.x, node.y, node.yaw))

        self.search = RadiusHashSearch(self.graph.vertices, res=self.loop_search_dist)

    def find_possible_loop_closure_chains(self, scan):
        vert = self.graph.vertices[scan.num]
        near_linked_verts = set(do_breadth_first_traversal(vert, self.near_scan_visitor))
        # print("found {} near_linked_verts".format(len(near_linked_verts)))
        chains = []

        vertices = self.search.crude_radius_search(scan.corrected_pose, self.loop_search_dist)
        # print("found {} vertices".format(len(vertices)))
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

    def process_scan(self, scan):
        # Scan has corrected_pose, odom_pose, num, and whatever the matcher needs
        query = scan

        if len(self.running_scans) == 0:
            query.num = 0
            self.running_scans.append(query)
            self.add_vertex(query)
            return None, None

        last_scan = self.running_scans[-1]
        query.num = last_scan.num + 1
        # Initialize starting location for matching

        odom_diff = query.odom_pose - last_scan.odom_pose

        sm_correction = last_scan.corrected_pose + odom_diff

        query.corrected_pose = sm_correction

        res = self.seq_matcher.match_scan(query, self.running_scans, True, True)
        query.corrected_pose = res.best_pose

        # add to graph
        self.add_vertex(query)
        self.add_edges(query, res.covariance)

        closed = self.try_to_close_loop(query)
        # closed = False

        self.running_scans.append(query)
        self.running_scans = self.running_scans[-self.scan_buffer_len:]

        return res, closed

    def make_occupancy_grid(self, resolution=0.05, range_threshold=12):
        return create_occupancy_grid([v.obj._scan for v in slam.graph.vertices[len(scans):]], 0.05, 12)
