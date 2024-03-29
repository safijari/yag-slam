#!/usr/bin/env python
import rospy
from yag_slam.graph_slam import GraphSlam, make_near_scan_visitor
from yag_slam.scan_matching import Scan2DMatcherCpp
from yag_slam.graph import do_breadth_first_traversal
from yag_slam_cpp import create_occupancy_grid, Pose2
from yag_slam.models import LocalizedRangeScan
from tiny_tf.tf import Transform
from tiny_tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMapResponse, GetMap
import tf2_ros
from collections import namedtuple
import time
import cv2
from threading import Thread, Lock
import traceback
from Queue import Queue
import json


def tftopose2(msg):
    t = msg.transform.translation
    r = msg.transform.rotation
    return Pose2(t.x, t.y, euler_from_quaternion((r.x, r.y, r.z, r.w))[-1])

def tftoTransform(msg):
    t = msg.transform.translation
    r = msg.transform.rotation
    return Transform(t.x, t.y, t.z, r.x, r.y, r.z, r.w)


def pose2toPose(pose):
    p = Pose()
    q = p.orientation
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, 0)
    p.position.x = pose.x
    p.position.y = pose.y
    return p


def TransformToTfTransform(xform, frame_id, child_frame_id):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    ts = t.transform.translation
    ts.x = xform.x; ts.y = xform.y
    tr = t.transform.rotation
    tr.x, tr.y, tr.z, tr.w = xform.quaternion
    return t


def _p(name, default):
    val = rospy.get_param(name, default)
    print("Queried param {} with default {} and got {}".format(name, default, val))
    return val


class YAGSlamRos1(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.setup = False
        self.tfb = tf2_ros.Buffer()
        self.tbr = tf2_ros.TransformBroadcaster()
        self.tfl = tf2_ros.TransformListener(self.tfb)

        self.odom_frame = _p('~odom_frame', 'odom')
        self.map_frame = _p('~map_frame', 'map')
        self.sensor_frame = _p('~sensor_frame', 'base_laser_link')
        self.min_distance = _p('~min_distance', 0.5)
        self.min_rotation = _p('~min_rotation', 0.5)
        self.loop_search_min_chain_size = _p('~loop_search_min_chain_size', 10)
        self.loop_search_distance = _p('~loop_search_distance', 6.0)
        self.min_response_coarse = _p('~min_response_coarse', 0.35)
        self.min_response_fine = _p('~min_response_fine', 0.45)
        self.range_threshold_for_map = _p('~range_threshold_for_map', 12)
        self.range_threshold = _p('~range_threshold', 30)
        self.scan_buffer_len = _p('~scan_buffer_len', 10)
        self.map_resolution = _p('~map_resolution', 0.05)

        self.scan_config = None
        self.mapper = None
        self.last_pose = None
        self.scans = []

        self.map_pub = rospy.Publisher("/map", OccupancyGrid)
        self.map_meta_pub = rospy.Publisher("/map_metadata", MapMetaData)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.map_srv = rospy.Service('dynamic_map', GetMap, self._map_callback)


        self.queue = Queue()
        self.map_queue = Queue()
        self.daemon = True
        self.map_thread = Thread(target=self._send_map)
        self.map_thread.daemon = True
        self.map_thread.start()
        self.start()

    def setup_mapper(self, msg):
        seq_scan_matcher_config = {
            "angle_variance_penalty": _p("~angle_variance_penalty", 1.0),
            "distance_variance_penalty": _p("~distance_variance_penalty", 0.5),
            "coarse_search_angle_offset": _p("~coarse_search_angle_offset", 0.349),
            "coarse_angle_resolution": _p("~coarse_angle_resolution", 0.0349),
            "fine_search_angle_resolution": _p("~fine_search_angle_resolution", 0.00349),
            "use_response_expansion": _p("~use_response_expansion", True),
            "range_threshold": _p("~range_threshold", 30),
            "minimum_angle_penalty": _p("~minimum_angle_penalty", 0.9),
            "search_size": _p("~sequential_matching_search_size", 0.5),
            "resolution": _p("~sequential_matching_resolution", 0.01),
            "smear_deviation": _p("~sequential_matching_smear_deviation", 0.1),
        }

        loop_scan_matcher_config = seq_scan_matcher_config.copy()
        loop_scan_matcher_config.update({
            "search_size": _p("~loop_matching_search_size", 8.0),
            "resolution": _p("~loop_matching_resolution", 0.05),
            "smear_deviation": _p("~loop_matching_smear_deviation", 0.03),
        })

        seq_matcher = Scan2DMatcherCpp(config_dict=seq_scan_matcher_config)
        loop_matcher = Scan2DMatcherCpp(config_dict=loop_scan_matcher_config, loop=True)

        self.mapper = GraphSlam(
            seq_matcher, loop_matcher,
            loop_search_min_chain_size=self.loop_search_min_chain_size,
            scan_buffer_len=self.scan_buffer_len,
            min_response_coarse=self.min_response_coarse,
            min_response_fine=self.min_response_fine)

    def _map_callback(self, req):
        resp = GetMapResponse()
        resp.map = self._make_map()
        return resp

    def _make_map(self, msg=True):
        grid = create_occupancy_grid([v.obj._scan for v in self.mapper.graph.vertices], self.map_resolution, self.range_threshold_for_map)
        im = grid.image.astype('int16')
        im[im == 0] = 100
        im[im == 200] = -1
        im[im == 255] = 0
        if not msg:
            return im
        map_msg = OccupancyGrid()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.height, map_msg.info.width,  = grid.height, grid.width
        map_msg.data = im.flatten().astype('int8').tolist()
        map_msg.info.origin = pose2toPose(grid.offset)
        map_msg.header.frame_id = self.map_frame
        return map_msg

    def _send_map(self):
        while True:
            self.map_queue.get()
            map_msg = self._make_map()
            self.map_pub.publish(map_msg)
            self.map_meta_pub.publish(map_msg.info)

    def run(self):
        map_counter = 0
        while True:
            msg, pose, invert_scan = self.queue.get()
            print(str(time.time()) + " got scan")

            ranges = msg.ranges[::-1] if invert_scan else msg.ranges

            # TODO Flip ranges?
            scan = LocalizedRangeScan(
                ranges, msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.range_min, msg.range_max, self.range_threshold, pose.x, pose.y, pose.yaw)
            res, closed = self.mapper.process_scan(scan)

            if len(self.mapper.running_scans) == 1:
                self.map_queue.put(True)

            map_counter += 1
            if (map_counter >= 10 or closed) and self.map_queue.qsize() == 0:
                self.map_queue.put(True)
                map_counter = 0

    def process_scan(self, msg):
        try:
            t = self.tfb.lookup_transform(
                self.odom_frame, self.sensor_frame, msg.header.stamp, rospy.Duration(0.1))
        except Exception:
            traceback.print_exc()
            print("{}: tf failed".format(time.time()))
            return

        pose = tftopose2(t)

        up_xform = tftoTransform(t) + Transform(0, 0, 100, 0, 0, 0, 1)

        # self.queue.put((msg, pose, up_xform.z < 0))
        # print(msg)
        # print(pose)
        # print(up_xform.z)

        ts = msg.header.stamp.secs + msg.header.stamp.nsecs*10.0**-9

        out_dict = {'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'timestamp': ts,
                    'seq': msg.header.seq,
                    'ranges': msg.ranges,
                    'scan_time': msg.scan_time,
                    'time_increment': msg.time_increment,
                    'x': pose.x,
                    'y': pose.y,
                    'yaw': pose.yaw,}

        with open("processed/" + str(ts) + ".json", "w") as ff:
            json.dump(out_dict, ff)


if __name__ == '__main__':
    rospy.init_node('serialize', anonymous=True)
    mapper = YAGSlamRos1()
    rospy.spin()