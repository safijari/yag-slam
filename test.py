import numpy as np
import cv2

from mp_slam_cpp import LocalizedRangeScan, LaserScanConfig, Pose2

from mp_slam_cpp import Wrapper, ScanMatcherConfig

def main():
    config = ScanMatcherConfig()
    mw = Wrapper(config)

    config = LaserScanConfig(-1.0, 1.0, np.deg2rad(0.5), 0, 10, 5, "")

    base = [LocalizedRangeScan(config, [3.0] * 230, Pose2(0, 0, 0), Pose2(0, 0, 0), 0, 0.0)]
    query = LocalizedRangeScan(config, [3.0] * 230, Pose2(1.0, 0, 1.57), Pose2(1.0, 0, 1.57), 1, 0.0)

    print(query.corrected_pose)

    res = mw.match_scan(query, base, True, True)
    print(res.best_pose)
    query.corrected_pose = res.best_pose
    print(np.array(res.covariance))

    print(query.corrected_pose)

if __name__ == '__main__':
    main()
    print('ran')
