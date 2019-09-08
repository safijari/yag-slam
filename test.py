import numpy as np
import cv2

from mp_slam_cpp import LocalizedRangeScan, LaserScanConfig, Pose2

from mp_slam_cpp import Wrapper, ScanMatcherConfig

def main():
    config = ScanMatcherConfig()
    mw = Wrapper('laser0', np.deg2rad(0.5), -1.0, 1.0, config)

    config = LaserScanConfig(-1.0, 1.0, np.deg2rad(0.5), 0, 10, 5, "")

    base = [mw.make_scan(config, [3.0] * 230, 0, 0, 0)]
    query = mw.make_scan(config, [3.0] * 230, 1.0, 0, 1.57)

    print(query.get_corrected_pose())

    res = mw.match_scan(query, base, True, True)
    print(res.best_pose)
    query.set_corrected_pose(res.best_pose)
    print(np.array(res.covariance))

    print(query.get_corrected_pose())

if __name__ == '__main__':
    main()
    print('ran')
