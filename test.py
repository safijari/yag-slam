import numpy as np
import cv2

from mp_slam_cpp import Wrapper, Pose2

def main():
    mw = Wrapper('laser0', np.deg2rad(0.5), -1.0, 1.0)

    mw.range_finder.set_offset_pose(Pose2(1.0, 0.0, 0.0))

    base = [mw.make_scan([3.0] * 230, 0, 0, 0)]
    query = mw.make_scan([3.0] * 230, 1.0, 0, 1.57)

    print(query.get_corrected_pose())

    res = mw.match_scan(query, base)
    print(res.best_pose)
    query.set_corrected_pose(res.best_pose)
    print(np.array(res.covariance))

    print(query.get_corrected_pose())

if __name__ == '__main__':
    main()
    print('ran')
