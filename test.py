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

import numpy as np

from karto_scanmatcher import LocalizedRangeScan, LaserScanConfig, Pose2

from karto_scanmatcher import Wrapper, ScanMatcherConfig


def main():
    config = ScanMatcherConfig()
    mw = Wrapper(config)

    config = LaserScanConfig(-1.0, 1.0, np.deg2rad(0.5), 0, 10, 5, "")

    base = [
        LocalizedRangeScan(config, [3.0] * 230, Pose2(0, 0, 0), Pose2(0, 0, 0),
                           0, 0.0)
    ]
    query = LocalizedRangeScan(config, [3.0] * 230, Pose2(1.0, 0, 1.57),
                               Pose2(1.0, 0, 1.57), 1, 0.0)

    print(query.corrected_pose)

    res = mw.match_scan(query, base, True, True)
    print(res.best_pose)
    query.corrected_pose = res.best_pose
    print(np.array(res.covariance))

    print(query.corrected_pose)


if __name__ == '__main__':
    main()
    print('ran')
