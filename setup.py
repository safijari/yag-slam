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

from setuptools import setup, find_packages

setup(
    name="yag_slam",
    version="0.3.0",
    author="Jariullah Safi",
    author_email="safijari@isu.edu",
    description="A complete 2D and 3D graph SLAM implementation using plagiarized code from SRI's OpenKarto",
    long_description="""
# YAG SLAM (Yet Another Graph SLAM)

Quick blurb on project goals: YAG SLAM is meant to be a complete graph SLAM system for life long mapping for robots using either 2D or 3D sensors. In its current form it is basically the same as Open Karto, even keeping the scan matcher from Karto mostly as is. The graph bits (including serialization/deserialization) however are implemented in Python and SBA is being used to do the graph optimization.

Here are the rough goals of this project:

- Code should be easy to understand, maintain, and add to (hence the focus on Python as an interface). 
- Support ROS without needing ROS as I intend to use the API exposed by this codebase in a variety of situations/cloud services that are related to robotics but aren't "on a robot".
- Do map saving, loading, and modification using portable formats (currently `Graph state -> dict -> msgpack`) to allow for tool development in a variety of ways.
- Support any sensor so long as a scan matcher and a loop closure system are supplied.
    """,
    packages=find_packages(),
    install_requires=[
        "sparse_bundle_adjustment",
        "numpy",
        "tiny_tf",
        "numba",
        "scipy",
        "scikit-image",
        "tqdm",
        "threeviz",
        "karto_scanmatcher==1.0.0",
    ],
    scripts=["ros1/slam_node_ros1"],
    zip_safe=False,
)
