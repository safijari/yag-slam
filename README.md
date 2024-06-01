# YAG SLAM (Yet Another Graph SLAM)

<table>
<tr>
<td><img src="https://github.com/safijari/yag-slam/assets/5191844/bf384bba-073f-450d-9490-63a302fa11e9" width="300" height="300"/> </td>
<td><img src="https://user-images.githubusercontent.com/5191844/64484217-e443fe80-d1c3-11e9-8f27-9fa95e7b845b.png" width="300" height="300"/></td>
</tr>
</table>

This project is meant to be both an occupancy grid mapping package to be used in ROS as well as a library which can be used to build mapping related tools and algorithms. The core of the library is the scan matcher lifted straight from Karto and exposed to Python through the use of Pybind11 (see `src/PythonInterface.cpp`)

The goals of the project are:
- Code should be easy to understand, maintain, and add to. The main interface for this library is Python with the computationally intensive parts relegated to either jit compiled Python (using numba) or C++ (scan matching, sparse bundle adjustment) 
- Be usable in ROS but don't require it. This package is released on PyPI and should work in any modern python version. I use the API exposed by this codebase in a variety of situations/cloud services that are related to robotics but aren't "on a robot". A recent example of this is a lifelong mapping system.
- Do map saving, loading, and modification using portable formats (currently `Graph state -> dict -> compressed msgpack`) to allow for tool development outside of ROS. I shouldn't need to provide specialized code to my web developer colleagues for them to just render the state of the underlying graph (see above for an example).
- Support any sensor type so long as a scan matcher and a loop closure checker are supplied.

## How to use this package
For mapping in ROS 1 simply running the `slam_node_ros1` script is sufficient. Parameters can be set through ROS and include:

| Parameter | Description | Default Value |
|:----------|:------------|:--------------|
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
|           |             |               |
