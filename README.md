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

See this [colab notebook](https://colab.research.google.com/drive/1aRWEo1zC9_3JGv1zg_DOmhNwHoUCXfdh#scrollTo=75chReEdmlmN) for live examples of how to use the various aspects of this library. This is currently under construction and new examples will be added soon.

## Installation
There isn't yet a ROS package but YAG SLAM can be installed via pip (`pip install yag-slam`). For ROS Noetic you might need to install the packages using the commands below due to the pip version on Ubuntu 20.04

```
pip install https://files.pythonhosted.org/packages/4f/39/bda0165a68b59ca277625791b788510a6d93b160476fec4e2f0585f9b581/sparse_bundle_adjustment-0.9.6-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
pip install https://files.pythonhosted.org/packages/e2/dc/035e39a94f3bcfe795194e8026fc778dcd89b8394c0d424f00420b05f05a/yag_slam-0.2.5-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

Alternatively you can build the Dockerfile.

## How to use this package
For mapping in ROS 1 simply running the `slam_node_ros1` script is sufficient. Parameters can be set through ROS and are the same as the ones used in Karto and you can get more details on this from the Karto readme. Instead of describing them I'll give comments on how to tune them.

| Parameter                        | Default | Comment                                                                                                                                                                                                          |
|:---------------------------------|:--------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `angle_variance_penalty`         | 0.3     | Increase if your odom is good                                                                                                                                                                                    |
| `distance_variance_penalty`      | 0.5     | Increase if your odom is good                                                                                                                                                                                    |
| `coarse_search_angle_offset`     | 0.349   | Can decrease if odom is good, impacts compute time                                                                                                                                                               |
| `coarse_angle_resolution`        | 0.0349  | Probably don't need to change this                                                                                                                                                                               |
| `fine_search_angle_resolution`   | 0.00349 | Probably don't need to change this                                                                                                                                                                               |
| `use_response_expansion`         | True    | Can probably disable is odom is good, can increase compute time                                                                                                                                                  |
| `range_threshold`                | 30      | Set based on reliable range for your sensor                                                                                                                                                                      |
| `search_size`                    | 0.5     | dx and dy for scan matcher, can decrease if odom is good, impacts compute time                                                                                                                                   |
| `resolution`                     | 0.01    | Steps in x and y direction for scan matcher, search_size must be an integer multiple, impacts compute time                                                                                                       |
| `smear_deviation`                | 0.05    | Must be between `0.5 * resolution` and `10 * resolution`, determines size of gaussian described in the [SRI paper](https://april.eecs.umich.edu/pdfs/olson2009icra.pdf), larger values can help when odom is bad |
| `loop_matching_search_size`      | 4       | Same as search size but for finding loops, larger values would be needed if odom and sensor quality are bad but that can also lead to more incorrect loop closures, impacts compute time                         |
| `loop_matching_resolution`       | 0.05    | Same as resolution but for loop search                                                                                                                                                                           |
| `loop_meatching_smear_deviation` | 0.05    | Same ... you know                                                                                                                                                                                                |
| `odom_frame`                     | `odom`  |                                                                                                                                                                                                                  |
| `map_frame`                      | `map`   |                                                                                                                                                                                                                  |
| `min_distance`                   | 0.5     | Linear distance (m) that the robot must travel to trigger integrating a scan, smaller values could lead to error accumulation but might be needed if odom is bad                                                 |
| `min_rotation`                   | 0.5     | Rotation (rad) ...                                                                                                                                                                                               |
| `loop_search_min_chain_size`     | 10      | Number of connected together scans from a previous traversal in one area to consider for loop closure, depends a bit on `min_distance` and `min_rotation`, higher values lead to less likely loop closures       |
| `min_response_coarse`            | 0.35    | Minimum confidence of loop closure scan matcher for considering a loop closure, larger values lead to higher quality but less likely loop closures                                                               |
| `min_response_fine`              | 0.45    | Same as previous but for the second stage of accepting a loop closure candidate                                                                                                                                  |
| `range_threshold_for_map`        | 12      | Effective range of sensor for creating the map. Larger values could lead to a less clean map, smaller values require more data near all obstacles                                                                |
| `scan_buffer_len`                | 10      | How many running scans to keep to correct odom errors against, maybe don't decrease this, larger values can lead to higher compute time and possibly worse performance as well                                   |
| `map_resolution`                 | 0.05    | How many meters to a pixel in the final map                                                                                                                                                                      |
