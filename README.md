# YAG SLAM (Yet Another Graph SLAM)

<img src="https://github.com/safijari/yag-slam/assets/5191844/bf384bba-073f-450d-9490-63a302fa11e9" width="250" height="250"/>

Quick blurb on project goals: YAG SLAM is meant to be a complete graph SLAM system for life long mapping for robots using either 2D or 3D sensors. In its current form it is basically the same as Open Karto, even keeping the scan matcher from Karto mostly as is. The graph bits (including serialization/deserialization) however are implemented in Python and SBA is being used to do the graph optimization.

Here are the rough goals of this project:

- Code should be easy to understand, maintain, and add to (hence the focus on Python as an interface). 
- Support ROS without needing ROS as I intend to use the API exposed by this codebase in a variety of situations/cloud services that are related to robotics but aren't "on a robot".
- Do map saving, loading, and modification using portable formats (currently `Graph state -> dict -> msgpack`) to allow for tool development in a variety of ways.
- Support any sensor so long as a scan matcher and a loop closure system are supplied.

Underlying Graph
![graph slam](https://user-images.githubusercontent.com/5191844/64484217-e443fe80-d1c3-11e9-8f27-9fa95e7b845b.png)

Occupancy Grid Map
![occupancy grid map](https://user-images.githubusercontent.com/5191844/64672265-cffc3d80-d41f-11e9-915b-f984df9bb1d9.png)
