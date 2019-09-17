# YAG SLAM (Yet Another Graph SLAM)

Quick blurb on project goals: YAG SLAM is meant to be a complete graph SLAM system for life long mapping for robots using either 2D or 3D sensors. In its current form it is basically the same as Open Karto, even keeping the scan matcher from Karto mostly as is. The graph bits (including serialization/deserialization) however are implemented in Python and SBA is being used to do the graph optimization.

Here are the rough goals of this project:

- Code should be easy to understand, maintain, and add to (hence the focus on Python as an interface). 
- Support ROS without needing ROS as I intend to use the API exposed by this codebase in a variety of situations/cloud services that are related to robotics but aren't "on a robot".
- Do map saving, loading, and modification using portable formats (currently `Graph state -> dict -> msgpack`) to allow for tool development in a variety of ways.
- Support any sensor so long as a scan matcher and a loop closure system are supplied.
