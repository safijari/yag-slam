#!/bin/bash
source /catkin_ws/install/setup.bash
cd /catkin_ws/src/yag-slam/ros1/
# cd /home/jari/Work/yag_ws/src/yag-slam/ros1/
./slam_node_ros1 "$@"

    # container_jailbreak_if_needed("docker run -it --name=yag-slam --rm --net host gcr.io/jari-cloud/yag-slam /catkin_ws/src/yag-slam/run_docker.sh")
    # container_jailbreak_if_needed("docker container kill yag-slam")

    # initialPose = get_last_pose_in_map_frame()
    # xx, yy, tt = [initialPose["x"], initialPose["y"], initialPose["yaw_rad"]]
    # map_path = get_current_map_path()
    # container_jailbreak_if_needed('docker run -it -v $HOME:$HOME --name=yag-slam --rm --net host gcr.io/jari-cloud/yag-slam /catkin_ws/src/yag-slam/run_docker.sh -b "{}" -x {} -y {} -t {}'.format(map_path, xx, yy, tt))
    # container_jailbreak_if_needed("docker container kill yag-slam")

