FROM ros:noetic-ros-base

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils python3-pip git ros-noetic-tf2-geometry-msgs libsm6

# slam_toolbox
RUN mkdir -p catkin_ws/src
RUN cd catkin_ws/src && git clone https://github.com/safijari/sba_python.git && cd sba_python && git submodule update --init --recursive

RUN source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=noetic -y

RUN apt install python3-catkin-tools -y

RUN pip install tiny-tf numba opencv-python-headless argh

# RUN apt install python3-scipy -y
RUN pip install --force-reinstall numpy scipy scikit-image tqdm

RUN pip install --force-reinstall msgpack ipython ipdb

COPY ./ catkin_ws/src/yag-slam/

RUN source /opt/ros/noetic/setup.bash \ 
    && cd catkin_ws/src \
    && catkin_init_workspace \
    && cd .. \
    && catkin config --install \
    && catkin build -DCMAKE_BUILD_TYPE=Release

CMD /catkin_ws/src/yag-slam/run_docker.sh