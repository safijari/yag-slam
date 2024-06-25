FROM ros:noetic-ros-base

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils python3-pip git ros-noetic-tf2-geometry-msgs libsm6
RUN apt install python3-catkin-tools ros-noetic-slam-toolbox-msgs -y

ARG USER=yag

RUN addgroup --gid 1000 ${USER} || true
RUN useradd -rm -d /home/${USER} -s /bin/bash -u 1000 -g 1000 ${USER} || true
USER ${USER}
ENV USER ${USER}

RUN mkdir -p ~/catkin_ws/src
RUN cd ~/catkin_ws/src && git clone https://github.com/safijari/sba_python.git && cd sba_python && git submodule update --init --recursive

USER root

RUN source /opt/ros/noetic/setup.bash \
    && cd /home/$USER/catkin_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=noetic -y

USER ${USER}

RUN pip install tiny-tf numba opencv-python-headless argh

RUN pip install --force-reinstall numpy scipy scikit-image tqdm

RUN pip install --force-reinstall msgpack ipython ipdb

COPY ./ /home/$USER/catkin_ws/src/yag-slam/

RUN source /opt/ros/noetic/setup.bash

RUN source /opt/ros/noetic/setup.bash \ 
    && cd /home/$USER/catkin_ws/src \
    && catkin_init_workspace \
    && cd .. \
    && catkin config --install \
    && catkin build -DCMAKE_BUILD_TYPE=Release

CMD /home/$USER/catkin_ws/src/yag-slam/run_docker.sh