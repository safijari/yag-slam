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

cmake_minimum_required(VERSION 3.0.0)
project(yag_slam)

find_package(catkin QUIET)

if (catkin_FOUND)
  include_directories(include)
  catkin_package(
    INCLUDE_DIRS include
    )
else()
  include_directories(include)
endif()

add_subdirectory(pybind11)

pybind11_add_module(yag_slam_cpp src/PythonInterface.cpp src/Impls.cpp src/ScanMatcher.cpp)
target_link_libraries(yag_slam_cpp)

if (catkin_FOUND)
  install(TARGETS yag_slam_cpp
    LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
    )
  install(DIRECTORY yag_slam DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
endif()