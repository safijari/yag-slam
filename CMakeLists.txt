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

cmake_minimum_required(VERSION 2.8.12)
project(mp-slam)
# Note: The lines below allows a pip install pybind11 to be found by cmake.
#       Do not remove.
execute_process(COMMAND python -c
                "import pybind11_cmake; print(pybind11_cmake.__path__[0])"
                OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
# set(pybind11_DIR /usr/lib/python3.7/site-packages/pybind11_cmake)
include_directories(include)
find_package(pybind11 REQUIRED)
# find_package(Boost COMPONENTS thread REQUIRED)
pybind11_add_module(yag_slam_cpp src/PythonInterface.cpp src/Impls.cpp src/ScanMatcher.cpp)
# target_link_libraries(yag_slam_cpp PRIVATE Boost::thread)
