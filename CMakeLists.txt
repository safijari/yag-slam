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
pybind11_add_module(mp_slam_cpp main.cpp src/Impls.cpp src/ScanMatcher.cpp src/ScanManager.cpp)
