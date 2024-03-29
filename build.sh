#!/bin/bash

#docker run --rm -v `pwd`:/repo --rm safijari/manylinux_2_28-x64 /repo/run_build.sh

cd /repo

rm -r build
rm -r dist

set -e

yum install boost-thread boost-chrono boost-devel -y

/opt/python/cp311-cp311/bin/pip install -U auditwheel

for folder in /opt/python/cp311*
do
    echo $folder
    $folder/bin/pip install pybind11 pybind11-cmake
    $folder/bin/python setup.py bdist_wheel
done

for folder in /opt/python/cp37*
do
    echo $folder
    $folder/bin/pip install pybind11 pybind11-cmake
    $folder/bin/python setup.py bdist_wheel
done

cd dist

for file in ./*
do
    /opt/python/cp311-cp311/bin/auditwheel -v repair --plat manylinux_2_28_x86_64 $file
done
