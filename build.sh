#!/bin/bash

#docker run --rm -v `pwd`:/repo --rm safijari/manylinux_2_28-x64 /repo/run_build.sh

cd /repo

rm -r build
rm -r dist

set -e

yum install boost-thread boost-atomic boost-chrono boost-devel -y

/opt/python/cp38-cp38/bin/pip install -U auditwheel

for folder in /opt/python/cp3*
do
    echo $folder
    $folder/bin/pip install pybind11 pybind11-cmake
    $folder/bin/python setup.py bdist_wheel
done

cd dist

for file in ./*
do
    /opt/python/cp38-cp38/bin/auditwheel -v repair --plat $1 $file
done
