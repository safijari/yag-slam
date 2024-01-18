#!/bin/bash

rm /usr/bin/python
ln -s /opt/python/cp311-cp311/bin/python /usr/bin/python
/opt/python/cp311-cp311/bin/pip install -U auditwheel

cd /repo

rm -r build
rm -r dist

for folder in /opt/python/cp311*
do
    echo $folder
    rm /usr/bin/python
    ln -s $folder/bin/python /usr/bin/python
    $folder/bin/pip install pybind11 pybind11-cmake
    $folder/bin/python setup.py bdist_wheel
done

cd dist

ln -s /opt/python/cp311-cp311m/bin/python /usr/bin/python
for file in ./*
do
    /opt/python/cp311-cp311/bin/auditwheel -v repair --plat manylinux_2_28_x86_64 $file
done