#!/bin/bash

rm /usr/bin/python
ln -s /opt/python/cp37-cp37m/bin/python /usr/bin/python
/opt/python/cp37-cp37m/bin/pip install -U auditwheel

cd /repo

rm -r build
rm -r dist

for folder in /opt/python/*
do
    echo $folder
    rm /usr/bin/python
    ln -s $folder/bin/python /usr/bin/python
    $folder/bin/pip install pybind11 pybind11-cmake
    $folder/bin/python setup.py bdist_wheel
done

cd dist

ln -s /opt/python/cp37-cp37m/bin/python /usr/bin/python
for file in ./*
do
    /opt/python/cp37-cp37m/bin/auditwheel repair --plat manylinux2010_x86_64 $file
done
