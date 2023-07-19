#!/bin/bash

rm -r build
rm -r dist
rm /usr/bin/python
ln -s /opt/python/cp37-cp37m/bin/python /usr/bin/python
cd /yag-slam && /opt/python/cp37-cp37m/bin/pip install pybind11 pybind11_cmake && /opt/python/cp37-cp37m/bin/python setup.py bdist_wheel

rm -r build
rm /usr/bin/python
ln -s /opt/python/cp311-cp311/bin/python /usr/bin/python
cd /yag-slam && /opt/python/cp311-cp311/bin/pip install pybind11 pybind11_cmake && /opt/python/cp311-cp311/bin/python setup.py bdist_wheel

for FN in /yag-slam/dist/*; do
    auditwheel repair $FN
done
