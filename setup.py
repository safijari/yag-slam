from setuptools import setup, find_packages
try:
    import pybind11_cmake
except ImportError:
    print("pybind11-cmake must be installed."
          "Try \n \t pip install pybind11_cmake")
    import sys
    sys.exit()

from pybind11_cmake import CMakeExtension, CMakeBuild

setup(
    name='mp-slam',
    version='0.0.1',
    author='',
    author_email='',
    description='',
    long_description='',
    packages=find_packages(),
    setup_requires=['pybind11_cmake'],
    ext_modules=[CMakeExtension('mp_slam_cpp')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
