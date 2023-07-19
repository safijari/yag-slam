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

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
import sys
import setuptools

# try:
#     import pybind11_cmake
# except ImportError:
#     print("pybind11-cmake must be installed."
#           "Try \n \t pip install pybind11_cmake")
#     import sys
#     sys.exit()

# from pybind11_cmake import CMakeExtension, CMakeBuild

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path

    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked."""

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11

        pybind_include_path = pybind11.get_include(self.user)
        print("pybind11 include found at " + pybind_include_path)
        return pybind_include_path

ext_modules = [
    Extension(
        "yag_slam_cpp",
        ["src/Impls.cpp", "src/PythonInterface.cpp", "src/ScanMatcher.cpp"],
        include_dirs=[
            # Path to pybind11 headers
            str(get_pybind_include()),
            str(get_pybind_include(user=True)),
            "include"
        ],
        language="c++",
    ),
]


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile

    with tempfile.NamedTemporaryFile("w", suffix=".cpp") as f:
        f.write("int main (int argc, char **argv) { return 0; }")
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/14] compiler flag.

    The c++14 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, "-std=c++14"):
        return "-std=c++14"
    elif has_flag(compiler, "-std=c++11"):
        return "-std=c++11"
    else:
        raise RuntimeError("Unsupported compiler -- at least C++11 support " "is needed!")

class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        "msvc": ["/EHsc"],
        "unix": [],
    }

    if sys.platform == "darwin":
        c_opts["unix"] += ["-stdlib=libc++", "-mmacosx-version-min=10.7"]

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        if ct == "unix":
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append(cpp_flag(self.compiler))
            if has_flag(self.compiler, "-fvisibility=hidden"):
                opts.append("-fvisibility=hidden")
        elif ct == "msvc":
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts
        build_ext.build_extensions(self)


setup(
    name='yag_slam',
    version='0.1.1',
    author='Jariullah Safi',
    author_email='safijari@isu.edu',
    description=
    'A complete 2D and 3D graph SLAM implementation using plagiarized code from SRI\'s OpenKarto',
    long_description='''
# YAG SLAM (Yet Another Graph SLAM)

Quick blurb on project goals: YAG SLAM is meant to be a complete graph SLAM system for life long mapping for robots using either 2D or 3D sensors. In its current form it is basically the same as Open Karto, even keeping the scan matcher from Karto mostly as is. The graph bits (including serialization/deserialization) however are implemented in Python and SBA is being used to do the graph optimization.

Here are the rough goals of this project:

- Code should be easy to understand, maintain, and add to (hence the focus on Python as an interface). 
- Support ROS without needing ROS as I intend to use the API exposed by this codebase in a variety of situations/cloud services that are related to robotics but aren't "on a robot".
- Do map saving, loading, and modification using portable formats (currently `Graph state -> dict -> msgpack`) to allow for tool development in a variety of ways.
- Support any sensor so long as a scan matcher and a loop closure system are supplied.
    ''',
    packages=find_packages(),
    install_requires=[
        'sparse_bundle_adjustment', 'numpy', 'tiny_tf', 'numba', 'pybind11'
    ],
    setup_requires=['pybind11_cmake'],
    ext_modules=ext_modules,
    cmdclass={"build_ext": BuildExt},
    zip_safe=False,
)
