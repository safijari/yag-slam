#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;


PYBIND11_MODULE(mp_slam_cpp, m) {
    m.doc() = R"pbdoc(
        pybind11_cmake example
        -----------------------

        .. currentmodule:: mp_slam_cpp

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    // Adding a regular function through reference.
    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    // Adding a function via lambda.
    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
