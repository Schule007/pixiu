#include <pybind11/pybind11.h>
#include "exponential_swept_sine.h"

namespace py = pybind11;

PYBIND11_MODULE(sysid_tools, m) {
    m.doc() = "System identification tools."; // optional module docstring
    py::class_<sysid_tools::Add>(m, "Add")
      .def(py::init<>())
      .def("call", &sysid_tools::Add::call);
    py::class_<sysid_tools::ExponentialSweptSine>(m, "ExponentialSweptSine")
      .def(py::init<double, double, double, double>())
      .def("step", &sysid_tools::ExponentialSweptSine::step)
      .def("reset", &sysid_tools::ExponentialSweptSine::reset);
}
