#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <pybind11/pybind11.h>

float my_sqrt(float value)
{
  return sqrt(value);
}


PYBIND11_MODULE(bind_example, m) {
  m.doc() = "pybind11 example";
  m.def("my_sqrt", &my_sqrt, "A function that computes the squre root.");
}
