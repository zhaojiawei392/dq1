#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "dq1.hpp"

PYBIND11_MODULE(dq1, m) {
    pybind11::class_<MyClass>(m, "MyClass")
        .def(pybind11::init<int>())
        .def("set_value", &MyClass::set_value)
        .def("get_value", &MyClass::get_value)
        .def("print", &MyClass::print);
}