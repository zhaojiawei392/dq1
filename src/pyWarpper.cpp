/** 
 *     This file is part of dq1.
 *  
 *     dq1 is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dq1 is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dq1. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file src/pyWarpper.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include "dq1.hpp"

namespace py = pybind11;

namespace dq1{
namespace Template{


template<typename qScalar_>
void bind_quaternion(py::module& m, const std::string& class_name){
    py::class_<Quaternion<qScalar_>>(m, class_name.c_str())
        // Constructor
        .def(py::init<>()) // Default constructor
        .def(py::init<const Vector4<qScalar_>&>())
        .def(py::init<qScalar_, qScalar_, qScalar_, qScalar_>())
        .def(py::init<const Vector3<qScalar_>, qScalar_, qScalar_>())
        // Expose Members
        .def("normalize", &Quaternion<qScalar_>::normalize)
        .def("norm", &Quaternion<qScalar_>::norm)
        .def("rotation_angle", &Quaternion<qScalar_>::rotation_angle)
        .def("rotation_axis_vec3", &Quaternion<qScalar_>::rotation_axis_vec3)
        .def("conj", &Quaternion<qScalar_>::conj)
        .def("inv", &Quaternion<qScalar_>::inv)
        .def("to_string", &Quaternion<qScalar_>::to_string)
        .def("__repr__", [](const Quaternion<qScalar_>& q) {
            return "<Quaternion " + q.to_string() + ">";
        })
        // Operators
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= py::self)
        .def(py::self *= qScalar_())
        .def(py::self /= qScalar_())
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(-py::self)
        // Query 
        .def_property_readonly("w", &Quaternion<qScalar_>::w)
        .def_property_readonly("x", &Quaternion<qScalar_>::x)
        .def_property_readonly("y", &Quaternion<qScalar_>::y)
        .def_property_readonly("z", &Quaternion<qScalar_>::z);
}

}
}

PYBIND11_MODULE(dq1, m){
    dq1::Template::bind_quaternion<float>(m, "Quatf");
    dq1::Template::bind_quaternion<double>(m, "Quatd");
}
