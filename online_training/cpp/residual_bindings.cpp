// pybind11 bindings for the C++ residual-dynamics regressor.
// Exposes the same interface as the numpy reference implementation
// (residual_learnt_dynamics/error_regression.py) so verify_cpp_port.py can
// drive both through identical protocols.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "error_regression.hpp"

namespace py = pybind11;
using residual::ErrorDynamicsRegressor;

PYBIND11_MODULE(residual_core, m) {
  m.doc() = "C++ residual (error) dynamics regressor — port of "
            "error_regression.py (math from Racing-LMPC-ROS2 safe_set.cpp)";

  py::class_<ErrorDynamicsRegressor>(m, "ErrorDynamicsRegressor")
      .def(py::init<double, double, double, double, int, double, int, bool,
                    int>(),
           py::arg("l_f"), py::arg("l_r"), py::arg("Ts"),
           py::arg("dist_max") = 2.0, py::arg("k_max") = 256,
           py::arg("ridge") = 1e-3, py::arg("min_pts") = 10,
           py::arg("std_scale") = true, py::arg("max_samples") = 0)
      .def("add_samples", &ErrorDynamicsRegressor::add_samples,
           py::arg("X"), py::arg("U"), py::arg("X_next"))
      .def_property_readonly("n_samples", &ErrorDynamicsRegressor::n_samples)
      .def("predict",
           [](ErrorDynamicsRegressor& self, const Eigen::MatrixXd& X,
              const Eigen::MatrixXd& U) {
             auto res = self.predict(X, U);
             py::array_t<bool> used(res.second.size());
             auto* p = used.mutable_data();
             for (size_t i = 0; i < res.second.size(); ++i)
               p[i] = res.second[i] != 0;
             return py::make_tuple(res.first, used);
           },
           py::arg("X"), py::arg("U"),
           "returns (X_next_pred (n,3), used_regression (n,) bool)")
      .def("linearize",
           [](ErrorDynamicsRegressor& self, const Eigen::Vector3d& x,
              const Eigen::Vector2d& u) {
             Eigen::Matrix3d dA;
             Eigen::Matrix<double, 3, 2> dB;
             Eigen::Vector3d dC;
             self.linearize(x, u, dA, dB, dC);
             return py::make_tuple(dA, dB, dC);
           },
           py::arg("x"), py::arg("u"),
           "returns (dA (3,3), dB (3,2), dC (3,)) discrete corrections")
      .def("predict_nominal",
           [](ErrorDynamicsRegressor& self, const Eigen::MatrixXd& X,
              const Eigen::MatrixXd& U) {
             Eigen::MatrixXd out(X.rows(), 3);
             for (Eigen::Index i = 0; i < X.rows(); ++i)
               out.row(i) = self.nominal()
                   .predict(X.row(i).head<3>(), U.row(i).head<2>())
                   .transpose();
             return out;
           },
           py::arg("X"), py::arg("U"),
           "kinematic-bicycle nominal prediction only (n,3)");
}
