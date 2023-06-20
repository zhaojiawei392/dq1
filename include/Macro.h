#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
template<typename Scalar_, int size>
using Evec=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Evec3=Eigen::Matrix<Scalar_, 3, 1>;
using Evec3d=Eigen::Matrix<double, 3, 1>;
using Evec3f=Eigen::Matrix<float, 3, 1>;
template<typename Scalar_>
using Evec4=Eigen::Matrix<Scalar_, 4, 1>;
using Evec4d=Eigen::Matrix<double, 4, 1>;
using Evec4f=Eigen::Matrix<float, 4, 1>;


template<typename Scalar_, int rows_, int cols_>
using Emtx=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Emtx3=Eigen::Matrix<Scalar_, 3, 3>;
using Emtx3d=Eigen::Matrix<double, 3, 3>;
using Emtx3f=Eigen::Matrix<float, 3, 3>;
template<typename Scalar_>
using Emtx4=Eigen::Matrix<Scalar_, 4, 4>;
using Emtx4d=Eigen::Matrix<double, 4, 4>;
using Emtx4f=Eigen::Matrix<float, 4, 4>;



}