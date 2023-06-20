#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
template<typename Scalar_, int size>
using Evec=Eigen::Matrix<Scalar_, size, 1>;

template<typename Scalar_, int rows_, int cols_>
using Emtx=Eigen::Matrix<Scalar_, rows_, cols_>;


}