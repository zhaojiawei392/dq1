#pragma once
#include <eigen3/Eigen/Dense>

namespace dq1
{
#define PRINT_PRECISION 18
#define OMIT_THRESHOLD 0.00000000001

template<typename Scalar_, int size>
using Vec=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Vec3=Eigen::Matrix<Scalar_, 3, 1>;
using Vec3f=Eigen::Matrix<float, 3, 1>;
using Vec3d=Eigen::Matrix<double, 3, 1>;
template<typename Scalar_>
using Vec4=Eigen::Matrix<Scalar_, 4, 1>;
using Vec4f=Eigen::Matrix<float, 4, 1>;
using Vec4d=Eigen::Matrix<double, 4, 1>;
template<typename Scalar_>
using Vec6=Eigen::Matrix<Scalar_, 6, 1>;
using Vec6f=Eigen::Matrix<float, 6, 1>;
using Vec6d=Eigen::Matrix<double, 6, 1>;
template<typename Scalar_>
using Vec8=Eigen::Matrix<Scalar_, 8, 1>;
using Vec8f=Eigen::Matrix<float, 8, 1>;
using Vec8d=Eigen::Matrix<double, 8, 1>;
template<typename Scalar_>
using Vecx=Eigen::Matrix<Scalar_, -1, 1>;
using Vecxf=Eigen::Matrix<float, -1, 1>;
using Vecxd=Eigen::Matrix<double, -1, 1>;

template<typename Scalar_, int rows_, int cols_>
using Mat=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Mat3=Eigen::Matrix<Scalar_, 3, 3>;
using Mat3f=Eigen::Matrix<float, 3, 3>;
using Mat3d=Eigen::Matrix<double, 3, 3>;
template<typename Scalar_>
using Mat4=Eigen::Matrix<Scalar_, 4, 4>;
using Mat4f=Eigen::Matrix<float, 4, 4>;
using Mat4d=Eigen::Matrix<double, 4, 4>;
template<typename Scalar_>
using Mat6=Eigen::Matrix<Scalar_, 6, 6>;
using Mat6f=Eigen::Matrix<float, 6, 6>;
using Mat6d=Eigen::Matrix<double, 6, 6>;
template<typename Scalar_>
using Mat8=Eigen::Matrix<Scalar_, 8, 8>;
using Mat8f=Eigen::Matrix<float, 8, 8>;
using Mat8d=Eigen::Matrix<double, 8, 8>;
template<typename Scalar_>
using Matx=Eigen::Matrix<Scalar_, -1, -1>;
using Matxf=Eigen::Matrix<float, -1, -1>;
using Matxd=Eigen::Matrix<double, -1, -1>;

}