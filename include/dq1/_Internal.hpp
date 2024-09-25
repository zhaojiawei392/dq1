#pragma once
#include "Pose.hpp"

using scalar_t = double;
using size_t = int;

template<size_t size>
using Vec=Eigen::Matrix<scalar_t, size, 1>;
using Vec3=Eigen::Matrix<scalar_t, 3, 1>;
using Vec4=Eigen::Matrix<scalar_t, 4, 1>;
using Vec6=Eigen::Matrix<scalar_t, 6, 1>;
using Vec8=Eigen::Matrix<scalar_t, 8, 1>;
using Vecx=Eigen::Matrix<scalar_t, -1, 1>;

template<size_t size>
using RowVec=Eigen::Matrix<scalar_t, 1, size>;
using RowVec3=Eigen::Matrix<scalar_t, 1, 3>;
using RowVec4=Eigen::Matrix<scalar_t, 1, 4>;
using RowVec6=Eigen::Matrix<scalar_t, 1, 6>;
using RowVec8=Eigen::Matrix<scalar_t, 1, 8>;
using RowVecx=Eigen::Matrix<scalar_t, 1, -1>;

template<size_t rows_, size_t cols_>
using Mat=Eigen::Matrix<scalar_t, rows_, cols_>;
using Mat3=Eigen::Matrix<scalar_t, 3, 3>;
using Mat4=Eigen::Matrix<scalar_t, 4, 4>;
using Mat6=Eigen::Matrix<scalar_t, 6, 6>;
using Mat8=Eigen::Matrix<scalar_t, 8, 8>;
using Matx=Eigen::Matrix<scalar_t, -1, -1>;

using Quat = dq1::Quaternion<scalar_t>;
using Rot = dq1::Rotation<scalar_t>;
using Tran = dq1::Translation<scalar_t>;
using BaseAxis = dq1::BaseAxis<scalar_t>;
using DQ = dq1::DualQuaternion<scalar_t>;
using Pose = dq1::Pose<scalar_t>;

using Pose_jcb_t = Mat<8, -1>;
using Rot_jcb_t = Mat<4, -1>;
using Tran_jcb_t = Mat<4, -1>;

const BaseAxis i_(1,0,0);
const BaseAxis j_(0,1,0);
const BaseAxis k_(0,0,1);
const Mat4 C4_ = (Mat4() << 1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,-1).finished();
const Mat8 C8_ = (Mat8() << C4_, Mat4::Zero(), C4_, Mat4::Zero()).finished();
