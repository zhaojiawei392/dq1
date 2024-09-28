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
 *     \file include/Macro.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include <eigen3/Eigen/Dense>
#include "Pose.hpp"

namespace dq1
{


namespace Macro
{
using scalar_t = float;

using Vec3=Eigen::Vector<scalar_t, 3>;
using Vec4=Eigen::Vector<scalar_t, 4>;
using Vec6=Eigen::Vector<scalar_t, 6>;
using Vec8=Eigen::Vector<scalar_t, 8>;
using Vecx=Eigen::Vector<scalar_t, -1>;
using RowVec3=Eigen::RowVector<scalar_t, 3>;
using RowVec4=Eigen::RowVector<scalar_t, 4>;
using RowVec6=Eigen::RowVector<scalar_t, 6>;
using RowVec8=Eigen::RowVector<scalar_t, 8>;
using RowVecx=Eigen::RowVector<scalar_t, -1>;
using Mat3=Eigen::Matrix<scalar_t, 3, 3>;
using Mat4=Eigen::Matrix<scalar_t, 4, 4>;
using Mat6=Eigen::Matrix<scalar_t, 6, 6>;
using Mat8=Eigen::Matrix<scalar_t, 8, 8>;
using Matx=Eigen::Matrix<scalar_t, -1, -1>;

using Vecxd=Eigen::VectorXd;
using RowVecxd=Eigen::RowVectorXd;
using Matxd=Eigen::MatrixXd;

using Quat = dq1::Template::Quaternion<scalar_t>;
using Rot = dq1::Template::Rotation<scalar_t>;
using Tran = dq1::Template::Translation<scalar_t>;
using UAxis = dq1::Template::UnitAxis<scalar_t>;
using DQ = dq1::Template::DualQuaternion<scalar_t>;
using Pose = dq1::Template::Pose<scalar_t>;

using PoseJacobian = Eigen::Matrix<scalar_t, 8, -1>;
using RotationJacobian = Eigen::Matrix<scalar_t, 4, -1>;
using TranslationJacobian = Eigen::Matrix<scalar_t, 4, -1>;
using DHParam = Eigen::Matrix<scalar_t, 5, -1>;
using JointLimits = Eigen::Matrix<scalar_t, 4, -1>;

const UAxis i_(1,0,0);
const UAxis j_(0,1,0);
const UAxis k_(0,0,1);
const Mat4 C4_ = (Mat4() << 1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,-1).finished();
const Mat8 C8_ = (Mat8() << C4_, Mat4::Zero(), C4_, Mat4::Zero()).finished(); 


}

}