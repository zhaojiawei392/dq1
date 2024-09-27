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
 *     \file include/Modeling.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include "Modeling.hpp"
#include <stdexcept>
#include <iostream>
#include <qpOASES.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace dq1{

namespace kinematics{

const Vecx __closest_invariant_rotation_error(const Rot& r, const Rot& rd) {
    Quat er_plus = r.conj() * rd - Quat(1);
    Quat er_minus = r.conj() * rd + Quat(1);

    if (er_plus.norm() < er_minus.norm()) {
        return er_plus.vec4();
    } else {
        return er_minus.vec4();
    }
}

void __check_size_equality(const std::string& src_info, const std::string& des_info, size_t src_size, size_t des_size){
    if (src_size != des_size){
        throw std::range_error(src_info + " invalid size " + std::to_string(src_size) + ", should be equal to " + des_info + " " + std::to_string(des_size) + ".\n" );
    }
}

void __check_size_inequality(const std::string& src_info, const std::string& des_info, size_t src_size, size_t des_size){
    if (src_size == des_size){
        throw std::range_error(src_info + " invalid size " + std::to_string(src_size) + ", should NOT be equal to " + des_info + " " + std::to_string(des_size) + ".\n" );
    }
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Joint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


Joint::Joint(const Vec4 motion_limits)
    : limits_(motion_limits) {
    _check_limits();
}

void Joint::_check_limits() {
    if (limits_[0] > limits_[1] || limits_[2] > limits_[3]) {
        throw std::runtime_error("Joint complains about unreasonable joint limits, check if max position limits are all bigger than the min limits.\n");
    }
}

void Joint::_make_position_within_limits(scalar_t& position) {
    if (position < limits_[0]) 
        position = limits_[0];
    if (position > limits_[1]) 
        position = limits_[1];
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// RevoluteJoint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


RevoluteJoint::RevoluteJoint(const Vec4 DH_parameters, const Vec4 motion_limits)
    : Joint(motion_limits), DH_params_(DH_parameters) {
}

Pose RevoluteJoint::fkm(const scalar_t position) const {
    
    scalar_t theta_real = cos( 0.5 * (DH_params_[0] + position) );
    scalar_t theta_im = sin( 0.5 * (DH_params_[0] + position) );
    scalar_t alpha_real = cos( 0.5 * DH_params_[3] );
    scalar_t alpha_im = sin( 0.5 * DH_params_[3] );
    scalar_t d = DH_params_[1];
    scalar_t a = DH_params_[2];
    return Pose{
        theta_real * alpha_real,
        theta_real * alpha_im,
        theta_im * alpha_im,
        theta_im * alpha_real,
        0.5 * ( -theta_im * d * alpha_real - theta_real * a * alpha_im ),
        0.5 * ( -theta_im * d * alpha_im + theta_real * a * alpha_real ),
        0.5 * ( theta_im * a * alpha_real + theta_real * d * alpha_im ),
        0.5 * ( theta_real * d * alpha_real - theta_im * a * alpha_im )
    };
}

Vec8 RevoluteJoint::pose_jacobian(const scalar_t position) const {
    scalar_t theta_dot_real = -0.5 * sin(0.5 * (DH_params_[0] + position));
    scalar_t theta_dot_im = 0.5 * cos(0.5 * (DH_params_[0] + position));
    scalar_t alpha_real = cos(0.5 * DH_params_[3]);
    scalar_t alpha_im = sin(0.5 * DH_params_[3]);
    scalar_t d = DH_params_[1];
    scalar_t a = DH_params_[2];
    return Vec8{
        theta_dot_real * alpha_real,
        theta_dot_real * alpha_im,
        theta_dot_im * alpha_im,
        theta_dot_im * alpha_real,
        0.5 * (-theta_dot_im * d * alpha_real - theta_dot_real * a * alpha_im),
        0.5 * (-theta_dot_im * d * alpha_im + theta_dot_real * a * alpha_real),
        0.5 * (theta_dot_im * a * alpha_real + theta_dot_real * d * alpha_im),
        0.5 * (theta_dot_real * d * alpha_real - theta_dot_im * a * alpha_im)
    };
}        



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// PrismaticJoint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


PrismaticJoint::PrismaticJoint(const Vec4 DH_parameters, const Vec4 motion_limits)
    : Joint(motion_limits), DH_params_(DH_parameters) {}

Pose PrismaticJoint::fkm(const scalar_t position) const {
    scalar_t theta_real = cos( 0.5 * DH_params_[0] );
    scalar_t theta_im = sin( 0.5 * DH_params_[0] );
    scalar_t alpha_real = cos( 0.5 * DH_params_[3] );
    scalar_t alpha_im = sin( 0.5 * DH_params_[3] );
    scalar_t d = DH_params_[1] + position;
    scalar_t a = DH_params_[2];
    return Pose{
        theta_real * alpha_real,
        theta_real * alpha_im,
        theta_im * alpha_im,
        theta_im * alpha_real,
        0.5 * ( -theta_im * d * alpha_real - theta_real * a * alpha_im ),
        0.5 * ( -theta_im * d * alpha_im + theta_real * a * alpha_real ),
        0.5 * ( theta_im * a * alpha_real + theta_real * d * alpha_im ),
        0.5 * ( theta_real * d * alpha_real - theta_im * a * alpha_im )
    };
}

Vec8 PrismaticJoint::pose_jacobian(const scalar_t position) const { // Intentionally unused arg, only for satisfying virtual function signature
    
    scalar_t theta_real = sin(0.5 * DH_params_[0]);
    scalar_t theta_im = cos(0.5 * DH_params_[0]);
    scalar_t alpha_real = cos(0.5 * DH_params_[3]);
    scalar_t alpha_im = sin(0.5 * DH_params_[3]);
    // scalar_t d = 1;
    // scalar_t a = 0;
    return Vec8{
        0,
        0,
        0,
        0,
        0.5 * -theta_im * alpha_real,
        0.5 * -theta_im * alpha_im,
        0.5 * theta_real * alpha_im,
        0.5 * theta_real * alpha_real
    };
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// SerialManipulator class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


SerialManipulator::SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions){

    std::ifstream json_file(params_file_path);

    // Check if the file was opened successfully
    if (!json_file.is_open()) {
        throw std::runtime_error("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) Could not open the JSON file.");
    }

    // Parse the JSON file into a json object
    nlohmann::json data = nlohmann::json::parse(json_file);
    // Close the file
    json_file.close();

    // DH parameters
    std::vector<scalar_t> theta = data["DH_params"]["theta"];
    std::vector<scalar_t> d = data["DH_params"]["d"];
    std::vector<scalar_t> a = data["DH_params"]["a"];
    std::vector<scalar_t> alpha = data["DH_params"]["alpha"];
    std::vector<scalar_t> joint_types = data["DH_params"]["joint_types"];

    size_t DOF = theta.size(); 

    __check_size_inequality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 theta", "DoF", DOF, 0);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 d", "DoF",d.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 a", "DoF",a.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 alpha", "DoF",alpha.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 joint_types", "DoF",joint_types.size(), DOF);
    

    DH_mat DH_params;
    DH_params.resize(5, DOF);
    DH_params.row(0) = Eigen::Map<RowVecx>(theta.data(), DOF) / 180 * M_PI;
    DH_params.row(1) = Eigen::Map<RowVecx>(d.data(), DOF);
    DH_params.row(2) = Eigen::Map<RowVecx>(a.data(), DOF);
    DH_params.row(3) = Eigen::Map<RowVecx>(alpha.data(), DOF) / 180 * M_PI;
    DH_params.row(4) = Eigen::Map<RowVecx>(joint_types.data(), DOF);

    // Joint limits
    std::vector<scalar_t> min_joint_position = data["joint_limits"]["min_joint_position"];
    std::vector<scalar_t> max_joint_position = data["joint_limits"]["max_joint_position"];
    std::vector<scalar_t> min_joint_velocities = data["joint_limits"]["min_joint_velocities"];
    std::vector<scalar_t> max_joint_velocities = data["joint_limits"]["max_joint_velocities"];

    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 min_joint_position", "DoF",min_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 max_joint_position", "DoF",max_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 min_joint_velocities", "DoF",min_joint_velocities.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions) arg0 max_joint_velocities", "DoF",max_joint_velocities.size(), DOF);

    Joint_limit_mat joint_limits;
    joint_limits.resize(4, DOF);
    joint_limits.row(0) = Eigen::Map<RowVecx>(min_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(1) = Eigen::Map<RowVecx>(max_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(2) = Eigen::Map<RowVecx>(min_joint_velocities.data(), DOF) / 180 * M_PI;
    joint_limits.row(3) = Eigen::Map<RowVecx>(max_joint_velocities.data(), DOF) / 180 * M_PI;

    _construct(DH_params, joint_limits, joint_positions);
}

void SerialManipulator::set_base(const Pose& base) noexcept {
    base_ = base;
}

void SerialManipulator::set_effector(const Pose& effector) noexcept {
    effector_ = effector;
}

void SerialManipulator::update(const Pose& desired_pose) {
    USING_NAMESPACE_QPOASES;

    const Matx& r_rd_jacobian = desired_pose.rotation().haminus() * C4_ * r_jacobian_;
    const Matx& Ht = t_jacobian_.transpose() * t_jacobian_;
    const Matx& Hr = r_rd_jacobian.transpose() * r_rd_jacobian;
    const Vecx& damping_vec = Vecx::Ones(DoF()) * cfg_.joint_damping;
    const Matx& Hj = damping_vec.asDiagonal();
    const Matx& H = cfg_.translation_priority * Ht + (1-cfg_.translation_priority) * Hr + Hj;
    // the final data type must be of double, because the solver only accept double
    const Matxd Hd = H.cast<double>();

    const Vecx& vec_et = (end_pose_.translation() - desired_pose.translation()).vec4();
    const Vecx& vec_er = __closest_invariant_rotation_error(end_pose_.rotation(), desired_pose.rotation());
    const Vecx& ct = cfg_.error_gain * vec_et.transpose() * t_jacobian_;
    const Vecx& cr = cfg_.error_gain * vec_er.transpose() * r_rd_jacobian;
    const Vecx& g = cfg_.translation_priority * ct + (1-cfg_.translation_priority) * cr;

    // the final data type must be of double, because the solver only accept double
    const Vecxd& gd = g.cast<double>();

    const Matxd& constraint = Vecxd::Ones(DoF()).asDiagonal();

    static SQProblem qp(DoF(), 0);
    static Options options;
    static int_t nWSR = 500;

    options.printLevel = PL_LOW;
    qp.setOptions(options);

    const double* H_raw = Hd.data();
    const double* g_raw = gd.data();
    const double* A_raw = constraint.data();

    const Vecxd min_joint_velocirties_double = min_joint_velocities().cast<double>();
    const Vecxd max_joint_velocirties_double = max_joint_velocities().cast<double>();

    const double* lb_raw = min_joint_velocirties_double.data();
    const double* ub_raw = max_joint_velocirties_double.data();

    bool first_time(true);
    if (first_time){
        auto nWSR_in_use = nWSR;
        returnValue status = qp.init(H_raw, g_raw, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR_in_use); 
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }
        first_time = false;
    }else{
        auto nWSR_in_use = nWSR;
        returnValue status = qp.hotstart(H_raw, g_raw, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR_in_use);
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }
    }
    double xOpt[DoF()];
    qp.getPrimalSolution(xOpt);
    Eigen::Map<Vecxd> u(xOpt, DoF());

    // update joint positions
    joint_positions_ += u.cast<scalar_t>() * cfg_.sampling_time_sec;
    _update_kinematics();
}

void SerialManipulator::set_joint_positions(const Vecx& joint_positions) {
    __check_size_equality("update_joint_positions(const Vecx& joint_positions)", "DoF", joint_positions.size(), DoF());

    joint_positions_ = joint_positions;

    _update_kinematics();
}

Pose SerialManipulator::fkm(size_t index) const{
    if (index > DoF() || index < 1){
        throw std::runtime_error("SerialManipulator::fkm(size_t index) invalid arg0 " + std::to_string(index) + ", should be within [ 1, " + std::to_string(DoF()) + " ].\n" );
    }
    if (index == 1) 
        return base_;
    else
        return base_ * joint_poses_[index-2];
}


Vecx SerialManipulator::min_joint_positions() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->min_position();
    }
    return res;
}

Vecx SerialManipulator::max_joint_positions() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->max_position();
    }
    return res;
}

Vecx SerialManipulator::min_joint_velocities() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->min_velocitiy();
    }
    return res;
}

Vecx SerialManipulator::max_joint_velocities() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->max_velocitiy();
    }
    return res;
}

void SerialManipulator::_update_kinematics() {
    // Manually calculate the first joint pose and the first col of the pose jacobian
    joint_poses_[0] = joints_[0]->fkm(joint_positions_[0]);
    pose_jacobian_.col(0) = (joint_poses_[0].conj() * joint_poses_.back()).haminus() * joints_[0]->pose_jacobian(joint_positions_[0]);

    // Iterately calculate the following joint poses and the last cols of the pose jacobian
    for (size_t i=1; i<DoF(); ++i){
        joint_poses_[i] = joint_poses_[i-1] * joints_[i]->fkm(joint_positions_[i]);
        pose_jacobian_.col(i) = (joint_poses_[i].conj() * joint_poses_.back()).haminus() * joint_poses_[i-1].hamiplus() * joints_[i]->pose_jacobian(joint_positions_[i]);
    }
    // Get the final pose and jacobians
    end_pose_ = base_ * joint_poses_.back() * effector_;
    pose_jacobian_ = effector_.haminus() * base_.hamiplus() * pose_jacobian_;
    r_jacobian_ = pose_jacobian_.block(0,0,4,DoF());
    t_jacobian_ = 2 * end_pose_.rotation().conj().haminus() * pose_jacobian_.block(4,0,4,DoF()) + 2 * end_pose_.dual().hamiplus() * C4_ * r_jacobian_;
}

void SerialManipulator::_construct(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecx& joint_positions){
    
    size_t DOF = DH_params.cols();

    joint_poses_.resize(DOF);
    pose_jacobian_.resize(8, DOF);
    r_jacobian_.resize(4, DOF);
    t_jacobian_.resize(4, DOF);

    // instantiate remaining joints
    for (size_t i=0; i<DOF; ++i){
        if (DH_params(4, i) == 0){
            joints_.push_back(std::make_unique<RevoluteJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i)));
        } else if (DH_params(4, i) == 1){
            joints_.push_back(std::make_unique<PrismaticJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i)));
        } else {
            throw std::runtime_error("SerialManipulator(const Matx& DH_params, const Matx& joint_limits, const Vecx& joint_positions) arg0 5-th row DH_params[4, " + std::to_string(i) + "] = " + std::to_string(DH_params(4, i)) + ", this row should consist only of 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
        }
    }

    joint_positions_ = joint_positions;
    _update_kinematics();

    std::cout << "A Kinematics::SerialManipulator constructed!!! DoF = " + std::to_string(DOF) + ".\n" ;
}

} // namespace kinematics

} // namespace dq1

