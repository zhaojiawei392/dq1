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
 *     \file src/Kinematics.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include "Kinematics.hpp"
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
    : _limits(motion_limits) {
    _check_limits();
}

void Joint::_check_limits() {
    if (_limits[0] > _limits[1] || _limits[2] > _limits[3]) {
        throw std::runtime_error("Joint complains about unreasonable joint limits, check if max position limits are all bigger than the min limits.\n");
    }
}

void Joint::_make_position_within_limits(scalar_t& position) {
    if (position < _limits[0]) 
        position = _limits[0];
    if (position > _limits[1]) 
        position = _limits[1];
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// RevoluteJoint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


RevoluteJoint::RevoluteJoint(const Vec4 DH_parameters, const Vec4 motion_limits)
    : Joint(motion_limits), _DH_params(DH_parameters) {
}

Pose RevoluteJoint::fkm(const scalar_t position) const {
    
    scalar_t theta_real = cos( 0.5 * (_DH_params[0] + position) );
    scalar_t theta_im = sin( 0.5 * (_DH_params[0] + position) );
    scalar_t alpha_real = cos( 0.5 * _DH_params[3] );
    scalar_t alpha_im = sin( 0.5 * _DH_params[3] );
    scalar_t d = _DH_params[1];
    scalar_t a = _DH_params[2];
    return Pose{
        theta_real * alpha_real,
        theta_real * alpha_im,
        theta_im * alpha_im,
        theta_im * alpha_real,
        (scalar_t)0.5 * ( -theta_im * d * alpha_real - theta_real * a * alpha_im ),
        (scalar_t)0.5 * ( -theta_im * d * alpha_im + theta_real * a * alpha_real ),
        (scalar_t)0.5 * ( theta_im * a * alpha_real + theta_real * d * alpha_im ),
        (scalar_t)0.5 * ( theta_real * d * alpha_real - theta_im * a * alpha_im )
    };
}

Vec8 RevoluteJoint::end_pose_jacobian(const scalar_t position) const {
    scalar_t theta_dot_real = -0.5 * sin(0.5 * (_DH_params[0] + position));
    scalar_t theta_dot_im = 0.5 * cos(0.5 * (_DH_params[0] + position));
    scalar_t alpha_real = cos(0.5 * _DH_params[3]);
    scalar_t alpha_im = sin(0.5 * _DH_params[3]);
    scalar_t d = _DH_params[1];
    scalar_t a = _DH_params[2];
    return Vec8{
        theta_dot_real * alpha_real,
        theta_dot_real * alpha_im,
        theta_dot_im * alpha_im,
        theta_dot_im * alpha_real,
        (scalar_t)0.5 * (-theta_dot_im * d * alpha_real - theta_dot_real * a * alpha_im),
        (scalar_t)0.5 * (-theta_dot_im * d * alpha_im + theta_dot_real * a * alpha_real),
        (scalar_t)0.5 * (theta_dot_im * a * alpha_real + theta_dot_real * d * alpha_im),
        (scalar_t)0.5 * (theta_dot_real * d * alpha_real - theta_dot_im * a * alpha_im)
    };
}        



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// PrismaticJoint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


PrismaticJoint::PrismaticJoint(const Vec4 DH_parameters, const Vec4 motion_limits)
    : Joint(motion_limits), _DH_params(DH_parameters) {}

Pose PrismaticJoint::fkm(const scalar_t position) const {
    scalar_t theta_real = cos( 0.5 * _DH_params[0] );
    scalar_t theta_im = sin( 0.5 * _DH_params[0] );
    scalar_t alpha_real = cos( 0.5 * _DH_params[3] );
    scalar_t alpha_im = sin( 0.5 * _DH_params[3] );
    scalar_t d = _DH_params[1] + position;
    scalar_t a = _DH_params[2];
    return Pose{
        theta_real * alpha_real,
        theta_real * alpha_im,
        theta_im * alpha_im,
        theta_im * alpha_real,
        (scalar_t)0.5 * ( -theta_im * d * alpha_real - theta_real * a * alpha_im ),
        (scalar_t)0.5 * ( -theta_im * d * alpha_im + theta_real * a * alpha_real ),
        (scalar_t)0.5 * ( theta_im * a * alpha_real + theta_real * d * alpha_im ),
        (scalar_t)0.5 * ( theta_real * d * alpha_real - theta_im * a * alpha_im )
    };
}

Vec8 PrismaticJoint::end_pose_jacobian(const scalar_t position) const { // IGNORE THIS WARNING! Intentionally unused arg, only for satisfying virtual function signature
    
    scalar_t theta_real = sin(0.5 * _DH_params[0]);
    scalar_t theta_im = cos(0.5 * _DH_params[0]);
    scalar_t alpha_real = cos(0.5 * _DH_params[3]);
    scalar_t alpha_im = sin(0.5 * _DH_params[3]);
    // scalar_t d = 1;
    // scalar_t a = 0;
    return Vec8{
        0,
        0,
        0,
        0,
        (scalar_t)0.5 * -theta_im * alpha_real,
        (scalar_t)0.5 * -theta_im * alpha_im,
        (scalar_t)0.5 * theta_real * alpha_im,
        (scalar_t)0.5 * theta_real * alpha_real
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
        throw std::runtime_error("SerialManipulator(const std::string& , const Vecx& ) Could not open the " + params_file_path + " JSON file.");
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

    __check_size_inequality("SerialManipulator(const std::string& , const Vecx& ) arg0 theta", "DoF", DOF, 0);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 d", "DoF",d.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 a", "DoF",a.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 alpha", "DoF",alpha.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 joint_types", "DoF",joint_types.size(), DOF);
    

    DHParam DH_params;
    DH_params.resize(5, DOF);
    DH_params.row(0) = Eigen::Map<RowVecx>(theta.data(), DOF) / 180 * M_PI;
    DH_params.row(1) = Eigen::Map<RowVecx>(d.data(), DOF);
    DH_params.row(2) = Eigen::Map<RowVecx>(a.data(), DOF);
    DH_params.row(3) = Eigen::Map<RowVecx>(alpha.data(), DOF) / 180 * M_PI;
    DH_params.row(4) = Eigen::Map<RowVecx>(joint_types.data(), DOF);

    // Joint limits
    std::vector<scalar_t> min_joint_position = data["joint_limits"]["min_joint_positions"];
    std::vector<scalar_t> max_joint_position = data["joint_limits"]["max_joint_positions"];
    std::vector<scalar_t> min_joint_velocities = data["joint_limits"]["min_joint_velocities"];
    std::vector<scalar_t> max_joint_velocities = data["joint_limits"]["max_joint_velocities"];

    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 min_joint_position", "DoF",min_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 max_joint_position", "DoF",max_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 min_joint_velocities", "DoF",min_joint_velocities.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& , const Vecx& ) arg0 max_joint_velocities", "DoF",max_joint_velocities.size(), DOF);

    // solver_config
    _cfg.translation_priority = data["solver_config"]["translation_priority"];
    _cfg.error_gain = data["solver_config"]["error_gain"];
    _cfg.joint_damping = data["solver_config"]["joint_damping"];
    _cfg.sampling_time_sec = data["solver_config"]["sampling_time_sec"];

    JointLimits joint_limits;
    joint_limits.resize(4, DOF);
    // convert limits' unit to rad per loop from degree per second
    joint_limits.row(0) = Eigen::Map<RowVecx>(min_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(1) = Eigen::Map<RowVecx>(max_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(2) = Eigen::Map<RowVecx>(min_joint_velocities.data(), DOF) / 180 * M_PI;
    joint_limits.row(3) = Eigen::Map<RowVecx>(max_joint_velocities.data(), DOF) / 180 * M_PI;

    _construct(DH_params, joint_limits, joint_positions);
}

void SerialManipulator::set_base(const Pose& base) noexcept {
    _data.base = base;
}

void SerialManipulator::set_effector(const Pose& effector) noexcept {
    _data.effector = effector;
}

void SerialManipulator::update(const Pose& desired_pose) {
    USING_NAMESPACE_QPOASES;

    const Matx& r_rd_jacobian = desired_pose.rotation().haminus() * C4_ * _data.end_rotation_jacobian;
    const Matx& Ht = _data.end_translation_jacobian.transpose() * _data.end_translation_jacobian;
    const Matx& Hr = r_rd_jacobian.transpose() * r_rd_jacobian;
    const Vecx& damping_vec = Vecx::Ones(DoF()) * _cfg.joint_damping;
    const Matx& Hj = damping_vec.asDiagonal();
    const Matx& H = _cfg.translation_priority * Ht + (1-_cfg.translation_priority) * Hr + Hj;
    // the final data type must be of double, because the solver only accept double
    const Matxd Hd = H.cast<double>();

    const Vecx& vec_et = (_data.end_pose.translation() - desired_pose.translation()).vec4();
    const Vecx& vec_er = __closest_invariant_rotation_error(_data.end_pose.rotation(), desired_pose.rotation());
    const Vecx& ct = _cfg.error_gain * vec_et.transpose() * _data.end_translation_jacobian;
    const Vecx& cr = _cfg.error_gain * vec_er.transpose() * r_rd_jacobian;
    const Vecx& g = _cfg.translation_priority * ct + (1-_cfg.translation_priority) * cr;
    const Vecxd& gd = g.cast<double>();

    const Matxd& constraint = Matxd::Identity(DoF(), DoF());
    const Vecxd lower_joint_distance_double = min_joint_positions().cast<double>() - joint_positions().cast<double>();
    const Vecxd upper_joint_distance_double = max_joint_positions().cast<double>() - joint_positions().cast<double>();
    const Vecxd min_joint_velocirties_double = min_joint_velocities().cast<double>();
    const Vecxd max_joint_velocirties_double = max_joint_velocities().cast<double>();

    const double* H_raw = Hd.data();
    const double* g_raw = gd.data();
    const double* A_raw = constraint.data();
    const double* lb_raw = min_joint_velocirties_double.data();
    const double* ub_raw = max_joint_velocirties_double.data();
    const double* lb_A_raw = lower_joint_distance_double.data();
    const double* ub_A_raw = upper_joint_distance_double.data();

    static SQProblem qp(DoF(), DoF());
    static Options options;
    static int_t nWSR = 500;

    bool first_time(true);
    if (first_time){
        options.printLevel = PL_LOW;
        qp.setOptions(options);
        auto nWSR_in_use = nWSR;
        returnValue status = qp.init(H_raw, g_raw, A_raw, lb_raw, ub_raw, lb_A_raw, ub_A_raw, nWSR_in_use); 
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }
        first_time = false;
    }else{
        auto nWSR_in_use = nWSR;
        returnValue status = qp.hotstart(H_raw, g_raw, A_raw, lb_raw, ub_raw, lb_A_raw, ub_A_raw, nWSR_in_use);
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }
    }
    double xOpt[DoF()];
    qp.getPrimalSolution(xOpt);
    Eigen::Map<Vecxd> u(xOpt, DoF());

    // update joint positions
    _data.joint_positions += u.cast<scalar_t>() * _cfg.sampling_time_sec;
    _update_kinematics();
}

Vecx SerialManipulator::min_joint_positions() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = _joints[i]->min_position();
    }
    return res;
}

Vecx SerialManipulator::max_joint_positions() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = _joints[i]->max_position();
    }
    return res;
}

Vecx SerialManipulator::min_joint_velocities() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = _joints[i]->min_velocitiy();
    }
    return res;
}

Vecx SerialManipulator::max_joint_velocities() const noexcept {
    Vecx res(DoF());
    for (size_t i = 0; i < DoF(); ++i) {
        res[i] = _joints[i]->max_velocitiy();
    }
    return res;
}

void SerialManipulator::_update_kinematics() {
    // Manually calculate the first joint pose and the first col of the pose jacobian
    _data.joint_poses[0] = _joints[0]->fkm(_data.joint_positions[0]);
    for (size_t i=1; i<DoF(); ++i){
        _data.joint_poses[i] = _data.joint_poses[i-1] * _joints[i]->fkm(_data.joint_positions[i]);
    }
    _data.end_pose_jacobian.col(0) = (_data.joint_poses[0].conj() * _data.joint_poses.back()).haminus() * _joints[0]->end_pose_jacobian(_data.joint_positions[0]);
    // Iterately calculate the following joint poses and the last cols of the pose jacobian
    for (size_t i=1; i<DoF(); ++i){
        _data.end_pose_jacobian.col(i) = (_data.joint_poses[i].conj() * _data.joint_poses.back()).haminus()
                                         * _data.joint_poses[i-1].hamiplus()
                                         * _joints[i]->end_pose_jacobian(_data.joint_positions[i]);
    }
    // Get the final pose and jacobians
    _data.end_pose = _data.base * _data.joint_poses.back() * _data.effector;
    _data.end_pose_jacobian = _data.effector.haminus() * _data.base.hamiplus() * _data.end_pose_jacobian;
    _data.end_rotation_jacobian = _data.end_pose_jacobian.block(0,0,4,DoF());
    _data.end_translation_jacobian = 2 * _data.end_pose.rotation().conj().haminus() * _data.end_pose_jacobian.block(4,0,4,DoF()) + 2 * _data.end_pose.dual().hamiplus() * C4_ * _data.end_rotation_jacobian;
}

void SerialManipulator::_construct(const DHParam& DH_params, const JointLimits& joint_limits, const Vecx& joint_positions){
    
    size_t DOF = DH_params.cols();

    _data.joint_poses.resize(DOF);
    _data.end_pose_jacobian.resize(8, DOF);
    _data.end_rotation_jacobian.resize(4, DOF);
    _data.end_translation_jacobian.resize(4, DOF);

    // instantiate remaining joints
    for (size_t i=0; i<DOF; ++i){
        if (DH_params(4, i) == 0){
            _joints.push_back(std::make_unique<RevoluteJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i)));
        } else if (DH_params(4, i) == 1){
            _joints.push_back(std::make_unique<PrismaticJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i)));
        } else {
            throw std::runtime_error("SerialManipulator(const Matx& DH_params, const Matx& joint_limits, const Vecx& joint_positions) arg0 5-th row DH_params[4, " + std::to_string(i) + "] = " + std::to_string(DH_params(4, i)) + ", this row should consist only of 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
        }
    }

    _data.joint_positions = joint_positions;
    _update_kinematics();

    std::cout << "Constructed a " + std::to_string(DOF) + "-DoF Kinematics::SerialManipulator.\n" ;
}

} // namespace kinematics

} // namespace dq1

