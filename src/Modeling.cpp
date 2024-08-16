#include "Modeling.hpp"
#include <stdexcept>
#include <iostream>

namespace dq1{

namespace kinematics{

const Vecxd __closest_invariant_rotation_error(const Rot& r, const Rot& rd) {
    Quat er_plus = r.conj() * rd - Quat(1);
    Quat er_minus = r.conj() * rd + Quat(1);

    if (er_plus.norm() < er_minus.norm()) {
        return er_plus.vec4();
    } else {
        return er_minus.vec4();
    }
}

void __check_size(const std::string& src_info, const std::string& des_info, int src_size, int des_size){
    if (src_size != des_size){
        throw std::range_error(src_info + " invalid size " + std::to_string(src_size) + ", should be equal to " + des_info + " " + std::to_string(des_size) + ".\n" );
    }
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Joint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


Joint::Joint(const Vec4d& motion_limits, double position)
    : pos_(position), limits_(motion_limits), fkm_(1) {
    _check_limits();
}

void Joint::_check_limits() {
    if (limits_[0] > limits_[1] || limits_[2] > limits_[3]) {
        throw std::runtime_error("Joint complains about unreasonable joint limits, check if max position limits are all bigger than the min limits.\n");
    }
}

void Joint::_make_position_within_limits() {
    if (pos_ < limits_[0]) 
        pos_ = limits_[0];
    if (pos_ > limits_[1]) 
        pos_ = limits_[1];
}

void Joint::update(const double position) noexcept{
    pos_ = position;
    _make_position_within_limits();
    fkm_ = calculate_fkm(pos_);
    jcb_ = calculate_jacobian(pos_);
}

void Joint::update_signal(const double signal) noexcept{
    update(pos_ + signal);
}

double Joint::position() const noexcept {
    return pos_;
}

Vec4d Joint::motion_limits() const noexcept {
    return limits_;
}

Pose Joint::fkm() const noexcept {
    return fkm_;
}

Vec8d Joint::jacobian() const noexcept {
    return jcb_;
}

double Joint::min_position() const noexcept {
    return limits_[0];
}

double Joint::max_position() const noexcept {
    return limits_[1];
}

double Joint::min_speed() const noexcept {
    return limits_[2];
}

double Joint::max_speed() const noexcept {
    return limits_[3];
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// RevoluteJoint class *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


RevoluteJoint::RevoluteJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position)
    : Joint(motion_limits, position), DH_params_(DH_parameters) {
    update(position);
}

Pose RevoluteJoint::calculate_fkm(const double position) const {
    double theta_real = cos( 0.5 * (DH_params_[0] + position) );
    double theta_im = sin( 0.5 * (DH_params_[0] + position) );
    double alpha_real = cos( 0.5 * DH_params_[3] );
    double alpha_im = sin( 0.5 * DH_params_[3] );
    double d = DH_params_[1];
    double a = DH_params_[2];
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

Vec8d RevoluteJoint::calculate_jacobian(const double position) const {
    double theta_dot_real = -0.5 * sin(0.5 * (DH_params_[0] + position));
    double theta_dot_im = 0.5 * cos(0.5 * (DH_params_[0] + position));
    double alpha_real = cos(0.5 * DH_params_[3]);
    double alpha_im = sin(0.5 * DH_params_[3]);
    double d = DH_params_[1];
    double a = DH_params_[2];
    return Vec8d{
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


PrismaticJoint::PrismaticJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position)
    : Joint(motion_limits, position), DH_params_(DH_parameters) {}

Pose PrismaticJoint::calculate_fkm(const double position) const {
    double theta_real = cos( 0.5 * DH_params_[0] );
    double theta_im = sin( 0.5 * DH_params_[0] );
    double alpha_real = cos( 0.5 * DH_params_[3] );
    double alpha_im = sin( 0.5 * DH_params_[3] );
    double d = DH_params_[1] + position;
    double a = DH_params_[2];
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

Vec8d PrismaticJoint::calculate_jacobian(const double position) const { // Intentionally unused arg, only for satisfying virtual function signature
    
    double theta_real = sin(0.5 * DH_params_[0]);
    double theta_im = cos(0.5 * DH_params_[0]);
    double alpha_real = cos(0.5 * DH_params_[3]);
    double alpha_im = sin(0.5 * DH_params_[3]);
    // double d = 1;
    // double a = 0;
    return Vec8d{
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


SerialManipulator::SerialManipulator(const Matd<5, -1>& DH_params, const Matd<4, -1>& joint_limits, const Vecxd& joint_positions) {
    if (DH_params.cols() == 0 ){
        throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) arg0 cols invalid size 0, should be DoF bigger than 0.\n");
    }
    size_t DOF = DH_params.cols();
    __check_size("SerialManipulator(const Matd<5, -1>& DH_params, const Matd<4, -1>& joint_limits, const Vecxd& joint_positions) arg1 cols", "DoF",joint_limits.cols(), DOF);
    __check_size("SerialManipulator(const Matd<5, -1>& DH_params, const Matd<4, -1>& joint_limits, const Vecxd& joint_positions) arg2", "DoF",joint_positions.size(), DOF);
    
    joint_poses_.resize(DOF);
    pose_jacobian_.resize(8, DOF);
    r_jacobian_.resize(4, DOF);
    t_jacobian_.resize(4, DOF);


    // instantiate remaining joints
    for (size_t i=0; i<DOF; ++i){
        if (DH_params(4, i) == 0){
            joints_.push_back(std::make_unique<RevoluteJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
        } else if (DH_params(4, i) == 1){
            joints_.push_back(std::make_unique<PrismaticJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
        } else {
            throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) arg0 5-th row DH_params[4, " + std::to_string(i) + "] = " + std::to_string(DH_params(4, i)) + ", this row should consist only of 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
        }
    }
    const Vecxd initial_signals = Vecxd::Zero(joints_.size());
    update_joint_signals(initial_signals);

    std::cout << "A " + std::to_string(DOF) + " DoF Kinematics::SerialManipulator constructed!\n" ;
}

void SerialManipulator::set_base(const Pose& base) noexcept {
    base_ = base;
}

void SerialManipulator::set_effector(const Pose& effector) noexcept {
    effector_ = effector;
}

void SerialManipulator::update(const Pose& desired_pose) {
    USING_NAMESPACE_QPOASES;

    const Matxd& r_rd_jacobian = desired_pose.rotation().conj().haminus() * r_jacobian_;
    const Matxd& Ht = (t_jacobian_.transpose() * t_jacobian_) * 2;
    const Matxd& Hr = (r_rd_jacobian.transpose() * r_rd_jacobian) * 2;
    const Vecxd& damping_vec = Vecxd::Ones(joints_.size()) * 0.0001;
    const Matxd& Hj = damping_vec.asDiagonal();
    const Matxd& H = 0.9999 * Ht + (1-0.9999) * Hr + Hj;

    const Vecxd& vec_et = (abs_end_.translation() - desired_pose.translation()).vec4();
    const Vecxd& vec_er = __closest_invariant_rotation_error(abs_end_.rotation(), desired_pose.rotation());
    const Vecxd& ct = 2 * 50 * vec_et.transpose() * t_jacobian_;
    const Vecxd& cr = 2 * 50 * vec_er.transpose() * r_rd_jacobian;
    const Vecxd& g = 0.9999 * ct + (1-0.9999) * cr;

    const Matxd& constraint = Vecxd::Ones(joints_.size()).asDiagonal();

    SQProblem qp(joints_.size(), 0);
    Options options;
    options.printLevel = PL_LOW;
    qp.setOptions(options);

    int_t nWSR = 500;
    const double* H_raw = H.data();
    const double* g_raw = g.data();
    const double* A_raw = constraint.data();
    const double* lb_raw = min_joint_speeds().data();
    const double* ub_raw = max_joint_speeds().data();

    bool first_time{true};
    if (first_time){
        returnValue status = qp.init(H_raw, g_raw, A_raw, nullptr, nullptr, lb_raw, ub_raw, nWSR); // intentionally raw pointers
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }
        first_time = false;
    }else{
        returnValue status = qp.hotstart(H_raw, g_raw, A_raw, nullptr, nullptr, lb_raw, ub_raw, nWSR);
        if (status != SUCCESSFUL_RETURN){
            throw std::runtime_error("Failed to solve QP problem.\n");
        }

    }
    real_t xOpt[joints_.size()];
    qp.getPrimalSolution(xOpt);
    Eigen::Map<Vecxd> u(xOpt, joints_.size());
    update_joint_signals(u*0.0004);
}

void SerialManipulator::update_joint_positions(const Vecxd& joint_positions) {
    __check_size("update_joint_positions(const Vecxd& joint_positions)", "DoF", joint_positions.size(), joints_.size());
    
    joints_[0]->update(joint_positions[0]);
    joint_poses_[0] = joints_[0]->fkm();

    for (size_t i=1; i<joints_.size(); ++i){
        joints_[i]->update(joint_positions[i]);
        joint_poses_[i] = joint_poses_[i-1] * joints_[i]->fkm();
    }
    abs_end_ = base_ * joint_poses_.back() * effector_;
    _update_jacobians();
}

void SerialManipulator::update_joint_signals(const Vecxd& joint_signals) {
    __check_size("update_joint_positions(const Vecxd& joint_signals)", "DoF", joint_signals.size(), joints_.size());
    
    joints_[0]->update_signal(joint_signals[0]);
    joint_poses_[0] = joints_[0]->fkm();

    for (size_t i=1; i<joints_.size(); ++i){
        joints_[i]->update_signal(joint_signals[i]);
        joint_poses_[i] = joint_poses_[i-1] * joints_[i]->fkm();
    }
    abs_end_ = base_ * joint_poses_.back() * effector_;
    _update_jacobians();
}

Vecxd SerialManipulator::joint_positions() const noexcept {
    Vecxd res(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        res[i] = joints_[i]->position();
    }
    return res;
}

Vecxd SerialManipulator::min_joint_positions() const noexcept {
    Vecxd res(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        res[i] = joints_[i]->min_position();
    }
    return res;
}

Vecxd SerialManipulator::max_joint_positions() const noexcept {
    Vecxd res(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        res[i] = joints_[i]->max_position();
    }
    return res;
}

Vecxd SerialManipulator::min_joint_speeds() const noexcept {
    Vecxd res(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        res[i] = joints_[i]->min_speed();
    }
    return res;
}

Vecxd SerialManipulator::max_joint_speeds() const noexcept {
    Vecxd res(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        res[i] = joints_[i]->max_speed();
    }
    return res;
}

Pose SerialManipulator::end_pose() const noexcept {
    return abs_end_;
}

int SerialManipulator::DoF() const noexcept {
    return joints_.size();
}

void SerialManipulator::_update_jacobians() {
    for (size_t i=0; i<joints_.size(); ++i){
        const Vec8d& pose_jacobian_i = (joint_poses_[i].conj() * joint_poses_.back()).haminus() * joints_[i]->jacobian();
        pose_jacobian_.col(i) = pose_jacobian_i;
    }
    pose_jacobian_ = effector_.haminus() * base_.hamiplus() * pose_jacobian_;

    r_jacobian_ = pose_jacobian_.block(0,0,4,joints_.size());

    t_jacobian_ = abs_end_.rotation().conj().haminus() * (pose_jacobian_.block(4,0,4,joints_.size()) * 2 - abs_end_.translation().hamiplus() * r_jacobian_);
}

} // namespace kinematics

} // namespace dq1

