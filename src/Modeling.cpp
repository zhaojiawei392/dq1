#include "Modeling.hpp"
#include <stdexcept>
#include <iostream>
#include <qpOASES.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

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

void __check_size_equality(const std::string& src_info, const std::string& des_info, int src_size, int des_size){
    if (src_size != des_size){
        throw std::range_error(src_info + " invalid size " + std::to_string(src_size) + ", should be equal to " + des_info + " " + std::to_string(des_size) + ".\n" );
    }
}

void __check_size_inequality(const std::string& src_info, const std::string& des_info, int src_size, int des_size){
    if (src_size == des_size){
        throw std::range_error(src_info + " invalid size " + std::to_string(src_size) + ", should NOT be equal to " + des_info + " " + std::to_string(des_size) + ".\n" );
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


SerialManipulator::SerialManipulator(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions) {
    int DOF = DH_params.cols();
    __check_size_inequality("SerialManipulator(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions) arg0 cols", "DoF", DOF, 0);
    __check_size_equality("SerialManipulator(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions) arg1 cols", "DoF",joint_limits.cols(), DOF);
    __check_size_equality("SerialManipulator(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions) arg2", "DoF",joint_positions.size(), DOF);
    
    _construct(DH_params, joint_limits, joint_positions);
}

SerialManipulator::SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions){

    std::ifstream json_file(params_file_path);

    // Check if the file was opened successfully
    if (!json_file.is_open()) {
        throw std::runtime_error("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) Could not open the JSON file.");
    }

    // Parse the JSON file into a json object
    nlohmann::json data = nlohmann::json::parse(json_file);
    // Close the file
    json_file.close();

    // DH parameters
    std::vector<double> theta = data["DH_params"]["theta"];
    std::vector<double> d = data["DH_params"]["d"];
    std::vector<double> a = data["DH_params"]["a"];
    std::vector<double> alpha = data["DH_params"]["alpha"];
    std::vector<double> joint_types = data["DH_params"]["joint_types"];

    int DOF = theta.size(); 

    __check_size_inequality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 theta", "DoF", DOF, 0);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 d", "DoF",d.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 a", "DoF",a.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 alpha", "DoF",alpha.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 joint_types", "DoF",joint_types.size(), DOF);
    

    DH_mat DH_params;
    DH_params.resize(5, DOF);
    DH_params.row(0) = Eigen::Map<RowVecxd>(theta.data(), DOF) / 180 * M_PI;
    DH_params.row(1) = Eigen::Map<RowVecxd>(d.data(), DOF);
    DH_params.row(2) = Eigen::Map<RowVecxd>(a.data(), DOF);
    DH_params.row(3) = Eigen::Map<RowVecxd>(alpha.data(), DOF) / 180 * M_PI;
    DH_params.row(4) = Eigen::Map<RowVecxd>(joint_types.data(), DOF);

    // Joint limits
    std::vector<double> min_joint_position = data["joint_limits"]["min_joint_position"];
    std::vector<double> max_joint_position = data["joint_limits"]["max_joint_position"];
    std::vector<double> min_joint_velocities = data["joint_limits"]["min_joint_velocities"];
    std::vector<double> max_joint_velocities = data["joint_limits"]["max_joint_velocities"];

    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 min_joint_position", "DoF",min_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 max_joint_position", "DoF",max_joint_position.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 min_joint_velocities", "DoF",min_joint_velocities.size(), DOF);
    __check_size_equality("SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions) arg0 max_joint_velocities", "DoF",max_joint_velocities.size(), DOF);

    Joint_limit_mat joint_limits;
    joint_limits.resize(4, DOF);
    joint_limits.row(0) = Eigen::Map<RowVecxd>(min_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(1) = Eigen::Map<RowVecxd>(max_joint_position.data(), DOF) / 180 * M_PI;
    joint_limits.row(2) = Eigen::Map<RowVecxd>(min_joint_velocities.data(), DOF) / 180 * M_PI;
    joint_limits.row(3) = Eigen::Map<RowVecxd>(max_joint_velocities.data(), DOF) / 180 * M_PI;

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

    const Matxd& r_rd_jacobian = desired_pose.rotation().haminus() * C4_ * r_jacobian_;
    const Matxd& Ht = t_jacobian_.transpose() * t_jacobian_;
    const Matxd& Hr = r_rd_jacobian.transpose() * r_rd_jacobian;
    const Vecxd& damping_vec = Vecxd::Ones(DoF()) * 0.0001;
    const Matxd& Hj = damping_vec.asDiagonal();
    const Matxd& H = cfg_.translation_priority * Ht + (1-cfg_.translation_priority) * Hr + Hj;

    const Vecxd& vec_et = (end_pose_.translation() - desired_pose.translation()).vec4();
    const Vecxd& vec_er = __closest_invariant_rotation_error(end_pose_.rotation(), desired_pose.rotation());
    const Vecxd& ct = cfg_.error_gain * vec_et.transpose() * t_jacobian_;
    const Vecxd& cr = cfg_.error_gain * vec_er.transpose() * r_rd_jacobian;
    const Vecxd& g = cfg_.translation_priority * ct + (1-cfg_.translation_priority) * cr;

    const Matxd& constraint = Vecxd::Ones(DoF()).asDiagonal();

    SQProblem qp(DoF(), 0);
    Options options;
    options.printLevel = PL_LOW;
    qp.setOptions(options);

    int_t nWSR = 500;
    const double* H_raw = H.data();
    const double* g_raw = g.data();
    const double* A_raw = constraint.data();
    const double* lb_raw = min_joint_velocities().data();
    const double* ub_raw = max_joint_velocities().data();

    bool first_time{true};
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
    real_t xOpt[DoF()];
    qp.getPrimalSolution(xOpt);
    Eigen::Map<Vecxd> u(xOpt, DoF());
    update_joint_signals(u*cfg_.sampling_time_sec);
}

void SerialManipulator::update_joint_positions(const Vecxd& joint_positions) {
    __check_size_equality("update_joint_positions(const Vecxd& joint_positions)", "DoF", joint_positions.size(), DoF());
    
    joints_[0]->update(joint_positions[0]);
    joint_poses_[0] = joints_[0]->fkm();

    for (int i=1; i<DoF(); ++i){
        joints_[i]->update(joint_positions[i]);
        joint_poses_[i] = joint_poses_[i-1] * joints_[i]->fkm();
    }
    end_pose_ = base_ * joint_poses_.back() * effector_;
    _update_jacobians();
}

void SerialManipulator::update_joint_signals(const Vecxd& joint_signals) {
    __check_size_equality("update_joint_positions(const Vecxd& joint_signals)", "DoF", joint_signals.size(), DoF());
    
    joints_[0]->update_signal(joint_signals[0]);
    joint_poses_[0] = joints_[0]->fkm();

    for (int i=1; i<DoF(); ++i){
        joints_[i]->update_signal(joint_signals[i]);
        joint_poses_[i] = joint_poses_[i-1] * joints_[i]->fkm();
    }
    end_pose_ = base_ * joint_poses_.back() * effector_;
    _update_jacobians();
}

Vecxd SerialManipulator::joint_positions() const noexcept {
    Vecxd res(DoF());
    for (int i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->position();
    }
    return res;
}

Vecxd SerialManipulator::min_joint_positions() const noexcept {
    Vecxd res(DoF());
    for (int i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->min_position();
    }
    return res;
}

Vecxd SerialManipulator::max_joint_positions() const noexcept {
    Vecxd res(DoF());
    for (int i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->max_position();
    }
    return res;
}

Vecxd SerialManipulator::min_joint_velocities() const noexcept {
    Vecxd res(DoF());
    for (int i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->min_velocitiy();
    }
    return res;
}

Vecxd SerialManipulator::max_joint_velocities() const noexcept {
    Vecxd res(DoF());
    for (int i = 0; i < DoF(); ++i) {
        res[i] = joints_[i]->max_velocitiy();
    }
    return res;
}

void SerialManipulator::_update_jacobians() {
    // Manually calculate first col of the pose jacobian
    const Vec8d& pose_jacobian_0 = (joint_poses_[0].conj() * joint_poses_.back()).haminus() * joints_[0]->jacobian();
    pose_jacobian_.col(0) = pose_jacobian_0;

    // Iterately calculate last cols of the pose jacobian
    for (int i=1; i<DoF(); ++i){
        const Vec8d& pose_jacobian_i = (joint_poses_[i].conj() * joint_poses_.back()).haminus() * joint_poses_[i-1].hamiplus() * joints_[i]->jacobian();
        pose_jacobian_.col(i) = pose_jacobian_i;
    }
    pose_jacobian_ = effector_.haminus() * base_.hamiplus() * pose_jacobian_;
    r_jacobian_ = pose_jacobian_.block(0,0,4,DoF());
    t_jacobian_ = 2 * end_pose_.rotation().conj().haminus() * pose_jacobian_.block(4,0,4,DoF()) + 2 * end_pose_.dual().hamiplus() * C4_ * r_jacobian_;
}

void SerialManipulator::_construct(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions){

    int DOF = DH_params.cols();

    joint_poses_.resize(DOF);
    pose_jacobian_.resize(8, DOF);
    r_jacobian_.resize(4, DOF);
    t_jacobian_.resize(4, DOF);

    // instantiate remaining joints
    for (int i=0; i<DOF; ++i){
        if (DH_params(4, i) == 0){
            joints_.push_back(std::make_unique<RevoluteJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
        } else if (DH_params(4, i) == 1){
            joints_.push_back(std::make_unique<PrismaticJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
        } else {
            throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) arg0 5-th row DH_params[4, " + std::to_string(i) + "] = " + std::to_string(DH_params(4, i)) + ", this row should consist only of 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
        }
    }
    const Vecxd initial_signals = Vecxd::Zero(DoF());
    update_joint_signals(initial_signals);

    std::cout << "A Kinematics::SerialManipulator constructed!! DoF = " + std::to_string(DOF) + ".\n" ;
}

} // namespace kinematics

} // namespace dq1

