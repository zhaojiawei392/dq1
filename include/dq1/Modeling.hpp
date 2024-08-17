#pragma once
#include <memory>
#include <vector>
#include "Pose.hpp"

namespace dq1
{
namespace kinematics
{

class Joint {
protected:
    Vec4d limits_; // {min position, max position, min velocitiy, max velocitiy}
    void _check_limits();
    void _make_position_within_limits(double& position);

public:
    Joint() = delete;
    Joint(const Vec4d motion_limits);
    virtual ~Joint() = default;

    virtual Pose fkm(const double position) const = 0;
    virtual Vec8d pose_jacobian(const double position) const = 0;

    inline Vec4d motion_limits() const noexcept {return limits_;};
    inline double min_position() const noexcept {return limits_[0];};
    inline double max_position() const noexcept {return limits_[1];};
    inline double min_velocitiy() const noexcept {return limits_[2];};
    inline double max_velocitiy() const noexcept {return limits_[3];};
};

class RevoluteJoint : public Joint {
protected:
    Vec4d DH_params_; 
public:
    RevoluteJoint() = delete;
    RevoluteJoint(const Vec4d DH_parameters, const Vec4d motion_limits);
    virtual ~RevoluteJoint() = default;
    
    virtual Pose fkm(const double position) const override;
    virtual Vec8d pose_jacobian(const double position) const override;
};

class PrismaticJoint : public Joint {
protected:
    Vec4d DH_params_; 
public:
    PrismaticJoint() = delete;
    PrismaticJoint(const Vec4d DH_parameters, const Vec4d motion_limits);
    virtual ~PrismaticJoint() = default;

    virtual Pose fkm(const double position) const override;
    virtual Vec8d pose_jacobian(const double position=0) const override;
};

struct SerialManipulatorConfig{
    double translation_priority{1};
    double error_gain{50};
    double sampling_time_sec{0.0004};
    double joint_damping{0.0001};
};

using DH_mat = Matd<5, -1>;
using Joint_limit_mat = Matd<4, -1>;

class SerialManipulator {
protected:
    SerialManipulatorConfig cfg_;
    std::vector<std::unique_ptr<Joint>> joints_;
    Vecxd joint_positions_;
    Pose base_;
    Pose effector_;
    Pose end_pose_;
    std::vector<Pose> joint_poses_; 
    Pose_jcb pose_jacobian_;
    Rot_jcb r_jacobian_;
    Tran_jcb t_jacobian_;
public:
    SerialManipulator() = delete;
    SerialManipulator(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions);
    SerialManipulator(const std::string& params_file_path, const Vecxd& joint_positions);
    virtual ~SerialManipulator() = default;

    void set_base(const Pose& base) noexcept;
    void set_effector(const Pose& effector) noexcept;
    void update(const Pose& desired_pose);

    void set_joint_positions(const Vecxd& joint_positions);

    inline Vecxd joint_positions() const noexcept {return joint_positions_;}
    Vecxd min_joint_positions() const noexcept;
    Vecxd max_joint_positions() const noexcept;
    Vecxd min_joint_velocities() const noexcept;
    Vecxd max_joint_velocities() const noexcept;
    inline Pose end_pose() const noexcept {return end_pose_;}
    inline int DoF() const noexcept {return joints_.size();}
    inline SerialManipulatorConfig& config() noexcept {return cfg_;}

private:
    void _update_kinematics();
    void _construct(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecxd& joint_positions);
};

} // namespace kinematics

} // namespace dq1

