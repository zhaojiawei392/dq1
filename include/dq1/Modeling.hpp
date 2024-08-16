#pragma once
#include "Pose.hpp"
#include <memory>
#include <qpOASES.hpp>
#include <vector>

namespace dq1
{
namespace kinematics
{

class Joint {
protected:
    double pos_;  // joint position
    Vec4d limits_; // {min position, max position, min speed, max speed}
    Pose fkm_;  // forward kinematics, pose transformation from reference to end  
    Vec8d jcb_;  // jacobian of reference end pose with respect to joint position
    void _check_limits();
    void _make_position_within_limits();

public:
    Joint() = delete;
    Joint(const Vec4d& motion_limits, double position = 0);
    virtual ~Joint() = default;

    virtual void update(const double position) noexcept;
    virtual void update_signal(const double signal) noexcept;

    virtual Pose calculate_fkm(const double position) const = 0;
    virtual Vec8d calculate_jacobian(const double position) const = 0;

    double position() const noexcept;
    Vec4d motion_limits() const noexcept;
    Pose fkm() const noexcept;
    Vec8d jacobian() const noexcept;
    double min_position() const noexcept;
    double max_position() const noexcept;
    double min_speed() const noexcept;
    double max_speed() const noexcept;
};

class RevoluteJoint : public Joint {
protected:
    Vec4d DH_params_; 
public:
    RevoluteJoint() = delete;
    RevoluteJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position = 0);
    virtual ~RevoluteJoint() = default;
    
    virtual Pose calculate_fkm(const double position) const override;
    virtual Vec8d calculate_jacobian(const double position) const override;
};

class PrismaticJoint : public Joint {
protected:
    Vec4d DH_params_; 
public:
    PrismaticJoint() = delete;
    PrismaticJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position = 0);
    virtual ~PrismaticJoint() = default;

    virtual Pose calculate_fkm(const double position) const override;
    virtual Vec8d calculate_jacobian(const double position=0) const override;
};

class SerialManipulator {
protected:
    std::vector<std::unique_ptr<Joint>> joints_;
    Pose base_;
    Pose effector_;
    Pose end_pose_;
    std::vector<Pose> joint_poses_; 
    Pose_jcb pose_jacobian_;
    Rot_jcb r_jacobian_;
    Tslt_jcb t_jacobian_;
public:
    SerialManipulator() = delete;
    SerialManipulator(const Matd<5, -1>& DH_params, const Matd<4, -1>& joint_limits, const Vecxd& joint_positions);
    virtual ~SerialManipulator() = default;

    void set_base(const Pose& base) noexcept;
    void set_effector(const Pose& effector) noexcept;
    void update(const Pose& desired_pose);

    void update_joint_positions(const Vecxd& joint_positions);
    void update_joint_signals(const Vecxd& joint_signals);

    Vecxd joint_positions() const noexcept;
    Vecxd min_joint_positions() const noexcept;
    Vecxd max_joint_positions() const noexcept;
    Vecxd min_joint_speeds() const noexcept;
    Vecxd max_joint_speeds() const noexcept;
    Pose end_pose() const noexcept;
    int DoF() const noexcept;

private:
    void _update_jacobians();
};

} // namespace kinematics
} // namespace dq1

