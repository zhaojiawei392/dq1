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
    Vec4 limits_; // {min position, max position, min velocitiy, max velocitiy}
    void _check_limits();
    void _make_position_within_limits(scalar_t& position);

public:
    Joint() = delete;
    Joint(const Vec4 motion_limits);
    virtual ~Joint() = default;

    virtual Pose fkm(const scalar_t position) const = 0;
    virtual Vec8 pose_jacobian(const scalar_t position) const = 0;

    inline Vec4 motion_limits() const noexcept {return limits_;};
    inline scalar_t min_position() const noexcept {return limits_[0];};
    inline scalar_t max_position() const noexcept {return limits_[1];};
    inline scalar_t min_velocitiy() const noexcept {return limits_[2];};
    inline scalar_t max_velocitiy() const noexcept {return limits_[3];};
};

class RevoluteJoint : public Joint {
protected:
    Vec4 DH_params_; 
public:
    RevoluteJoint() = delete;
    RevoluteJoint(const Vec4 DH_parameters, const Vec4 motion_limits);
    virtual ~RevoluteJoint() = default;
    
    virtual Pose fkm(const scalar_t position) const override;
    virtual Vec8 pose_jacobian(const scalar_t position) const override;
};

class PrismaticJoint : public Joint {
protected:
    Vec4 DH_params_; 
public:
    PrismaticJoint() = delete;
    PrismaticJoint(const Vec4 DH_parameters, const Vec4 motion_limits);
    virtual ~PrismaticJoint() = default;

    virtual Pose fkm(const scalar_t position) const override;
    virtual Vec8 pose_jacobian(const scalar_t position=0) const override;
};

struct SerialManipulatorConfig{
    scalar_t translation_priority{1};
    scalar_t error_gain{50};
    scalar_t sampling_time_sec{0.0004};
    scalar_t joint_damping{0.0001};
};


class SerialManipulator {
protected:
    SerialManipulatorConfig cfg_;
    std::vector<std::unique_ptr<Joint>> joints_;
    Vecx joint_positions_;
    Pose base_;
    Pose effector_;
    Pose end_pose_;
    std::vector<Pose> joint_poses_; 
    Pose_jcb pose_jacobian_;
    Rot_jcb r_jacobian_;
    Tran_jcb t_jacobian_;
public:
    SerialManipulator() = delete;
    SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions);
    virtual ~SerialManipulator() = default;

    void set_base(const Pose& base) noexcept;
    void set_effector(const Pose& effector) noexcept;
    void update(const Pose& desired_pose);

    void set_joint_positions(const Vecx& joint_positions);
    
    // query
    Pose fkm(size_t index) const;
    inline Vecx joint_positions() const noexcept {return joint_positions_;}
    Vecx min_joint_positions() const noexcept;
    Vecx max_joint_positions() const noexcept;
    Vecx min_joint_velocities() const noexcept;
    Vecx max_joint_velocities() const noexcept;
    inline Pose end_pose() const noexcept {return end_pose_;}
    inline size_t DoF() const noexcept {return joints_.size();}
    inline SerialManipulatorConfig& config() noexcept {return cfg_;}

private:
    void _update_kinematics();
    void _construct(const DH_mat& DH_params, const Joint_limit_mat& joint_limits, const Vecx& joint_positions);
};

} // namespace kinematics

} // namespace dq1

