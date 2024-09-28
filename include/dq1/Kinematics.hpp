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
 *     \file include/Kinematics.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include <memory>
#include <vector>
#include "Pose.hpp"
#include "Macro.hpp"

namespace dq1
{

namespace kinematics
{

using namespace Macro;

class Joint {
protected:
    Vec4 _limits; // {min position, max position, min velocitiy, max velocitiy}
    void _check_limits();
    void _make_position_within_limits(scalar_t& position);

public:
    Joint() = delete;
    Joint(const Vec4 motion_limits);
    virtual ~Joint() = default;

    virtual Pose fkm(const scalar_t position) const = 0;
    virtual Vec8 end_pose_jacobian(const scalar_t position) const = 0;

    inline Vec4 motion_limits() const noexcept {return _limits;};
    inline scalar_t min_position() const noexcept {return _limits[0];};
    inline scalar_t max_position() const noexcept {return _limits[1];};
    inline scalar_t min_velocitiy() const noexcept {return _limits[2];};
    inline scalar_t max_velocitiy() const noexcept {return _limits[3];};
};

class RevoluteJoint : public Joint {
protected:
    Vec4 _DH_params; 
public:
    RevoluteJoint() = delete;
    RevoluteJoint(const Vec4 DH_parameters, const Vec4 motion_limits);
    virtual ~RevoluteJoint() = default;
    
    virtual Pose fkm(const scalar_t position) const override;
    virtual Vec8 end_pose_jacobian(const scalar_t position) const override;
};

class PrismaticJoint : public Joint {
protected:
    Vec4 _DH_params; 
public:
    PrismaticJoint() = delete;
    PrismaticJoint(const Vec4 DH_parameters, const Vec4 motion_limits);
    virtual ~PrismaticJoint() = default;

    virtual Pose fkm(const scalar_t position) const override;
    virtual Vec8 end_pose_jacobian(const scalar_t position=0) const override;
};

struct SerialManipulatorConfig{
    scalar_t translation_priority{1};
    scalar_t error_gain{50};
    scalar_t sampling_time_sec{0.0004};
    scalar_t joint_damping{0.0001};
};

struct SerialManipulatorData{
    Vecx joint_positions;
    std::vector<Pose> joint_poses; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
    Pose base;
    Pose effector;
    Pose end_pose;
    PoseJacobian end_pose_jacobian;
    RotationJacobian end_rotation_jacobian;
    TranslationJacobian end_translation_jacobian;
};


class SerialManipulator {
protected:
    SerialManipulatorConfig _cfg;
    std::vector<std::unique_ptr<Joint>> _joints;
    SerialManipulatorData _data;
public:
    SerialManipulator() = delete;
    SerialManipulator(const std::string& params_file_path, const Vecx& joint_positions);
    virtual ~SerialManipulator() = default;

    void set_base(const Pose& base) noexcept;
    void set_effector(const Pose& effector) noexcept;
    void update(const Pose& desired_pose);

    // query
    inline Vecx joint_positions() const noexcept {return _data.joint_positions;}
    Vecx min_joint_positions() const noexcept;
    Vecx max_joint_positions() const noexcept;
    Vecx min_joint_velocities() const noexcept;
    Vecx max_joint_velocities() const noexcept;
    inline Pose end_pose() const noexcept {return _data.end_pose;}
    inline size_t DoF() const noexcept {return _joints.size();}
    inline SerialManipulatorConfig& config() noexcept {return _cfg;}
    inline const SerialManipulatorData& query_data() const noexcept {return _data;}

private:
    void _update_kinematics();
    void _construct(const DHParam& DH_params, const JointLimits& joint_limits, const Vecx& joint_positions);
};

} // namespace kinematics

} // namespace dq1

