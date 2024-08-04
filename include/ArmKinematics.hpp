#include "Pose.hpp"

namespace dq1
{

namespace kinematics
{

class Joint{
protected:
    double pos_; // joint position
    Vec4d limits_; // {max position, max speed, min position, min speed}
    Pose fkm_; // forward kinematics, pose transformation from reference to end
    Vec8d jcb_; // jacobian of reference end pose with respect to joint position
public:

    // Mutable 

    Joint()=delete;
    Joint(const Vec4d& motion_limits, double position=0);
    virtual ~Joint();

    virtual void update(double position)=0;

    // Const
    virtual Pose calculate_fkm(double position)=0;
    virtual Vec8d& calculate_jacobian(double position)=0;

    double position(){ return pos_; }
    const Vec4d& motion_limits() { return limits_; }
    const Pose& fkm() { return fkm_; }
    const Vec8d& jacobian() { return jcb_; } 
};

class RevoluteJoint: public Joint{
protected:
    Vec4d DH_params_; // {Theta, d, a, alpha}
public:

    // Mutable 

    RevoluteJoint()=delete;
    RevoluteJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position=0): Joint(motion_limits, position), DH_params_(DH_parameters){

    }
    virtual ~RevoluteJoint()=default;

    virtual void update(double position){
        fkm_ = calculate_fkm(position);
        jcb_ = calculate_jacobian(position);
    }

    // Const
    virtual Pose calculate_fkm(double position) {
        Rot theta{k_, DH_params_[0]};
        Tslt d{0, 0, DH_params_[1]};
        Tslt a{DH_params_[2], 0, 0};
        Rot alpha{i_, DH_params_[3]};
        return Pose::build_from(theta, d, a, alpha);
    }
    virtual Vec8d& calculate_jacobian(double position);
};

class PrismaticJoint: public Joint{
protected:
    Vec4d DH_params_; // {Theta, d, a, alpha}
public:

    // Mutable 

    PrismaticJoint()=delete;
    PrismaticJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position);
    virtual ~PrismaticJoint()=default;

    virtual void update(double position);

    // Const
    virtual Pose calculate_fkm(double position) {

    }
    virtual Vec8d& calculate_jacobian(double position);
};

}
    
}