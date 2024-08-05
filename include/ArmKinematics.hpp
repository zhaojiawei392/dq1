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
    Pose base_;
    Pose end_;
public:

    // Mutable 

    Joint()=delete;
    Joint(const Vec4d& motion_limits, double position=0);
    virtual ~Joint();

    virtual void update(double position)=0;
    virtual void update(double position, const Pose& base)=0;
    virtual void update(double position, Pose&& base)=0;

    // Const
    virtual Pose calculate_fkm(double position)const =0 ;
    virtual Vec8d calculate_jacobian(double position)const =0 ;

    void set_base(const Pose& base) { base_ = base;}
    void set_base(Pose&& base) { base_ = std::move(base);}
    double position() const noexcept{ return pos_; }
    const Vec4d& motion_limits()  const noexcept{ return limits_; }
    const Pose& fkm() const  noexcept{ return fkm_; }
    const Vec8d& jacobian() const noexcept{ return jcb_; } 
    const Pose& base() const  noexcept{ return base_; }
    const Pose& end() const  noexcept{ return end_; }
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
    virtual void update(double position, const Pose& base){
        base_ = base;
        fkm_ = calculate_fkm(position);
        jcb_ = calculate_jacobian(position);
        end_ = base_ * end_;
    }
    virtual void update(double position, Pose&& base){
        base_ = std::move(base);
        fkm_ = calculate_fkm(position);
        jcb_ = calculate_jacobian(position);
        end_ = base_ * end_;
    }

    // Const
    virtual Pose calculate_fkm(double position) {

        double theta_real = cos( 0.5* (DH_params_[0] + pos_) );
        double theta_im = sin( 0.5* (DH_params_[0] + pos_) );
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
            0.5 * (theta_real * d * alpha_real - theta_im * a * alpha_im)
        }; // test past
    }
    virtual Vec8d calculate_jacobian(double position){

        double theta_dot_real = -0.5 * sin( 0.5* (DH_params_[0] + pos_) );
        double theta_dot_im = 0.5 * cos( 0.5* (DH_params_[0] + pos_) );
        double alpha_real = cos( 0.5 * DH_params_[3] );
        double alpha_im = sin( 0.5 * DH_params_[3] );
        double d = DH_params_[1];
        double a = DH_params_[2];
        return Vec8d{
            theta_dot_real * alpha_real,
            theta_dot_real * alpha_im,
            theta_dot_im * alpha_im,
            theta_dot_im * alpha_real,
            0.5 * ( -theta_dot_im * d * alpha_real - theta_dot_real * a * alpha_im ),
            0.5 * ( -theta_dot_im * d * alpha_im + theta_dot_real * a * alpha_real ),
            0.5 * ( theta_dot_im * a * alpha_real + theta_dot_real * d * alpha_im ),
            0.5 * (theta_dot_real * d * alpha_real - theta_dot_im * a * alpha_im)
        }; // test past
    }
};

class PrismaticJoint: public Joint{
protected:
    Vec4d DH_params_; // {Theta, d, a, alpha}
public:

    // Mutable 

    PrismaticJoint()=delete;
    PrismaticJoint(const Vec4d& DH_parameters, const Vec4d& motion_limits, double position);
    virtual ~PrismaticJoint()=default;

    virtual void update(double position){}
    virtual void update(double position, const Pose& base){

    }
    virtual void update(double position, Pose&& base){}

    // Const
    virtual Pose calculate_fkm(double position) {

    }
    virtual Vec8d calculate_jacobian(double position){

    }
};


class SerialManipulatorAbstract{
protected:
    std::vector<Joint> joints_;
public:
    SerialManipulatorAbstract()=delete;
    SerialManipulatorAbstract(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions);
    virtual ~SerialManipulatorAbstract()=default;

    void set_base();
    void link(const SerialManipulatorAbstract& serial);
    void link(SerialManipulatorAbstract&& serial);

    void update(const Pose& desired_pose, double& sampling_time);

    const Vecxd& joint_positions() const noexcept;
    const Vecxd& joint_signals() const noexcept;


};

}
    
}