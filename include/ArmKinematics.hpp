#pragma once
#include "Pose.hpp"
#include <memory>

namespace dq1
{

namespace kinematics
{

class Joint{
protected:
    double pos_; // joint position
    Vec4d limits_; // {max position, max speed, min position, min speed}
    Pose fkm_{1}; // forward kinematics, pose transformation from reference to end    
    Pose base_{1};
    Pose end_{1};
    Vec8d jcb_; // jacobian of reference end pose with respect to joint position

public:

    // Mutable 

    Joint()=delete;
    Joint(const Vec4d& motion_limits, double position=0): limits_(motion_limits), pos_(position) {
         //????
    }
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
    Vec4d motion_limits()  const noexcept{ return limits_; }
    Pose fkm() const  noexcept{ return fkm_; }
    Vec8d jacobian() const noexcept{ return jcb_; } 
    Pose base() const  noexcept{ return base_; }
    Pose end() const  noexcept{ return end_; }
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
    virtual Pose calculate_fkm(double position) const override{

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
    virtual Vec8d calculate_jacobian(double position) const override{

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
    virtual Pose calculate_fkm(double position) const override {

    }
    virtual Vec8d calculate_jacobian(double position) const override{

    }
};


class SerialManipulator{
protected:
    std::vector<std::unique_ptr<Joint>> joints_;
public:
    SerialManipulator()=delete;
    SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions){
        if (DH_params.rows() != 5 || DH_params.cols() == 0 ){
            throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) complains about DH_params' invalid size (" + std::to_string(DH_params.rows()) + ", " + std::to_string(DH_params.cols()) + "), which should be (5, DOF), where DOF is bigger than 0.\n");
        }
        int DOF = DH_params.cols();
        if (joint_limits.rows() != 4 || joint_limits.cols() != DOF ){
            throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) complains about joint_limits' invalid size (" + std::to_string(joint_limits.rows()) + ", " + std::to_string(joint_limits.cols()) + "), which should be (5, " + std::to_string(DOF) + ").\n");
        }
        if (joint_positions.size() == DOF ){
            throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) complains about joint_positions' invalid size (" + std::to_string(joint_limits.rows()) + ", " + "), which should be (DOF, " + std::to_string(DOF) + ").\n");
        }

        for (int i=0; i<DOF; ++i){
            if (DH_params[4, i] == 0){
                joints_.push_back(std::make_unique<RevoluteJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
            } else if (DH_params[4, i] == 1){
                joints_.push_back(std::make_unique<PrismaticJoint>(DH_params.block<4,1>(0,i), joint_limits.block<4,1>(0,i), joint_positions[i]));
            } else {
                throw std::runtime_error("SerialManipulator(const Matxd& DH_params, const Matxd& joint_limits, const Vecxd& joint_positions) complains about DH_params' 5-th row member DH_params[4, " + std::to_string(i) + "] = " + std::to_string(DH_params[4,i]) + ", this row should consist of 0: Revolute joint or 1: Prismatic joint.\n");
            }
        }


        std::cout << "A Kinematics::SerialManipulator constructed!\n" ;
    }
    virtual ~SerialManipulator()=default;

    void set_reference(const Pose& reference) noexcept{
        joints_[0]->set_base(reference);
    }    
    void set_reference(Pose&& reference) noexcept{
        joints_[0]->set_base(std::move(reference));
    }
    void link(SerialManipulator&& other){
        for (const auto& joint : other.joints_){
            joints_.push_back(std::move(joint));
        }
    }

    void update(const Pose& desired_pose){

    }

    Pose calculate_fkm(const Vecxd& joint_positions) const{
        if (joint_positions.size() != joints_.size()){
            throw std::range_error("SerialManipulator::calculate_fkm(const Vecxd& joint_positions) complains about joint_positions' size " + std::to_string(joint_positions.size()) + ", which should be " + std::to_string(joints_.size()) + ".\n" );
        }
        Pose res{joints_[0]->base()};
        for (int i=0; i<joints_.size(); ++i){
            res *= joints_[i]->calculate_fkm(joint_positions[i]); // *= operators may introduce floating error cause there are no floating check for such functions
        }
        return Pose(std::move(res));
    }

    Vecxd joint_positions() const noexcept{
        Vecxd res;
        for (int i=0; i<joints_.size(); ++i){
            res << joints_[i]->position();
        }
        return std::move(res);
    }
    // Vecxd joint_signals() const noexcept;
private:
    Matxd _calculate_quadratic_matrix(const Pose& xd)const{
        const Matxd& J_r_rd = xd.rotation().haminus() * C4_ * 
        const MatrixXd& J_r_rd = haminus4(rotation(xd)) * C4() * kc_.J_r;
        const MatrixXd& Ht = (kc_.J_t.transpose() * kc_.J_t) * 2;
        const MatrixXd& Hr = (J_r_rd.transpose() * J_r_rd) * 2;
        const MatrixXd& Hj = cfg_.damping_diagonal.asDiagonal();
        return cfg_.alpha * Ht + (1-cfg_.alpha) * Hr + Hj;
    }
    Matxd _calculate_quadratic_vector(const Pose& xd)const{
        const MatrixXd& J_r_rd = haminus4(rotation(xd)) * C4() * kc_.J_r;
        const VectorXd& vec_et = vec4(translation(kc_.x) - translation(xd));
        const VectorXd& vec_er = __closest_invariant_rotation_error(kc_.x, xd);
        const VectorXd& ct = 2 * vec_et.transpose() * cfg_.n * kc_.J_t;
        const VectorXd& cr = 2 * vec_er.transpose() * cfg_.n * J_r_rd;
        return cfg_.alpha * ct + (1 - cfg_.alpha) * cr;
    }
    Matxd _calculate_pose_jacobian() const{
        
    }
    Matxd _calculate_rotation_jacobian() const{

    }
    Matxd _calculate_translation_jacobian() const{

    }



};

}
    
}