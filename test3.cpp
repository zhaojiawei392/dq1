
#include <iostream>
#include "Pose.hpp"


int main()
{   
    using namespace dq1;
    std::cout << "i: " << i_ << "\n";
    std::cout << "j: " << j_ << "\n";
    std::cout << "k: " << k_ << "\n";

    Rot r1{i_, M_PI_2};
    Rot r2{j_, M_PI_2};
    Rot r3{k_, M_PI_2};

    std::cout << "r1: " << r1 << "\n";
    std::cout << "r2: " << r2 << "\n";
    std::cout << "r3: " << r3 << "\n";

    Tslt t1{1,2,3};
    Tslt t2{4,5,6};
    Tslt t3{7,8,9};

    std::cout << "t1: " << t1 << "\n";
    std::cout << "t2: " << t2 << "\n";
    std::cout << "t3: " << t3 << "\n";

    Pose p1 = Pose::build_from(t1, r1);
    Pose p2 = Pose::build_from(t2, r2);
    Pose p3 = Pose::build_from(t3, r3);

    std::cout << "p1: " << p1 << "\n";
    std::cout << "p2: " << p2 << "\n";
    std::cout << "p3: " << p3 << "\n";

    Pose p4 = Pose::build_from(p1, p2, p3, r1, t1, t2, r2, t3);

    std::cout << "p4: " << p4 << "\n";


    double pos_ = M_PI / 6;
    double theta = M_PI_2;
    double alpha = M_PI_2;
    double d = 0.1;
    double a = 0.2;

    // dq1::Quaternion<double> theta_dot{-0.5 * sin( 0.5 * (theta + pos_) ), 0, 0, 0.5 * cos( 0.5 * (theta + pos_) )};
    // dq1::DualQuaternion<double> theta_dot_dq(theta_dot);
    // Tslt t{a,0,d};
    // Rot r_alpha{i_, alpha};

    // std::cout << theta_dot_dq * Pose::build_from(t, r_alpha) << "\n";
    

    // double theta_dot_real = -0.5 * sin( 0.5* (theta + pos_) );
    // double theta_dot_im = 0.5 * cos( 0.5* (theta + pos_) );
    // double alpha_real = cos( 0.5 * alpha );
    // double alpha_im = sin( 0.5 * alpha );
    // dq1::Vec8d vec8{
    //     theta_dot_real * alpha_real,
    //     theta_dot_real * alpha_im,
    //     theta_dot_im * alpha_im,
    //     theta_dot_im * alpha_real,
    //     0.5 * ( -theta_dot_im * d * alpha_real - theta_dot_real * a * alpha_im ),
    //     0.5 * ( -theta_dot_im * d * alpha_im + theta_dot_real * a * alpha_real ),
    //     0.5 * ( theta_dot_im * a * alpha_real + theta_dot_real * d * alpha_im ),
    //     0.5 * (theta_dot_real * d * alpha_real - theta_dot_im * a * alpha_im)
    // };
    
    // std::cout <<vec8<<'\n';

    Quat quat0(1);
    Quat quat1(1);


    Rot r_theta{k_, theta+pos_};
    Tslt td{0, 0, d};
    Tslt ta{a, 0, 0};
    Rot r_alpha{i_, alpha};
    Pose pp0 =  Pose::build_from(std::move(r_theta), std::move(td), std::move(ta), std::move(r_alpha));
    DQ dq1(1);
    DQ dq2(2);
    PureQuaternion<double> pq0;
    UnitPureQuaternion<double> upq0;
    UnitQuaternion<double> uq0;

    uq0 *= uq0;

    Pose pp2;

        double theta_real = cos( 0.5* (theta + pos_) );
        double theta_im = sin( 0.5* (theta + pos_) );
        double alpha_real = cos( 0.5 * alpha );
        double alpha_im = sin( 0.5 * alpha );

        Pose pp1 =  Pose{
            theta_real * alpha_real,
            theta_real * alpha_im,
            theta_im * alpha_im,
            theta_im * alpha_real,
            0.5 * ( -theta_im * d * alpha_real - theta_real * a * alpha_im ),
            0.5 * ( -theta_im * d * alpha_im + theta_real * a * alpha_real ),
            0.5 * ( theta_im * a * alpha_real + theta_real * d * alpha_im ),
            0.5 * (theta_real * d * alpha_real - theta_im * a * alpha_im)
        };
    std::cout << pp0 << "\n" << pp1 << "\n";

    std::cout << "  " <<  (pp0 - pp1) <<  "  ";

}