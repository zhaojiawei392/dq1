
#include <iostream>
#include "../include/dq1.hpp"

using namespace dq1;

void test1()
{

    std::cout << "i: " << i_ << "\n";
    std::cout << "j: " << j_ << "\n";
    std::cout << "k: " << k_ << "\n";

    Rot r0;
    Rot r1{i_, M_PI_2};
    Rot r2{j_, M_PI_2};
    Rot r3{k_, M_PI_2};

    std::cout << "r0: " << r0 << "\n";
    std::cout << "r1: " << r1 << "\n";
    std::cout << "r2: " << r2 << "\n";
    std::cout << "r3: " << r3 << "\n";

    Tran t0;
    Tran t1{1,2,3};
    Tran t2{4,5,6};
    Tran t3{7,8,9};

    std::cout << "t0: " << t0 << "\n";
    std::cout << "t1: " << t1 << "\n";
    std::cout << "t2: " << t2 << "\n";
    std::cout << "t3: " << t3 << "\n";

    Pose p0;
    Pose p1 = Pose::build_from(t1, r1);
    Pose p2 = Pose::build_from(t2, r2);
    Pose p3 = Pose::build_from(t3, r3);

    std::cout << "p0: " << p0 << "\n";
    std::cout << "p1: " << p1 << "\n";
    std::cout << "p2: " << p2 << "\n";
    std::cout << "p3: " << p3 << "\n";

    Pose p4 = Pose::build_from(p1, p2, p3, r1, t1, t2, r2, t3);

    std::cout << "p4: " << p4 << "\n";
}

void test2(){
    Matd<5, 6> dh; 
    dh << M_PI, M_PI_2, -M_PI_2, 0, M_PI, 0,
            0.345, 0, 0, 0.255, 0, 0.255,
            0, 0.25, 0.01, 0, 0, 0,
            M_PI_2, 0, -M_PI_2, M_PI_2, M_PI_2, 0,
            0,0,0,0,0,0;
    
    
    Matd<4, 6> limits;
    limits << -150, -10, -10, -260, -90, -350,
             150, 110, 150, 260, 100, 350,
             -33, -32.1, -59.40, -60.50, -60.50, -90.80,
             33, 32.1, 59.40, 60.50, 60.50, 90.80;

    Vec6d pos;
    pos << 0,0,0,0,0,0;

    limits = limits * M_PI / 180;
    kinematics::SerialManipulator sm(dh, limits, pos);

    Rot r1(1);
    Tran t1{-0.1,-0.2,-0.3};
    Pose desired_pose = Pose::build_from(sm.end_pose(), r1, t1);

    int i{0};
    while (i<1000000000){

        sm.update(desired_pose);
        std::cout << "joint positions: " << sm.joint_positions().transpose() <<"\n";
        std::cout << "rotation error: " << sm.end_pose().rotation().conj() * desired_pose.rotation() <<"\n";
        std::cout << "translation error: " << sm.end_pose().translation() - desired_pose.translation() <<"\n";
        ++i;
    }
}

int main()
{   

    test2();


}