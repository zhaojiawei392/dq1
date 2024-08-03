
#include <iostream>
#include "Pose.hpp"


int main()
{   
    std::cout << "i: " << i_ << "\n";
    std::cout << "j: " << j_ << "\n";
    std::cout << "k: " << k_ << "\n";

    Rot r1{i_, M_PI_2};
    Rot r2{j_, M_PI_2};
    Rot r3{k_, M_PI_2};

    std::cout << "r1: " << r1 << "\n";
    std::cout << "r2: " << r2 << "\n";
    std::cout << "r3: " << r3 << "\n";

    Trans t1{1,2,3};
    Trans t2{4,5,6};
    Trans t3{7,8,9};

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
    



}