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
 *     \file examples/example1.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */


#include <iostream>
#include "../include/dq1.hpp"

namespace example{

void display()
{
using namespace dq1;
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

    auto a1 = r1 * t1;
    Pose p5 = r1 * p1;
    Pose p6 = p2 * t2;
    r2 * 3;
    r2 + 3;
    t2 * 3;
    t2 + 3;
    p2 * 3;
    p2 + 3;

}

void test2(){

}
}

int main()
{   

    example::display();


}

