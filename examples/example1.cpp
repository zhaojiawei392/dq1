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
#include <chrono>
#include "../include/dq1.hpp"

namespace example{

void display()
{
using namespace dq1::Macro;
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
    t2 * 3;
    t2 + t2;
    p2 * 3;
    p2 + 3;
}
void display2()
{
using namespace dq1::Pose2;
const UnitPureQuaternion<float> i_(1,0,0);
const UnitPureQuaternion<float> j_(0,1,0);
const UnitPureQuaternion<float> k_(0,0,1);
    std::cout << std::setprecision(10);
    std::cout << "i: " << i_ << "\n";
    std::cout << "j: " << j_ << "\n";
    std::cout << "k: " << k_ << "\n";
using Quat = Quaternion<float>;
    std::array<float, 4> arr4 = {1,0,0,0};
    Quat q0;
    Quat q1(arr4);
    Quat q2(1.4567823427837527857823782,0,0,0);
    Quat q3(q2);
    std::cout << "q0: " << q0 << "\n";
    std::cout << "q1: " << q1 << "\n";
    std::cout << "q2: " << q2 << "\n";
    std::cout << "q3: " << q3 << "\n";

using Rot = UnitQuaternion<float>;
    Rot r0;
    Rot r1(1,0,0,0);
    Rot r2(arr4);
    Rot r3(q1);
    Rot r4(r3);

    std::cout << "r0: " << r0 << "\n";
    std::cout << "r1: " << r1 << "\n";
    std::cout << "r2: " << r2 << "\n";
    std::cout << "r3: " << r3 << "\n";
    std::cout << "r4: " << r3 << "\n";
using Tran = PureQuaternion<float>;
    std::array<float, 3> arr3 = {1,0,1.4567823427837527857823782};
    Tran t0;
    Tran t1{1,2,3};
    Tran t2{t1};
    Tran t3{arr3};
    Tran t4{t3};

    std::cout << "t0: " << t0 << "\n";
    std::cout << "t1: " << t1 << "\n";
    std::cout << "t2: " << t2 << "\n";
    std::cout << "t3: " << t3 << "\n";
    std::cout << "t4: " << t4 << "\n";

using dq = DualQuaternion<float>;
    std::array<float, 8> arr8 = {1,0,0,1.4567823427837527857823782,0,1.4567823427837527857823782,0,0};
    dq dq0;
    dq dq1(arr8);
    dq dq2(1,1.4567823427837527857823782,0,0);
    dq dq3(dq2);

    std::cout << "dq0: " << dq0 << "\n";
    std::cout << "dq1: " << dq1 << "\n";
    std::cout << "dq2: " << dq2 << "\n";
    std::cout << "dq3: " << dq3 << "\n";
using Po = UnitDualQuaternion<float>;
    Po p0;
    Po p1(r2,t1);
    Po p2(dq2); 
    Po p3(q2, t1);

    std::cout << "p0: " << p0 << "\n";
    std::cout << "p1: " << p1 << "\n";
    std::cout << "p2: " << p2 << "\n";

    auto a1 = r1 * t1;
    Po p5 = Po(r1 * p1);
    Po p6 = Po(p2 * t2);
    r2 * 3;
    t2 * 3;
    t2 + t2;
    p2 * 3;
    std::cout << "r2 * 3: " << r2 * 3 << "\n";
    std::cout << "t2 * 3: " << t2 * 3 << "\n";
    std::cout << "t2 + t2: " << t2 + t2 << "\n";
    std::cout << "p2 * 3: " << p2 * 3 << "\n";
}

void construct1(){
using namespace dq1::Macro;
    Vec4 vec = {1,2,3,4};
    std::vector<Vec4> vec_vec;
    for (size_t i=0; i<10000000; i++){
        vec_vec.emplace_back(Vec4(vec));
    }
    std::vector<Quat> q_vec;
    
    auto start0 = std::chrono::high_resolution_clock::now();

    for (size_t i=0; i<10000000; i++){
        q_vec.emplace_back(Quat(std::move(vec_vec[i])));
    }
    auto end0 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed0 = end0 - start0;

    std::cout << "move Elapsed time: " << elapsed0.count() <<" ms\n";

    auto start1 = std::chrono::high_resolution_clock::now();

    for (size_t i=0; i<10000000; i++){
        q_vec.emplace_back(Quat(vec_vec[i]));
    }
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed1 = end1 - start1;

    std::cout << "reference Elapsed time: " << elapsed1.count() <<" ms\n";

    // reference > move > copy  in arguments passing
}

void construct2(){

using namespace dq1::Macro;
    Vec3 vec = {0,0,1};

    std::vector<Quat> q_vec;
    
    auto start0 = std::chrono::high_resolution_clock::now();

    for (size_t i=0; i<10000000; i++){
        q_vec.emplace_back(Quat(vec, 0.5, 1));
    }
    auto end0 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed0 = end0 - start0;

    std::cout << "reference Elapsed time: " << elapsed0.count() <<" ms\n";

    // reference > move > copy  in arguments passing
}

}

int main()
{   
    example::display2();

}

