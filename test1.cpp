#include "Quaternion.hpp"
#include "DualQuaternion.hpp"
#include "Pose.hpp"
#include <iostream>

using namespace dq1;
using q = Quaternion<double>;
using pq = PureQuaternion<double>;
using uq = UnitQuaternion<double>;
using upq = UnitPureQuaternion<double>;
using dq = DualQuaternion<double>;
using rot = Rotation<double>;
using trans = Translation<double>;
using uaxis = UnitAxis<double>;
using pose = Pose<double>;

void print(const q& qx)
{
    std::cout << qx << "\n";
}

void test_constuctor()
{
    Vec4d vec4({1,2,3,4});
    Vec3d vec3({5,6,7});
    q q0;
    q q1(vec4);
    q q2(Vec4d({1,2,3,4}));
    q q3(q2);
    q q4(q{});
    q q5(4,vec3);
    q q6(vec3, M_PI_2, 2);
    q q7(7,8,9,10);
    print(q0);print(q1);print(q2);print(q3);print(q4);print(q5);print(q6);
    pq pq0{};
    pq pq1(q0);
    pq pq2(q{});
    pq pq3(vec3);
    pq pq4(Vec3d({5,6,7}));
    pq pq5(pq0);
    pq pq6(pq{});
    pq pq7{1,2,3};
    print(pq0);print(pq1);print(pq2);print(pq3);print(pq4);print(pq5);print(pq6);
    uq uq1(q0);
    uq uq2(q{});
    uq uq3(2,vec3);
    uq uq4(Vec3d({5,6,7}));
    uq uq5(pq0);
    uq uq6(pq{});
    uq uq7{1,2,3};
    print(pq0);print(pq1);print(pq2);print(pq3);print(pq4);print(pq5);print(pq6);
    upq upq1(q0);
    upq upq2(q{});
    upq upq3(vec3);
    upq upq4(Vec3d({5,6,7}));
    upq upq5(pq0);
    upq upq6(pq{});
    upq upq7{1,2,3};
    print(pq0);print(pq1);print(pq2);print(pq3);print(pq4);print(pq5);print(pq6);
}



int main()
{
    test_constuctor();
    Vec4d v1({1,2,3,4});
    Vec4d v2({4,3,2,1});
    std::cout <<v1.dot(v2);

    q q0;
    q q1(v1);
    q q2(v2);

    dq dq0;
    dq dq1;
    dq dq2(Vec8d::Zero());
    dq dq3(dq2);
    dq dq4(q1, q2);
 
    std::cout <<dq0 << dq1<<dq2<<dq3<<dq4;

    // uaxis axis0{0,1,0};
    // rot r0{axis0.vec3(), 60*M_PI/180};
    // trans t1{1,2,3};
    // pose p0 = pose::build_from(r0, t1);
    // pose p1 = pose::build_from(p0, r0, t1, p0, t1, r0);

    // std::cout <<axis0 << "\n"
    //             <<r0 << "\n"
    //             <<t1 << "\n"
    //             <<p0 << "\n"
    //             <<p1 << "\n";


    return 0;
}