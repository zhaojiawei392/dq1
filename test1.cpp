#include "Quaternion.h"
#include <iostream>

using DQ = dq1::Quaternion<double>;

void test(double a, double b, double c, double d)
{
    DQ q1(a,b,c,d);
    std::cout << "rot_angle: " << q1.rot_angle()/M_PI*180 << " rot_axis: " << q1.rot_axis().to_string();
}

int main()
{
    test(3,0,0,0);
    test(3*cos(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34) );
}