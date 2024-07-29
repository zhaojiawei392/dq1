#include "Quaternion.h"
#include <iostream>

using DQ = dq1::Quaternion<double>;
using pdq = dq1::PureQuaternion<double>;
using udq = dq1::UnitQuaternion<double>;
using updq = dq1::UnitPureQuaternion<double>;

void test_string(std::string str)
{
    std::cout << str;
}

void test1(DQ dq)
{
std::cout<< dq.to_string() << "\n";
}


void test2(DQ dq)
{        
    std::cout << "dq norm: " << dq.norm() << "\n";  
    double exp0 = std::exp(dq.w());
    DQ exp1 = DQ(cos(dq.x()), sin(dq.x()), 0 ,0);
    DQ exp2 = DQ(cos(dq.y()), 0, sin(dq.y()), 0);
    DQ exp3 = DQ(cos(dq.z()), 0, 0, sin(dq.z()));

    DQ dq_exp = exp1*exp2*exp3*exp0;
    DQ dq_exp_ln = dq_exp.ln();
    std::cout << "dq.exp: " << dq.exp().to_string() <<
                 "\ndq_exp: " << dq_exp.to_string()<< 
                 "\ndq_exp_ln: " << dq_exp_ln.to_string() << 
                 "\ndq.exp_ln: " << dq.exp().ln().to_string() << "\n";


    std::cout << "dq.ln: " << dq.ln().to_string() << 
                "\ndq_ln_exp: " << dq.ln().exp().to_string() << "\n";

    std::cout << "dq.pow 2: " << dq.pow(2).to_string() << "\ndq * dq: " << (dq * dq ).to_string() << "\n";
    std::cout << "dq.pow 3: " << dq.pow(3).to_string() << "\ndq * dq * dq: " << (dq * dq * dq).to_string() << "\n";

    std::cout << "dq conj: " << dq.conj().to_string() << "\n";
    std::cout << "dq inv: " << dq.inv().to_string() << "\n";
}

void test3(DQ dq)
{

    std::cout << "dq norm: " << dq.norm() << "\n";  
    std::cout << "dq rot angle: " << dq.rotation_angle() / M_PI * 180 << " rad " << dq.rotation_angle() << " degree\n";
    std::cout << "dq rot axis: " << dq.rotation_axis().to_string() << "\n";
}

void test4(const dq1::Vec3d& vec)
{
    std::cout <<vec;
}

void test4(const dq1::Vec4d& vec)
{
    std::cout <<vec;}

void test5(pdq x)
{

}



int main()
{
    
    pdq q1(3,4,5);
    pdq q2(6,7,8);

    std::cout <<q1*q2.conj() << "\n" << q2*q1.conj();

    DQ q3(dq1::Vec4d({1,2,3,4}), 0.5*M_PI, 2);




}