#include "Quaternion.h"
#include <iostream>

using DQ = dq1::Quaternion<double>;

void test_string(std::string str)
{
    std::cout << str;
}

void test1(DQ dq)
{
std::cout<< dq.to_string() << "\n";
}

void test_multiply(DQ dq1, DQ dq2)
{
    test1(dq1); test1(dq2);
    test1(dq1 * dq2);
}

void test2(DQ dq)
{        
    test1(dq);
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

void test4(const dq1::Tvec3d& vec)
{
    std::cout <<vec;
}

void test4(const dq1::Tvec4d& vec)
{
    std::cout <<vec;}

int main()
{
    // test1(3,0,0,0);
    // test1(3*cos(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34), std::sqrt(3)*sin(13*M_PI/34) );

    test2(DQ(1,2,3,4));
    test2(DQ(5,6,7,8));
    test2(DQ(0,0,0.7854,0));
    test_string(DQ(1));

    std::cout << DQ(1);


    // test3(DQ(1,2,3,4));
    // test1(DQ(cos(2.77438 * 0.5), sin(2.77438 * 0.5) * dq1::Tvec3<double>(0.371390676354103721, 0.557086014531155582, 0.742781352708207443)) * 5.47723);

    // dq1::Tvec3d vec1;
    // vec1 << 1,2,3;
    // dq1::Tvec4d vec2;
    // vec2 << 4,5,6,7;
    
    // DQ dq1(vec1);
    // DQ dq2(vec2);
    // DQ dq3((dq1::Tvec4d()<<4,3,2,1).finished());

    // test4(vec1);
    // test4(vec2);
}