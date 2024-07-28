#include <iostream>
#include <cmath>
#include <complex>

// Quaternion structure
struct Quaternion {
    double a, b, c, d;
    Quaternion(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}
};

// Function to compute the exponential of a quaternion
Quaternion exp(const Quaternion& q) {
    double a = q.a;
    double b = q.b;
    double c = q.c;
    double d = q.d;
    
    double v_magnitude = std::sqrt(b*b + c*c + d*d);
    double exp_a = std::exp(a);
    
    double cos_v = std::cos(v_magnitude);
    double sin_v = std::sin(v_magnitude);
    
    Quaternion result(
        exp_a * cos_v,
        exp_a * sin_v * b / v_magnitude,
        exp_a * sin_v * c / v_magnitude,
        exp_a * sin_v * d / v_magnitude
    );
    
    return result;
}

void test(Quaternion q)
{
    Quaternion result = exp(q);
    
    std::cout << result.a << " + " << result.b << "i + " 
              << result.c << "j + " << result.d << "k" 
              << std::endl;
}

int main() {
    Quaternion q1(1, 2, 3, 4);
    test(q1);
    Quaternion q2(5, 6, 7, 8);
    test(q2);
    
    return 0;
}