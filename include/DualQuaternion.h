#pragma once
#include "Quaternion.h"


namespace dq1
{

template<typename Scalar_>
class DualQuaternion{
protected:
    Quaternion<Scalar_> p_; // Primary part
    Quaternion<Scalar_> d_; // Dual part
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(const DualQuaternion& dn)=default;
    DualQuaternion(DualQuaternion&& dn)=default;

    virtual ~DualQuaternion()=default;
    DualQuaternion& operator=(const DualQuaternion& dn)=default;
    DualQuaternion& operator=(DualQuaternion&& dn)=default;

    // non-const operators

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Scalar_& scalar) noexcept;
    Quaternion& operator/=(const Scalar_& scalar) noexcept;
    Quaternion& normalize();

    // const operators

    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator+(Quaternion&& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    Quaternion operator-(Quaternion&& other) const noexcept;
    Quaternion operator*(const Quaternion& other) const noexcept;
    Quaternion operator*(Quaternion&& other) const noexcept;
    Quaternion operator*(const Scalar_& scalar) const noexcept;
    Quaternion operator/(const Scalar_& scalar) const noexcept;
    Quaternion operator-() const noexcept;
    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 
    operator std::string() const;

    // service functions

    Scalar_ norm() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    Quaternion pow(const Scalar_& index) const noexcept;
    Quaternion normalized() const noexcept;
    Mat8<Scalar_> hamiplus() const noexcept;
    Mat8<Scalar_> haminus() const noexcept;
    std::string to_string() const;

    Quaternion<Scalar_> P() const noexcept;
    Quaternion<Scalar_> D() const noexcept;
    Vec6<Scalar_> vec6() const noexcept;
    Vec8<Scalar_> vec8() const noexcept;


};

}