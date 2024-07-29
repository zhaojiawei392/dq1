#pragma once
#include "Quaternion.h"


namespace dq1
{

template<typename Scalar_>
class DualQuaternion{
protected:
    Quaternion<Scalar_> primary_; // Primary part
    Quaternion<Scalar_> dual_; // Dual part
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(const Vec8<Scalar_>& vec);
    DualQuaternion(Vec8<Scalar_>&& vec);
    DualQuaternion(const Quaternion<Scalar_>& primary, const Quaternion<Scalar_>& dual);
    DualQuaternion(const Quaternion<Scalar_>& primary, Quaternion<Scalar_>&& dual);
    DualQuaternion(Quaternion<Scalar_>&& primary, const Quaternion<Scalar_>& dual);
    DualQuaternion(Quaternion<Scalar_>&& primary, Quaternion<Scalar_>&& dual);
    DualQuaternion(const Scalar_& h0, const Scalar_& h1=0, const Scalar_& h2=0, const Scalar_& h3=0, const Scalar_& h4=0, const Scalar_& h5=0, const Scalar_& h6=0, const Scalar_& h7=0);
    
    DualQuaternion(const DualQuaternion& dn)=default;
    DualQuaternion(DualQuaternion&& dn)=default;
    virtual ~DualQuaternion()=default;
    DualQuaternion& operator=(const DualQuaternion& dn)=default;
    DualQuaternion& operator=(DualQuaternion&& dn)=default;

    // mutable operators

    DualQuaternion& operator+=(const DualQuaternion& other) noexcept;
    DualQuaternion& operator-=(const DualQuaternion& other) noexcept;
    DualQuaternion& operator*=(const DualQuaternion& other) noexcept;
    DualQuaternion& operator*=(const Scalar_& scalar) noexcept;
    DualQuaternion& operator/=(const Scalar_& scalar) noexcept;
    DualQuaternion& normalize();

    // const operators

    DualQuaternion operator+(const DualQuaternion& other) const noexcept;
    DualQuaternion operator+(DualQuaternion&& other) const noexcept;
    DualQuaternion operator-(const DualQuaternion& other) const noexcept;
    DualQuaternion operator-(DualQuaternion&& other) const noexcept;
    DualQuaternion operator*(const DualQuaternion& other) const noexcept;
    DualQuaternion operator*(DualQuaternion&& other) const noexcept;
    DualQuaternion operator*(const Scalar_& scalar) const noexcept;
    DualQuaternion operator/(const Scalar_& scalar) const noexcept;
    DualQuaternion operator-() const noexcept;
    bool operator==(const DualQuaternion& other) const noexcept;
    bool operator!=(const DualQuaternion& other) const noexcept; 
    operator std::string() const;

    // service functions

    Scalar_ norm() const noexcept;
    DualQuaternion conj() const noexcept;
    DualQuaternion inv() const noexcept;
    DualQuaternion ln() const noexcept;
    DualQuaternion exp() const noexcept;
    DualQuaternion pow(const Scalar_& index) const noexcept;
    DualQuaternion normalized() const noexcept;
    Mat8<Scalar_> hamiplus() const noexcept;
    Mat8<Scalar_> haminus() const noexcept;
    std::string to_string() const;

    DualQuaternion<Scalar_> P() const noexcept;
    DualQuaternion<Scalar_> D() const noexcept;
    Vec6<Scalar_> vec6() const noexcept;
    Vec8<Scalar_> vec8() const noexcept;

    // Friends "const"
    
    template<typename qScalar_>
    friend DualQuaternion<qScalar_> operator*(const qScalar_& scalar, const DualQuaternion<qScalar_>& quaternion);
    template<typename qScalar_>
    friend DualQuaternion<qScalar_> operator*(const qScalar_& scalar, DualQuaternion<qScalar_>&& quaternion);    
    template<typename qScalar_>
    friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<qScalar_>& q);

};

}