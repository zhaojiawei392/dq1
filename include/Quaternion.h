#pragma once
#include "Macro.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{

template<typename Scalar_>
class Quaternion;
template<typename Scalar_>
class PureQuaternion;
template<typename Scalar_>
class unitQuaternion;
template<typename Scalar_>
class UnitPureQuaternion;

template<typename Scalar_>
class Quaternion{
protected:
    Vec4<Scalar_> vals_;
public:

    // Constructors and Assignments

             Quaternion();
    explicit Quaternion(const Vecx<Scalar_>& vec);
    explicit Quaternion(Vecx<Scalar_>&& vec);
    explicit Quaternion(const Scalar_& w, const Vecx<Scalar_>& vec);
    explicit Quaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
    explicit Quaternion(const Vecx<Scalar_>& rotation_vec, const Scalar_& rotation_angle, const Scalar_& norm=1);

    virtual ~Quaternion()=default;
             Quaternion(const Quaternion& other)=default;
             Quaternion(Quaternion&& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;

    // mutable operators

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

    // service functions const

    Scalar_ norm() const noexcept;
    Scalar_ rotation_angle() const noexcept;
    Quaternion rotation_axis() const noexcept;
    Vec3<Scalar_> rotation_vec() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    Quaternion pow(const Scalar_& index) const noexcept;
    Quaternion normalized() const noexcept;
    Mat4<Scalar_> hamiplus() const noexcept;
    Mat4<Scalar_> haminus() const noexcept;
    std::string to_string() const;

    Scalar_ w() const noexcept;
    Scalar_ x() const noexcept;
    Scalar_ y() const noexcept;
    Scalar_ z() const noexcept;
    Vec3<Scalar_> vec3() const noexcept;
    Vec4<Scalar_> vec4() const noexcept;

    // Friends "const"
    
    template<typename qScalar_>
    friend Quaternion<qScalar_> operator*(const qScalar_& scalar, const Quaternion<qScalar_>& quaternion);
    template<typename qScalar_>
    friend Quaternion<qScalar_> operator*(const qScalar_& scalar, Quaternion<qScalar_>&& quaternion);    
    template<typename qScalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<qScalar_>& q);
    template<typename qScalar_>
    friend void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<qScalar_>& quaternion) noexcept;
    template<typename qScalar_>
    friend void _norm_should_be_one(std::string&& calling_fn, Quaternion<qScalar_>& quaternion) noexcept;
};

template<typename Scalar_>
class PureQuaternion : public Quaternion<Scalar_>
{
public:

    // Constructors and Assignments

           explicit PureQuaternion(const Vecx<Scalar_>& vec);
           explicit PureQuaternion(const Scalar_& x, const Scalar_& y, const Scalar_& z);
           explicit PureQuaternion(const Quaternion<Scalar_>& other);
           explicit PureQuaternion(Quaternion<Scalar_>&& other);
    PureQuaternion& operator=(const Quaternion<Scalar_>& other);
    PureQuaternion& operator=(Quaternion<Scalar_>&& other);

                    PureQuaternion()=default;
                    PureQuaternion(const PureQuaternion& other)=default;
                    PureQuaternion(PureQuaternion& other)=default;
            virtual ~PureQuaternion()=default;
    PureQuaternion& operator=(const PureQuaternion& other)=default;
    PureQuaternion& operator=(PureQuaternion&& other)=default;

    // operators const

    bool operator==(const PureQuaternion& other) const noexcept;
    bool operator!=(const PureQuaternion& other) const noexcept; 

};

template<typename Scalar_>
class UnitQuaternion : public Quaternion<Scalar_>
{
public:

    // Constructors and Assignments

           explicit UnitQuaternion(const Vecx<Scalar_>& vec);
           explicit UnitQuaternion(Vecx<Scalar_>&& vec);
           explicit UnitQuaternion(const Scalar_& w, const Vecx<Scalar_>& vec);
           explicit UnitQuaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
           explicit UnitQuaternion(const Vecx<Scalar_>& rotation_vec, const Scalar_& rotation_angle);
           explicit UnitQuaternion(const Quaternion<Scalar_>& other);
           explicit UnitQuaternion(Quaternion<Scalar_>&& other);
    UnitQuaternion& operator=(const Quaternion<Scalar_>& other);
    UnitQuaternion& operator=(Quaternion<Scalar_>&& other);

                    UnitQuaternion()=delete;
                    UnitQuaternion(const UnitQuaternion& other)=default;
                    UnitQuaternion(UnitQuaternion& other)=default;
    UnitQuaternion& operator=(const UnitQuaternion& other)=default;
    UnitQuaternion& operator=(UnitQuaternion&& other)=default;
            virtual ~UnitQuaternion()=default;

};

template<typename Scalar_>
class UnitPureQuaternion : public Quaternion<Scalar_>
{
public:

    // Constructors and Assignments

               explicit UnitPureQuaternion(const Vecx<Scalar_>& vec);
               explicit UnitPureQuaternion(const Scalar_& x, const Scalar_& y, const Scalar_& z);
               explicit UnitPureQuaternion(const Quaternion<Scalar_>& other);
               explicit UnitPureQuaternion(Quaternion<Scalar_>&& other);
    UnitPureQuaternion& operator=(const Quaternion<Scalar_>& other);
    UnitPureQuaternion& operator=(Quaternion<Scalar_>&& other);

                        UnitPureQuaternion()=delete;
                        UnitPureQuaternion(const UnitPureQuaternion& other)=default;
                        UnitPureQuaternion(UnitPureQuaternion& other)=default;
                virtual ~UnitPureQuaternion()=default;
    UnitPureQuaternion& operator=(const UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(UnitPureQuaternion&& other)=default;

};
const Quaternion<double> i_(0,1,0,0);
const Quaternion<double> j_(0,1,0,0);
const Quaternion<double> k_(0,1,0,0);

}
