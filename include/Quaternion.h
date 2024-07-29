#pragma once
#include "Macro.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{

template<typename qScalar_>
class Quaternion;
template<typename qScalar_>
class PureQuaternion;
template<typename qScalar_>
class unitQuaternion;
template<typename qScalar_>
class UnitPureQuaternion;

template<typename qScalar_>
class Quaternion{
protected:
    Vec4<qScalar_> vals_;
public:

    // Constructors and Assignments

             Quaternion();
    explicit Quaternion(const Vecx<qScalar_>& vec);
    explicit Quaternion(Vecx<qScalar_>&& vec);
    explicit Quaternion(const qScalar_& w, const Vecx<qScalar_>& vec);
    explicit Quaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
    explicit Quaternion(const Vecx<qScalar_>& rotation_vec, const qScalar_& rotation_angle, const qScalar_& norm=1);

    virtual ~Quaternion()=default;
             Quaternion(const Quaternion& other)=default;
             Quaternion(Quaternion&& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;

    // mutable operators

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    template<typename Scalar_>
    Quaternion& operator*=(const Scalar_& scalar) noexcept;
    template<typename Scalar_>
    Quaternion& operator/=(const Scalar_& scalar) noexcept;
    Quaternion& normalize();

    // const operators

    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator+(Quaternion&& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    Quaternion operator-(Quaternion&& other) const noexcept;
    Quaternion operator*(const Quaternion& other) const noexcept;
    Quaternion operator*(Quaternion&& other) const noexcept;
    template<typename Scalar_>
    Quaternion operator*(const Scalar_& scalar) const noexcept;
    template<typename Scalar_>
    Quaternion operator/(const Scalar_& scalar) const noexcept;
    Quaternion operator-() const noexcept;
    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 
    operator std::string() const;

    // service functions const

    qScalar_ norm() const noexcept;
    qScalar_ rotation_angle() const noexcept;
    Quaternion rotation_axis() const noexcept;
    Vec3<qScalar_> rotation_vec() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    template<typename Scalar_>
    Quaternion pow(const Scalar_& index) const noexcept;//??? cannot generate instantiations automatically
    Quaternion normalized() const noexcept;
    Mat4<qScalar_> hamiplus() const noexcept;
    Mat4<qScalar_> haminus() const noexcept;
    std::string to_string() const;

    qScalar_ w() const noexcept;
    qScalar_ x() const noexcept;
    qScalar_ y() const noexcept;
    qScalar_ z() const noexcept;
    Vec3<qScalar_> vec3() const noexcept;
    Vec4<qScalar_> vec4() const noexcept;

    // Friends "const"
    
    template<typename Scalar_>
    friend Quaternion<Scalar_> operator*(const Scalar_& scalar, const Quaternion<Scalar_>& quaternion);
    template<typename Scalar_>
    friend Quaternion<Scalar_> operator*(const Scalar_& scalar, Quaternion<Scalar_>&& quaternion);    
    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& q);
    template<typename Scalar_>
    friend void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept;
    template<typename Scalar_>
    friend void _norm_should_be_one(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept;
};

template<typename qScalar_>
class PureQuaternion : public Quaternion<qScalar_>
{
public:

    // Constructors and Assignments

           explicit PureQuaternion(const Vecx<qScalar_>& vec);
           explicit PureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z);
           explicit PureQuaternion(const Quaternion<qScalar_>& other);
           explicit PureQuaternion(Quaternion<qScalar_>&& other);
    PureQuaternion& operator=(const Quaternion<qScalar_>& other);
    PureQuaternion& operator=(Quaternion<qScalar_>&& other);

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

template<typename qScalar_>
class UnitQuaternion : public Quaternion<qScalar_>
{
public:

    // Constructors and Assignments

           explicit UnitQuaternion(const Vecx<qScalar_>& vec);
           explicit UnitQuaternion(Vecx<qScalar_>&& vec);
           explicit UnitQuaternion(const qScalar_& w, const Vecx<qScalar_>& vec);
           explicit UnitQuaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
           explicit UnitQuaternion(const Vecx<qScalar_>& rotation_vec, const qScalar_& rotation_angle);
           explicit UnitQuaternion(const Quaternion<qScalar_>& other);
           explicit UnitQuaternion(Quaternion<qScalar_>&& other);
    UnitQuaternion& operator=(const Quaternion<qScalar_>& other);
    UnitQuaternion& operator=(Quaternion<qScalar_>&& other);

                    UnitQuaternion()=delete;
                    UnitQuaternion(const UnitQuaternion& other)=default;
                    UnitQuaternion(UnitQuaternion& other)=default;
    UnitQuaternion& operator=(const UnitQuaternion& other)=default;
    UnitQuaternion& operator=(UnitQuaternion&& other)=default;
            virtual ~UnitQuaternion()=default;

};

template<typename qScalar_>
class UnitPureQuaternion : public Quaternion<qScalar_>
{
public:

    // Constructors and Assignments

               explicit UnitPureQuaternion(const Vecx<qScalar_>& vec);
               explicit UnitPureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z);
               explicit UnitPureQuaternion(const Quaternion<qScalar_>& other);
               explicit UnitPureQuaternion(Quaternion<qScalar_>&& other);
    UnitPureQuaternion& operator=(const Quaternion<qScalar_>& other);
    UnitPureQuaternion& operator=(Quaternion<qScalar_>&& other);

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
