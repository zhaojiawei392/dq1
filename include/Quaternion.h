#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
#define PRINT_PRECISION 18
#define OMIT_THRESHOLD 0.00000000001

template<typename Scalar_, int size>
using Tvec=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Tvec3=Eigen::Matrix<Scalar_, 3, 1>;
using Tvec3d=Eigen::Matrix<double, 3, 1>;
template<typename Scalar_>
using Tvec4=Eigen::Matrix<Scalar_, 4, 1>;
using Tvec4d=Eigen::Matrix<double, 4, 1>;
template<typename Scalar_>
using Tvecx=Eigen::Matrix<Scalar_, -1, 1>;

template<typename Scalar_, int rows_, int cols_>
using Tmat=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Tmat3=Eigen::Matrix<Scalar_, 3, 3>;
using Tmat3d=Eigen::Matrix<double, 3, 3>;
template<typename Scalar_>
using Tmat4=Eigen::Matrix<Scalar_, 4, 4>;
using Tmat4d=Eigen::Matrix<double, 4, 4>;
template<typename Scalar_>
using Tmatx=Eigen::Matrix<Scalar_, -1, -1>;
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
    Tvec4<Scalar_> vals_;
public:
    // Special members

    Quaternion();
    Quaternion(const Quaternion& other)=default;
    Quaternion(Quaternion&& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;
    virtual ~Quaternion()=default;
    
    // Customized constructors

    explicit Quaternion(const Tvecx<Scalar_>& vec);
    explicit Quaternion(Tvecx<Scalar_>&& vec);
    explicit Quaternion(const Scalar_& w, const Tvecx<Scalar_>& vec);
    explicit Quaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
    explicit Quaternion(const Tvecx<Scalar_>& rotation_vec, const Scalar_& rotation_angle, const Scalar_& norm=1);

    Scalar_ w() const noexcept;
    Scalar_ x() const noexcept;
    Scalar_ y() const noexcept;
    Scalar_ z() const noexcept;

    Scalar_ norm() const noexcept;
    Scalar_ rotation_angle() const noexcept;
    Quaternion rotation_axis() const noexcept;
    Tvec3<Scalar_> rotation_vec() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    Quaternion pow(const Scalar_& index) const noexcept;
    Quaternion normalized() const noexcept;

    Tvec3<Scalar_> vec3() const noexcept;
    Tvec4<Scalar_> vec4() const noexcept;

    Tmat4<Scalar_> hamiplus() const noexcept;
    Tmat4<Scalar_> haminus() const noexcept;

    std::string to_string() const;
    operator std::string() const;

    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator+(Quaternion&& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    Quaternion operator-(Quaternion&& other) const noexcept;
    Quaternion operator*(const Quaternion& other) const noexcept;
    Quaternion operator*(Quaternion&& other) const noexcept;
    Quaternion operator*(const Scalar_& scalar) const noexcept;
    Quaternion operator/(const Scalar_& scalar) const noexcept;
    Quaternion operator-() const noexcept;

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Scalar_& scalar) noexcept;
    Quaternion& operator/=(const Scalar_& scalar) noexcept;

    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 

    // Friends
    
    template<typename qScalar_>
    friend Quaternion<qScalar_> operator*(const qScalar_& scalar, const Quaternion<qScalar_>& quaternion);
    template<typename qScalar_>
    friend Quaternion<qScalar_> operator*(const qScalar_& scalar, Quaternion<qScalar_>&& quaternion);    
    template<typename qScalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<qScalar_>& q);
};

template<typename Scalar_>
class PureQuaternion : public Quaternion<Scalar_>
{
public:

    // Special members

    PureQuaternion()=default;
    PureQuaternion(const PureQuaternion& other)=default;
    PureQuaternion(PureQuaternion& other)=default;
    PureQuaternion& operator=(const PureQuaternion& other)=default;
    PureQuaternion& operator=(PureQuaternion&& other)=default;
    virtual ~PureQuaternion()=default;

    // Custom constructors

    explicit PureQuaternion(const Tvecx<Scalar_>& vec);
    explicit PureQuaternion(const Scalar_& x, const Scalar_& y, const Scalar_& z);

    // Special members with base class argument

    explicit PureQuaternion(const Quaternion<Scalar_>& other);
    explicit PureQuaternion(Quaternion<Scalar_>&& other);
    PureQuaternion& operator=(const Quaternion<Scalar_>& other);
    PureQuaternion& operator=(Quaternion<Scalar_>&& other);

    bool operator==(const PureQuaternion& other) const noexcept;
    bool operator!=(const PureQuaternion& other) const noexcept; 

protected:
    void _real_part_should_be_zero(std::string&& calling_fn) noexcept;
};

template<typename Scalar_>
class UnitQuaternion : public Quaternion<Scalar_>
{
public:

    // Special members

    UnitQuaternion()=delete;
    UnitQuaternion(const UnitQuaternion& other)=default;
    UnitQuaternion(UnitQuaternion& other)=default;
    UnitQuaternion& operator=(const UnitQuaternion& other)=default;
    UnitQuaternion& operator=(UnitQuaternion&& other)=default;
    virtual ~UnitQuaternion()=default;

    // Custom constructors

    explicit UnitQuaternion(const Tvecx<Scalar_>& vec);
    explicit UnitQuaternion(Tvecx<Scalar_>&& vec);
    explicit UnitQuaternion(const Scalar_& w, const Tvecx<Scalar_>& vec);
    explicit UnitQuaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
    explicit UnitQuaternion(const Tvecx<Scalar_>& rotation_vec, const Scalar_& rotation_angle);

    // Special members with base class argument

    explicit UnitQuaternion(const Quaternion<Scalar_>& other);
    explicit UnitQuaternion(Quaternion<Scalar_>&& other);
    UnitQuaternion& operator=(const Quaternion<Scalar_>& other);
    UnitQuaternion& operator=(Quaternion<Scalar_>&& other);
    
protected:
    void _norm_should_be_one(std::string&& calling_fn) noexcept;

};

template<typename Scalar_>
class UnitPureQuaternion : virtual public PureQuaternion<Scalar_>, virtual public UnitQuaternion<Scalar_>
{
public:

    // Special members

    UnitPureQuaternion()=delete;
    UnitPureQuaternion(const UnitPureQuaternion& other)=default;
    UnitPureQuaternion(UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(const UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(UnitPureQuaternion&& other)=default;
    virtual ~UnitPureQuaternion()=default;

    // Custom constructors

    explicit UnitPureQuaternion(const Tvecx<Scalar_>& vec);
    explicit UnitPureQuaternion(Tvecx<Scalar_>&& vec);
    explicit UnitPureQuaternion(const Scalar_& w, const Tvecx<Scalar_>& vec);
    explicit UnitPureQuaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
    explicit UnitPureQuaternion(const Tvecx<Scalar_>& rotation_vec, const Scalar_& rotation_angle);

    // Special members with base class argument

    explicit UnitPureQuaternion(const Quaternion<Scalar_>& other);
    explicit UnitPureQuaternion(Quaternion<Scalar_>&& other);
    UnitPureQuaternion& operator=(const Quaternion<Scalar_>& other);
    UnitPureQuaternion& operator=(Quaternion<Scalar_>&& other);

    explicit UnitPureQuaternion(const UnitQuaternion<Scalar_>& other);
    explicit UnitPureQuaternion(UnitQuaternion<Scalar_>&& other);
    UnitPureQuaternion& operator=(const UnitQuaternion<Scalar_>& other);
    UnitPureQuaternion& operator=(UnitQuaternion<Scalar_>&& other);
      
    explicit UnitPureQuaternion(const PureQuaternion<Scalar_>& other);
    explicit UnitPureQuaternion(PureQuaternion<Scalar_>&& other);
    UnitPureQuaternion& operator=(const PureQuaternion<Scalar_>& other);
    UnitPureQuaternion& operator=(PureQuaternion<Scalar_>&& other);
};

// template<typename Scalar_>
// using i_=Quaternion<Scalar_>(0,1,0,0);
// template<typename Scalar_>
// using j_=Quaternion<Scalar_>(0,0,1,0);
// template<typename Scalar_>
// using k_=Quaternion<Scalar_>(0,0,0,1);
}