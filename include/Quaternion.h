#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
#define PRINT_PRECISION 18
#define OMIT_THRESHOLD 0.0000000000001

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
    // Destructor, always identified as virtual, otherwise the derived class resources may not be released correctly
    virtual ~Quaternion()=default;
    
    // Customized constructors
    explicit Quaternion(const Tvecx<Scalar_>& vec);
    explicit Quaternion(Tvecx<Scalar_>&& vec);
    explicit Quaternion(const Scalar_& w, const Tvecx<Scalar_>& vec);
    explicit Quaternion(const Scalar_& w, const Scalar_& x=0, const Scalar_& y=0, const Scalar_& z=0);
    explicit Quaternion(const Scalar_& norm, const Scalar_& rotation_angle, const Tvecx<Scalar_>& rotation_vec);

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
    virtual Quaternion operator*(const Scalar_& scalar) const noexcept;
    virtual Quaternion operator/(const Scalar_& scalar) const noexcept;
    virtual Quaternion operator-() const noexcept;

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    virtual Quaternion& operator*=(const Scalar_& scalar) noexcept;
    virtual Quaternion& operator/=(const Scalar_& scalar) noexcept;

    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 

    // Friends
    // friend Quaternion<Scalar_> operator*(const Scalar_& scalar, const Quaternion<Scalar_>& quaternion);
    // friend Quaternion<Scalar_> operator*(const Scalar_& scalar, Quaternion<Scalar_>&& quaternion);    
    template<typename osScalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<osScalar_>& q);
};

template<typename Scalar_>
class PureQuaternion : public Quaternion<Scalar_>
{
public:

    // Special members
    PureQuaternion();
    PureQuaternion(const PureQuaternion& other)=default;
    PureQuaternion(PureQuaternion& other)=default;
    PureQuaternion& operator=(const PureQuaternion& other)=default;
    PureQuaternion& operator=(PureQuaternion&& other)=default;
    // Destructor, always identified as virtual, otherwise the derived class resources may not be released correctly
    virtual ~PureQuaternion()=default;

    // Special members with base class argument
    explicit PureQuaternion(const Quaternion<Scalar_>& other);
    explicit PureQuaternion(Quaternion<Scalar_>&& other);
    PureQuaternion& operator=(const Quaternion<Scalar_>& other);
    PureQuaternion& operator=(Quaternion<Scalar_>&& other);
    
    // Customized constructors
    explicit PureQuaternion(const Tvecx<Scalar_>& vec);
    explicit PureQuaternion(const Scalar_& x, const Scalar_& y, const Scalar_& z);

    PureQuaternion operator+(const PureQuaternion& other) const noexcept; //overload
    PureQuaternion operator+(PureQuaternion&& other) const noexcept; //overload
    PureQuaternion operator-(const PureQuaternion& other) const noexcept; //overload
    PureQuaternion operator-(PureQuaternion&& other) const noexcept; //overload
    PureQuaternion operator*(const PureQuaternion& other) const noexcept; //overload
    PureQuaternion operator*(PureQuaternion&& other) const noexcept; //overload
    virtual PureQuaternion operator*(const Scalar_& scalar) const noexcept override;
    virtual PureQuaternion operator/(const Scalar_& scalar) const noexcept override;
    virtual PureQuaternion operator-() const noexcept override;

    PureQuaternion& operator+=(const PureQuaternion& other) noexcept; //overload
    PureQuaternion& operator-=(const PureQuaternion& other) noexcept; //overload
    PureQuaternion& operator*=(const PureQuaternion& other) noexcept; //overload
    virtual PureQuaternion& operator*=(const Scalar_& scalar) noexcept override;
    virtual PureQuaternion& operator/=(const Scalar_& scalar) noexcept override;

    bool operator==(const PureQuaternion& other) const noexcept;
    bool operator!=(const PureQuaternion& other) const noexcept; 

    // // Friends
    // friend Quaternion operator*(const Scalar_& scalar, const PureQuaternion<Scalar_>& pure_quaternion);
    // friend Quaternion operator*(const Scalar_& scalar, PureQuaternion<Scalar_>&& pure_quaternion);

};

template<typename Scalar_>
class UnitQuaternion : public Quaternion<Scalar_>
{

};

template<typename Scalar_>
class UnitPureQuaternion : virtual public PureQuaternion<Scalar_>, virtual public UnitQuaternion<Scalar_>
{
    
};

// template<typename Scalar_>
// using i_=Quaternion<Scalar_>(0,1,0,0);
// template<typename Scalar_>
// using j_=Quaternion<Scalar_>(0,0,1,0);
// template<typename Scalar_>
// using k_=Quaternion<Scalar_>(0,0,0,1);
}