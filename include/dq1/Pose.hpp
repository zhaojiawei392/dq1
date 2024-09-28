/** 
 *     This file is part of dq1.
 *  
 *     dq1 is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dq1 is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dq1. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file include/dq1/Pose.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>

namespace dq1
{

using scalar_t = float;
using size_t = int;


template <typename T>
T square(const T& x) {
    return x * x;
}

constexpr int PRINT_PRECISION = 18;
constexpr double FLOAT_OMIT_THRESHOLD = 0.00000001;
constexpr double FLOAT_ERROR_THRESHOLD = 0.0001;
constexpr bool VERY_VERBOSE = false;

// Template type alias
template<typename Scalar_, int size>
using Vec_t=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Vec3_t=Eigen::Matrix<Scalar_, 3, 1>;
template<typename Scalar_>
using Vec4_t=Eigen::Matrix<Scalar_, 4, 1>;
template<typename Scalar_>
using Vec6_t=Eigen::Matrix<Scalar_, 6, 1>;
template<typename Scalar_>
using Vec8_t=Eigen::Matrix<Scalar_, 8, 1>;
template<typename Scalar_>
using Vecx_t=Eigen::Matrix<Scalar_, -1, 1>;

template<typename Scalar_, int size>
using RowVec_t=Eigen::Matrix<Scalar_, 1, size>;
template<typename Scalar_>
using RowVec3_t=Eigen::Matrix<Scalar_, 1, 3>;
template<typename Scalar_>
using RowVec4_t=Eigen::Matrix<Scalar_, 1, 4>;
template<typename Scalar_>
using RowVec6_t=Eigen::Matrix<Scalar_, 1, 6>;
template<typename Scalar_>
using RowVec8_t=Eigen::Matrix<Scalar_, 1, 8>;
template<typename Scalar_>
using RowVecx_t=Eigen::Matrix<Scalar_, 1, -1>;

template<typename Scalar_, int rows_, int cols_>
using Mat_t=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Mat3_t=Eigen::Matrix<Scalar_, 3, 3>;
template<typename Scalar_>
using Mat4_t=Eigen::Matrix<Scalar_, 4, 4>;
template<typename Scalar_>
using Mat6_t=Eigen::Matrix<Scalar_, 6, 6>;
template<typename Scalar_>
using Mat8_t=Eigen::Matrix<Scalar_, 8, 8>;
template<typename Scalar_>
using Matx_t=Eigen::Matrix<Scalar_, -1, -1>;


// Instance type alias
using Vec3=Vec3_t<scalar_t>;
using Vec4=Vec4_t<scalar_t>;
using Vec6=Vec6_t<scalar_t>;
using Vec8=Vec8_t<scalar_t>;
using Vecx=Vecx_t<scalar_t>;
using Vecxf=Vecx_t<float>;
using Vecxd=Vecx_t<double>;

using RowVec3=RowVec3_t<scalar_t>;
using RowVec4=RowVec4_t<scalar_t>;
using RowVec6=RowVec6_t<scalar_t>;
using RowVec8=RowVec8_t<scalar_t>;
using RowVecx=RowVecx_t<scalar_t>;
using RowVecxf=RowVecx_t<float>;
using RowVecxd=RowVecx_t<double>;

using Mat3=Mat3_t<scalar_t>;
using Mat4=Mat4_t<scalar_t>;
using Mat6=Mat6_t<scalar_t>;
using Mat8=Mat8_t<scalar_t>;
using Matx=Matx_t<scalar_t>;
using Matxf=Matx_t<float>;
using Matxd=Matx_t<double>;

template<typename qScalar_>
class Quaternion;
template<typename qScalar_>
class PureQuaternion;
template<typename qScalar_>
class UnitQuaternion;
template<typename qScalar_>
class UnitPureQuaternion;
template<typename qScalar_>
class DualQuaternion;
template<typename qScalar_>
class UnitDualQuaternion;

template<typename qScalar_>
class Quaternion{
protected:
    Vec4_t<qScalar_> vals_;
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const qScalar_ w, const Vec3_t<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const UnitPureQuaternion<qScalar_> rotation_axis, const qScalar_ rotation_angle, const qScalar_ norm=1);
public:
    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             Quaternion();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Quaternion(Vec4_t<qScalar_> vec4);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const qScalar_ w, const qScalar_ x=0, const qScalar_ y=0, const qScalar_ z=0);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const Vec3_t<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle, const qScalar_ norm=1);

    // mutable operators

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion&>
    operator*=(const Scalar_ scalar) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion&>
    operator/=(const Scalar_ scalar) noexcept;
    Quaternion<qScalar_>& normalize();

    // const operators

    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
    operator/(const Scalar_ scalar) const noexcept;
    Quaternion operator-() const noexcept;
    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 
    operator std::string() const;

    // service functions const

    qScalar_ norm() const noexcept;
    qScalar_ rotation_angle() const noexcept;
    UnitPureQuaternion<qScalar_> rotation_axis() const noexcept;
    Vec3_t<qScalar_> rotation_axis_vec3() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
             pow(const Scalar_ index) const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    UnitQuaternion<qScalar_> normalized() const noexcept;
    Mat4_t<qScalar_> hamiplus() const noexcept;
    Mat4_t<qScalar_> haminus() const noexcept;

    // "Get" const

    std::string to_string() const;
    qScalar_ w() const noexcept;
    qScalar_ x() const noexcept;
    qScalar_ y() const noexcept;
    qScalar_ z() const noexcept;
    Vec3_t<qScalar_> vec3() const noexcept;
    Vec4_t<qScalar_> vec4() const noexcept;

    // Friends "const"
    
    template<typename Scalar_>
    friend void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion);
    template<typename Scalar_>
    friend void _norm_should_be_one(std::string&& calling_fn, Quaternion<Scalar_>& quaternion);

    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& quaternion);

    template<typename fScalar_>
    friend Quaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion1, const Quaternion<fScalar_>& quaternion2) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
    operator*(const Quaternion<fScalar_>& quaternion, const Scalar_ scalar) noexcept;
    template<typename Scalar_, typename fScalar_> 
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
    operator*(const Scalar_ scalar, const Quaternion<fScalar_>& quaternion) noexcept;
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion, const DualQuaternion<fScalar_>& dual_quaternion) noexcept;
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dual_quaternion, const Quaternion<fScalar_>& quaternion) noexcept;

    // Defaults

    virtual ~Quaternion()=default;
             Quaternion(const Quaternion& other)=default;
             Quaternion(Quaternion&& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;
};

template<typename qScalar_>
class PureQuaternion : public Quaternion<qScalar_>
{
public:

    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureQuaternion(const Vec3_t<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureQuaternion(const qScalar_ x, const qScalar_ y, const qScalar_ z);

    PureQuaternion(const Quaternion<qScalar_>& q);
    PureQuaternion(Quaternion<qScalar_>&& q);
    PureQuaternion& operator=(const Quaternion<qScalar_>& q);
    PureQuaternion& operator=(Quaternion<qScalar_>&& q);

    // mutable operators
    PureQuaternion& operator+=(const PureQuaternion& other);
    PureQuaternion& operator-=(const PureQuaternion& other);
    PureQuaternion& operator*=(const PureQuaternion& other) noexcept=delete;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, PureQuaternion&>
    operator*=(const Scalar_ scalar) ;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, PureQuaternion&>
    operator/=(const Scalar_ scalar) ;
    PureQuaternion& normalize();

    PureQuaternion active_rotate(const UnitQuaternion<qScalar_>& rotation){
        return PureQuaternion(rotation * (*this) * rotation.conj());
    }
    PureQuaternion passive_rotate(const UnitQuaternion<qScalar_>& rotation){
        return PureQuaternion(rotation.conj() * (*this) * rotation);
    }

    // Service functions
    qScalar_ rotation_angle() const noexcept =delete;
    UnitPureQuaternion<qScalar_> rotation_axis() const noexcept =delete;
    Vec3_t<qScalar_> rotation_axis_vec3() const noexcept =delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>>
             pow(const Scalar_ index) const noexcept =delete;
    PureQuaternion conj() const noexcept;
    PureQuaternion inv() const noexcept;
    Quaternion<qScalar_> ln() const noexcept =delete;
    Quaternion<qScalar_> exp() const noexcept =delete;

    // Defaults

                    PureQuaternion()=default;
                    PureQuaternion(const PureQuaternion& other)=default;
                    PureQuaternion(PureQuaternion& other)=default;
            virtual ~PureQuaternion()=default;
    PureQuaternion& operator=(const PureQuaternion& other)=default;
    PureQuaternion& operator=(PureQuaternion&& other)=default;
};

template<typename qScalar_>
class UnitQuaternion : public Quaternion<qScalar_>
{
protected:
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit UnitQuaternion(const qScalar_ w, const Vec3_t<qScalar_> vec3);
public:

    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
                    UnitQuaternion();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit UnitQuaternion(Vec4_t<qScalar_> vec4);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit UnitQuaternion(const qScalar_ w, const qScalar_ x=0, const qScalar_ y=0, const qScalar_ z=0);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit UnitQuaternion(const Vec3_t<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit UnitQuaternion(const UnitPureQuaternion<qScalar_> rotation_axis, const qScalar_ rotation_angle);

    UnitQuaternion(const Quaternion<qScalar_>& q);
    UnitQuaternion(Quaternion<qScalar_>&& q);
    UnitQuaternion& operator=(const Quaternion<qScalar_>& q);
    UnitQuaternion& operator=(Quaternion<qScalar_>&& q);

    // mutable operators
    UnitQuaternion& operator+=(const UnitQuaternion& other) noexcept=delete;
    UnitQuaternion& operator-=(const UnitQuaternion& other) noexcept=delete;
    UnitQuaternion& operator*=(const UnitQuaternion& other) ;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitQuaternion&>
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitQuaternion&>
    operator/=(const Scalar_ scalar) noexcept=delete;
    UnitQuaternion& normalize();
    
    // Service functions
    qScalar_ rotation_angle() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitQuaternion>
             pow(const Scalar_ index) const noexcept;
    UnitQuaternion conj() const noexcept;
    UnitQuaternion inv() const noexcept;
    Quaternion<qScalar_> ln() const noexcept=delete;
    Quaternion<qScalar_> exp() const noexcept=delete;

    // Defaults
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

    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             UnitPureQuaternion();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureQuaternion(const Vec3_t<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureQuaternion(const qScalar_ x, const qScalar_ y, const qScalar_ z);

    UnitPureQuaternion(const Quaternion<qScalar_>& q);
    UnitPureQuaternion(Quaternion<qScalar_>&& q);
    UnitPureQuaternion& operator=(const Quaternion<qScalar_>& q);
    UnitPureQuaternion& operator=(Quaternion<qScalar_>&& q);

    UnitPureQuaternion& normalize();

    UnitPureQuaternion& operator+=(const UnitPureQuaternion& other) noexcept=delete;
    UnitPureQuaternion& operator-=(const UnitPureQuaternion& other) noexcept=delete;
    UnitPureQuaternion& operator*=(const UnitPureQuaternion& other) noexcept=delete;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitPureQuaternion&>
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitPureQuaternion&>
    operator/=(const Scalar_ scalar) noexcept=delete;

    Quaternion<qScalar_> ln() const noexcept=delete;
    Quaternion<qScalar_> exp() const noexcept=delete;

                        UnitPureQuaternion(const UnitPureQuaternion& other)=default;
                        UnitPureQuaternion(UnitPureQuaternion& other)=default;
                virtual ~UnitPureQuaternion()=default;
    UnitPureQuaternion& operator=(const UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(UnitPureQuaternion&& other)=default;
};
}






// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************







namespace dq1
{

// Friend Functions *************************************************************************

template<typename Scalar_>
void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) {
    double real = std::abs(quaternion.vals_[0]);
    if (real > FLOAT_OMIT_THRESHOLD){
        if (VERY_VERBOSE)
            std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                     std::move(calling_fn) << " detected Pure Quaternion with real part " << quaternion.vals_[0] << ".\n";
        if (real > FLOAT_ERROR_THRESHOLD)
            throw std::runtime_error(std::move(calling_fn) + " detected bad Pure Quaternion real part " + std::to_string(quaternion.vals_[0]) + ".\n");
    }
    // quaternion.vals_[0] = 0;
}

template<typename Scalar_>
void _norm_should_be_one(std::string&& calling_fn, Quaternion<Scalar_>& quaternion){
    double norm_err = std::abs(quaternion.norm() - 1);
    if (norm_err > FLOAT_OMIT_THRESHOLD){
        if (VERY_VERBOSE)
            std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                     std::move(calling_fn) << " detected Unit Quaternion with norm " << quaternion.norm() << ".\n";
        if (norm_err > FLOAT_ERROR_THRESHOLD)
            throw std::runtime_error(std::move(calling_fn) + " detected bad Unit Quaternion norm " + std::to_string(quaternion.norm()) + ".\n");
    } 
    // quaternion.vals_.normalize();
}

template<typename Scalar_>
std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& q){
    os << q.operator std::string();  
return os;
} 

template<typename fScalar_>
Quaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion1, const Quaternion<fScalar_>& quaternion2) noexcept{
    return Quaternion<fScalar_>(quaternion1.hamiplus() * quaternion2.vals_); 
}

template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>> 
operator*(const Quaternion<fScalar_>& quaternion, const Scalar_ scalar) noexcept{
    return Quaternion<fScalar_>(quaternion.vals_ * scalar);
}

template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
operator*(const Scalar_ scalar, const Quaternion<fScalar_>& quaternion) noexcept {
    return Quaternion<fScalar_>(quaternion.vals_ * scalar);
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Quaternion ******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion()
    :vals_(Vec4_t<qScalar_>::Zero()){}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(Vec4_t<qScalar_> vec4): vals_(vec4){

}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const qScalar_ w, const Vec3_t<qScalar_> vec3)
{
    vals_ << w, vec3;
}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z) :vals_((Vec4_t<qScalar_>() << w, x, y, z).finished()){}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const Vec3_t<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle, const qScalar_ norm){
    vals_ << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized();
    vals_ *= norm;
}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const UnitPureQuaternion<qScalar_> rotation_axis, const qScalar_ rotation_angle, const qScalar_ norm){
    vals_ << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3();
    vals_ *= norm;
}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator+=(const Quaternion& other) noexcept {vals_ += other.vals_; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator-=(const Quaternion& other) noexcept {vals_ -= other.vals_; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator*=(const Quaternion& other) noexcept {vals_ = hamiplus() * other.vals_; return *this;}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator*=(const Scalar_ scalar) noexcept {vals_ *= scalar; return *this;}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator/=(const Scalar_ scalar) noexcept {vals_ /= scalar; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::normalize(){
    vals_.normalize();
    return *this;
}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(vals_ + other.vals_);}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(vals_ - other.vals_);}

template<typename qScalar_>
template<typename Scalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::operator/(const Scalar_ scalar) const noexcept {return Quaternion<qScalar_>(vals_ / scalar);}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-() const noexcept {return Quaternion<qScalar_>(-vals_);}

template<typename qScalar_>
bool Quaternion<qScalar_>::operator==(const Quaternion& other) const noexcept {return vals_ == other.vals_;}

template<typename qScalar_>
bool Quaternion<qScalar_>::operator!=(const Quaternion& other) const noexcept {return vals_ != other.vals_;}

template<typename qScalar_>
Quaternion<qScalar_>::operator std::string() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << vals_[0] << " + " << vals_[1] << " î + " << vals_[2] << " ĵ + " << vals_[3] << " k̂ ";
    return oss.str();
}   

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::norm() const noexcept {return vals_.norm();}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::rotation_angle() const noexcept {return 2 * acos(vals_[0] / norm());}

template<typename qScalar_>
UnitPureQuaternion<qScalar_> Quaternion<qScalar_>::rotation_axis() const noexcept {return UnitPureQuaternion<qScalar_>( rotation_axis_vec3() );}

template<typename qScalar_>
Vec3_t<qScalar_> Quaternion<qScalar_>::rotation_axis_vec3() const noexcept
{ 
    if (vec3().norm() == 0)
        return Vec3_t<qScalar_>{0,0,1}; // convention
    else 
        return vec3().normalized();
}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::conj() const noexcept {return Quaternion( (Vec4_t<qScalar_>() << vals_[0], -vec3()).finished() ); }

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::inv() const noexcept {return conj() / square(norm());}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::ln() const noexcept 
{
    return Quaternion<qScalar_>(std::log(norm()), 0.5 * rotation_angle() * rotation_axis_vec3());
} 

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::exp() const noexcept 
{
    return Quaternion<qScalar_>(std::exp(vals_[0]) * cos(vec3().norm()), std::exp(vals_[0]) * sin( vec3().norm() ) * vec3().normalized());
} 

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::pow(const Scalar_ index) const noexcept 
{
    return Quaternion<qScalar_>(rotation_axis_vec3(), index * rotation_angle(), std::pow(norm(), index));
} 

template<typename qScalar_>
UnitQuaternion<qScalar_> Quaternion<qScalar_>::normalized() const noexcept {return UnitQuaternion<qScalar_>((*this) / norm());}

template<typename qScalar_>
Mat4_t<qScalar_> Quaternion<qScalar_>::hamiplus() const noexcept 
{      
    const qScalar_ w = vals_[0];
    const qScalar_ x = vals_[1];
    const qScalar_ y = vals_[2];
    const qScalar_ z = vals_[3];
    return (Mat4_t<qScalar_>() << w, -x, -y, -z,
                                x,  w, -z,  y,
                                y,  z,  w, -x,
                                z, -y,  x,  w).finished();
}

template<typename qScalar_>
Mat4_t<qScalar_> Quaternion<qScalar_>::haminus() const noexcept 
{
    const qScalar_ w = vals_[0];
    const qScalar_ x = vals_[1];
    const qScalar_ y = vals_[2];
    const qScalar_ z = vals_[3];
    return (Mat4_t<qScalar_>() << w, -x, -y, -z,
                                x,  w,  z, -y,
                                y, -z,  w,  x,
                                z,  y, -x,  w).finished();
}

template<typename qScalar_>
std::string Quaternion<qScalar_>::to_string() const {return operator std::string();}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::w() const noexcept {return vals_[0];}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::x() const noexcept {return vals_[1];}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::y() const noexcept {return vals_[2];}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::z() const noexcept {return vals_[3];}

template<typename qScalar_>
Vec3_t<qScalar_> Quaternion<qScalar_>::vec3() const noexcept {return vals_.template tail<3>();}

template<typename qScalar_>
Vec4_t<qScalar_> Quaternion<qScalar_>::vec4() const noexcept {return vals_;}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class PureQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
PureQuaternion<qScalar_>::PureQuaternion(const Vec3_t<qScalar_> vec3) : Quaternion<qScalar_>(0, vec3) {}

template<typename qScalar_>
template<typename T, typename>
PureQuaternion<qScalar_>::PureQuaternion(const qScalar_ x, const qScalar_ y, const qScalar_ z) 
    : Quaternion<qScalar_>(0, x,y,z) {}

template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _real_part_should_be_zero("PureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("PureQuaternion(Quaternion<qScalar_>&& other)", *this);
}

template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("PureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("PureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator+=(const PureQuaternion& other) {
    this->vals_ += other.vals_;
    _real_part_should_be_zero("PureQuaternion<qScalar_>::operator+=(const PureQuaternion& other)", *this);
    return *this;
}
template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator-=(const PureQuaternion& other) {
    this->vals_ -= other.vals_;
    _real_part_should_be_zero("PureQuaternion<qScalar_>::operator-=(const PureQuaternion& other)", *this);
    return *this;
}
template<typename qScalar_>
template<typename Scalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, PureQuaternion<qScalar_>&>
PureQuaternion<qScalar_>::operator*=(const Scalar_ scalar) {
    this->vals_ *= scalar;
    _real_part_should_be_zero("PureQuaternion<qScalar_>::operator*=(const PureQuaternion& other)", *this);
    return *this;
}
template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, PureQuaternion<qScalar_>&>
PureQuaternion<qScalar_>::operator/=(const Scalar_ scalar) {
    this->vals_ /= scalar;
    _real_part_should_be_zero("PureQuaternion<qScalar_>::operator/=(const PureQuaternion& other)", *this);
    return *this;
}
template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::normalize(){
    _real_part_should_be_zero("PureQuaternion<qScalar_>::normalize()", *this);
    this->vals_.normalize();
    return *this;
}

template<typename qScalar_>
PureQuaternion<qScalar_> PureQuaternion<qScalar_>::conj() const noexcept{
    return PureQuaternion<qScalar_>(-*this);
}

template<typename qScalar_>
PureQuaternion<qScalar_> PureQuaternion<qScalar_>::inv() const noexcept{
    return PureQuaternion<qScalar_>(-*this / square(this->norm()));
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion() : Quaternion<qScalar_>(1) {
    
}

template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion(Vec4_t<qScalar_> vec4) : Quaternion<qScalar_>(vec4) {
    _norm_should_be_one("UnitQuaternion(const Vec4_t<qScalar_>& vec4)", *this);
}

template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion(const qScalar_ w, const Vec3_t<qScalar_> vec3) 
    : Quaternion<qScalar_>(w, vec3) {
    _norm_should_be_one("UnitQuaternion(const qScalar_ w, const Vec3_t<qScalar_> vec3)", *this);
}

template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z) 
    : Quaternion<qScalar_>(w, x, y, z) {
    _norm_should_be_one("UnitQuaternion(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z)", *this);
}

template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion(const Vec3_t<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle) 
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized()) {
    _norm_should_be_one("UnitQuaternion(const Vec3_t<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle)", *this);

}

template<typename qScalar_>
template<typename T, typename>
UnitQuaternion<qScalar_>::UnitQuaternion(const UnitPureQuaternion<qScalar_> rotation_axis, const qScalar_ rotation_angle)
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3()){
    _norm_should_be_one("UnitQuaternion(const UnitPureQuaternion<qScalar_> rotation_axis, const qScalar_ rotation_angle)", *this);
}

template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _norm_should_be_one("UnitQuaternion(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _norm_should_be_one("UnitQuaternion(Quaternion<qScalar_>&& other)", *this);
}

template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other); // Use the base class assignment operator
        _norm_should_be_one("UnitQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other)); // Use the base class assignment operator
        _norm_should_be_one("UnitQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::operator*=(const UnitQuaternion& other) {
    this->vals_ = this->hamiplus() * other.vals_;
    _norm_should_be_one("UnitQuaternion<qScalar_>::operator*=(const UnitQuaternion& other)", *this);
    return *this;
}
template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::normalize(){
    this->vals_.normalize();
    return *this;
}

template<typename qScalar_>
qScalar_ UnitQuaternion<qScalar_>::rotation_angle() const noexcept{
    return 2 * acos(this->vals_[0]);
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitQuaternion<qScalar_>>
            UnitQuaternion<qScalar_>::pow(const Scalar_ index) const noexcept{
     return UnitQuaternion<qScalar_>(this->rotation_axis_vec3(), index * rotation_angle());
}

template<typename qScalar_>
UnitQuaternion<qScalar_> UnitQuaternion<qScalar_>::conj() const noexcept{
    return UnitQuaternion<qScalar_>(this->vals_[0], -this->vals_.template tail<3>());
}

template<typename qScalar_>
UnitQuaternion<qScalar_> UnitQuaternion<qScalar_>::inv() const noexcept{
    return conj();
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitPureQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(): Quaternion<qScalar_>(0,1,0,0) {

}
template<typename qScalar_>
template<typename T, typename>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const Vec3_t<qScalar_> vec3): Quaternion<qScalar_>(0, vec3) {
    _norm_should_be_one("UnitPureQuaternion(const Vecx<qScalar_>& vec)", *this);
}

template<typename qScalar_>
template<typename T, typename>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const qScalar_ x, const qScalar_ y, const qScalar_ z):Quaternion<qScalar_>(0, x,y,z){
    _norm_should_be_one("UnitPureQuaternion(const qScalar_ x, const qScalar_ y, const qScalar_ z)", *this);
}

template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const Quaternion<qScalar_>& other): Quaternion<qScalar_>(other){
    _real_part_should_be_zero("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(Quaternion<qScalar_>&& other): Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
UnitPureQuaternion<qScalar_>& UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
        _norm_should_be_one("UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
UnitPureQuaternion<qScalar_>& UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
        _norm_should_be_one("UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}
template<typename qScalar_>
UnitPureQuaternion<qScalar_>& UnitPureQuaternion<qScalar_>::normalize(){
    this->vals_.normalize();
    return *this;
}



}





































namespace dq1
{

template<typename qScalar_>
class DualQuaternion{
protected:
    Quaternion<qScalar_> primary_;
    Quaternion<qScalar_> dual_;
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(Quaternion<qScalar_> primary, Quaternion<qScalar_> dual=Quaternion<qScalar_>());
    explicit DualQuaternion(const Vec8_t<qScalar_>& vec8);
    explicit DualQuaternion(Vec8_t<qScalar_>&& vec8);
    DualQuaternion(const qScalar_ h0, const qScalar_ h1=0, const qScalar_ h2=0, const qScalar_ h3=0, const qScalar_ h4=0, const qScalar_ h5=0, const qScalar_ h6=0, const qScalar_ h7=0);
    
    DualQuaternion(const DualQuaternion& dq)=default;
    DualQuaternion(DualQuaternion&& dq)=default;
    virtual ~DualQuaternion()=default;
    DualQuaternion& operator=(const DualQuaternion& dq)=default;
    DualQuaternion& operator=(DualQuaternion&& dq)=default;

    // mutable operators    

    DualQuaternion& operator+=(const DualQuaternion& other) noexcept;
    DualQuaternion& operator-=(const DualQuaternion& other) noexcept;
    DualQuaternion& operator*=(const DualQuaternion& other) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion&> 
    operator*=(const Scalar_ scalar) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion&> 
    operator/=(const Scalar_ scalar) noexcept;
    DualQuaternion& normalize() noexcept;

    // Const operator    
    
    DualQuaternion operator+(const DualQuaternion& other) const noexcept;
    DualQuaternion operator-(const DualQuaternion& other) const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    operator/(const Scalar_ scalar) const noexcept;
    DualQuaternion operator-() const noexcept;
    qScalar_ operator[](const int index) const;
    bool operator==(const DualQuaternion& other) const noexcept;
    bool operator!=(const DualQuaternion& other) const noexcept; 
    operator std::string() const;

    // service functions

    DualQuaternion norm() const noexcept;
    DualQuaternion conj() const noexcept;
    DualQuaternion normalized() const noexcept;
    Mat8_t<qScalar_> hamiplus() const noexcept;
    Mat8_t<qScalar_> haminus() const noexcept;
    Quaternion<qScalar_> primary() const noexcept;
    Quaternion<qScalar_> dual() const noexcept;
    Vec6_t<qScalar_> vec6() const noexcept;
    Vec8_t<qScalar_> vec8() const noexcept;
    std::string to_string() const;
    
    // Friend functions
    template<typename Scalar_>
    friend void _primary_part_should_be_unit(std::string&& calling_fn, DualQuaternion<Scalar_>& dq);

    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& dq);

    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dq1, const DualQuaternion<fScalar_>& dq2) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>> 
    operator*(const DualQuaternion<fScalar_>& dq, const Scalar_ scalar) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
    operator*(const Scalar_ scalar, const DualQuaternion<fScalar_>& dq) noexcept;
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion, const DualQuaternion<fScalar_>& dual_quaternion) noexcept;
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dual_quaternion, const Quaternion<fScalar_>& quaternion) noexcept;

private:
    // not inplemented
    DualQuaternion inv() const noexcept;
    DualQuaternion ln() const noexcept;
    DualQuaternion exp() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    pow(const Scalar_ index) const noexcept;
};


template<typename qScalar_>
class UnitDualQuaternion: public DualQuaternion<qScalar_>{
public:

    // Constructors and Assignments

    UnitDualQuaternion();
    UnitDualQuaternion(UnitQuaternion<qScalar_> primary, Quaternion<qScalar_> dual=Quaternion<qScalar_>());
    explicit UnitDualQuaternion(const Vec8_t<qScalar_>& vec8); 
    explicit UnitDualQuaternion(Vec8_t<qScalar_>&& vec8);
    UnitDualQuaternion(const qScalar_ h0, const qScalar_ h1=0, const qScalar_ h2=0, const qScalar_ h3=0, const qScalar_ h4=0, const qScalar_ h5=0, const qScalar_ h6=0, const qScalar_ h7=0);
    
    UnitDualQuaternion(const DualQuaternion<qScalar_>& dq);
    UnitDualQuaternion(DualQuaternion<qScalar_>&& dq);
    UnitDualQuaternion& operator=(const DualQuaternion<qScalar_>& dq);
    UnitDualQuaternion& operator=(DualQuaternion<qScalar_>&& dq);

    // mutable operators    
    UnitDualQuaternion& operator+=(const UnitDualQuaternion& other) noexcept=delete;
    UnitDualQuaternion& operator-=(const UnitDualQuaternion& other) noexcept=delete;
    UnitDualQuaternion& operator*=(const UnitDualQuaternion& other) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitDualQuaternion&> 
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitDualQuaternion&> 
    operator/=(const Scalar_ scalar) noexcept=delete;
    UnitDualQuaternion& normalize() noexcept;


    template<typename First_, typename... Args_>
    static UnitDualQuaternion build_from(First_ first, Args_... args){
        return UnitDualQuaternion(build_from(first) * build_from(args...));
    }  
    static UnitDualQuaternion build_from(UnitQuaternion<qScalar_> rotation){
        return UnitDualQuaternion(rotation);
    }
    static UnitDualQuaternion build_from(PureQuaternion<qScalar_> translation){
        return UnitDualQuaternion(UnitQuaternion<qScalar_>(1), translation / 2);
    }
    static UnitDualQuaternion build_from(UnitDualQuaternion pose){
        return pose;
    }

    UnitQuaternion<qScalar_> rotation() const noexcept {return UnitQuaternion(this->primary_);}
    PureQuaternion<qScalar_> translation() const noexcept {return PureQuaternion(this->dual_ * this->primary_.conj() * 2);}

    UnitDualQuaternion(const UnitDualQuaternion& dq)=default;
    UnitDualQuaternion(UnitDualQuaternion&& dq)=default;
    virtual ~UnitDualQuaternion()=default;
    UnitDualQuaternion& operator=(const UnitDualQuaternion& dq)=default;
    UnitDualQuaternion& operator=(UnitDualQuaternion&& dq)=default;

};

}





// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************





namespace dq1
{

// Friend functions

template<typename Scalar_>
std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& dq) {
    os << dq.operator std::string();  
    return os;
}

template<typename Scalar_>
void _primary_part_should_be_unit(std::string&& calling_fn, DualQuaternion<Scalar_>& dq){
    double primary_norm_err = std::abs(dq.primary_.norm() - 1);
    if (primary_norm_err > FLOAT_OMIT_THRESHOLD){
        if (VERY_VERBOSE)
            std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                        std::move(calling_fn) << " detected primary part norm " << dq.primary_.norm() << ".\n";
        if (primary_norm_err > FLOAT_ERROR_THRESHOLD)
            throw std::runtime_error(std::move(calling_fn) + " detected bad primary part norm " + std::to_string(dq.primary_.norm()) + ".\n");
    }    
    // dq.normalize();
}

template<typename fScalar_>
DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dq1, const DualQuaternion<fScalar_>& dq2) noexcept{
    return DualQuaternion<fScalar_>(dq1.primary_ * dq2.primary_, dq1.primary_ * dq2.dual_ + dq1.dual_ * dq2.primary_);
}
template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
operator*(const DualQuaternion<fScalar_>& dq, const Scalar_ scalar) noexcept{
    return DualQuaternion<fScalar_>(dq.primary_ * scalar, dq.dual_ * scalar);
}
template<typename Scalar_, typename fScalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
operator*(const Scalar_ scalar, const DualQuaternion<fScalar_>& dq) noexcept{
    return DualQuaternion<fScalar_>(dq.primary_ * scalar, dq.dual_ * scalar);
}

template<typename fScalar_>
DualQuaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion, const DualQuaternion<fScalar_>& dual_quaternion) noexcept{
    return DualQuaternion<fScalar_>(quaternion * dual_quaternion.primary_, quaternion * dual_quaternion.dual_);
}

template<typename fScalar_>
DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dual_quaternion, const Quaternion<fScalar_>& quaternion) noexcept{
    return DualQuaternion<fScalar_>(quaternion * dual_quaternion.primary_, quaternion * dual_quaternion.dual_);
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class DualQuaternion *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

// Default constructor
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(): primary_(), dual_() {}

/**
 * @brief Construct a DualQuaternion from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Quaternion<qScalar_> primary, Quaternion<qScalar_> dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from a Vec8.
 * @param vec8 A Vec8 representing the dual quaternion values.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(const Vec8_t<qScalar_>& vec8)
    : primary_(vec8.template head<4>()), dual_(vec8.template tail<4>()) {}

/**
 * @brief Construct a DualQuaternion from an rvalue Vec8.
 * @param vec8 An rvalue Vec8 representing the dual quaternion values.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Vec8_t<qScalar_>&& vec8)
    : primary_(std::move(vec8.template head<4>())), dual_(std::move(vec8.template tail<4>())) {}

/**
 * @brief Construct a DualQuaternion from individual scalar values.
 * @param h0 The first scalar value.
 * @param h1 The second scalar value.
 * @param h2 The third scalar value.
 * @param h3 The fourth scalar value.
 * @param h4 The fifth scalar value.
 * @param h5 The sixth scalar value.
 * @param h6 The seventh scalar value.
 * @param h7 The eighth scalar value.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(const qScalar_ h0, const qScalar_ h1, const qScalar_ h2, const qScalar_ h3,
                                        const qScalar_ h4, const qScalar_ h5, const qScalar_ h6, const qScalar_ h7)     
                                        : primary_(h0, h1, h2, h3), dual_(h4, h5, h6, h7) {}

// Mutable operators    

template<typename qScalar_>
DualQuaternion<qScalar_>& DualQuaternion<qScalar_>::operator+=(const DualQuaternion& other) noexcept{
    primary_ += other.primary_;
    dual_ += other.dual_;
    return *this;
}

template<typename qScalar_>
DualQuaternion<qScalar_>& DualQuaternion<qScalar_>::operator-=(const DualQuaternion& other) noexcept{
    primary_ -= other.primary_;
    dual_ -= other.dual_;
    return *this;
}

template<typename qScalar_>
DualQuaternion<qScalar_>& DualQuaternion<qScalar_>::operator*=(const DualQuaternion& other) noexcept{
    primary_ *= other.primary_;
    dual_ = primary_ * other.dual_ + dual_ * other.primary_;
    return *this;
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>&> 
DualQuaternion<qScalar_>::operator*=(const Scalar_ scalar) noexcept{
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>&> 
DualQuaternion<qScalar_>::operator/=(const Scalar_ scalar) noexcept{
    primary_ /= scalar;
    dual_ /= scalar;
    return *this;
}

/**
 * @brief Normalize this DualQuaternion.
 * @return A reference to the normalized DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>& DualQuaternion<qScalar_>::normalize() noexcept {
    const qScalar_ norm = primary_.norm();
    primary_ /= norm;
    dual_ /= norm;
    return *this;
}

// Const operator    

template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::operator+(const DualQuaternion<qScalar_>& other) const noexcept{
    return DualQuaternion<qScalar_>(primary_ + other.primary_, dual_ + other.dual_);
}
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::operator-(const DualQuaternion<qScalar_>& other) const noexcept{
    return DualQuaternion<qScalar_>(primary_ - other.primary_, dual_ - other.dual_);
}
template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>> 
DualQuaternion<qScalar_>::operator/(const Scalar_ scalar) const noexcept{
    return DualQuaternion<qScalar_>(primary_ / scalar, dual_ / scalar);
}
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::operator-() const noexcept{
    return DualQuaternion<qScalar_>(-primary_, -dual_);
}

template<typename qScalar_>
qScalar_ DualQuaternion<qScalar_>::operator[](const int index) const{
    return vec8()[index];
}
template<typename qScalar_>
bool DualQuaternion<qScalar_>::operator==(const DualQuaternion<qScalar_>& other) const noexcept{
    return primary_ == other.primary_ && dual_ == other.dual_;
}
template<typename qScalar_>
bool DualQuaternion<qScalar_>::operator!=(const DualQuaternion<qScalar_>& other) const noexcept{
    return primary_ != other.primary_ || dual_ != other.dual_;
}
template<typename qScalar_>
DualQuaternion<qScalar_>::operator std::string() const{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << primary_ << " + " << " ϵ ( " << dual_ << " )";
    return oss.str();
}

// Service functions

/**
 * @brief Compute the norm of this DualQuaternion.
 * @return The norm of the DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::norm() const noexcept {
    if (primary_.norm() == 0) 
        return DualQuaternion<qScalar_>();
    const qScalar_ primary_norm = primary_.norm();
    const qScalar_ dual_norm = primary_.vec4().dot(dual_.vec4()) / primary_norm;

    return DualQuaternion<qScalar_>(Quaternion<qScalar_>(primary_norm), Quaternion<qScalar_>(dual_norm));
}

template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::conj() const noexcept {
    return DualQuaternion<qScalar_>(primary_.conj(), dual_.conj());
}

template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::inv() const noexcept {
    // Implementation details here...
}

template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::ln() const noexcept {
    // Implementation details here...
}

template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::exp() const noexcept {
    // Implementation details here...
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>>
DualQuaternion<qScalar_>::pow(const Scalar_ index) const noexcept{

}

/**
 * @brief Return a normalized version of this DualQuaternion.
 * @return A normalized DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::normalized() const noexcept {
    return operator/=(primary_.norm());
}

/**
 * @brief Compute the Hamilton plus matrix of this DualQuaternion.
 * @return The Hamilton plus matrix of the DualQuaternion.
 */
template<typename qScalar_>
Mat8_t<qScalar_> DualQuaternion<qScalar_>::hamiplus() const noexcept {
    return (Mat8_t<qScalar_>() << primary_.hamiplus(), Mat4_t<qScalar_>::Zero(), dual_.hamiplus(), primary_.hamiplus()).finished();
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename qScalar_>
Mat8_t<qScalar_> DualQuaternion<qScalar_>::haminus() const noexcept {
    return (Mat8_t<qScalar_>() << primary_.haminus(), Mat4_t<qScalar_>::Zero(), dual_.haminus(), primary_.haminus()).finished();
}

template<typename qScalar_>
Quaternion<qScalar_> DualQuaternion<qScalar_>::primary() const noexcept{
    return primary_;
}

template<typename qScalar_>
Quaternion<qScalar_> DualQuaternion<qScalar_>::dual() const noexcept{
    return dual_;
}

/**
 * @brief Convert this DualQuaternion to a Vec6 representation.
 * @return A Vec6 representation of the DualQuaternion.
 */
template<typename qScalar_>
Vec6_t<qScalar_> DualQuaternion<qScalar_>::vec6() const noexcept {
    return (Vec6_t<qScalar_>() << primary_.vec3(), dual_.vec3()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename qScalar_>
Vec8_t<qScalar_> DualQuaternion<qScalar_>::vec8() const noexcept {
    return (Vec8_t<qScalar_>() << primary_.vec4(), dual_.vec4()).finished();
}

template<typename qScalar_>
std::string DualQuaternion<qScalar_>::to_string() const{
    return operator std::string();
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitDualQuaternion *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************



// Constructors and Assignments


template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(): DualQuaternion<qScalar_>(UnitQuaternion<qScalar_>(1)){

}

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(UnitQuaternion<qScalar_> primary, Quaternion<qScalar_> dual)
    : DualQuaternion<qScalar_>(primary, dual){ } 

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const Vec8_t<qScalar_>& vec8)
    : DualQuaternion<qScalar_>( UnitQuaternion<qScalar_>(vec8.template head<4>()), Quaternion<qScalar_>(vec8.template tail<4>()) ){ }

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(Vec8_t<qScalar_>&& vec8) 
    : DualQuaternion<qScalar_>( UnitQuaternion<qScalar_>(std::move(vec8.template head<4>())), Quaternion<qScalar_>(std::move(vec8.template tail<4>())) ){ }
template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const qScalar_ h0, const qScalar_ h1, const qScalar_ h2, const qScalar_ h3, const qScalar_ h4, const qScalar_ h5, const qScalar_ h6, const qScalar_ h7)
    : DualQuaternion<qScalar_>( UnitQuaternion<qScalar_>(h0, h1, h2, h3), Quaternion<qScalar_>(h4, h5, h6, h7) ){ }
template<typename qScalar_>  
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const DualQuaternion<qScalar_>& dq): DualQuaternion<qScalar_>(dq){
    _primary_part_should_be_unit("UnitDualQuaternion(const DualQuaternion<qScalar_>& dq)", *this);
}
template<typename qScalar_>  
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(DualQuaternion<qScalar_>&& dq): DualQuaternion<qScalar_>(std::move(dq)){
    _primary_part_should_be_unit("UnitDualQuaternion(DualQuaternion<qScalar_>&& dq)", *this);
}
template<typename qScalar_>  
UnitDualQuaternion<qScalar_>& UnitDualQuaternion<qScalar_>::operator=(const DualQuaternion<qScalar_>& dq){
    if (this != &dq) {
        DualQuaternion<qScalar_>::operator=(dq);
        _primary_part_should_be_unit("UnitDualQuaternion<qScalar_>::operator=(const DualQuaternion<qScalar_>& dq)", *this);
    }
    return *this;
}
template<typename qScalar_>  
UnitDualQuaternion<qScalar_>& UnitDualQuaternion<qScalar_>::operator=(DualQuaternion<qScalar_>&& dq){
    if (this != &dq) {
        DualQuaternion<qScalar_>::operator=(std::move(dq));
        _primary_part_should_be_unit("UnitDualQuaternion<qScalar_>::operator=(DualQuaternion<qScalar_>&& dq)", *this);
    }
    return *this;
}

template<typename qScalar_>  
UnitDualQuaternion<qScalar_>& UnitDualQuaternion<qScalar_>::operator*=(const UnitDualQuaternion& other) noexcept{
    this->primary_ *= other.primary_;
    this->dual_ = this->primary_ * other.dual_ + this->dual_ * other.primary_;
    return *this;
}

template<typename qScalar_>  
UnitDualQuaternion<qScalar_>& UnitDualQuaternion<qScalar_>::normalize() noexcept{
    this->normalize();
}
    
}
















namespace dq1{

template<typename Scalar_>
using Quat_t = Quaternion<Scalar_>;
template<typename Scalar_>
using Rot_t = UnitQuaternion<Scalar_>;
template<typename Scalar_>
using Tran_t = PureQuaternion<Scalar_>;
template<typename Scalar_>
using BaseAxis_t = UnitPureQuaternion<Scalar_>;
template<typename Scalar_>
using DQ_t = DualQuaternion<Scalar_>;
template<typename Scalar_>
using Pose_t = UnitDualQuaternion<Scalar_>;

using Quat = Quat_t<scalar_t>;
using Rot = Rot_t<scalar_t>;
using Tran = Tran_t<scalar_t>;
using BaseAxis = BaseAxis_t<scalar_t>;
using DQ = DQ_t<scalar_t>;
using Pose = Pose_t<scalar_t>;

using Pose_jcb = Mat_t<scalar_t, 8, -1>;
using Rot_jcb = Mat_t<scalar_t, 4, -1>;
using Tran_jcb = Mat_t<scalar_t, 4, -1>;
using DH_mat = Mat_t<scalar_t, 5, -1>;
using Joint_limit_mat = Mat_t<scalar_t, 4, -1>;

const BaseAxis i_(1,0,0);
const BaseAxis j_(0,1,0);
const BaseAxis k_(0,0,1);
const Mat4 C4_ = (Mat4() << 1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,-1).finished();
const Mat8 C8_ = (Mat8() << C4_, Mat4::Zero(), C4_, Mat4::Zero()).finished();



}
