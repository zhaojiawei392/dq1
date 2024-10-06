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

// Template type alias
using Eigen::VectorX;
using Eigen::Vector3;
using Eigen::Vector4;
template<typename Scalar_>
using Vector6 = Eigen::Vector<Scalar_, 6>;
template<typename Scalar_>
using Vector8 = Eigen::Vector<Scalar_, 8>;

using Eigen::RowVectorX;
using Eigen::RowVector3;
using Eigen::RowVector4;
template<typename Scalar_>
using RowVector6 = Eigen::RowVector<Scalar_, 6>;
template<typename Scalar_>
using RowVector8 = Eigen::RowVector<Scalar_, 8>;

using Eigen::MatrixX;
using Eigen::Matrix3;
using Eigen::Matrix4;
template<typename Scalar_>
using Matrix6 = Eigen::Matrix<Scalar_, 6, 6>;
template<typename Scalar_>
using Matrix8 = Eigen::Matrix<Scalar_, 8, 8>;

namespace Template
{

template <typename T>
T square(const T& x) {
    return x * x;
}

constexpr int PRINT_PRECISION = 18;
constexpr double FLOAT_OMIT_THRESHOLD = 0.00000001;
constexpr double FLOAT_ERROR_THRESHOLD = 0.0001;
constexpr bool VERY_VERBOSE = false;
template<typename qScalar_>
class Quaternion;
template<typename qScalar_>
class Translation;
template<typename qScalar_>
class Rotation;
template<typename qScalar_>
class UnitAxis;
template<typename qScalar_>
class DualQuaternion;
template<typename qScalar_>
class Pose;

template<typename qScalar_>
class Quaternion{
protected:
    Vector4<qScalar_> _vec4;
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const qScalar_ w, const Vector3<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    Quaternion(const UnitAxis<qScalar_> rotation_axis, const qScalar_ rotation_angle, const qScalar_ norm=1);
public:
    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             Quaternion();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Quaternion(const Vector4<qScalar_>& vec4);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             Quaternion(const qScalar_ w, const qScalar_ x=0, const qScalar_ y=0, const qScalar_ z=0);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             Quaternion(const Vector3<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle, const qScalar_ norm=1);

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

    // Service functions const

    qScalar_ norm() const noexcept;
    qScalar_ rotation_angle() const noexcept;
    UnitAxis<qScalar_> rotation_axis() const noexcept;
    Vector3<qScalar_> rotation_axis_vec3() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
             pow(const Scalar_ index) const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    Rotation<qScalar_> normalized() const noexcept;
    Matrix4<qScalar_> hamiplus() const noexcept;
    Matrix4<qScalar_> haminus() const noexcept;

    // Query const

    inline std::string to_string() const {return operator std::string();};
    inline qScalar_ w() const noexcept {return _vec4[0];};
    inline qScalar_ x() const noexcept {return _vec4[1];}; 
    inline qScalar_ y() const noexcept {return _vec4[2];};
    inline qScalar_ z() const noexcept {return _vec4[3];};
    inline Vector3<qScalar_> vec3() const noexcept {return _vec4.template tail<3>();};
    inline Vector4<qScalar_> vec4() const noexcept {return _vec4;};
    inline qScalar_* data() const noexcept {return vec4().data();}
    inline qScalar_* vrep_data() const noexcept {return (Vector4<qScalar_>()<< vec3(), w()).finished().data(); }

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
class Translation : public Quaternion<qScalar_> 
{ // PureQuaternion
public:

    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const Vector3<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const qScalar_ x, const qScalar_ y, const qScalar_ z);

    Translation(const Quaternion<qScalar_>& q); // implicitly convert a quaternion to a translation
    Translation(Quaternion<qScalar_>&& q);
    Translation& operator=(const Quaternion<qScalar_>& q);
    Translation& operator=(Quaternion<qScalar_>&& q);

    // mutable operators
    Translation& operator+=(const Translation& other) noexcept;
    Translation& operator-=(const Translation& other) noexcept;
    Translation& operator*=(const Translation& other) noexcept=delete;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Translation&>
    operator*=(const Scalar_ scalar) ;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Translation&>
    operator/=(const Scalar_ scalar) ;
    Translation& normalize();

    Translation active_rotate(const Rotation<qScalar_>& rotation){
        return Translation(rotation * (*this) * rotation.conj());
    }
    Translation passive_rotate(const Rotation<qScalar_>& rotation){
        return Translation(rotation.conj() * (*this) * rotation);
    }

    Quaternion<qScalar_> operator+(const Quaternion<qScalar_>& other) const noexcept=delete;
    Quaternion<qScalar_> operator-(const Quaternion<qScalar_>& other) const noexcept=delete;
    Translation operator+(const Translation& other) const noexcept;
    Translation operator-(const Translation& other) const noexcept;

    // Service functions
    qScalar_ rotation_angle() const noexcept =delete;
    UnitAxis<qScalar_> rotation_axis() const noexcept =delete;
    Vector3<qScalar_> rotation_axis_vec3() const noexcept =delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>>
             pow(const Scalar_ index) const noexcept =delete;
    Translation conj() const noexcept;
    Translation inv() const noexcept;
    Quaternion<qScalar_> ln() const noexcept =delete;
    Quaternion<qScalar_> exp() const noexcept =delete;

    // query
    inline qScalar_* data() const noexcept {return Quaternion<qScalar_>::vec3().data();}

    // Defaults

                    Translation()=default;
                    Translation(const Translation& other)=default;
                    Translation(Translation& other)=default;
            virtual ~Translation()=default;
    Translation& operator=(const Translation& other)=default;
    Translation& operator=(Translation&& other)=default;
};

template<typename qScalar_>
class Rotation : public Quaternion<qScalar_>
{ // UnitQuaternion
protected:
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit Rotation(const qScalar_ w, const Vector3<qScalar_> vec3);
public:

    // Constructors and Assignments
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
                    Rotation();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit Rotation(const Vector4<qScalar_>& vec4);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit Rotation(const qScalar_ w, const qScalar_ x=0, const qScalar_ y=0, const qScalar_ z=0);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit Rotation(const Vector3<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
           explicit Rotation(const UnitAxis<qScalar_> rotation_axis, const qScalar_ rotation_angle);

    Rotation(const Quaternion<qScalar_>& q); // implicitly convert a quaternion to a rotation
    Rotation(Quaternion<qScalar_>&& q);
    Rotation& operator=(const Quaternion<qScalar_>& q);
    Rotation& operator=(Quaternion<qScalar_>&& q);

    // mutable operators
    Rotation& operator+=(const Rotation& other) noexcept=delete;
    Rotation& operator-=(const Rotation& other) noexcept=delete;
    Rotation& operator*=(const Rotation& other) noexcept;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&>
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&>
    operator/=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Rotation&>
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Rotation&>
    operator/=(const Scalar_ scalar) noexcept=delete;
    Rotation& normalize();
    
    Quaternion<qScalar_> operator+(const Quaternion<qScalar_>& other) const noexcept=delete;
    Quaternion<qScalar_> operator-(const Quaternion<qScalar_>& other) const noexcept=delete;

    // Service functions
    qScalar_ rotation_angle() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Rotation>
             pow(const Scalar_ index) const noexcept;
    Rotation conj() const noexcept;
    Rotation inv() const noexcept;
    Quaternion<qScalar_> ln() const noexcept=delete;
    Quaternion<qScalar_> exp() const noexcept=delete;

    // Defaults
                    Rotation(const Rotation& other)=default;
                    Rotation(Rotation& other)=default;
    Rotation& operator=(const Rotation& other)=default;
    Rotation& operator=(Rotation&& other)=default;
            virtual ~Rotation()=default;
};

template<typename qScalar_>
class UnitAxis : public Quaternion<qScalar_>
{ // UnitPureQuaternion
public:

    // Constructors and Assignments

    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
             UnitAxis();
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const Vector3<qScalar_> vec3);
    template <typename T = qScalar_, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const qScalar_ x, const qScalar_ y, const qScalar_ z);

    UnitAxis(const Quaternion<qScalar_>& q); // implicitly convert a quaternion to a UnitAxis
    UnitAxis(Quaternion<qScalar_>&& q);
    UnitAxis& operator=(const Quaternion<qScalar_>& q);
    UnitAxis& operator=(Quaternion<qScalar_>&& q);

    UnitAxis& normalize();

    UnitAxis& operator+=(const UnitAxis& other) noexcept=delete;
    UnitAxis& operator-=(const UnitAxis& other) noexcept=delete;
    UnitAxis& operator*=(const UnitAxis& other) noexcept=delete;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitAxis&>
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, UnitAxis&>
    operator/=(const Scalar_ scalar) noexcept=delete;

    Quaternion<qScalar_> operator+(const Quaternion<qScalar_>& other) const noexcept=delete;
    Quaternion<qScalar_> operator-(const Quaternion<qScalar_>& other) const noexcept=delete;

    Quaternion<qScalar_> ln() const noexcept=delete;
    Quaternion<qScalar_> exp() const noexcept=delete;

                        UnitAxis(const UnitAxis& other)=default;
                        UnitAxis(UnitAxis& other)=default;
                virtual ~UnitAxis()=default;
    UnitAxis& operator=(const UnitAxis& other)=default;
    UnitAxis& operator=(UnitAxis&& other)=default;
};






// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************







// Friend Functions *************************************************************************

template<typename Scalar_>
void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) {
    double real = std::abs(quaternion._vec4[0]);
    if (real > FLOAT_OMIT_THRESHOLD){
        if (VERY_VERBOSE)
            std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                     std::move(calling_fn) << " detected Pure Quaternion with real part " << quaternion._vec4[0] << ".\n";
        if (real > FLOAT_ERROR_THRESHOLD)
            throw std::runtime_error(std::move(calling_fn) + " detected bad Pure Quaternion real part " + std::to_string(quaternion._vec4[0]) + ".\n");
    }
    // quaternion._vec4[0] = 0;
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
    // quaternion._vec4.normalize();
}

template<typename Scalar_>
std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& q){
    os << q.operator std::string();  
return os;
} 

template<typename fScalar_>
Quaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion1, const Quaternion<fScalar_>& quaternion2) noexcept{
    return Quaternion<fScalar_>(quaternion1.hamiplus() * quaternion2._vec4); 
}

template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>> 
operator*(const Quaternion<fScalar_>& quaternion, const Scalar_ scalar) noexcept{
    return Quaternion<fScalar_>(quaternion._vec4 * scalar);
}

template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
operator*(const Scalar_ scalar, const Quaternion<fScalar_>& quaternion) noexcept {
    return Quaternion<fScalar_>(quaternion._vec4 * scalar);
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Quaternion ******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion()
    :_vec4(Vector4<qScalar_>::Zero()){}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const Vector4<qScalar_>& vec4): _vec4(vec4){

}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const qScalar_ w, const Vector3<qScalar_> vec3)
{
    _vec4 << w, vec3;
}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z) :_vec4((Vector4<qScalar_>() << w, x, y, z).finished()){}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const Vector3<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle, const qScalar_ norm){
    _vec4 << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized();
    _vec4 *= norm;
}

template<typename qScalar_>
template<typename T, typename>
Quaternion<qScalar_>::Quaternion(const UnitAxis<qScalar_> rotation_axis, const qScalar_ rotation_angle, const qScalar_ norm){
    _vec4 << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3();
    _vec4 *= norm;
}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator+=(const Quaternion& other) noexcept {_vec4 += other._vec4; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator-=(const Quaternion& other) noexcept {_vec4 -= other._vec4; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator*=(const Quaternion& other) noexcept {_vec4 = hamiplus() * other._vec4; return *this;}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator*=(const Scalar_ scalar) noexcept {_vec4 *= scalar; return *this;}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator/=(const Scalar_ scalar) noexcept {_vec4 /= scalar; return *this;}

template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::normalize(){
    _vec4.normalize();
    return *this;
}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(_vec4 + other._vec4);}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(_vec4 - other._vec4);}

template<typename qScalar_>
template<typename Scalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::operator/(const Scalar_ scalar) const noexcept {return Quaternion<qScalar_>(_vec4 / scalar);}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-() const noexcept {return Quaternion<qScalar_>(-_vec4);}

template<typename qScalar_>
bool Quaternion<qScalar_>::operator==(const Quaternion& other) const noexcept {return _vec4 == other._vec4;}

template<typename qScalar_>
bool Quaternion<qScalar_>::operator!=(const Quaternion& other) const noexcept {return _vec4 != other._vec4;}

template<typename qScalar_>
Quaternion<qScalar_>::operator std::string() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << _vec4[0] << " + " << _vec4[1] << " î + " << _vec4[2] << " ĵ + " << _vec4[3] << " k̂ ";
    return oss.str();
}   

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::norm() const noexcept {return _vec4.norm();}

template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::rotation_angle() const noexcept {return 2 * acos(_vec4[0] / norm());}

template<typename qScalar_>
UnitAxis<qScalar_> Quaternion<qScalar_>::rotation_axis() const noexcept {return UnitAxis<qScalar_>( rotation_axis_vec3() );}

template<typename qScalar_>
Vector3<qScalar_> Quaternion<qScalar_>::rotation_axis_vec3() const noexcept
{ 
    if (vec3().norm() == 0)
        return Vector3<qScalar_>{0,0,1}; // convention
    else 
        return vec3().normalized();
}

template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::conj() const noexcept {return Quaternion( (Vector4<qScalar_>() << _vec4[0], -vec3()).finished() ); }

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
    return Quaternion<qScalar_>(std::exp(_vec4[0]) * cos(vec3().norm()), std::exp(_vec4[0]) * sin( vec3().norm() ) * vec3().normalized());
} 

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::pow(const Scalar_ index) const noexcept 
{
    return Quaternion<qScalar_>(rotation_axis_vec3(), index * rotation_angle(), std::pow(norm(), index));
} 

template<typename qScalar_>
Rotation<qScalar_> Quaternion<qScalar_>::normalized() const noexcept {return Rotation<qScalar_>((*this) / norm());}

template<typename qScalar_>
Matrix4<qScalar_> Quaternion<qScalar_>::hamiplus() const noexcept 
{      
    const qScalar_ w = _vec4[0];
    const qScalar_ x = _vec4[1];
    const qScalar_ y = _vec4[2];
    const qScalar_ z = _vec4[3];
    return (Matrix4<qScalar_>() << w, -x, -y, -z,
                                x,  w, -z,  y,
                                y,  z,  w, -x,
                                z, -y,  x,  w).finished();
}

template<typename qScalar_>
Matrix4<qScalar_> Quaternion<qScalar_>::haminus() const noexcept 
{
    const qScalar_ w = _vec4[0];
    const qScalar_ x = _vec4[1];
    const qScalar_ y = _vec4[2];
    const qScalar_ z = _vec4[3];
    return (Matrix4<qScalar_>() << w, -x, -y, -z,
                                x,  w,  z, -y,
                                y, -z,  w,  x,
                                z,  y, -x,  w).finished();
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Translation **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
Translation<qScalar_>::Translation(const Vector3<qScalar_> vec3) : Quaternion<qScalar_>(0, vec3) {}

template<typename qScalar_>
template<typename T, typename>
Translation<qScalar_>::Translation(const qScalar_ x, const qScalar_ y, const qScalar_ z) 
    : Quaternion<qScalar_>(0, x,y,z) {}

template<typename qScalar_>
Translation<qScalar_>::Translation(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _real_part_should_be_zero("Translation(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
Translation<qScalar_>::Translation(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("Translation(Quaternion<qScalar_>&& other)", *this);
}

template<typename qScalar_>
Translation<qScalar_>& Translation<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("Translation<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
Translation<qScalar_>& Translation<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("Translation<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
Translation<qScalar_>& Translation<qScalar_>::operator+=(const Translation& other) noexcept {
    this->_vec4 += other._vec4;
    _real_part_should_be_zero("Translation<qScalar_>::operator+=(const Translation& other)", *this);
    return *this;
}
template<typename qScalar_>
Translation<qScalar_>& Translation<qScalar_>::operator-=(const Translation& other) noexcept {
    this->_vec4 -= other._vec4;
    _real_part_should_be_zero("Translation<qScalar_>::operator-=(const Translation& other)", *this);
    return *this;
}
template<typename qScalar_>
template<typename Scalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Translation<qScalar_>&>
Translation<qScalar_>::operator*=(const Scalar_ scalar) {
    this->_vec4 *= scalar;
    _real_part_should_be_zero("Translation<qScalar_>::operator*=(const Translation& other)", *this);
    return *this;
}
template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Translation<qScalar_>&>
Translation<qScalar_>::operator/=(const Scalar_ scalar) {
    this->_vec4 /= scalar;
    _real_part_should_be_zero("Translation<qScalar_>::operator/=(const Translation& other)", *this);
    return *this;
}
template<typename qScalar_>
Translation<qScalar_>& Translation<qScalar_>::normalize(){
    _real_part_should_be_zero("Translation<qScalar_>::normalize()", *this);
    this->_vec4.normalize();
    return *this;
}

template<typename qScalar_>
Translation<qScalar_> Translation<qScalar_>::operator+(const Translation<qScalar_>& other) const noexcept{
    return Translation(_vec4 + other._vec4);
}

template<typename qScalar_>
Translation<qScalar_> Translation<qScalar_>::operator-(const Translation<qScalar_>& other) const noexcept{
    return Translation(_vec4 - other._vec4);
}

template<typename qScalar_>
Translation<qScalar_> Translation<qScalar_>::conj() const noexcept{
    return Translation<qScalar_>(-*this);
}

template<typename qScalar_>
Translation<qScalar_> Translation<qScalar_>::inv() const noexcept{
    return Translation<qScalar_>(-*this / square(this->norm()));
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Rotation **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation() : Quaternion<qScalar_>(1) {
    
}

template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation(const Vector4<qScalar_>& vec4) : Quaternion<qScalar_>(vec4) {
    _norm_should_be_one("Rotation(const Vector4<qScalar_>& vec4)", *this);
}

template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation(const qScalar_ w, const Vector3<qScalar_> vec3) 
    : Quaternion<qScalar_>(w, vec3) {
    _norm_should_be_one("Rotation(const qScalar_ w, const Vector3<qScalar_> vec3)", *this);
}

template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z) 
    : Quaternion<qScalar_>(w, x, y, z) {
    _norm_should_be_one("Rotation(const qScalar_ w, const qScalar_ x, const qScalar_ y, const qScalar_ z)", *this);
}

template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation(const Vector3<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle) 
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized()) {
    _norm_should_be_one("Rotation(const Vector3<qScalar_> rotation_axis_vec3, const qScalar_ rotation_angle)", *this);

}

template<typename qScalar_>
template<typename T, typename>
Rotation<qScalar_>::Rotation(const UnitAxis<qScalar_> rotation_axis, const qScalar_ rotation_angle)
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3()){
    _norm_should_be_one("Rotation(const UnitAxis<qScalar_> rotation_axis, const qScalar_ rotation_angle)", *this);
}

template<typename qScalar_>
Rotation<qScalar_>::Rotation(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _norm_should_be_one("Rotation(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
Rotation<qScalar_>::Rotation(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _norm_should_be_one("Rotation(Quaternion<qScalar_>&& other)", *this);
}

template<typename qScalar_>
Rotation<qScalar_>& Rotation<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other); // Use the base class assignment operator
        _norm_should_be_one("Rotation<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
Rotation<qScalar_>& Rotation<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other)); // Use the base class assignment operator
        _norm_should_be_one("Rotation<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
Rotation<qScalar_>& Rotation<qScalar_>::operator*=(const Rotation& other) noexcept {
    this->_vec4 = this->hamiplus() * other._vec4;
    _norm_should_be_one("Rotation<qScalar_>::operator*=(const Rotation& other)", *this);
    return *this;
}
template<typename qScalar_>
Rotation<qScalar_>& Rotation<qScalar_>::normalize(){
    this->_vec4.normalize();
    return *this;
}

template<typename qScalar_>
qScalar_ Rotation<qScalar_>::rotation_angle() const noexcept{
    return 2 * acos(this->_vec4[0]);
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Rotation<qScalar_>>
            Rotation<qScalar_>::pow(const Scalar_ index) const noexcept{
     return Rotation<qScalar_>(this->rotation_axis_vec3(), index * rotation_angle());
}

template<typename qScalar_>
Rotation<qScalar_> Rotation<qScalar_>::conj() const noexcept{
    return Rotation<qScalar_>(this->_vec4[0], -this->_vec4.template tail<3>());
}

template<typename qScalar_>
Rotation<qScalar_> Rotation<qScalar_>::inv() const noexcept{
    return conj();
}

// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitAxis **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

template<typename qScalar_>
template<typename T, typename>
UnitAxis<qScalar_>::UnitAxis(): Quaternion<qScalar_>(0,1,0,0) {

}
template<typename qScalar_>
template<typename T, typename>
UnitAxis<qScalar_>::UnitAxis(const Vector3<qScalar_> vec3): Quaternion<qScalar_>(0, vec3) {
    _norm_should_be_one("UnitAxis(const Vecx<qScalar_>& vec)", *this);
}

template<typename qScalar_>
template<typename T, typename>
UnitAxis<qScalar_>::UnitAxis(const qScalar_ x, const qScalar_ y, const qScalar_ z):Quaternion<qScalar_>(0, x,y,z){
    _norm_should_be_one("UnitAxis(const qScalar_ x, const qScalar_ y, const qScalar_ z)", *this);
}

template<typename qScalar_>
UnitAxis<qScalar_>::UnitAxis(const Quaternion<qScalar_>& other): Quaternion<qScalar_>(other){
    _real_part_should_be_zero("UnitAxis(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitAxis(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
UnitAxis<qScalar_>::UnitAxis(Quaternion<qScalar_>&& other): Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("UnitAxis(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitAxis(const Quaternion<qScalar_>& other)", *this);
}

template<typename qScalar_>
UnitAxis<qScalar_>& UnitAxis<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("UnitAxis<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
        _norm_should_be_one("UnitAxis<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

template<typename qScalar_>
UnitAxis<qScalar_>& UnitAxis<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("UnitAxis<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
        _norm_should_be_one("UnitAxis<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}
template<typename qScalar_>
UnitAxis<qScalar_>& UnitAxis<qScalar_>::normalize(){
    this->_vec4.normalize();
    return *this;
}
















template<typename qScalar_>
class DualQuaternion{
protected:
    Quaternion<qScalar_> primary_;
    Quaternion<qScalar_> dual_;
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(Quaternion<qScalar_> primary, Quaternion<qScalar_> dual=Quaternion<qScalar_>());
    explicit DualQuaternion(const Vector8<qScalar_>& vec8);
    explicit DualQuaternion(Vector8<qScalar_>&& vec8);
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
    Matrix8<qScalar_> hamiplus() const noexcept;
    Matrix8<qScalar_> haminus() const noexcept;
    Quaternion<qScalar_> primary() const noexcept;
    Quaternion<qScalar_> dual() const noexcept;
    Vector6<qScalar_> vec6() const noexcept;
    Vector8<qScalar_> vec8() const noexcept;
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
class Pose: public DualQuaternion<qScalar_>{
public:

    // Constructors and Assignments

    Pose();
    Pose(Rotation<qScalar_> primary, Quaternion<qScalar_> dual=Quaternion<qScalar_>());
    explicit Pose(const Vector8<qScalar_>& vec8); 
    explicit Pose(Vector8<qScalar_>&& vec8);
    Pose(const qScalar_ h0, const qScalar_ h1=0, const qScalar_ h2=0, const qScalar_ h3=0, const qScalar_ h4=0, const qScalar_ h5=0, const qScalar_ h6=0, const qScalar_ h7=0);
    
    Pose(const DualQuaternion<qScalar_>& dq);
    Pose(DualQuaternion<qScalar_>&& dq);
    Pose& operator=(const DualQuaternion<qScalar_>& dq);
    Pose& operator=(DualQuaternion<qScalar_>&& dq);

    // mutable operators    
    Pose& operator+=(const Pose& other) noexcept=delete;
    Pose& operator-=(const Pose& other) noexcept=delete;
    Pose& operator*=(const Pose& other) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Pose&> 
    operator*=(const Scalar_ scalar) noexcept=delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Pose&> 
    operator/=(const Scalar_ scalar) noexcept=delete;
    Pose& normalize() noexcept;


    template<typename First_, typename... Args_>
    static Pose build_from(const First_& first, const Args_&... args){
        return Pose(build_from(first) * build_from(args...));
    }  
    static Pose build_from(const Rotation<qScalar_>& rotation){
        return Pose(rotation);
    }
    static Pose build_from(const Translation<qScalar_>& translation){
        return Pose(Rotation<qScalar_>(1), translation / 2);
    }
    static Pose build_from(const Pose& pose){
        return pose;
    }

    Rotation<qScalar_> rotation() const noexcept {return Rotation(this->primary_);}
    Translation<qScalar_> translation() const noexcept {return Translation(this->dual_ * this->primary_.conj() * 2);}

    Pose(const Pose& dq)=default;
    Pose(Pose&& dq)=default;
    virtual ~Pose()=default;
    Pose& operator=(const Pose& dq)=default;
    Pose& operator=(Pose&& dq)=default;

};







// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************





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

template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Quaternion<qScalar_> primary, Quaternion<qScalar_> dual)
    : primary_(primary), dual_(dual) {}

template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(const Vector8<qScalar_>& vec8)
    : primary_(vec8.template head<4>()), dual_(vec8.template tail<4>()) {}

template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Vector8<qScalar_>&& vec8)
    : primary_(std::move(vec8.template head<4>())), dual_(std::move(vec8.template tail<4>())) {}

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
Matrix8<qScalar_> DualQuaternion<qScalar_>::hamiplus() const noexcept {
    return (Matrix8<qScalar_>() << primary_.hamiplus(), Matrix4<qScalar_>::Zero(), dual_.hamiplus(), primary_.hamiplus()).finished();
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename qScalar_>
Matrix8<qScalar_> DualQuaternion<qScalar_>::haminus() const noexcept {
    return (Matrix8<qScalar_>() << primary_.haminus(), Matrix4<qScalar_>::Zero(), dual_.haminus(), primary_.haminus()).finished();
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
Vector6<qScalar_> DualQuaternion<qScalar_>::vec6() const noexcept {
    return (Vector6<qScalar_>() << primary_.vec3(), dual_.vec3()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename qScalar_>
Vector8<qScalar_> DualQuaternion<qScalar_>::vec8() const noexcept {
    return (Vector8<qScalar_>() << primary_.vec4(), dual_.vec4()).finished();
}

template<typename qScalar_>
std::string DualQuaternion<qScalar_>::to_string() const{
    return operator std::string();
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Pose *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************



// Constructors and Assignments


template<typename qScalar_>
Pose<qScalar_>::Pose(): DualQuaternion<qScalar_>(Rotation<qScalar_>(1)){

}

template<typename qScalar_>
Pose<qScalar_>::Pose(Rotation<qScalar_> primary, Quaternion<qScalar_> dual)
    : DualQuaternion<qScalar_>(primary, dual){ } 

template<typename qScalar_>
Pose<qScalar_>::Pose(const Vector8<qScalar_>& vec8)
    : DualQuaternion<qScalar_>( Rotation<qScalar_>(vec8.template head<4>()), Quaternion<qScalar_>(vec8.template tail<4>()) ){ }

template<typename qScalar_>
Pose<qScalar_>::Pose(Vector8<qScalar_>&& vec8) 
    : DualQuaternion<qScalar_>( Rotation<qScalar_>(std::move(vec8.template head<4>())), Quaternion<qScalar_>(std::move(vec8.template tail<4>())) ){ }
template<typename qScalar_>
Pose<qScalar_>::Pose(const qScalar_ h0, const qScalar_ h1, const qScalar_ h2, const qScalar_ h3, const qScalar_ h4, const qScalar_ h5, const qScalar_ h6, const qScalar_ h7)
    : DualQuaternion<qScalar_>( Rotation<qScalar_>(h0, h1, h2, h3), Quaternion<qScalar_>(h4, h5, h6, h7) ){ }
template<typename qScalar_>  
Pose<qScalar_>::Pose(const DualQuaternion<qScalar_>& dq): DualQuaternion<qScalar_>(dq){
    _primary_part_should_be_unit("Pose(const DualQuaternion<qScalar_>& dq)", *this);
}
template<typename qScalar_>  
Pose<qScalar_>::Pose(DualQuaternion<qScalar_>&& dq): DualQuaternion<qScalar_>(std::move(dq)){
    _primary_part_should_be_unit("Pose(DualQuaternion<qScalar_>&& dq)", *this);
}
template<typename qScalar_>  
Pose<qScalar_>& Pose<qScalar_>::operator=(const DualQuaternion<qScalar_>& dq){
    if (this != &dq) {
        DualQuaternion<qScalar_>::operator=(dq);
        _primary_part_should_be_unit("Pose<qScalar_>::operator=(const DualQuaternion<qScalar_>& dq)", *this);
    }
    return *this;
}
template<typename qScalar_>  
Pose<qScalar_>& Pose<qScalar_>::operator=(DualQuaternion<qScalar_>&& dq){
    if (this != &dq) {
        DualQuaternion<qScalar_>::operator=(std::move(dq));
        _primary_part_should_be_unit("Pose<qScalar_>::operator=(DualQuaternion<qScalar_>&& dq)", *this);
    }
    return *this;
}

template<typename qScalar_>  
Pose<qScalar_>& Pose<qScalar_>::operator*=(const Pose& other) noexcept{
    this->primary_ *= other.primary_;
    this->dual_ = this->primary_ * other.dual_ + this->dual_ * other.primary_;
    return *this;
}

template<typename qScalar_>  
Pose<qScalar_>& Pose<qScalar_>::normalize() noexcept{
    this->normalize();
}
    
}  // namespace Template

}  // namespace dq1