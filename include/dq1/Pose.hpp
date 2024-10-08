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
 *     \file include/dq1/UnitDualQuaternion.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>

namespace dq1
{

namespace Template
{

template <typename T>
inline T square(const T& x) {
    return x * x;
}

constexpr int PRINT_PRECISION = 12;
constexpr double FLOAT_OMIT_THRESHOLD = 0.00000001;
constexpr double FLOAT_ERROR_THRESHOLD = 0.0001;
constexpr bool VERY_VERBOSE = false;

template<typename qScalar>
class Quaternion;
template<typename qScalar>
class PureQuaternion;
template<typename qScalar>
class UnitQuaternion;
template<typename qScalar>
class UnitPureQuaternion;
template<typename qScalar>
class DualQuaternion;
template<typename qScalar>
class UnitDualQuaternion;

template<typename Scalar_>
void __real_should_be_zero(const std::string& calling_fn, Quaternion<Scalar_>& quaternion) {
    const double real = std::abs(quaternion.w());
    if (real > FLOAT_ERROR_THRESHOLD) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << " Error: " << calling_fn << " cannot purify a Quaternion: " << quaternion << " with a too-non-0 real " << std::to_string(real) << " \n";
        throw std::runtime_error(oss.str());
    }
    if (real > FLOAT_OMIT_THRESHOLD && VERY_VERBOSE){
        quaternion.data()[0] = 0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << " Warning: " << calling_fn << " purifie a Quaternion: " << quaternion << " with a real " << std::to_string(real) << " \n";
        std::cout << oss.str();
    }
}

template<typename Scalar_>
void __norm_should_be_one(const std::string& calling_fn, Quaternion<Scalar_>& quaternion){
    const double norm_err = std::abs(quaternion.norm() - 1);    
    if (norm_err > FLOAT_ERROR_THRESHOLD) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << " Error: " << calling_fn << " cannot normalize a Quaternion: " << quaternion << " with a too-non-1 norm " << std::to_string(quaternion.norm()) << " \n";
        throw std::runtime_error(oss.str());
    }
    if (norm_err > FLOAT_OMIT_THRESHOLD && VERY_VERBOSE){
        quaternion.normalize();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << " Warning: " << calling_fn << " normalize a Quaternion: " << quaternion << " with a norm " << std::to_string(quaternion.norm()) << " \n";
        std::cout << oss.str();
    }
}

template<typename Scalar, size_t size>
using Arr=std::array<Scalar, size>;
template<typename Scalar, size_t rows, size_t cols>
using Mat=std::array<std::array<Scalar, cols>, rows>;

template<typename qScalar>
class Quaternion{

using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    Arr4 _arr4;
public:
    // Constructors and Assignments    
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Quaternion()
    :_arr4{ 0, 0, 0, 0 } {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Quaternion(const Arr4& arr4)
    :_arr4{ arr4 } {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Quaternion(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0)
    :_arr4{ static_cast<qScalar>(w), static_cast<qScalar>(x), static_cast<qScalar>(y), static_cast<qScalar>(z) } {

    }

    // mutable operators

    Quaternion& operator+=(const Quaternion& other) noexcept {
        _arr4[0] += other.w();  
        _arr4[1] += other.x(); 
        _arr4[2] += other.y(); 
        _arr4[3] += other.z();
        return *this; 
    }
    Quaternion& operator-=(const Quaternion& other) noexcept {
        _arr4[0] -= other.w();  
        _arr4[1] -= other.x(); 
        _arr4[2] -= other.y(); 
        _arr4[3] -= other.z();
        return *this; 
    }
    Quaternion& operator*=(const Quaternion& other) noexcept {

        _arr4[0] = w()*other.w() - x()*other.x() - y()*other.y() - z()*other.z();  
        _arr4[1] = x()*other.w() + w()*other.x() - z()*other.y() + y()*other.z(); 
        _arr4[2] = y()*other.w() + z()*other.x() + w()*other.y() - x()*other.z(); 
        _arr4[3] = z()*other.w() - y()*other.x() + x()*other.y() + w()*other.z(); 

        return *this;
    }
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion&>
    operator*=(const Scalar_ scalar) noexcept {
        
        _arr4[0] *= scalar;  
        _arr4[1] *= scalar; 
        _arr4[2] *= scalar; 
        _arr4[3] *= scalar; 
        
        return *this;
    }
    Quaternion& normalize() {
        this->operator*=( 1 / norm() );
        return *this;
    }

    // const operators

    Quaternion operator+(const Quaternion& other) const noexcept {
        
        const qScalar w_ = w() + other.w();  // local variable cannot have same name as member functions
        const qScalar x_ = x() + other.x(); 
        const qScalar y_ = y() + other.y(); 
        const qScalar z_ = z() + other.z(); 

        return Quaternion(w_, x_, y_, z_);
    }
    Quaternion operator-(const Quaternion& other) const noexcept {
        
        const qScalar w_ = w() - other.w();  
        const qScalar x_ = x() - other.x(); 
        const qScalar y_ = y() - other.y(); 
        const qScalar z_ = z() - other.z(); 

        return Quaternion(w_, x_, y_, z_);
    }
    Quaternion operator*(const Quaternion& other) const noexcept{

        const qScalar w_ = w()*other.w() - x()*other.x() - y()*other.y() - z()*other.z();  
        const qScalar x_ = x()*other.w() + w()*other.x() - z()*other.y() + y()*other.z(); 
        const qScalar y_ = y()*other.w() + z()*other.x() + w()*other.y() - x()*other.z(); 
        const qScalar z_ = z()*other.w() - y()*other.x() + x()*other.y() + w()*other.z(); 

        return Quaternion(w_, x_, y_, z_);
    }
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
    operator*(const Scalar_ scalar) const noexcept {

        const qScalar w_ = w() * scalar;  
        const qScalar x_ = x() * scalar; 
        const qScalar y_ = y() * scalar; 
        const qScalar z_ = z() * scalar; 

        return Quaternion(w_, x_, y_, z_);
    }
    Quaternion operator-() const noexcept {

        const qScalar w_ = - w();  
        const qScalar x_ = - x(); 
        const qScalar y_ = - y(); 
        const qScalar z_ = - z(); 

        return Quaternion(w_, x_, y_, z_);
    }
    bool operator==(const Quaternion& other) const noexcept{
        return _arr4 == other._arr4;
    }
    bool operator!=(const Quaternion& other) const noexcept {
        return _arr4 != other._arr4;
    }

    // Service functions const

    Arr3 rotation_axis() const noexcept {
        if (rotation_angle() == 0){
            const Arr3 res = {0, 0, 1};
            return res;
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ) );
        const qScalar x_ = x() / vec3_norm;
        const qScalar y_ = y() / vec3_norm;
        const qScalar z_ = z() / vec3_norm;
        const Arr3 res = {x_, y_, z_};
        return res;
    }
    qScalar rotation_angle() const noexcept {
        return 2 * acos(w() / norm());
    }
    qScalar norm() const noexcept {
        return std::sqrt( square( w() ) + square( x() ) + square( y() ) + square( z() ));
    }
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
    pow(const Scalar_ index) const noexcept{
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / init_norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::pow(w(), index);
            return Quaternion(w);
        }
        const qScalar res_norm = std::pow(init_norm, index);
        const qScalar res_rotate_angle = init_rotate_angle * index;
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));

        const qScalar cos_ = cos(res_rotate_angle);
        const qScalar sin_ = sin(res_rotate_angle);

        const qScalar w_ = cos_ * res_norm;
        const qScalar x_ = sin_ * x() / vec3_norm * res_norm;
        const qScalar y_ = sin_ * y() / vec3_norm * res_norm;
        const qScalar z_ = sin_ * z() / vec3_norm * res_norm;
        return Quaternion(w_, x_, y_, z_);   
    }
    Quaternion conj() const noexcept {
        const qScalar w_ = w();  
        const qScalar x_ = - x(); 
        const qScalar y_ = - y(); 
        const qScalar z_ = - z(); 
        return Quaternion(w_, x_, y_, z_);  
    }
    Quaternion inv() const noexcept {
        const qScalar norm2 = square(norm());
        const qScalar w_ = w() / norm2;  
        const qScalar x_ = - x() / norm2; 
        const qScalar y_ = - y() / norm2; 
        const qScalar z_ = - z() / norm2; 
        return Quaternion(w_, x_, y_, z_);  
    }
    Quaternion log() const noexcept {
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / init_norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::log(w());
            return Quaternion(w);
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));

        const qScalar w_ = std::log(init_norm);
        const qScalar x_ = 0.5 * init_rotate_angle * x() / vec3_norm;
        const qScalar y_ = 0.5 * init_rotate_angle * y() / vec3_norm;
        const qScalar z_ = 0.5 * init_rotate_angle * z() / vec3_norm;
        return Quaternion(w_, x_, y_, z_);  
    }
    Quaternion exp() const noexcept {        
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::exp(w());
            return Quaternion(w);
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));
        
        const qScalar exp_ = std::exp(w());
        const qScalar cos_ = cos(vec3_norm);
        const qScalar sin_ = sin(vec3_norm);

        const qScalar w_ = exp_ * cos_;
        const qScalar x_ = exp_ * sin_ * x() / vec3_norm;
        const qScalar y_ = exp_ * sin_ * y() / vec3_norm;
        const qScalar z_ = exp_ * sin_ * z() / vec3_norm;
        return Quaternion(w_, x_, y_, z_); 
    }
    Quaternion normalized() const noexcept {
        return *this * (1 / norm()); 
    }
    Mat4 hamiplus() const noexcept {
        return Mat4 { { w(), -x(), -y(), -z() },
                      { x(),  w(), -z(),  y() },
                      { y(),  z(),  w(), -x() },
                      { z(), -y(),  x(),  w() } };
    }
    Mat4 haminus() const noexcept {
        return Mat4 { { w(), -x(), -y(), -z() },
                      { x(),  w(),  z(), -y() },
                      { y(), -z(),  w(),  x() },
                      { z(),  y(), -x(),  w() } };            
    }

    // Query const
    inline qScalar w() const noexcept {return _arr4[0];};
    inline qScalar x() const noexcept {return _arr4[1];}; 
    inline qScalar y() const noexcept {return _arr4[2];};
    inline qScalar z() const noexcept {return _arr4[3];};

    inline std::string to_string() const {    
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) << w() << " + " << x() << " î + " << y() << " ĵ + " << z() << " k̂";
        return oss.str();
    };
    inline qScalar* data() noexcept {return _arr4.data();}
    inline const qScalar* data() const noexcept {return _arr4.data();}
    inline const qScalar* vrep_data() const noexcept {
        const qScalar res[4] = {x(), y(), z(), w()};
        return res; 
    }

    // Friends "const"

    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& quat) {
        os << quat.to_string();  
        return os;
    }
    template<typename Scalar_, typename fScalar_> 
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
    operator*(const Scalar_ scalar, const Quaternion<fScalar_>& quat) noexcept {return quat * scalar;}

    // Defaults

        virtual ~Quaternion()=default;
                Quaternion(const Quaternion& other)=default;
                Quaternion(Quaternion&& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;
};

template<typename qScalar>
class PureQuaternion : public Quaternion<qScalar> 
{ 
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
public:
    // Constructors and Assignments
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureQuaternion(const Arr3 arr3)
    : Quaternion<qScalar>(0, arr3[0], arr3[1], arr3[2]) {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureQuaternion(const qScalar x, const qScalar y, const qScalar z)
    :Quaternion<qScalar>(0, static_cast<qScalar>(x), static_cast<qScalar>(y), static_cast<qScalar>(z)) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureQuaternion(const Quaternion<qScalar>& quat)
    : Quaternion<qScalar>(quat) {
        __real_should_be_zero("explicit PureQuaternion(const Quaternion&)", *this);
    }

    PureQuaternion& operator=(const Quaternion<qScalar>& quat) {
        this->operator=(quat);
        __real_should_be_zero("PureQuaternion& operator=(const Quaternion&)", *this);
    }

    inline const qScalar* data() const noexcept {
        return this->_arr4.data()+1;
    }

    // Defaults

            virtual ~PureQuaternion()=default;
                    PureQuaternion()=default;
                    PureQuaternion(const PureQuaternion& other)=default;
                    PureQuaternion(PureQuaternion& other)=default;
    PureQuaternion& operator=(const PureQuaternion& other)=default;
    PureQuaternion& operator=(PureQuaternion&& other)=default;
};

template<typename qScalar>
class UnitQuaternion : public Quaternion<qScalar>
{
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
public:
    // Constructors and Assignments
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitQuaternion()
    :Quaternion<qScalar>(1) {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitQuaternion(const Arr4& arr4)
    :Quaternion<qScalar>( arr4 ) {
        __norm_should_be_one("explicit UnitQuaternion(const std::array&)", *this);
    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitQuaternion(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0)
    :Quaternion<qScalar>( w, x, y, z ) {
        __norm_should_be_one("explicit UnitQuaternion(const qScalar, const qScalar, const qScalar, const qScalar)", *this);
    }

    explicit UnitQuaternion(const Quaternion<qScalar>& quat): Quaternion<qScalar>(quat) {
        __norm_should_be_one("explicit UnitQuaternion(const Quaternion&)", *this);
    }
    UnitQuaternion& operator=(const Quaternion<qScalar>& quat) {
        this->operator=(quat);
        __norm_should_be_one("UnitQuaternion& operator=(const Quaternion&)", *this);
    }

    // Defaults
            virtual ~UnitQuaternion()=default;
                    UnitQuaternion(const UnitQuaternion& other)=default;
                    UnitQuaternion(UnitQuaternion& other)=default;
    UnitQuaternion& operator=(const UnitQuaternion& other)=default;
    UnitQuaternion& operator=(UnitQuaternion&& other)=default;
};

template<typename qScalar>
class UnitPureQuaternion : public Quaternion<qScalar>
{ // UnitPureQuaternion
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureQuaternion(const Arr3 arr3)
    : Quaternion<qScalar>( 0, arr3[0], arr3[1], arr3[2] ){
        __norm_should_be_one("explicit UnitPureQuaternion(const std::array)", *this);
    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureQuaternion(const qScalar x, const qScalar y, const qScalar z)
    : Quaternion<qScalar>(0, x, y, z){
        __norm_should_be_one("explicit UnitPureQuaternion(const qScalar, const qScalar, const qScalar)", *this);
    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureQuaternion(const Quaternion<qScalar>& quat)
    : Quaternion<qScalar>(quat) {
        __real_should_be_zero("explicit UnitPureQuaternion(const Quaternion&)", *this);
        __norm_should_be_one("explicit UnitPureQuaternion(const Quaternion&)", *this);
    }

    UnitPureQuaternion& operator=(const Quaternion<qScalar>& quat) {
        this->operator=(quat);
        __real_should_be_zero("UnitPureQuaternion& operator=(const Quaternion&)", *this);
        __norm_should_be_one("UnitPureQuaternion& operator=(const Quaternion&)", *this);
    }    
    
    inline const qScalar* data() const noexcept {
        return this->_arr4.data()+1;
    }

    // Delete
    UnitPureQuaternion()=delete;
    // Default
                virtual ~UnitPureQuaternion()=default;
                        UnitPureQuaternion(const UnitPureQuaternion& other)=default;
                        UnitPureQuaternion(UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(const UnitPureQuaternion& other)=default;
    UnitPureQuaternion& operator=(UnitPureQuaternion&& other)=default;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename qScalar>
class DualQuaternion{
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:

    Quaternion<qScalar> _primary;
    Quaternion<qScalar> _dual;
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit DualQuaternion()
    : _primary(1), _dual() {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    DualQuaternion(const Quaternion<qScalar>& primary, const Quaternion<qScalar>& dual=Quaternion<qScalar>())
    : _primary(primary), _dual(dual) {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit DualQuaternion(const Arr8& arr8)
    : _primary(arr8[0], arr8[1], arr8[2], arr8[3]), _dual(arr8[4], arr8[5], arr8[6], arr8[7]) {

    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit DualQuaternion(const qScalar h0, const qScalar h1=0, const qScalar h2=0, const qScalar h3=0, const qScalar h4=0, const qScalar h5=0, const qScalar h6=0, const qScalar h7=0)
    : _primary(h0, h1, h2, h3), _dual(h4, h5, h6, h7) {

    }

    // mutable operators    

    DualQuaternion& operator+=(const DualQuaternion& other) noexcept {
        _primary += other._primary;
        _dual += other._dual; 
        return *this;
    }
    DualQuaternion& operator-=(const DualQuaternion& other) noexcept {
        _primary -= other._primary;
        _dual -= other._dual;   
        return *this;
    }
    DualQuaternion& operator*=(const DualQuaternion& other) noexcept {
        _primary *= other._primary;
        _dual = _primary * other._dual + _dual * other._primary;
        return *this;
    }
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion&> 
    operator*=(const Scalar_ scalar) noexcept {
        _primary *= scalar;
        _dual *= scalar;
        return *this;
    }
    DualQuaternion& normalize() noexcept {
        const qScalar norm = _primary.norm();
        _primary *= ( 1 / norm );
        _dual *= ( 1 / norm );
        return *this;
    }

    // Const operator    
    
    DualQuaternion operator+(const DualQuaternion& other) const noexcept {
        return DualQuaternion( _primary + other._primary, _dual + other._dual );
    }
    DualQuaternion operator-(const DualQuaternion& other) const noexcept {
        return DualQuaternion( _primary - other._primary, _dual - other._dual );
    }
    DualQuaternion operator*(const DualQuaternion& other) const noexcept {
        return DualQuaternion( _primary * other._primary, _primary * other._dual + _dual * other._primary);
    }
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    operator*(const Scalar_ scalar) const noexcept {
        return DualQuaternion( _primary * scalar, _dual * scalar );
    }
    DualQuaternion operator-() const noexcept {
        return DualQuaternion( -_primary, -_dual );
    }
    bool operator==(const DualQuaternion& other) const noexcept {
        return _primary == other._primary && _dual == other._dual;
    }
    bool operator!=(const DualQuaternion& other) const noexcept {
        return _primary != other._primary || _dual != other._dual;
    }

    // service functions

    DualQuaternion norm() const noexcept {
        const qScalar primary_norm = _primary.norm();
        if (primary_norm == 0) 
            return DualQuaternion();
        const qScalar res_dual_norm = ( _primary.w() * _dual.w() + _primary.x() * _dual.x() + _primary.y() * _dual.y() + _primary.z() * _dual.z() ) / primary_norm;

        return DualQuaternion(Quaternion<qScalar>(primary_norm), Quaternion<qScalar>(res_dual_norm));
    }
    DualQuaternion conj() const noexcept {
        return DualQuaternion(_primary.conj(), _dual.conj());
    }
    DualQuaternion normalized() const noexcept {
        return *this * ( 1 / _primary.norm() );
    }
    Mat8 hamiplus() const noexcept {
        const Mat4 pri_hami = _primary.hamiplus();
        const Mat4 dua_hami = _dual.hamiplus();
        Mat8 res;
        for (int i=0; i<4; ++i) {
            std::copy(pri_hami[i].begin(), pri_hami[i].end(), res[i].begin());
        }
        for (int i=0; i<4; ++i) {
            for (int j=4; j<8; ++j) {
                res[i][j] = 0;
            }
        }
        for (int i=4; i<8; ++i) {
            std::copy(dua_hami[i].begin(), dua_hami[i].end(), res[i].begin());
        }
        for (int i=4; i<8; ++i) {
            std::copy(pri_hami[i].begin(), pri_hami[i].end(), res[i].begin()+4);
        }
        return res;
    }
    Mat8 haminus() const noexcept {        
        const Mat4 pri_hami = _primary.haminus();
        const Mat4 dua_hami = _dual.haminus();
        Mat8 res;
        for (int i=0; i<4; ++i) {
            std::copy(pri_hami[i].begin(), pri_hami[i].end(), res[i].begin());
        }
        for (int i=0; i<4; ++i) {
            for (int j=4; j<8; ++j) {
                res[i][j] = 0;
            }
        }
        for (int i=4; i<8; ++i) {
            std::copy(dua_hami[i].begin(), dua_hami[i].end(), res[i].begin());
        }
        for (int i=4; i<8; ++i) {
            std::copy(pri_hami[i].begin(), pri_hami[i].end(), res[i].begin()+4);
        }
        return res;
    }
    inline Quaternion<qScalar>& primary() noexcept {
        return _primary;
    }
    inline Quaternion<qScalar>& dual() noexcept {
        return _dual;
    }
    inline const Quaternion<qScalar>& primary() const noexcept {
        return _primary;
    }
    inline const Quaternion<qScalar>& dual() const noexcept {
        return _dual;
    }
    inline const qScalar* data() const noexcept {
        const Arr8 res = { _primary.w(), _primary.x(), _primary.y(), _primary.z(), _dual.w(), _dual.x(), _dual.y(), _dual.z() };
        return res;
    }
    inline std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) <<  _primary << " + " << " ϵ ( " << _dual << " )";
        return oss.str();
    }
    
    // Friend functions

    template<typename fScalar_>
    friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<fScalar_>& dq) {
        os << dq.to_string();  
        return os;
    }
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
    operator*(const Scalar_ scalar, const DualQuaternion<fScalar_>& dq) noexcept {return dq * scalar;}
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const Quaternion<fScalar_>& quat, const DualQuaternion<fScalar_>& dq) noexcept {
        return DualQuaternion( quat * dq._primary, quat * dq._dual );
    }
    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dq, const Quaternion<fScalar_>& quat) noexcept {
        return DualQuaternion( dq._primary * quat, dq._dual * quat );
    }
    
    // Default
            virtual ~DualQuaternion()=default;
                    DualQuaternion(const DualQuaternion& dq)=default;
                    DualQuaternion(DualQuaternion&& dq)=default;
    DualQuaternion& operator=(const DualQuaternion& dq)=default;
    DualQuaternion& operator=(DualQuaternion&& dq)=default;

private:
    // not inplemented
    DualQuaternion inv() const noexcept {}
    DualQuaternion log() const noexcept {}
    DualQuaternion exp() const noexcept {}
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    pow(const Scalar_ index) const noexcept {}
};


template<typename qScalar>
class UnitDualQuaternion: public DualQuaternion<qScalar>{
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuaternion()
    : DualQuaternion<qScalar>(1){

    }    
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuaternion(const Quaternion<qScalar>& primary, const Quaternion<qScalar>& dual)
    : DualQuaternion<qScalar>(primary, dual){
        __norm_should_be_one("explicit UnitDualQuaternion(const Quaternion&, const Quaternion&)", this->primary());
    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuaternion(const DualQuaternion<qScalar>& dq)
    : DualQuaternion<qScalar>(dq) {
        __norm_should_be_one("explicit UnitDualQuaternion(const DualQuaternion&)", this->primary());
    }
    UnitDualQuaternion& operator=(const DualQuaternion<qScalar>& dq) {
        DualQuaternion<qScalar>::operator=(dq);
        __norm_should_be_one("UnitDualQuaternion& operator=(const DualQuaternion&)", this->primary());
    }

    // Default
                virtual ~UnitDualQuaternion()=default;
                        UnitDualQuaternion(const UnitDualQuaternion& dq)=default;
                        UnitDualQuaternion(UnitDualQuaternion&& dq)=default;
    UnitDualQuaternion& operator=(const UnitDualQuaternion& dq)=default;
    UnitDualQuaternion& operator=(UnitDualQuaternion&& dq)=default;
};



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename qScalar>
class Rotation;
template<typename qScalar>
class Translation;
template<typename qScalar>
class Pose;
template<typename qScalar>
class UnitAxis;

template<typename qScalar>
class Rotation {
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    UnitQuaternion<qScalar> _quat; 
public:
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Rotation()
    : _quat() {// default UnitQuaternion is set to 1

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Rotation(const Quaternion<qScalar>& quat)
    : _quat(quat) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    static Rotation build_from(const UnitAxis& rotate_axis, const qScalar rotate_angle) {
        const qScalar w_ = cos(0.5 * rotate_angle);
        const qScalar sin_ = sin(0.5 * rotate_angle);
        const qScalar x_ = rotate_axis.x() * sin_;
        const qScalar y_ = rotate_axis.y() * sin_;
        const qScalar z_ = rotate_axis.z() * sin_;

        const UnitQuaternion<qScalar> uquat(w_, x_, y_, z_);
        return Rotation(uquat);
    }

    static Rotation build_from(const UnitAxis& uaxis1, const UnitAxis& uaxis2) {
        if ()
    }    
    
    static Rotation build_from(const Rotation& rotation1, const Rotation& rotation2) {
        if ()
    }

    inline Quaternion<qScalar> quaternion() const noexcept { return _quat; }

    // Default
        virtual ~Rotation()=default;
                Rotation(const Rotation&)=default;
                Rotation(Rotation&&)=default;
    Rotation& operator=(const Rotation&)=default;
    Rotation& operator=(Rotation&&)=default;
};


template<typename qScalar>
class Translation {
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    PureQuaternion<qScalar> _quat; 
public:

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const qScalar x, const qScalar y, const qScalar z)
    : _quat(x, y, z) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const Quaternion<qScalar>& quat)
    : _quat(quat) {

    }

    inline Translation& active_rotate(const Rotation& rotation) {
        Quaternion<qScalar> rotation_quat = rotation.quaternion();
        _quat = rotation_quat * _quat * rotation_quat.conj();
        return &this;
    }    
    
    inline Translation& passive_rotate(const Rotation& rotation) {
        Quaternion<qScalar> rotation_quat = rotation.quaternion();
        _quat = rotation_quat.conj() * _quat * rotation_quat;
        return &this;
    }

    inline Translation active_rotated(const Rotation& rotation) const {
        Quaternion<qScalar> rotation_quat = rotation.quaternion();
        Quaternion<qScalar> res = rotation_quat * _quat * rotation_quat.conj();
        return Translation(res);
    }    
    
    inline Translation passive_rotated(const Rotation& rotation) const {
        Quaternion<qScalar> rotation_quat = rotation.quaternion();
        Quaternion<qScalar> res = rotation_quat.conj() * _quat * rotation_quat;
        return Translation(res);
    }

    inline Quaternion<qScalar> quaternion() const noexcept { return _quat; }

    // Default
        virtual ~Translation()=default;
                Translation()=default; // {0,0,0}
                Translation(const Translation&)=default;
                Translation(Translation&&)=default;
    Translation& operator=(const Translation&)=default;
    Translation& operator=(Translation&&)=default;
};    
}  // namespace Pose

}  // namespace dq1
