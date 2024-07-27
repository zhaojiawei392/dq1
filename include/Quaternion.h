#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
template<typename Scalar_, int size>
using Tvec=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Tvec3=Eigen::Matrix<Scalar_, 3, 1>;
template<typename Scalar_>
using Tvec4=Eigen::Matrix<Scalar_, 4, 1>;

template<typename Scalar_, int rows_, int cols_>
using Tmat=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Tmat3=Eigen::Matrix<Scalar_, 3, 3>;
template<typename Scalar_>
using Tmat4=Eigen::Matrix<Scalar_, 4, 4>;

template<typename Scalar_>
class Quaternion{
protected:
    Tvec4<Scalar_> data_;
public:
    // Special members
    Quaternion();
    Quaternion(const Quaternion& other)=default;
    Quaternion(Quaternion& other)=default;
    Quaternion& operator=(const Quaternion& other)=default;
    Quaternion& operator=(Quaternion&& other)=default;
    // Destructor, always identified as virtual, otherwise the derived class resources may not be released correctly
    virtual ~Quaternion()=default;
    
    // Customized constructors
    explicit Quaternion(const Tvec3<Scalar_>& xyz);
    explicit Quaternion(Tvec3<Scalar_>&& xyz);
    explicit Quaternion(const Tvec4<Scalar_>& wxyz);
    explicit Quaternion(Tvec4<Scalar_>&& wxyz);
    explicit Quaternion(const Scalar_& w, const Scalar_& x, const Scalar_& y, const Scalar_& z);
    explicit Quaternion(Scalar_&& w, Scalar_&& x, Scalar_&& y, Scalar_&& z);

    Scalar_ w() const noexcept;
    Scalar_ x() const noexcept;
    Scalar_ y() const noexcept;
    Scalar_ z() const noexcept;

    Scalar_ norm() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
    Quaternion pow(const double& a) const noexcept;
    Quaternion normalized() const noexcept;
    std::string to_string() const noexcept;

    Tvec3<Scalar_> vec3() const noexcept;
    Tvec4<Scalar_> vec4() const noexcept;

    Tmat4<Scalar_> hamiplus() const noexcept;
    Tmat4<Scalar_> haminus() const noexcept;

    /**
     * @brief Calculates the sum of this quaternion and argument quaternion.
     * This function takes one quaternion and returns the sum of this quaternion and the argument quaternion.
     * @param quaternion The argument quaternion.
     * @return the sum of this quaternion and argument quaternion.
     */
    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator+(Quaternion&& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    Quaternion operator-(Quaternion&& other) const noexcept;
    Quaternion operator*(const Quaternion& other) const noexcept;
    Quaternion operator*(Quaternion&& other) const noexcept;
    Quaternion operator*(const Scalar_& scalar) const noexcept;
    Quaternion operator*(Scalar_&& scalar) const noexcept;
    Quaternion operator/(const Scalar_& scalar) const noexcept;
    Quaternion operator/(Scalar_&& scalar) const noexcept;

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator+=(Quaternion&& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator-=(Quaternion&& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    Quaternion& operator*=(Quaternion&& other) noexcept;
    Quaternion& operator*=(const Scalar_& scalar) noexcept;
    Quaternion& operator*=(Scalar_&& scalar) noexcept;
    Quaternion& operator/=(const Scalar_& scalar) noexcept;
    Quaternion& operator/=(Scalar_&& scalar) noexcept;

    bool operator==(const Quaternion& other) const noexcept;
    bool operator==(Quaternion&& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 
    bool operator!=(Quaternion&& other) const noexcept; 


};

template<typename Scalar_>
class PureQuaternion : public Quaternion{
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
    explicit PureQuaternion(Quaternion<Scalar_>& other);
    explicit PureQuaternion& operator=(const Quaternion<Scalar_>& other);
    explicit PureQuaternion& operator=(Quaternion<Scalar_>&& other);
    
    // Customized constructors
    explicit PureQuaternion(const Tvec3<Scalar_>& xyz);
    explicit PureQuaternion(Tvec3<Scalar_>&& xyz);
    explicit PureQuaternion(const Scalar_& x, const Scalar_& y, const Scalar_& z);
    explicit PureQuaternion(Scalar_&& x, Scalar_&& y, Scalar_&& z);

    


};

template<typename Scalar_>
class UnitQuaternion : public Quaternion{

};

template<typename Scalar_>
class UnitPureQuaternion : virtual public PureQuaternion, virtual public UnitQuaternion{
    
};

}