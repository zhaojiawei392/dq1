#include "Quaternion.h"
#include <iomanip>

namespace dq1
{
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion()
    :data_(vec4<Scalar_>::Zero()){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Tvec3<Scalar_>& xyz)  :data_((vec4<Scalar_>() << 0, xyz).finished()){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(Tvec3<Scalar_>&& xyz)       :data_((vec4<Scalar_>() << 0, xyz).finished()){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Tvec4<Scalar_>& wxyz) :data_(wxyz){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(Tvec4<Scalar_>&& wxyz)      :data_(std::move(wxyz)){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Scalar_& w, const Scalar_& x, const Scalar_& y, const Scalar_& z) :data_((vec4<Scalar_>() << w, x, y, z).finished()){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(Scalar_&& w, Scalar_&& x, Scalar_&& y, Scalar_&& z)                     :data_((vec4<Scalar_>() << w, x, y, z).finished()){}

template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::w() const noexcept {return data_[0];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::x() const noexcept {return data_[1];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::y() const noexcept {return data_[2];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::z() const noexcept {return data_[3];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::norm() const noexcept {return data_.norm();}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::conj() const noexcept {return Quaternion( (Tvec4<Scalar_>() << data_[0], -data_[1], -data_[2], -data_[3]).finished() ); }
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::inv() const noexcept {return conj() / norm();}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::ln() const noexcept {}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::exp() const noexcept {}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::pow(const double& a) const noexcept {}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::normalized() const noexcept {return (*this) / norm();}
template<typename Scalar_>
std::string Quaternion<Scalar_>::to_string() const noexcept {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6); // Set fixed-point notation and precision
    oss << data_[0] << " + " << data_[1] << " î + " << data_[2] << " ĵ + " << data_[3] << " k̂ ";
    return oss.str();
}

template<typename Scalar_>
Tvec3<Scalar_> Quaternion<Scalar_>::vec3() const noexcept {return data_.tail<3>();}
template<typename Scalar_>
Tvec4<Scalar_> Quaternion<Scalar_>::vec4() const noexcept {return data_;}

template<typename Scalar_>
Tmat4<Scalar_> Quaternion<Scalar_>::hamiplus() const noexcept 
{
    const Scalar_& w,x,y,z = data_[0], data_[1], data_[2], data_[3];
    return (Tmat4<Scalar_>() << w, -x, -y, -z,
                                x,  w, -z,  y,
                                y,  z,  w, -x,
                                z, -y,  x,  w).finished();
}
template<typename Scalar_>
Tmat4<Scalar_> Quaternion<Scalar_>::haminus() const noexcept 
{
    const Scalar_& w,x,y,z = data_[0], data_[1], data_[2], data_[3];
    return (Tmat4<Scalar_>() << w, -x, -y, -z,
                                x,  w,  z, -y,
                                y, -z,  w,  x,
                                z,  y, -x,  w).finished();
}

template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(data_ + other.data_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(Quaternion&& other) const noexcept {other.data_ += data_; return std::move(other);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(data_ - other.data_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(Quaternion&& other) const noexcept {return Quaternion<Scalar_>(data_ - other.data_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(other.haminus() * data_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(Quaternion&& other) const noexcept {return Quaternion<Scalar_>(other.haminus() * data_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(data_ * scalar);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(Scalar_&& scalar) const noexcept {return Quaternion<Scalar_>(data_ * scalar);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator/(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(data_ / scalar);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator/(Scalar_&& scalar) const noexcept {return Quaternion<Scalar_>(data_ / scalar);}

template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator+=(const Quaternion& other) noexcept {data_ += other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator+=(Quaternion&& other) noexcept {data_ += other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator-=(const Quaternion& other) noexcept {data_ -= other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator-=(Quaternion&& other) noexcept {data_ -= other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Quaternion& other) noexcept {data_ = hamiplus() * other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(Quaternion&& other) noexcept {data_ = hamiplus() * other.data_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Scalar_& scalar) noexcept {data_ *= scalar; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(Scalar_&& scalar) noexcept {data_ *= scalar; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator/=(const Scalar_& scalar) noexcept {data_ /= scalar; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator/=(Scalar_&& scalar) noexcept {data_ /= scalar; return *this;}

template<typename Scalar_>
bool Quaternion<Scalar_>::operator==(const Quaternion& other) const noexcept {return data_ == other.data_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator==(Quaternion&& other) const noexcept {return data_ == other.data_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator!=(const Quaternion& other) const noexcept {return data_ != other.data_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator!=(Quaternion&& other) const noexcept {return data_ != other.data_;}

}