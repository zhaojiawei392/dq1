#include "Quaternion.h"
#include "Math.h"
#include <iomanip>
#include <cmath>

namespace dq1
{
// Explicit instantiation
template class Quaternion<double>;
template class Quaternion<float>;

template<typename Scalar_>
Quaternion<Scalar_>::Quaternion()
    :vals_(Tvec4<Scalar_>::Zero()){}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Tvecx<Scalar_>& vec)
{
    switch(vec.size())
    {
    case 3:
        vals_ << 0, vec;
        break;
    case 4:
        vals_ << vec;
        break;
    default:
        throw std::range_error("Error of constructing a Quaternion with a vector size of " + std::to_string(vec.size()) + ", which should be 3 or 4.");
    }
}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(Tvecx<Scalar_>&& vec)
{
    switch(vec.size())
    {
    case 3:
        vals_ << 0, vec;
        break;
    case 4:
        vals_ = std::move(vec);
        break;
    default:
        throw std::range_error("Error of constructing a Quaternion with a vector size of " + std::to_string(vec.size()) + ", which should be 3 or 4.");
    }
}
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Scalar_& w, const Scalar_& x, const Scalar_& y, const Scalar_& z) :vals_((Tvec4<Scalar_>() << w, x, y, z).finished()){}

template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::w() const noexcept {return vals_[0];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::x() const noexcept {return vals_[1];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::y() const noexcept {return vals_[2];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::z() const noexcept {return vals_[3];}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::norm() const noexcept {return vals_.norm();}
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::rot_angle() const noexcept {return 2 * acos(vals_[0] / norm());}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::rot_axis() const noexcept {return Quaternion<Scalar_>( vec3() / (norm() * sin(0.5 * rot_angle())));}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::conj() const noexcept {return Quaternion( (Tvec4<Scalar_>() << vals_[0], -vals_[1], -vals_[2], -vals_[3]).finished() ); }
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::inv() const noexcept {return conj() / square(norm());}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::ln() const noexcept {return *this;} // pass
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::exp() const noexcept {return *this;} //pass
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::pow(const double& a) const noexcept {return *this;} //pass
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::normalized() const noexcept {return (*this) / norm();}
template<typename Scalar_>
std::string Quaternion<Scalar_>::to_string() const noexcept {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << vals_[0] << " + " << vals_[1] << " î + " << vals_[2] << " ĵ + " << vals_[3] << " k̂ ";
    return oss.str();
}

template<typename Scalar_>
Tvec3<Scalar_> Quaternion<Scalar_>::vec3() const noexcept {return vals_.template tail<3>();}
template<typename Scalar_>
Tvec4<Scalar_> Quaternion<Scalar_>::vec4() const noexcept {return vals_;}

template<typename Scalar_>
Tmat4<Scalar_> Quaternion<Scalar_>::hamiplus() const noexcept 
{      
    const Scalar_ w = vals_[0];
    const Scalar_ x = vals_[1];
    const Scalar_ y = vals_[2];
    const Scalar_ z = vals_[3];
    return (Tmat4<Scalar_>() << w, -x, -y, -z,
                                x,  w, -z,  y,
                                y,  z,  w, -x,
                                z, -y,  x,  w).finished();
}
template<typename Scalar_>
Tmat4<Scalar_> Quaternion<Scalar_>::haminus() const noexcept 
{
    const Scalar_ w = vals_[0];
    const Scalar_ x = vals_[1];
    const Scalar_ y = vals_[2];
    const Scalar_ z = vals_[3];
    return (Tmat4<Scalar_>() << w, -x, -y, -z,
                                x,  w,  z, -y,
                                y, -z,  w,  x,
                                z,  y, -x,  w).finished();
}

template<typename Scalar_>
Quaternion<Scalar_>::operator std::string() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << vals_[0] << " + " << vals_[1] << " î + " << vals_[2] << " ĵ + " << vals_[3] << " k̂ ";
    return oss.str();
}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(vals_ + other.vals_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(Quaternion&& other) const noexcept {other.vals_ += vals_; return std::move(other);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(vals_ - other.vals_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(Quaternion&& other) const noexcept {other.vals_ = vals_ - other.vals_; return std::move(other);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(other.haminus() * vals_);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(Quaternion&& other) const noexcept {other.vals_ = hamiplus() * other.vals_; return std::move(other);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(vals_ * scalar);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator/(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(vals_ / scalar);}
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-() const noexcept {return Quaternion<Scalar_>(-vals_);}

template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator+=(const Quaternion& other) noexcept {vals_ += other.vals_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator-=(const Quaternion& other) noexcept {vals_ -= other.vals_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Quaternion& other) noexcept {vals_ = hamiplus() * other.vals_; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Scalar_& scalar) noexcept {vals_ *= scalar; return *this;}
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator/=(const Scalar_& scalar) noexcept {vals_ /= scalar; return *this;}

template<typename Scalar_>
bool Quaternion<Scalar_>::operator==(const Quaternion& other) const noexcept {return vals_ == other.vals_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator==(Quaternion&& other) const noexcept {return vals_ == other.vals_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator!=(const Quaternion& other) const noexcept {return vals_ != other.vals_;}
template<typename Scalar_>
bool Quaternion<Scalar_>::operator!=(Quaternion&& other) const noexcept {return vals_ != other.vals_;}

}