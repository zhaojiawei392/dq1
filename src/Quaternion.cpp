#include "Quaternion.h"
#include "Math.h"
#include <iomanip>
#include <cmath>

namespace dq1
{

// Friend definitions
template<typename osScalar_>
std::ostream& operator<<(std::ostream& os, const Quaternion<osScalar_>& q){
    os << q.operator std::string();  
return os;
} 

// Explicit instantiation
template class Quaternion<double>;
template class Quaternion<float>;
template std::ostream& operator<< (std::ostream& os, const Quaternion<double>& q);
template std::ostream& operator<< (std::ostream& os, const Quaternion<float>& q);

/**
 * @brief Default constructor that initializes a Quaternion to zero.
 * 
 * This constructor initializes the quaternion with all components set to zero.
 * It uses the `Tvec4<Scalar_>::Zero()` function to create a 4-element vector 
 * with all elements set to zero, which is then used to initialize the quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion()
    :vals_(Tvec4<Scalar_>::Zero()){}
/**
 * @brief Constructs a Quaternion from a vector.
 * 
 * This constructor initializes a quaternion using the elements of a given vector, passed as an lvalue.
 * The vector can have either 3 or 4 elements:
 * - If the vector has 3 elements, the quaternion is initialized as (0, vec[0], vec[1], vec[2]),
 *   where 0 is the scalar part, and the vector elements are the imaginary parts.
 * - If the vector has 4 elements, the quaternion is initialized directly with these elements
 *   as (vec[0], vec[1], vec[2], vec[3]).
 * 
 * @param vec The lvalue reference to the vector used to initialize the quaternion. 
 *            The size of the vector should be 3 or 4.
 * @throws std::range_error if the size of the vector is not 3 or 4.
 */
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
/**
 * @brief Constructs a Quaternion from a vector.
 * 
 * This constructor initializes a quaternion using the elements of a given vector, passed as an rvalue.
 * The vector can have 4 elements:
 * - If the vector has 3 elements, the quaternion is initialized as (0, vec[0], vec[1], vec[2]),
 *   where 0 is the scalar part, and the vector elements are the imaginary parts.
 * - If the vector has 4 elements, the quaternion is initialized directly with these elements 
 *   using move semantics to transfer ownership of the data.
 * 
 * @param vec The rvalue reference to the vector used to initialize the quaternion.
 *            The size of the vector should be 3 or 4.
 * @throws std::range_error if the size of the vector is not 3 or 4.
 */
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
        throw std::range_error("Error in constructor Quaternion(Tvecx<Scalar_>&& vec) with a vector size of " + std::to_string(vec.size()) + ", which should be 3 or 4.");
    }
}
/**
 * @brief Constructs a Quaternion from a scalar and a 3-element vector.
 * 
 * This constructor initializes a quaternion using a scalar part and a 3-element vector.
 * The scalar part is used as the real component of the quaternion, while the vector elements
 * are used as the imaginary components.
 * 
 * @param w The scalar part of the quaternion.
 * @param vec The 3-element vector used to initialize the imaginary parts of the quaternion.
 * @throws std::range_error if the size of the vector is not 3.
 */
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Scalar_& w, const Tvecx<Scalar_>& vec)
{
    if (vec.size() != 3)
        throw std::range_error("Error in constructor Quaternion(const Scalar_& w, const Tvecx<Scalar_>& vec) with a vector size of " + std::to_string(vec.size()) + ", which should be 3.");
    vals_ << w, vec;
}
/**
 * @brief Constructs a Quaternion from four scalar values.
 * 
 * This constructor initializes a quaternion using four scalar values, representing
 * the real part (w) and the three imaginary parts (x, y, z) of the quaternion.
 * 
 * @param w The scalar part of the quaternion.
 * @param x The first imaginary part of the quaternion.
 * @param y The second imaginary part of the quaternion.
 * @param z The third imaginary part of the quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Scalar_& w, const Scalar_& x, const Scalar_& y, const Scalar_& z) :vals_((Tvec4<Scalar_>() << w, x, y, z).finished()){}
/**
 * @brief Constructs a Quaternion from a norm, rotation angle, and a 3-element rotation vector.
 * 
 * This constructor initializes a quaternion representing a rotation. It uses the provided norm,
 * rotation angle, and a 3-element rotation vector. The quaternion is constructed based on the
 * formula for a unit quaternion representing a rotation.
 * 
 * @param norm The norm of the quaternion.
 * @param rotation_angle The angle of rotation in radians.
 * @param rotation_vec The 3-element vector representing the axis of rotation.
 * @throws std::range_error if the size of the rotation vector is not 3.
 */
template<typename Scalar_>
Quaternion<Scalar_>::Quaternion(const Scalar_& norm, const Scalar_& rotation_angle, const Tvecx<Scalar_>& rotation_vec)
{
    if (rotation_vec.size() != 3)
        throw std::range_error("Error in constructor Quaternion(const Scalar_& norm, const Scalar_& rotation_angle, const Tvecx<Scalar_>& rotation_axis) with a rotation_axis size of " + std::to_string(rotation_vec.size()) + ", which should be 3.");
    vals_ << cos(0.5 * rotation_angle), 
             sin(0.5 * rotation_angle) * rotation_vec;
    vals_ *= norm;
}
/**
 * @brief Returns the scalar (real) part of the quaternion.
 * 
 * This function retrieves the first element of the quaternion, which represents
 * the scalar (real) part.
 * 
 * @return The scalar part of the quaternion.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::w() const noexcept {return vals_[0];}
/**
 * @brief Returns the first imaginary part of the quaternion.
 * 
 * This function retrieves the second element of the quaternion, which represents
 * the first imaginary part (x) of the quaternion.
 * 
 * @return The first imaginary part of the quaternion.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::x() const noexcept {return vals_[1];}
/**
 * @brief Returns the second imaginary part of the quaternion.
 * 
 * This function retrieves the third element of the quaternion, which represents
 * the second imaginary part (y) of the quaternion.
 * 
 * @return The second imaginary part of the quaternion.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::y() const noexcept {return vals_[2];}
/**
 * @brief Returns the third imaginary part of the quaternion.
 * 
 * This function retrieves the fourth element of the quaternion, which represents
 * the third imaginary part (z) of the quaternion.
 * 
 * @return The third imaginary part of the quaternion.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::z() const noexcept {return vals_[3];}
/**
 * @brief Calculates the norm (magnitude) of the quaternion.
 * 
 * This function calculates the norm of the quaternion, which is the Euclidean
 * norm of the underlying 4-element vector.
 * 
 * @return The norm of the quaternion.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::norm() const noexcept {return vals_.norm();}
/**
 * @brief Calculates the rotation angle represented by the quaternion.
 * 
 * This function calculates the rotation angle using the scalar part of the quaternion.
 * The formula used is 2 * acos(w / norm()), where w is the scalar part.
 * 
 * @return The rotation angle in radians.
 */
template<typename Scalar_>
Scalar_ Quaternion<Scalar_>::rotation_angle() const noexcept {return 2 * acos(vals_[0] / norm());}
/**
 * @brief Calculates the rotation axis represented by the quaternion.
 * 
 * This function calculates the rotation axis, which is returned as a quaternion
 * with only the imaginary parts set (x, y, z) and the scalar part set to zero.
 * 
 * @return A quaternion representing the rotation axis.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::rotation_axis() const noexcept {return Quaternion<Scalar_>( rotation_vec() );}
/**
 * @brief Calculates the rotation vector (normalized imaginary part) of the quaternion.
 * 
 * This function retrieves the imaginary parts of the quaternion and normalizes them.
 * If the norm of the imaginary part is zero, it returns a default vector (0, 0, 1).
 * 
 * @return The normalized rotation vector.
 */
template<typename Scalar_>
Tvec3<Scalar_> Quaternion<Scalar_>::rotation_vec() const noexcept
{ 
    if (vec3().norm() == 0)
        return (Tvec3<Scalar_>() << 0,0,1).finished(); // convention
    else 
        return vec3().normalized();
}
/**
 * @brief Calculates the conjugate of the quaternion.
 * 
 * This function calculates the conjugate of the quaternion by negating the imaginary parts
 * and keeping the scalar part unchanged.
 * 
 * @return A quaternion representing the conjugate.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::conj() const noexcept {return Quaternion( (Tvec4<Scalar_>() << vals_[0], -vec3()).finished() ); }
/**
 * @brief Calculates the inverse of the quaternion.
 * 
 * This function calculates the inverse of the quaternion using the formula:
 * inverse = conjugate / norm^2.
 * 
 * @return A quaternion representing the inverse.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::inv() const noexcept {return conj() / square(norm());}
 
/**
 * @brief Calculates the natural logarithm of the calling quaternion.
 * 
 * This function computes the logarithm of the calling quaternion. Note that the imaginary
 * part of the result may be incorrect if the calling quaternion is not a unit quaternion.
 * This is due to the use of the arccos function during the calculation. Specifically,
 * the arccosine of a cosine value can yield an angle that differs from the original angle.
 * For example, if the original angle is 100 degrees, arccos(cos(100 degrees)) will not
 * return 100 degrees, leading to a loss of information.
 * 
 * @return A quaternion representing the natural logarithm of the calling quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::ln() const noexcept 
{
    return Quaternion<Scalar_>(std::log(norm()), 0.5 * rotation_angle() * rotation_vec());
} 
/**
 * @brief Calculates the exponential of the quaternion.
 * 
 * This function computes the exponential of the quaternion using the formula:
 * exp(q) = exp(w) * (cos(||v||) + (v / ||v||) * sin(||v||)), where q = w + v,
 * and v is the imaginary part of the quaternion.
 * 
 * @return A quaternion representing the exponential of the original quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::exp() const noexcept 
{
    return Quaternion<Scalar_>(std::exp(vals_[0]) * cos(vec3().norm()), std::exp(vals_[0]) * sin( vec3().norm() ) * vec3().normalized());
} 
/**
 * @brief Raises the quaternion to the power of a given index.
 * 
 * This function computes the quaternion raised to the power of a given index using
 * the formula: q^index = (||q||^index) * (cos(index * θ) + (v / ||v||) * sin(index * θ)),
 * where q is the quaternion, θ is the rotation angle, and v is the imaginary part.
 * 
 * @param index The exponent to which the quaternion is raised.
 * @return A quaternion representing the original quaternion raised to the given power.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::pow(const Scalar_& index) const noexcept 
{
    return Quaternion<Scalar_>(std::pow(norm(), index), index * rotation_angle(), rotation_vec());
} 
/**
 * @brief Normalizes the quaternion.
 * 
 * This function normalizes the quaternion by dividing each component by the norm
 * of the quaternion.
 * 
 * @return A normalized quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::normalized() const noexcept {return (*this) / norm();}
/**
 * @brief Retrieves the imaginary part of the quaternion as a 3-element vector.
 * 
 * This function returns the last three elements of the quaternion, which represent
 * the imaginary part (x, y, z).
 * 
 * @return A 3-element vector representing the imaginary part of the quaternion.
 */
template<typename Scalar_>
Tvec3<Scalar_> Quaternion<Scalar_>::vec3() const noexcept {return vals_.template tail<3>();}
/**
 * @brief Retrieves the quaternion as a 4-element vector.
 * 
 * This function returns the entire quaternion as a 4-element vector, including both
 * the scalar and imaginary parts.
 * 
 * @return A 4-element vector representing the quaternion.
 */
template<typename Scalar_>
Tvec4<Scalar_> Quaternion<Scalar_>::vec4() const noexcept {return vals_;}
/**
 * @brief Constructs the Hamilton operator (H+) matrix of the calling quaternion.
 * 
 * This function constructs and returns a 4x4 Hamilton operator matrix (H+) for the calling quaternion.
 * The Hamilton operator matrix is used in quaternion algebra to represent the quaternion multiplication
 * in matrix form. 
 * For example, given two quaternions q1 and q2, the vector form of their product can be obtained as:
 * (q1 * q2).vec4() = q1.hamiplus() * q2.vec4()
 * 
 * @return A 4x4 matrix representing the Hamilton operator (H+) of the calling quaternion.
 */
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
/**
 * @brief Constructs the Hamilton operator (H-) matrix of the calling quaternion.
 * 
 * This function constructs and returns a 4x4 Hamilton operator matrix (H-) for the calling quaternion.
 * The Hamilton operator matrix is used in quaternion algebra to represent the quaternion multiplication
 * in matrix form.
 * For example, given two quaternions q1 and q2, the vector form of their product can be obtained as:
 * (q1 * q2).vec4() = q2.haminus() * q1.vec4()
 * 
 * @return A 4x4 matrix representing the Hamilton operator (H-) of the calling quaternion.
 */
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
/**
 * @brief Converts the quaternion to a string representation.
 * 
 * This function provides a string representation of the quaternion by utilizing
 * the `std::string` conversion operator. It formats the quaternion components
 * as a string with fixed-point notation and a specified precision.
 * 
 * @return A string representing the quaternion in the format "w + x î + y ĵ + z k̂".
 */
template<typename Scalar_>
std::string Quaternion<Scalar_>::to_string() const {return operator std::string();}
/**
 * @brief Converts the quaternion to a string representation.
 * 
 * This conversion operator formats the quaternion components into a string with
 * fixed-point notation and a specified precision. The format used is "w + x î + y ĵ + z k̂".
 * 
 * @return A string representing the quaternion in the format "w + x î + y ĵ + z k̂".
 */
template<typename Scalar_>
Quaternion<Scalar_>::operator std::string() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << vals_[0] << " + " << vals_[1] << " î + " << vals_[2] << " ĵ + " << vals_[3] << " k̂ ";
    return oss.str();
}   

/**
 * @brief Calculates the sum of the calling quaternion and another quaternion.
 * 
 * This function computes the sum of the calling quaternion and another quaternion
 * passed as an lvalue.
 * 
 * @param other The other quaternion to be added to the calling quaternion.
 * @return A quaternion representing the sum of the calling quaternion and the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(vals_ + other.vals_);}
/**
 * @brief Calculates the sum of the calling quaternion and another quaternion.
 * 
 * This function computes the sum of the calling quaternion and another quaternion
 * passed as an rvalue.
 * 
 * @param other The other quaternion to be added to the calling quaternion.
 * @return A quaternion representing the sum of the calling quaternion and the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator+(Quaternion&& other) const noexcept {other.vals_ += vals_; return std::move(other);}
/**
 * @brief Calculates the subtraction of another quaternion from the calling quaternion.
 * 
 * This function computes the result of subtracting another quaternion, passed as an lvalue,
 * from the calling quaternion.
 * 
 * @param other The other quaternion to be subtracted from the calling quaternion.
 * @return A quaternion representing the result of the calling quaternion subtracting the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(vals_ - other.vals_);}
/**
 * @brief Calculates the subtraction of another quaternion from the calling quaternion.
 * 
 * This function computes the result of subtracting another quaternion, passed as an rvalue,
 * from the calling quaternion.
 * 
 * @param other The other quaternion to be subtracted from the calling quaternion.
 * @return A quaternion representing the result of the calling quaternion subtracting the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-(Quaternion&& other) const noexcept {other.vals_ = vals_ - other.vals_; return std::move(other);}
/**
 * @brief Calculates the product of the calling quaternion and another quaternion.
 * 
 * This function computes the result of multiplying the calling quaternion by another quaternion
 * passed as an lvalue.
 * 
 * @param other The other quaternion to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Quaternion& other) const noexcept {return Quaternion<Scalar_>(other.haminus() * vals_);}
/**
 * @brief Calculates the product of the calling quaternion and another quaternion.
 * 
 * This function computes the result of multiplying the calling quaternion by another quaternion
 * passed as an rvalue.
 * 
 * @param other The other quaternion to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the other quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(Quaternion&& other) const noexcept {other.vals_ = hamiplus() * other.vals_; return std::move(other);}
/**
 * @brief Calculates the product of the calling quaternion and a scalar.
 * 
 * This function computes the result of multiplying the calling quaternion by a scalar
 * passed as an lvalue.
 * 
 * @param scalar The scalar to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the scalar.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator*(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(vals_ * scalar);}
/**
 * @brief Calculates the division of the calling quaternion by a scalar.
 * 
 * This function computes the result of dividing the calling quaternion by a scalar
 * passed as an lvalue.
 * 
 * @param scalar The scalar to divide the calling quaternion by.
 * @return A quaternion representing the result of the calling quaternion divided by the scalar.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator/(const Scalar_& scalar) const noexcept {return Quaternion<Scalar_>(vals_ / scalar);}
/**
 * @brief Calculates the negation of the calling quaternion.
 * 
 * This function computes the negation of the calling quaternion.
 * 
 * @return A quaternion which is the negation of the calling quaternion.
 */
template<typename Scalar_>
Quaternion<Scalar_> Quaternion<Scalar_>::operator-() const noexcept {return Quaternion<Scalar_>(-vals_);}
/**
 * @brief Adds another quaternion to the calling quaternion.
 * 
 * This operator overload adds the components of another quaternion to the
 * components of the calling quaternion and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be added.
 * @return A reference to the calling quaternion after the addition.
 */
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator+=(const Quaternion& other) noexcept {vals_ += other.vals_; return *this;}
/**
 * @brief Subtracts another quaternion from the calling quaternion.
 * 
 * This operator overload subtracts the components of another quaternion from the
 * components of the calling quaternion and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be subtracted.
 * @return A reference to the calling quaternion after the subtraction.
 */
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator-=(const Quaternion& other) noexcept {vals_ -= other.vals_; return *this;}
/**
 * @brief Multiplies the calling quaternion by another quaternion.
 * 
 * This operator overload multiplies the calling quaternion by another quaternion using
 * Hamilton product and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be multiplied by.
 * @return A reference to the calling quaternion after the multiplication.
 */
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Quaternion& other) noexcept {vals_ = hamiplus() * other.vals_; return *this;}
/**
 * @brief Multiplies the calling quaternion by a scalar.
 * 
 * This operator overload multiplies each component of the calling quaternion by
 * a scalar and assigns the result to the calling quaternion.
 * 
 * @param scalar The scalar value to multiply by.
 * @return A reference to the calling quaternion after the multiplication.
 */
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator*=(const Scalar_& scalar) noexcept {vals_ *= scalar; return *this;}
/**
 * @brief Divides the calling quaternion by a scalar.
 * 
 * This operator overload divides each component of the calling quaternion by
 * a scalar and assigns the result to the calling quaternion.
 * 
 * @param scalar The scalar value to divide by.
 * @return A reference to the calling quaternion after the division.
 */
template<typename Scalar_>
Quaternion<Scalar_>& Quaternion<Scalar_>::operator/=(const Scalar_& scalar) noexcept {vals_ /= scalar; return *this;}
/**
 * @brief Compares two quaternions for equality.
 * 
 * This operator overload compares the components of the calling quaternion and
 * another quaternion for equality.
 * 
 * @param other The quaternion to compare with.
 * @return True if the quaternions are equal, false otherwise.
 */
template<typename Scalar_>
bool Quaternion<Scalar_>::operator==(const Quaternion& other) const noexcept {return vals_ == other.vals_;}
/**
 * @brief Compares two quaternions for inequality.
 * 
 * This operator overload compares the components of the calling quaternion and
 * another quaternion for inequality.
 * 
 * @param other The quaternion to compare with.
 * @return True if the quaternions are not equal, false otherwise.
 */
template<typename Scalar_>
bool Quaternion<Scalar_>::operator!=(const Quaternion& other) const noexcept {return vals_ != other.vals_;}

}