#pragma once
#include "Macro.hpp"
#include "Math.hpp"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <cmath>

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
    explicit Quaternion(const qScalar_& w, const Vec3<qScalar_>& vec);
    explicit Quaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
    explicit Quaternion(const Vec3<qScalar_>& rotation_vec, const qScalar_& rotation_angle, const qScalar_& norm=1);

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
    explicit operator qScalar_() const;

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
    Quaternion pow(const Scalar_& index) const noexcept;
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

           explicit PureQuaternion(const Vec3<qScalar_>& vec);
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
           explicit UnitQuaternion(const qScalar_& w, const Vec3<qScalar_>& vec);
           explicit UnitQuaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
           explicit UnitQuaternion(const Vec3<qScalar_>& rotation_vec, const qScalar_& rotation_angle);
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

               explicit UnitPureQuaternion(const Vec3<qScalar_>& vec);
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






// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************







namespace dq1
{

// Helper Functions *************************************************************************

/**
 * @brief Multiplies a scalar by a quaternion.
 * 
 * This operator overload multiplies each component of a quaternion by a scalar.
 * 
 * @param scalar The scalar value to multiply by.
 * @param quaternion The quaternion to be multiplied.
 * @return A quaternion which is the result of the scalar multiplication.
 */
template<typename Scalar_>
Quaternion<Scalar_> operator*(const Scalar_& scalar, const Quaternion<Scalar_>& quaternion) {
    return quaternion * scalar;
}

/**
 * @brief Multiplies a scalar by a quaternion.
 * 
 * This operator overload multiplies each component of a quaternion by a scalar.
 * 
 * @param scalar The scalar value to multiply by.
 * @param quaternion The quaternion to be multiplied.
 * @return A quaternion which is the result of the scalar multiplication.
 */
template<typename Scalar_>
Quaternion<Scalar_> operator*(const Scalar_& scalar, Quaternion<Scalar_>&& quaternion) {
    return std::move(quaternion *= scalar);
}

/**
 * @brief Outputs the quaternion to an output stream.
 * 
 * This operator overload allows a quaternion to be output to an `std::ostream`.
 * It converts the quaternion to a string representation and then inserts it into the stream.
 * 
 * @tparam Scalar_ The scalar type of the quaternion.
 * @param os The output stream.
 * @param q The quaternion to be output.
 * @return The output stream after inserting the quaternion's string representation.
 */
template<typename Scalar_>
std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& q){
    os << q.operator std::string();  
return os;
} 

/**
 * @brief Ensures that the real part of the quaternion is zero.
 * 
 * This helper function checks if the real part (w component) of the quaternion is close to zero.
 * If it is not, it prints a warning message and sets the real part to zero.
 * 
 * @param calling_fn The name of the calling function, used for the warning message.
 * @param quaternion the quaternion that needs to be ensured.
 * 
 * @note The method uses a threshold defined by `OMIT_THRESHOLD` to determine significant deviation from one.
 *       The `PRINT_PRECISION` constant is used to format the printed norm value.
 */
template<typename Scalar_>
void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept {
    if (std::abs(quaternion.vals_[0]) > OMIT_THRESHOLD)
        std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                     std::move(calling_fn) << " omitted a real part " << quaternion.vals_[0] << ".\n";
    quaternion.vals_[0] = 0.;
}

/**
 * @brief Ensures the norm of the quaternion is one and prints a message if it's not.
 * 
 * This method checks if the norm of the quaternion deviates from one by more than a defined threshold.
 * If the deviation exceeds the threshold, a message is printed indicating the actual norm value. 
 * It then normalizes the quaternion to ensure its norm is exactly one.
 * 
 * @param calling_fn A string indicating the function or context from which this method is called.
 *                   This is useful for debugging purposes to know where the norm check was triggered.
 * @param quaternion the quaternion that needs to be ensured.
 * 
 * The norm of the quaternion is computed using the base class `Quaternion<qScalar_>::norm()` method.
 * If the norm is not close to one, a warning message is constructed and printed using `std::cout`.
 * The message includes the calling function's name and the actual norm of the quaternion.
 * Finally, the quaternion is normalized using the `normalized()` method of the base class.
 * 
 * @note The method uses a threshold defined by `OMIT_THRESHOLD` to determine significant deviation from one.
 *       The `PRINT_PRECISION` constant is used to format the printed norm value.
 */
template<typename Scalar_>
void _norm_should_be_one(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept{
    if (std::abs(quaternion.norm() - 1) > OMIT_THRESHOLD){
        std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                     std::move(calling_fn) << " normalized a quaternion with a norm " << quaternion.norm() << ".\n";
    } 
    quaternion.vals_.normalize();
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class Quaternion ******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

/**
 * @brief Default constructor that initializes a Quaternion to zero.
 */
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion()
    :vals_(Vec4<qScalar_>::Zero()){}

/**
 * @brief Constructs a Quaternion from a vector.
 * 
 * This constructor initializes a quaternion using the elements of a given vector, passed as an lvalue.
 * The vector can have either 3 or 4 elements:
 * - If the vector has 3 elements, the quaternion is initialized as (0, vec[0], vec[1], vec[2]),
 *   where 0 is the real part, and the vector elements are the imaginary parts.
 * - If the vector has 4 elements, the quaternion is initialized directly with these elements
 *   as (vec[0], vec[1], vec[2], vec[3]).
 * 
 * @param vec The lvalue reference to the vector used to initialize the quaternion. 
 *            The size of the vector should be 3 or 4.
 * @throws std::range_error if the size of the vector is not 3 or 4.
 */
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const Vecx<qScalar_>& vec)
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
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(Vecx<qScalar_>&& vec)
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
        throw std::range_error("Error in constructor Quaternion(Vecx<qScalar_>&& vec) with a vector size of " + std::to_string(vec.size()) + ", which should be 3 or 4.");
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
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const qScalar_& w, const Vec3<qScalar_>& vec)
{
    if (vec.size() != 3)
        throw std::range_error("Error in constructor Quaternion(const qScalar_& w, const Vecx<qScalar_>& vec) with a vector size of " + std::to_string(vec.size()) + ", which should be 3.");
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
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const qScalar_& w, const qScalar_& x, const qScalar_& y, const qScalar_& z) :vals_((Vec4<qScalar_>() << w, x, y, z).finished()){}

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
 * @throws std::runtime_error if the rotation vector is not unit 3-dimensional.
 */
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const Vec3<qScalar_>& rotation_vec, const qScalar_& rotation_angle, const qScalar_& norm)
{
    if (rotation_vec.size() != 3){
        throw std::runtime_error("Quaternion(const Vecx<qScalar_>& rotation_vec, const qScalar_& rotation_angle, const qScalar_& norm) complains a rotation_vec with a size " + std::to_string(rotation_vec.size()) + ", which should be 3.\n");
    }else if (std::abs(rotation_vec.norm() - 1) > OMIT_THRESHOLD){
        std::cout << "Quaternion(const Vecx<qScalar_>& rotation_vec, const qScalar_& rotation_angle, const qScalar_& norm) normalized a rotation_vec with a norm " << std::to_string(rotation_vec.norm()) << ".\n";
    }
        vals_ << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_vec;
    vals_ *= norm;
}

/**
 * @brief Adds another quaternion to the calling quaternion.
 * 
 * This operator overload adds the components of another quaternion to the
 * components of the calling quaternion and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be added.
 * @return A reference to the calling quaternion after the addition.
 */
template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator+=(const Quaternion& other) noexcept {vals_ += other.vals_; return *this;}

/**
 * @brief Subtracts another quaternion from the calling quaternion.
 * 
 * This operator overload subtracts the components of another quaternion from the
 * components of the calling quaternion and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be subtracted.
 * @return A reference to the calling quaternion after the subtraction.
 */
template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator-=(const Quaternion& other) noexcept {vals_ -= other.vals_; return *this;}

/**
 * @brief Multiplies the calling quaternion by another quaternion.
 * 
 * This operator overload multiplies the calling quaternion by another quaternion using
 * Hamilton product and assigns the result to the calling quaternion.
 * 
 * @param other The quaternion to be multiplied by.
 * @return A reference to the calling quaternion after the multiplication.
 */
template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator*=(const Quaternion& other) noexcept {vals_ = hamiplus() * other.vals_; return *this;}

/**
 * @brief Multiplies the calling quaternion by a scalar.
 * 
 * This operator overload multiplies each component of the calling quaternion by
 * a scalar and assigns the result to the calling quaternion.
 * 
 * @param scalar The scalar value to multiply by.
 * @return A reference to the calling quaternion after the multiplication.
 */
template<typename qScalar_>
template<typename Scalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator*=(const Scalar_& scalar) noexcept {vals_ *= scalar; return *this;}

/**
 * @brief Divides the calling quaternion by a scalar.
 * 
 * This operator overload divides each component of the calling quaternion by
 * a scalar and assigns the result to the calling quaternion.
 * 
 * @param scalar The scalar value to divide by.
 * @return A reference to the calling quaternion after the division.
 */
template<typename qScalar_>
template<typename Scalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::operator/=(const Scalar_& scalar) noexcept {vals_ /= scalar; return *this;}

/**
 * @brief Normalize the calling quaternion.
 * 
 * This function normalizes the calling quaternion by ensuring its magnitude becomes 1.
 * This is crucial for maintaining the correct properties of quaternions, particularly 
 * when they are used to represent rotations.
 * 
 * @return A reference to the calling quaternion after normalization, allowing for method chaining.
 */
template<typename qScalar_>
Quaternion<qScalar_>& Quaternion<qScalar_>::normalize(){
    vals_.normalize();
    return *this;
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
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator+(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(vals_ + other.vals_);}

/**
 * @brief Calculates the sum of the calling quaternion and another quaternion.
 * 
 * This function computes the sum of the calling quaternion and another quaternion
 * passed as an rvalue.
 * 
 * @param other The other quaternion to be added to the calling quaternion.
 * @return A quaternion representing the sum of the calling quaternion and the other quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator+(Quaternion&& other) const noexcept {other.vals_ += vals_; return std::move(other);}

/**
 * @brief Calculates the subtraction of another quaternion from the calling quaternion.
 * 
 * This function computes the result of subtracting another quaternion, passed as an lvalue,
 * from the calling quaternion.
 * 
 * @param other The other quaternion to be subtracted from the calling quaternion.
 * @return A quaternion representing the result of the calling quaternion subtracting the other quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(vals_ - other.vals_);}

/**
 * @brief Calculates the subtraction of another quaternion from the calling quaternion.
 * 
 * This function computes the result of subtracting another quaternion, passed as an rvalue,
 * from the calling quaternion.
 * 
 * @param other The other quaternion to be subtracted from the calling quaternion.
 * @return A quaternion representing the result of the calling quaternion subtracting the other quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-(Quaternion&& other) const noexcept {other.vals_ = vals_ - other.vals_; return std::move(other);}

/**
 * @brief Calculates the product of the calling quaternion and another quaternion.
 * 
 * This function computes the result of multiplying the calling quaternion by another quaternion
 * passed as an lvalue.
 * 
 * @param other The other quaternion to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the other quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator*(const Quaternion& other) const noexcept {return Quaternion<qScalar_>(other.haminus() * vals_);}

/**
 * @brief Calculates the product of the calling quaternion and another quaternion.
 * 
 * This function computes the result of multiplying the calling quaternion by another quaternion
 * passed as an rvalue.
 * 
 * @param other The other quaternion to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the other quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator*(Quaternion&& other) const noexcept {other.vals_ = hamiplus() * other.vals_; return std::move(other);}

/**
 * @brief Calculates the product of the calling quaternion and a scalar.
 * 
 * This function computes the result of multiplying the calling quaternion by a scalar
 * passed as an lvalue.
 * 
 * @param scalar The scalar to multiply with the calling quaternion.
 * @return A quaternion representing the product of the calling quaternion and the scalar.
 */
template<typename qScalar_>
template<typename Scalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator*(const Scalar_& scalar) const noexcept {return Quaternion<qScalar_>(vals_ * scalar);}

/**
 * @brief Calculates the division of the calling quaternion by a scalar.
 * 
 * This function computes the result of dividing the calling quaternion by a scalar
 * passed as an lvalue.
 * 
 * @param scalar The scalar to divide the calling quaternion by.
 * @return A quaternion representing the result of the calling quaternion divided by the scalar.
 */
template<typename qScalar_>
template<typename Scalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator/(const Scalar_& scalar) const noexcept {return Quaternion<qScalar_>(vals_ / scalar);}

/**
 * @brief Calculates the negation of the calling quaternion.
 * 
 * This function computes the negation of the calling quaternion.
 * 
 * @return A quaternion which is the negation of the calling quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::operator-() const noexcept {return Quaternion<qScalar_>(-vals_);}

/**
 * @brief Compares two quaternions for equality.
 * 
 * This operator overload compares the components of the calling quaternion and
 * another quaternion for equality.
 * 
 * @param other The quaternion to compare with.
 * @return True if the quaternions are equal, false otherwise.
 */
template<typename qScalar_>
bool Quaternion<qScalar_>::operator==(const Quaternion& other) const noexcept {return vals_ == other.vals_;}

/**
 * @brief Compares two quaternions for inequality.
 * 
 * This operator overload compares the components of the calling quaternion and
 * another quaternion for inequality.
 * 
 * @param other The quaternion to compare with.
 * @return True if the quaternions are not equal, false otherwise.
 */
template<typename qScalar_>
bool Quaternion<qScalar_>::operator!=(const Quaternion& other) const noexcept {return vals_ != other.vals_;}

/**
 * @brief Converts the quaternion to a string representation.
 * 
 * This conversion operator formats the quaternion components into a string with
 * fixed-point notation and a specified precision. The format used is "w + x î + y ĵ + z k̂".
 * 
 * @return A string representing the quaternion in the format "w + x î + y ĵ + z k̂".
 */
template<typename qScalar_>
Quaternion<qScalar_>::operator std::string() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << vals_[0] << " + " << vals_[1] << " î + " << vals_[2] << " ĵ + " << vals_[3] << " k̂ ";
    return oss.str();
}   

/**
 * @brief Calculates the norm (magnitude) of the quaternion.
 * 
 * This function calculates the norm of the quaternion, which is the Euclidean
 * norm of the underlying 4-element vector.
 * 
 * @return The norm of the quaternion.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::norm() const noexcept {return vals_.norm();}

/**
 * @brief Calculates the rotation angle represented by the quaternion.
 * 
 * This function calculates the rotation angle using the scalar part of the quaternion.
 * The formula used is 2 * acos(w / norm()), where w is the scalar part.
 * 
 * @return The rotation angle in radians.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::rotation_angle() const noexcept {return 2 * acos(vals_[0] / norm());}

/**
 * @brief Calculates the rotation axis represented by the quaternion.
 * 
 * This function calculates the rotation axis, which is returned as a quaternion
 * with only the imaginary parts set (x, y, z) and the scalar part set to zero.
 * 
 * @return A quaternion representing the rotation axis.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::rotation_axis() const noexcept {return Quaternion<qScalar_>( rotation_vec() );}

/**
 * @brief Calculates the rotation vector (normalized imaginary part) of the quaternion.
 * 
 * This function retrieves the imaginary parts of the quaternion and normalizes them.
 * If the norm of the imaginary part is zero, it returns a default vector (0, 0, 1).
 * 
 * @return The normalized rotation vector.
 */
template<typename qScalar_>
Vec3<qScalar_> Quaternion<qScalar_>::rotation_vec() const noexcept
{ 
    if (vec3().norm() == 0)
        return (Vec3<qScalar_>() << 0,0,1).finished(); // convention
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
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::conj() const noexcept {return Quaternion( (Vec4<qScalar_>() << vals_[0], -vec3()).finished() ); }

/**
 * @brief Calculates the inverse of the quaternion.
 * 
 * This function calculates the inverse of the quaternion using the formula:
 * inverse = conjugate / norm^2.
 * 
 * @return A quaternion representing the inverse.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::inv() const noexcept {return conj() / square(norm());}
 
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
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::ln() const noexcept 
{
    return Quaternion<qScalar_>(std::log(norm()), 0.5 * rotation_angle() * rotation_vec());
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
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::exp() const noexcept 
{
    return Quaternion<qScalar_>(std::exp(vals_[0]) * cos(vec3().norm()), std::exp(vals_[0]) * sin( vec3().norm() ) * vec3().normalized());
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
template<typename qScalar_>
template<typename Scalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::pow(const Scalar_& index) const noexcept 
{
    return Quaternion<qScalar_>(rotation_vec(), index * rotation_angle(), std::pow(norm(), index));
} 

/**
 * @brief Normalizes the quaternion.
 * 
 * This function normalizes the quaternion by dividing each component by the norm
 * of the quaternion.
 * 
 * @return A normalized quaternion.
 */
template<typename qScalar_>
Quaternion<qScalar_> Quaternion<qScalar_>::normalized() const noexcept {return (*this) / norm();}

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
template<typename qScalar_>
Mat4<qScalar_> Quaternion<qScalar_>::hamiplus() const noexcept 
{      
    const qScalar_ w = vals_[0];
    const qScalar_ x = vals_[1];
    const qScalar_ y = vals_[2];
    const qScalar_ z = vals_[3];
    return (Mat4<qScalar_>() << w, -x, -y, -z,
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
template<typename qScalar_>
Mat4<qScalar_> Quaternion<qScalar_>::haminus() const noexcept 
{
    const qScalar_ w = vals_[0];
    const qScalar_ x = vals_[1];
    const qScalar_ y = vals_[2];
    const qScalar_ z = vals_[3];
    return (Mat4<qScalar_>() << w, -x, -y, -z,
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
template<typename qScalar_>
std::string Quaternion<qScalar_>::to_string() const {return operator std::string();}

/**
 * @brief Returns the scalar (real) part of the quaternion.
 * 
 * This function retrieves the first element of the quaternion, which represents
 * the scalar (real) part.
 * 
 * @return The scalar part of the quaternion.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::w() const noexcept {return vals_[0];}

/**
 * @brief Returns the first imaginary part of the quaternion.
 * 
 * This function retrieves the second element of the quaternion, which represents
 * the first imaginary part (x) of the quaternion.
 * 
 * @return The first imaginary part of the quaternion.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::x() const noexcept {return vals_[1];}

/**
 * @brief Returns the second imaginary part of the quaternion.
 * 
 * This function retrieves the third element of the quaternion, which represents
 * the second imaginary part (y) of the quaternion.
 * 
 * @return The second imaginary part of the quaternion.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::y() const noexcept {return vals_[2];}

/**
 * @brief Returns the third imaginary part of the quaternion.
 * 
 * This function retrieves the fourth element of the quaternion, which represents
 * the third imaginary part (z) of the quaternion.
 * 
 * @return The third imaginary part of the quaternion.
 */
template<typename qScalar_>
qScalar_ Quaternion<qScalar_>::z() const noexcept {return vals_[3];}

/**
 * @brief Retrieves the imaginary part of the quaternion as a 3-element vector.
 * 
 * This function returns the last three elements of the quaternion, which represent
 * the imaginary part (x, y, z).
 * 
 * @return A 3-element vector representing the imaginary part of the quaternion.
 */
template<typename qScalar_>
Vec3<qScalar_> Quaternion<qScalar_>::vec3() const noexcept {return vals_.template tail<3>();}

/**
 * @brief Retrieves the quaternion as a 4-element vector.
 * 
 * This function returns the entire quaternion as a 4-element vector, including both
 * the scalar and imaginary parts.
 * 
 * @return A 4-element vector representing the quaternion.
 */
template<typename qScalar_>
Vec4<qScalar_> Quaternion<qScalar_>::vec4() const noexcept {return vals_;}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class PureQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

/**
 * @brief Constructs a PureQuaternion from a 3D vector, initializing the scalar part to 0.
 * 
 * This constructor initializes a PureQuaternion object from a given 3D vector.
 * The scalar part (w component) of the quaternion is set to 0, making it a pure quaternion.
 * 
 * @param vec The 3D vector to initialize the quaternion.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(const Vec3<qScalar_>& vec) : Quaternion<qScalar_>(0, vec) {}

/**
 * @brief Constructs a PureQuaternion from three scalar values.
 * 
 * This constructor initializes a PureQuaternion object from three scalar values representing the x, y, and z components.
 * The scalar part (w component) of the quaternion is set to 0, making it a pure quaternion.
 * 
 * @param x The x component.
 * @param y The y component.
 * @param z The z component.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z) 
    : Quaternion<qScalar_>(0, x,y,z) {}

/**
 * @brief Constructs a PureQuaternion from a base Quaternion.
 * 
 * This constructor initializes a PureQuaternion object from a given Quaternion.
 * It ensures that the real part (w component) of the quaternion is zero, as required for pure quaternions.
 * 
 * @param other The base quaternion to copy.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _real_part_should_be_zero("PureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

/**
 * @brief Constructs a PureQuaternion from a base Quaternion using move semantics.
 * 
 * This constructor initializes a PureQuaternion object by moving the data from a given Quaternion.
 * It ensures that the real part (w component) of the quaternion is zero, as required for pure quaternions.
 * 
 * @param other The base quaternion to move.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>::PureQuaternion(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("PureQuaternion(Quaternion<qScalar_>&& other)", *this);
}

/**
 * @brief Assigns a base Quaternion to this PureQuaternion.
 * 
 * This assignment operator assigns the values from a given Quaternion to this PureQuaternion.
 * It ensures that the real part (w component) of the quaternion is zero, as required for pure quaternions.
 * 
 * @param other The base quaternion to copy.
 * @return Reference to this PureQuaternion.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("PureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

/**
 * @brief Move assigns a base Quaternion to this PureQuaternion.
 * 
 * This move assignment operator assigns the values from a given Quaternion to this PureQuaternion, using move semantics.
 * It ensures that the real part (w component) of the quaternion is zero, as required for pure quaternions.
 * 
 * @param other The base quaternion to move.
 * @return Reference to this PureQuaternion.
 */
template<typename qScalar_>
PureQuaternion<qScalar_>& PureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("PureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}

/**
 * @brief Compares if two pure quaternions are equal.
 * 
 * This operator overload allows comparing two pure quaternions for equality.
 * Two pure quaternions are equal if their vector components (x, y, z) are equal.
 * 
 * @param other The other pure quaternion to compare with.
 * @return True if they are equal, false otherwise.
 */
template<typename qScalar_>
bool PureQuaternion<qScalar_>::operator==(const PureQuaternion<qScalar_>& other) const noexcept {
    return Quaternion<qScalar_>::vec3() == other.vec3();
}

/**
 * @brief Compares if two pure quaternions are not equal.
 * 
 * This operator overload allows comparing two pure quaternions for inequality.
 * Two pure quaternions are not equal if their vector components (x, y, z) are not equal.
 * 
 * @param other The other pure quaternion to compare with.
 * @return True if they are not equal, false otherwise.
 */
template<typename qScalar_>
bool PureQuaternion<qScalar_>::operator!=(const PureQuaternion<qScalar_>& other) const noexcept {
    return Quaternion<qScalar_>::vec3() != other.vec3();
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

/**
 * @brief Constructs a UnitQuaternion from a vector.
 * 
 * This constructor initializes a unit quaternion using the elements of a given vector, passed as an lvalue.
 * The vector can have either 3 or 4 elements:
 * - If the vector has 3 elements, the quaternion is initialized as (0, vec[0], vec[1], vec[2]),
 *   where 0 is the scalar part, and the vector elements are the imaginary parts.
 * - If the vector has 4 elements, the quaternion is initialized directly with these elements
 *   as (vec[0], vec[1], vec[2], vec[3]).
 * 
 * After initialization, the quaternion is normalized to ensure it has a unit norm.
 * 
 * @param vec The lvalue reference to the vector used to initialize the quaternion. 
 *            The size of the vector should be 3 or 4.
 * @throws std::range_error if the size of the vector is not 3 or 4.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const Vecx<qScalar_>& vec) 
    : Quaternion<qScalar_>(vec) {
    _norm_should_be_one("UnitQuaternion(const Vecx<qScalar_>& vec)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from a vector using move semantics.
 * 
 * This constructor initializes a unit quaternion using the elements of a given vector, passed as an rvalue.
 * The vector can have either 3 or 4 elements:
 * - If the vector has 3 elements, the quaternion is initialized as (0, vec[0], vec[1], vec[2]),
 *   where 0 is the scalar part, and the vector elements are the imaginary parts.
 * - If the vector has 4 elements, the quaternion is initialized directly with these elements
 *   as (vec[0], vec[1], vec[2], vec[3]).
 * 
 * After initialization, the quaternion is normalized to ensure it has a unit norm.
 * 
 * @param vec The rvalue reference to the vector used to initialize the quaternion. 
 *            The size of the vector should be 3 or 4.
 * @throws std::range_error if the size of the vector is not 3 or 4.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(Vecx<qScalar_>&& vec) 
    : Quaternion<qScalar_>(std::move(vec)) {
    _norm_should_be_one("UnitQuaternion(Vecx<qScalar_>&& vec)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from a scalar and a vector.
 * 
 * This constructor initializes a unit quaternion using a scalar (the real part) and the elements of a vector
 * (the imaginary parts). The vector should have exactly 3 elements.
 * 
 * After initialization, the quaternion is normalized to ensure it has a unit norm.
 * 
 * @param w The scalar part of the quaternion.
 * @param vec The vector part of the quaternion, which should have exactly 3 elements.
 * @throws std::range_error if the size of the vector is not 3.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const qScalar_& w, const Vec3<qScalar_>& vec) 
    : Quaternion<qScalar_>(w, vec) {
    _norm_should_be_one("UnitQuaternion(const qScalar_& w, const Vecx<qScalar_>& vec)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from four scalar values.
 * 
 * This constructor initializes a unit quaternion using four scalar values representing the real part
 * and the three imaginary parts.
 * 
 * After initialization, the quaternion is normalized to ensure it has a unit norm.
 * 
 * @param w The scalar part of the quaternion.
 * @param x The x component of the vector part of the quaternion.
 * @param y The y component of the vector part of the quaternion.
 * @param z The z component of the vector part of the quaternion.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const qScalar_& w, const qScalar_& x, const qScalar_& y, const qScalar_& z) 
    : Quaternion<qScalar_>(w, x, y, z) {
    _norm_should_be_one("UnitQuaternion(const qScalar_& w, const qScalar_& x, const qScalar_& y, const qScalar_& z)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from a rotation vector and rotation angle.
 * 
 * This constructor initializes a unit quaternion representing a rotation.
 * The rotation is defined by a vector and an angle. The quaternion is calculated as:
 * - The scalar part (real part) is `cos(rotation_angle / 2)`.
 * - The vector part (imaginary parts) is `rotation_vec.normalized() * sin(rotation_angle / 2)`.
 * 
 * After initialization, the quaternion is normalized to ensure it has a unit norm.
 * 
 * @param rotation_vec The vector defining the axis of rotation.
 * @param rotation_angle The angle of rotation in radians.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const Vec3<qScalar_>& rotation_vec, const qScalar_& rotation_angle) 
    : Quaternion<qScalar_>(rotation_vec, rotation_angle) {
    _norm_should_be_one("UnitQuaternion(const Vecx<qScalar_>& rotation_vec, const qScalar_& rotation_angle)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from a Quaternion.
 * 
 * This constructor initializes a UnitQuaternion using the elements of an existing Quaternion.
 * The quaternion is normalized to ensure it remains a unit quaternion.
 * 
 * @param other The lvalue reference to the Quaternion used to initialize the UnitQuaternion.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const Quaternion<qScalar_>& other) : Quaternion<qScalar_>(other) {
    _norm_should_be_one("UnitQuaternion(const Quaternion<qScalar_>& other)", *this);
}

/**
 * @brief Constructs a UnitQuaternion from a Quaternion.
 * 
 * This constructor initializes a UnitQuaternion using the elements of an existing Quaternion, passed as an rvalue.
 * The quaternion is normalized to ensure it remains a unit quaternion.
 * 
 * @param other The rvalue reference to the Quaternion used to initialize the UnitQuaternion.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(Quaternion<qScalar_>&& other) : Quaternion<qScalar_>(std::move(other)) {
    _norm_should_be_one("UnitQuaternion(Quaternion<qScalar_>&& other)", *this);
}

/**
 * @brief Assigns a Quaternion to a UnitQuaternion.
 * 
 * This assignment operator initializes a UnitQuaternion using the elements of an existing Quaternion.
 * The quaternion is normalized to ensure it remains a unit quaternion.
 * 
 * @param other The lvalue reference to the Quaternion used to initialize the UnitQuaternion.
 * @return A reference to the updated UnitQuaternion.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(other); // Use the base class assignment operator
        _norm_should_be_one("UnitQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

/**
 * @brief Assigns a Quaternion to a UnitQuaternion.
 * 
 * This assignment operator initializes a UnitQuaternion using the elements of an existing Quaternion, passed as an rvalue.
 * The quaternion is normalized to ensure it remains a unit quaternion.
 * 
 * @param other The rvalue reference to the Quaternion used to initialize the UnitQuaternion.
 * @return A reference to the updated UnitQuaternion.
 */
template<typename qScalar_>
UnitQuaternion<qScalar_>& UnitQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other) {
        Quaternion<qScalar_>::operator=(std::move(other)); // Use the base class assignment operator
        _norm_should_be_one("UnitQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitPureQuaternion **************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

/**
 * @brief Constructs a UnitPureQuaternion from a vector.
 * 
 * This constructor initializes a unit pure quaternion using the elements of a given vector.
 * The vector should have 3 elements. The quaternion is initialized as (0, vec[0], vec[1], vec[2]).
 * 
 * @param vec The lvalue reference to the vector used to initialize the quaternion. 
 *            The size of the vector should be 3.
 * @throws std::range_error if the size of the vector is not 3.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const Vec3<qScalar_>& vec): Quaternion<qScalar_>(0, vec) {
    _norm_should_be_one("UnitPureQuaternion(const Vecx<qScalar_>& vec)", *this);
}

/**
 * @brief Constructs a UnitPureQuaternion from three scalar values.
 * 
 * This constructor initializes a UnitPureQuaternion object from three scalar values representing the x, y, and z components.
 * The scalar part (w component) of the quaternion is set to 0, making it a pure quaternion.
 * 
 * @param x The x component.
 * @param y The y component.
 * @param z The z component.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z):Quaternion<qScalar_>(0, x,y,x){
    _norm_should_be_one("UnitPureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z)", *this);
}

/**
 * @brief Constructs a UnitPureQuaternion from a Quaternion.
 * 
 * This constructor initializes a unit pure quaternion from a base Quaternion object.
 * 
 * @param other The quaternion to initialize from.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const Quaternion<qScalar_>& other): Quaternion<qScalar_>(other){
    _real_part_should_be_zero("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

/**
 * @brief Constructs a UnitPureQuaternion from an rvalue Quaternion.
 * 
 * This constructor initializes a unit pure quaternion from an rvalue Quaternion object.
 * 
 * @param other The rvalue quaternion to initialize from.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(Quaternion<qScalar_>&& other): Quaternion<qScalar_>(std::move(other)) {
    _real_part_should_be_zero("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
    _norm_should_be_one("UnitPureQuaternion(const Quaternion<qScalar_>& other)", *this);
}

/**
 * @brief Assignment operator from a Quaternion.
 * 
 * This operator assigns a unit pure quaternion from a base Quaternion object.
 * 
 * @param other The quaternion to assign from.
 * @return A reference to the assigned UnitPureQuaternion object.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>& UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(other);
        _real_part_should_be_zero("UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
        _norm_should_be_one("UnitPureQuaternion<qScalar_>::operator=(const Quaternion<qScalar_>& other)", *this);
    }
    return *this;
}

/**
 * @brief Assignment operator from an rvalue Quaternion.
 * 
 * This operator assigns a unit pure quaternion from an rvalue Quaternion object.
 * 
 * @param other The rvalue quaternion to assign from.
 * @return A reference to the assigned UnitPureQuaternion object.
 */
template<typename qScalar_>
UnitPureQuaternion<qScalar_>& UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other) {
    if (this != &other){
        Quaternion<qScalar_>::operator=(std::move(other));
        _real_part_should_be_zero("UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
        _norm_should_be_one("UnitPureQuaternion<qScalar_>::operator=(Quaternion<qScalar_>&& other)", *this);
    }
    return *this;
}



}