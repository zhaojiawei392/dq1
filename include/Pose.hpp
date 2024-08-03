#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <cmath>
#include <type_traits>

namespace dq1
{
template <typename T>
T square(const T& x) {
    return x * x;
}

#define PRINT_PRECISION 18
#define OMIT_THRESHOLD 0.00000000001

template<typename Scalar_, int size>
using Vec=Eigen::Matrix<Scalar_, size, 1>;
template<typename Scalar_>
using Vec3=Eigen::Matrix<Scalar_, 3, 1>;
using Vec3f=Eigen::Matrix<float, 3, 1>;
using Vec3d=Eigen::Matrix<double, 3, 1>;
template<typename Scalar_>
using Vec4=Eigen::Matrix<Scalar_, 4, 1>;
using Vec4f=Eigen::Matrix<float, 4, 1>;
using Vec4d=Eigen::Matrix<double, 4, 1>;
template<typename Scalar_>
using Vec6=Eigen::Matrix<Scalar_, 6, 1>;
using Vec6f=Eigen::Matrix<float, 6, 1>;
using Vec6d=Eigen::Matrix<double, 6, 1>;
template<typename Scalar_>
using Vec8=Eigen::Matrix<Scalar_, 8, 1>;
using Vec8f=Eigen::Matrix<float, 8, 1>;
using Vec8d=Eigen::Matrix<double, 8, 1>;
template<typename Scalar_>
using Vecx=Eigen::Matrix<Scalar_, -1, 1>;
using Vecxf=Eigen::Matrix<float, -1, 1>;
using Vecxd=Eigen::Matrix<double, -1, 1>;

template<typename Scalar_, int rows_, int cols_>
using Mat=Eigen::Matrix<Scalar_, rows_, cols_>;
template<typename Scalar_>
using Mat3=Eigen::Matrix<Scalar_, 3, 3>;
using Mat3f=Eigen::Matrix<float, 3, 3>;
using Mat3d=Eigen::Matrix<double, 3, 3>;
template<typename Scalar_>
using Mat4=Eigen::Matrix<Scalar_, 4, 4>;
using Mat4f=Eigen::Matrix<float, 4, 4>;
using Mat4d=Eigen::Matrix<double, 4, 4>;
template<typename Scalar_>
using Mat6=Eigen::Matrix<Scalar_, 6, 6>;
using Mat6f=Eigen::Matrix<float, 6, 6>;
using Mat6d=Eigen::Matrix<double, 6, 6>;
template<typename Scalar_>
using Mat8=Eigen::Matrix<Scalar_, 8, 8>;
using Mat8f=Eigen::Matrix<float, 8, 8>;
using Mat8d=Eigen::Matrix<double, 8, 8>;
template<typename Scalar_>
using Matx=Eigen::Matrix<Scalar_, -1, -1>;
using Matxf=Eigen::Matrix<float, -1, -1>;
using Matxd=Eigen::Matrix<double, -1, -1>;

template<typename qScalar_>
class Quaternion;
template<typename qScalar_>
class PureQuaternion;
template<typename qScalar_>
class UnitQuaternion;
template<typename qScalar_>
class UnitPureQuaternion;

template<typename qScalar_>
class Quaternion{
protected:
    Vec4<qScalar_> vals_;
public:
    // Constructors and Assignments

             Quaternion();
    explicit Quaternion(const Vec4<qScalar_>& vec4);
    explicit Quaternion(Vec4<qScalar_>&& vec4);
    Quaternion(const qScalar_& w, const Vec3<qScalar_>& vec3);
    Quaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
    Quaternion(const Vec3<qScalar_>& rotation_axis_vec3, const qScalar_& rotation_angle, const qScalar_& norm=1);
    Quaternion(const UnitPureQuaternion<qScalar_>& rotation_axis, const qScalar_& rotation_angle, const qScalar_& norm=1);

    // mutable operators

    Quaternion& operator+=(const Quaternion& other) noexcept;
    Quaternion& operator-=(const Quaternion& other) noexcept;
    Quaternion& operator*=(const Quaternion& other) noexcept;
    template<typename Scalar_> 
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion&>
    operator*=(const Scalar_& scalar) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion&>
    operator/=(const Scalar_& scalar) noexcept;
    Quaternion& normalize();

    // const operators

    Quaternion operator+(const Quaternion& other) const noexcept;
    Quaternion operator-(const Quaternion& other) const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
    operator/(const Scalar_& scalar) const noexcept;
    Quaternion operator-() const noexcept;
    bool operator==(const Quaternion& other) const noexcept;
    bool operator!=(const Quaternion& other) const noexcept; 
    operator std::string() const;

    // service functions const

    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion>
             pow(const Scalar_& index) const noexcept;
    qScalar_ norm() const noexcept;
    qScalar_ rotation_angle() const noexcept;
    UnitPureQuaternion<qScalar_> rotation_axis() const noexcept;
    Vec3<qScalar_> rotation_axis_vec3() const noexcept;
    Quaternion conj() const noexcept;
    Quaternion inv() const noexcept;
    Quaternion ln() const noexcept;
    Quaternion exp() const noexcept;
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
    
    template<typename fScalar_>
    friend Quaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion1, const Quaternion<fScalar_>& quaternion2) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
    operator*(const Quaternion<fScalar_>& quaternion, const Scalar_& scalar) noexcept;
    template<typename Scalar_, typename fScalar_> 
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
    operator*(const Scalar_& scalar, const Quaternion<fScalar_>& quaternion) noexcept;
    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<Scalar_>& quaternion);
    template<typename Scalar_>
    friend void _real_part_should_be_zero(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept;
    template<typename Scalar_>
    friend void _norm_should_be_one(std::string&& calling_fn, Quaternion<Scalar_>& quaternion) noexcept;

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

            explicit PureQuaternion(const Vec3<qScalar_>& vec3);
            explicit PureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z);

    explicit PureQuaternion(const Quaternion<qScalar_>& q);
    explicit PureQuaternion(Quaternion<qScalar_>&& q);
    PureQuaternion& operator=(const Quaternion<qScalar_>& q);
    PureQuaternion& operator=(Quaternion<qScalar_>&& q);

    PureQuaternion active_rotate(const UnitQuaternion<qScalar_>& rotation){
        return PureQuaternion(rotation * (*this) * rotation.conj());
    }
    PureQuaternion passive_rotate(const UnitQuaternion<qScalar_>& rotation){
        return PureQuaternion(rotation.conj() * (*this) * rotation);
    }


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
public:

    // Constructors and Assignments

           explicit UnitQuaternion(const Vec4<qScalar_>& vec4);
           explicit UnitQuaternion(Vec4<qScalar_>&& vec4);
           explicit UnitQuaternion(const qScalar_& w, const Vec3<qScalar_>& vec3);
           explicit UnitQuaternion(const qScalar_& w, const qScalar_& x=0, const qScalar_& y=0, const qScalar_& z=0);
           explicit UnitQuaternion(const Vec3<qScalar_>& rotation_axis_vec3, const qScalar_& rotation_angle);
           explicit UnitQuaternion(const UnitPureQuaternion<qScalar_>& rotation_axis, const qScalar_& rotation_angle);

    explicit UnitQuaternion(const Quaternion<qScalar_>& q);
    explicit UnitQuaternion(Quaternion<qScalar_>&& q);
    UnitQuaternion& operator=(const Quaternion<qScalar_>& q);
    UnitQuaternion& operator=(Quaternion<qScalar_>&& q);

    // Defaults

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

               explicit UnitPureQuaternion(const Vec3<qScalar_>& vec3);
               explicit UnitPureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z);

    explicit UnitPureQuaternion(const Quaternion<qScalar_>& q);
    explicit UnitPureQuaternion(Quaternion<qScalar_>&& q);
    UnitPureQuaternion& operator=(const Quaternion<qScalar_>& q);
    UnitPureQuaternion& operator=(Quaternion<qScalar_>&& q);

                        UnitPureQuaternion()=delete;
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

// Helper Functions *************************************************************************

template<typename fScalar_>
Quaternion<fScalar_> operator*(const Quaternion<fScalar_>& quaternion1, const Quaternion<fScalar_>& quaternion2) noexcept{
    return Quaternion<fScalar_>(quaternion1.hamiplus() * quaternion2.vals_); 
}
template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>> 
operator*(const Quaternion<fScalar_>& quaternion, const Scalar_& scalar) noexcept{
    return Quaternion<fScalar_>(quaternion.vals_ * scalar);
}
template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<fScalar_>>
operator*(const Scalar_& scalar, const Quaternion<fScalar_>& quaternion) noexcept {
    return Quaternion<fScalar_>(quaternion.vals_ * scalar);
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
    quaternion.vals_[0] = 0;
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
Quaternion<qScalar_>::Quaternion(const Vec4<qScalar_>& vec4): vals_(vec4){

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
Quaternion<qScalar_>::Quaternion(Vec4<qScalar_>&& vec4): vals_(std::move(vec4)){

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
Quaternion<qScalar_>::Quaternion(const qScalar_& w, const Vec3<qScalar_>& vec3)
{
    vals_ << w, vec3;
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
 * @param rotation_axis_vec3 The 3-element vector representing the axis of rotation.
 * @throws std::runtime_error if the rotation vector is not unit 3-dimensional.
 */
template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const Vec3<qScalar_>& rotation_axis_vec3, const qScalar_& rotation_angle, const qScalar_& norm){
    vals_ << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized();
    vals_ *= norm;
}

template<typename qScalar_>
Quaternion<qScalar_>::Quaternion(const UnitPureQuaternion<qScalar_>& rotation_axis, const qScalar_& rotation_angle, const qScalar_& norm){
    vals_ << cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3();
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
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator*=(const Scalar_& scalar) noexcept {vals_ *= scalar; return *this;}

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
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>&> 
Quaternion<qScalar_>::operator/=(const Scalar_& scalar) noexcept {vals_ /= scalar; return *this;}

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
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::operator/(const Scalar_& scalar) const noexcept {return Quaternion<qScalar_>(vals_ / scalar);}

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
UnitPureQuaternion<qScalar_> Quaternion<qScalar_>::rotation_axis() const noexcept {return UnitPureQuaternion<qScalar_>( rotation_axis_vec3() );}

/**
 * @brief Calculates the rotation vector (normalized imaginary part) of the quaternion.
 * 
 * This function retrieves the imaginary parts of the quaternion and normalizes them.
 * If the norm of the imaginary part is zero, it returns a default vector (0, 0, 1).
 * 
 * @return The normalized rotation vector.
 */
template<typename qScalar_>
Vec3<qScalar_> Quaternion<qScalar_>::rotation_axis_vec3() const noexcept
{ 
    if (vec3().norm() == 0)
        return Vec3<qScalar_>{0,0,1}; // convention
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
    return Quaternion<qScalar_>(std::log(norm()), 0.5 * rotation_angle() * rotation_axis_vec3());
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
std::enable_if_t<std::is_arithmetic_v<Scalar_>, Quaternion<qScalar_>> 
Quaternion<qScalar_>::pow(const Scalar_& index) const noexcept 
{
    return Quaternion<qScalar_>(rotation_axis_vec3(), index * rotation_angle(), std::pow(norm(), index));
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
PureQuaternion<qScalar_>::PureQuaternion(const Vec3<qScalar_>& vec3) : Quaternion<qScalar_>(0, vec3) {}

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
UnitQuaternion<qScalar_>::UnitQuaternion(const Vec4<qScalar_>& vec4) : Quaternion<qScalar_>(vec4) {
    _norm_should_be_one("UnitQuaternion(const Vec4<qScalar_>& vec4)", *this);
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
UnitQuaternion<qScalar_>::UnitQuaternion(Vec4<qScalar_>&& vec4) : Quaternion<qScalar_>(std::move(vec4)) {
    _norm_should_be_one("UnitQuaternion(Vec4<qScalar_>&& vec4)", *this);
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
UnitQuaternion<qScalar_>::UnitQuaternion(const qScalar_& w, const Vec3<qScalar_>& vec3) 
    : Quaternion<qScalar_>(w, vec3) {
    _norm_should_be_one("UnitQuaternion(const qScalar_& w, const Vec3<qScalar_>& vec3)", *this);
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

template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const Vec3<qScalar_>& rotation_axis_vec3, const qScalar_& rotation_angle) 
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis_vec3.normalized()) {
    _norm_should_be_one("UnitQuaternion(const Vec3<qScalar_>& rotation_axis_vec3, const qScalar_& rotation_angle)", *this);

}

template<typename qScalar_>
UnitQuaternion<qScalar_>::UnitQuaternion(const UnitPureQuaternion<qScalar_>& rotation_axis, const qScalar_& rotation_angle)
    : Quaternion<qScalar_>(cos(0.5 * rotation_angle), sin(0.5 * rotation_angle) * rotation_axis.vec3()){
    _norm_should_be_one("UnitQuaternion(const UnitPureQuaternion<qScalar_>& rotation_axis, const qScalar_& rotation_angle)", *this);
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
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const Vec3<qScalar_>& vec3): Quaternion<qScalar_>(0, vec3) {
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
UnitPureQuaternion<qScalar_>::UnitPureQuaternion(const qScalar_& x, const qScalar_& y, const qScalar_& z):Quaternion<qScalar_>(0, x,y,z){
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
    DualQuaternion(const Quaternion<qScalar_>& primary, const Quaternion<qScalar_>& dual);
    DualQuaternion(const Quaternion<qScalar_>& primary, Quaternion<qScalar_>&& dual=Quaternion<qScalar_>());
    DualQuaternion(Quaternion<qScalar_>&& primary, const Quaternion<qScalar_>& dual);
    DualQuaternion(Quaternion<qScalar_>&& primary, Quaternion<qScalar_>&& dual=Quaternion<qScalar_>());
    explicit DualQuaternion(const Vec8<qScalar_>& vec8);
    explicit DualQuaternion(Vec8<qScalar_>&& vec8);
    DualQuaternion(const qScalar_& h0, const qScalar_& h1=0, const qScalar_& h2=0, const qScalar_& h3=0, const qScalar_& h4=0, const qScalar_& h5=0, const qScalar_& h6=0, const qScalar_& h7=0);
    
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
    operator*=(const Scalar_& scalar) noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion&> 
    operator/=(const Scalar_& scalar) noexcept;
    DualQuaternion& normalize() noexcept;

    // Const operator    
    
    DualQuaternion operator+(const DualQuaternion& other) const noexcept;
    DualQuaternion operator-(const DualQuaternion& other) const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    operator/(const Scalar_& scalar) const noexcept;
    DualQuaternion operator-() const noexcept;
    qScalar_ operator[](int index) const;
    bool operator==(const DualQuaternion& other) const noexcept;
    bool operator!=(const DualQuaternion& other) const noexcept; 
    operator std::string() const;

    // service functions

    DualQuaternion norm() const noexcept;
    DualQuaternion conj() const noexcept;
    DualQuaternion inv() const noexcept;
    DualQuaternion ln() const noexcept;
    DualQuaternion exp() const noexcept;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion> 
    pow(const Scalar_& index) const noexcept;
    DualQuaternion normalized() const noexcept;
    Mat8<qScalar_> hamiplus() const noexcept;
    Mat8<qScalar_> haminus() const noexcept;
    Quaternion<qScalar_> primary() const noexcept;
    Quaternion<qScalar_> dual() const noexcept;
    Vec6<qScalar_> vec6() const noexcept;
    Vec8<qScalar_> vec8() const noexcept;
    std::string to_string() const;
    
    // Friend functions

    template<typename fScalar_>
    friend DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dq1, const DualQuaternion<fScalar_>& dq2) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>> 
    operator*(const DualQuaternion<fScalar_>& dq, const Scalar_& scalar) noexcept;
    template<typename Scalar_, typename fScalar_>
    friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
    operator*(const Scalar_& scalar, const DualQuaternion<fScalar_>& dq) noexcept;
    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& dq);
    
    template<typename Scalar_>
    friend void _primary_part_should_be_unit(std::string&& calling_fn, DualQuaternion<Scalar_>& dq) noexcept;

};


template<typename qScalar_>
class UnitDualQuaternion: public DualQuaternion<qScalar_>{
public:

    // Constructors and Assignments

    UnitDualQuaternion(const UnitQuaternion<qScalar_>& primary, const Quaternion<qScalar_>& dual);
    UnitDualQuaternion(const UnitQuaternion<qScalar_>& primary, Quaternion<qScalar_>&& dual=Quaternion<qScalar_>());
    UnitDualQuaternion(UnitQuaternion<qScalar_>&& primary, const Quaternion<qScalar_>& dual);
    UnitDualQuaternion(UnitQuaternion<qScalar_>&& primary, Quaternion<qScalar_>&& dual=Quaternion<qScalar_>());
    explicit UnitDualQuaternion(const Vec8<qScalar_>& vec8); 
    explicit UnitDualQuaternion(Vec8<qScalar_>&& vec8);
    UnitDualQuaternion(const qScalar_& h0, const qScalar_& h1=0, const qScalar_& h2=0, const qScalar_& h3=0, const qScalar_& h4=0, const qScalar_& h5=0, const qScalar_& h6=0, const qScalar_& h7=0);
    
    explicit UnitDualQuaternion(const DualQuaternion<qScalar_>& q);
    explicit UnitDualQuaternion(DualQuaternion<qScalar_>&& q);
    UnitDualQuaternion& operator=(const DualQuaternion<qScalar_>& q);
    UnitDualQuaternion& operator=(DualQuaternion<qScalar_>&& q);

    template<typename First_, typename... Args_>
    static UnitDualQuaternion build_from(const First_& first, const Args_&... args){
        return UnitDualQuaternion(build_from(first) * build_from(args...));
    }
    static UnitDualQuaternion build_from(const UnitQuaternion<qScalar_>& rotation){
        return UnitDualQuaternion(rotation);
    }
    static UnitDualQuaternion build_from(const PureQuaternion<qScalar_>& translation){
        return UnitDualQuaternion(UnitQuaternion<qScalar_>(1), translation / 2);
    }
    static UnitDualQuaternion build_from(const UnitDualQuaternion& pose){
        return pose;
    }

    UnitQuaternion<qScalar_> rotation() const noexcept {return UnitQuaternion(this->primary_);}
    PureQuaternion<qScalar_> translation() const noexcept {return PureQuaternion(this->dual_ * this->primary_.conj() * 2);}

    UnitDualQuaternion()=delete;
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


template<typename fScalar_>
DualQuaternion<fScalar_> operator*(const DualQuaternion<fScalar_>& dq1, const DualQuaternion<fScalar_>& dq2) noexcept{
    return DualQuaternion<fScalar_>(dq1.primary_ * dq2.primary_, dq1.primary_ * dq2.dual_ + dq1.dual_ * dq2.primary_);
}
template<typename Scalar_, typename fScalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
operator*(const DualQuaternion<fScalar_>& dq, const Scalar_& scalar) noexcept{
    return DualQuaternion<fScalar_>(dq.primary_ * scalar, dq.dual_ * scalar);
}
template<typename Scalar_, typename fScalar_> 
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<fScalar_>>
operator*(const Scalar_& scalar, const DualQuaternion<fScalar_>& dq) noexcept{
    return DualQuaternion<fScalar_>(dq.primary_ * scalar, dq.dual_ * scalar);
}

/**
 * @brief Output a DualQuaternion to a stream.
 * @param os The output stream.
 * @param q The DualQuaternion to output.
 * @return The output stream with the DualQuaternion.
 */
template<typename Scalar_>
std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& dq) {
    os << dq.operator std::string();  
    return os;
}

template<typename Scalar_>
void _primary_part_should_be_unit(std::string&& calling_fn, DualQuaternion<Scalar_>& dq) noexcept{
    if (std::abs(dq.primary_.norm() - 1) > OMIT_THRESHOLD){
        std::cout << "Warning: " << std::fixed << std::setprecision(PRINT_PRECISION) << 
                        std::move(calling_fn) << " normalized a primary part with a norm " << dq.primary_.norm() << ".\n";
    }    
    dq.normalize();
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
DualQuaternion<qScalar_>::DualQuaternion(const Quaternion<qScalar_>& primary, const Quaternion<qScalar_>& dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(const Quaternion<qScalar_>& primary, Quaternion<qScalar_>&& dual)
    : primary_(primary), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Quaternion<qScalar_>&& primary, const Quaternion<qScalar_>& dual)
    : primary_(std::move(primary)), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Quaternion<qScalar_>&& primary, Quaternion<qScalar_>&& dual)
    : primary_(std::move(primary)), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a Vec8.
 * @param vec8 A Vec8 representing the dual quaternion values.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(const Vec8<qScalar_>& vec8)
    : primary_(vec8.template head<4>()), dual_(vec8.template tail<4>()) {}

/**
 * @brief Construct a DualQuaternion from an rvalue Vec8.
 * @param vec8 An rvalue Vec8 representing the dual quaternion values.
 */
template<typename qScalar_>
DualQuaternion<qScalar_>::DualQuaternion(Vec8<qScalar_>&& vec8)
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
DualQuaternion<qScalar_>::DualQuaternion(const qScalar_& h0, const qScalar_& h1, const qScalar_& h2, const qScalar_& h3,
                                        const qScalar_& h4, const qScalar_& h5, const qScalar_& h6, const qScalar_& h7)     
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
DualQuaternion<qScalar_>::operator*=(const Scalar_& scalar) noexcept{
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>&> 
DualQuaternion<qScalar_>::operator/=(const Scalar_& scalar) noexcept{
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
    const qScalar_& norm = primary_.norm();
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
DualQuaternion<qScalar_>::operator/(const Scalar_& scalar) const noexcept{
    return DualQuaternion<qScalar_>(primary_ / scalar, dual_ / scalar);
}
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::operator-() const noexcept{
    return DualQuaternion<qScalar_>(-primary_, -dual_);
}

template<typename qScalar_>
qScalar_ DualQuaternion<qScalar_>::operator[](int index) const{
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
    const qScalar_& primary_norm = primary_.norm();
    const qScalar_& dual_norm = primary_.vec4().dot(dual_.vec4()) / primary_norm;

    return DualQuaternion<qScalar_>(Quaternion<qScalar_>(primary_norm), Quaternion<qScalar_>(dual_norm));
}

/**
 * @brief Compute the conjugate of this DualQuaternion.
 * @return The conjugate of the DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::conj() const noexcept {
    return DualQuaternion<qScalar_>(primary_.conj(), dual_.conj());
}

/**
 * @brief Compute the inverse of this DualQuaternion.
 * @return The inverse of the DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::inv() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the natural logarithm of this DualQuaternion.
 * @return The natural logarithm of the DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::ln() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the exponential of this DualQuaternion.
 * @return The exponential of the DualQuaternion.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> DualQuaternion<qScalar_>::exp() const noexcept {
    // Implementation details here...
}

template<typename qScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<qScalar_>>
DualQuaternion<qScalar_>::pow(const Scalar_& index) const noexcept{

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
Mat8<qScalar_> DualQuaternion<qScalar_>::hamiplus() const noexcept {
    return (Mat8<qScalar_>() << primary_.hamiplus(), Mat4<qScalar_>::Zero(), dual_.hamiplus(), primary_.haminus()).finished();
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename qScalar_>
Mat8<qScalar_> DualQuaternion<qScalar_>::haminus() const noexcept {
    return (Mat8<qScalar_>() << primary_.haminus(), primary_.hamiplus(), dual_.haminus(), Mat4<qScalar_>::Zero()).finished();
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
Vec6<qScalar_> DualQuaternion<qScalar_>::vec6() const noexcept {
    return (Vec6<qScalar_>() << primary_.vec3(), dual_.vec3()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename qScalar_>
Vec8<qScalar_> DualQuaternion<qScalar_>::vec8() const noexcept {
    return (Vec8<qScalar_>() << primary_.vec4(), dual_.vec4()).finished();
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
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const UnitQuaternion<qScalar_>& primary, const Quaternion<qScalar_>& dual)
    : DualQuaternion<qScalar_>(primary, dual){ } 

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const UnitQuaternion<qScalar_>& primary, Quaternion<qScalar_>&& dual)
    : DualQuaternion<qScalar_>(primary, std::move(dual)){ }

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(UnitQuaternion<qScalar_>&& primary, const Quaternion<qScalar_>& dual)
    : DualQuaternion<qScalar_>(std::move(primary), dual){ }

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(UnitQuaternion<qScalar_>&& primary, Quaternion<qScalar_>&& dual)
    : DualQuaternion<qScalar_>(std::move(primary), std::move(dual)){ }

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const Vec8<qScalar_>& vec8)
    : DualQuaternion<qScalar_>( UnitQuaternion<qScalar_>(vec8.template head<4>()), Quaternion<qScalar_>(vec8.template tail<4>()) ){ }

template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(Vec8<qScalar_>&& vec8) 
    : DualQuaternion<qScalar_>( UnitQuaternion<qScalar_>(std::move(vec8.template head<4>())), Quaternion<qScalar_>(std::move(vec8.template tail<4>())) ){ }
template<typename qScalar_>
UnitDualQuaternion<qScalar_>::UnitDualQuaternion(const qScalar_& h0, const qScalar_& h1, const qScalar_& h2, const qScalar_& h3, const qScalar_& h4, const qScalar_& h5, const qScalar_& h6, const qScalar_& h7)
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
    
}
















namespace dq1{

template<typename Scalar_>
using T_Rotation = UnitQuaternion<Scalar_>;
template<typename Scalar_>
using T_Translation = PureQuaternion<Scalar_>;
template<typename Scalar_>
using T_Axis = UnitPureQuaternion<Scalar_>;
template<typename Scalar_>
using T_Pose = UnitDualQuaternion<Scalar_>; 

}

using Rot = dq1::T_Rotation<double>;
using Trans = dq1::T_Translation<double>;
using Axis = dq1::T_Axis<double>;
using Pose = dq1::T_Pose<double>;
const Axis i_(1.,0,0);
const Axis j_(0,1.,0);
const Axis k_(0,0,1.);