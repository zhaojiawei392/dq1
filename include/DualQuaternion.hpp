#pragma once
#include "DualNumber.hpp"
#include "Quaternion.hpp"



namespace dq1
{

template<typename dqScalar_>
class DualQuaternion: public DualNumber<Quaternion<dqScalar_>>{
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(const Quaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual);
    DualQuaternion(const Quaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual);
    DualQuaternion(Quaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual);
    DualQuaternion(Quaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual);
    DualQuaternion(const Vec8<dqScalar_>& vec);
    DualQuaternion(Vec8<dqScalar_>&& vec);
    DualQuaternion(const dqScalar_& h0, const dqScalar_& h1=0, const dqScalar_& h2=0, const dqScalar_& h3=0, const dqScalar_& h4=0, const dqScalar_& h5=0, const dqScalar_& h6=0, const dqScalar_& h7=0);
    
    DualQuaternion(const DualQuaternion& dq)=default;
    DualQuaternion(DualQuaternion&& dq)=default;
    virtual ~DualQuaternion()=default;
    DualQuaternion& operator=(const DualQuaternion& dq)=default;
    DualQuaternion& operator=(DualQuaternion&& dq)=default;

    // mutable operators

    DualQuaternion& normalize();

    // Const operator

    explicit operator dqScalar_() const;

    // service functions

    DualNumber<dqScalar_> norm() const noexcept;
    DualQuaternion conj() const noexcept;
    DualQuaternion inv() const noexcept;
    DualQuaternion ln() const noexcept;
    DualQuaternion exp() const noexcept;
    template<typename Scalar_>
    DualQuaternion pow(const Scalar_& index) const noexcept;
    DualQuaternion normalized() const noexcept;
    Mat8<dqScalar_> hamiplus() const noexcept;
    Mat8<dqScalar_> haminus() const noexcept;

    Vec6<dqScalar_> vec6() const noexcept;
    Vec8<dqScalar_> vec8() const noexcept;

    // Friends "const"
    
    // template<typename Scalar_>
    // friend DualQuaternion<Scalar_> operator*(const Scalar_& scalar, const DualQuaternion<Scalar_>& quaternion);
    // template<typename Scalar_>
    // friend DualQuaternion<Scalar_> operator*(const Scalar_& scalar, DualQuaternion<Scalar_>&& quaternion);    
    // template<typename Scalar_>
    // friend std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& q);

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

// /**
//  * @brief Multiply a scalar with a DualQuaternion.
//  * @param scalar The scalar to multiply.
//  * @param quaternion The DualQuaternion to multiply with.
//  * @return A new DualQuaternion resulting from the multiplication.
//  */
// template<typename Scalar_>
// DualQuaternion<Scalar_> operator*(const Scalar_& scalar, const DualQuaternion<Scalar_>& quaternion) {
//     // Implementation details here...
// }

// /**
//  * @brief Multiply a scalar with a rvalue DualQuaternion.
//  * @param scalar The scalar to multiply.
//  * @param quaternion The rvalue DualQuaternion to multiply with.
//  * @return A new DualQuaternion resulting from the multiplication.
//  */
// template<typename Scalar_>
// DualQuaternion<Scalar_> operator*(const Scalar_& scalar, DualQuaternion<Scalar_>&& quaternion) {
//     // Implementation details here...
// }

// /**
//  * @brief Output a DualQuaternion to a stream.
//  * @param os The output stream.
//  * @param q The DualQuaternion to output.
//  * @return The output stream with the DualQuaternion.
//  */
// template<typename Scalar_>
// std::ostream& operator<<(std::ostream& os, const DualQuaternion<Scalar_>& q) {
//     // Implementation details here...
// }


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class DualQuaternion *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

// Default constructor
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(): DualNumber<Quaternion<dqScalar_>>() {}

/**
 * @brief Construct a DualQuaternion from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Quaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual)
    : DualNumber<Quaternion<dqScalar_>>(primary, dual) {}

/**
 * @brief Construct a DualQuaternion from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Quaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual)
    : DualNumber<Quaternion<dqScalar_>>(primary, std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Quaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual)
    : DualNumber<Quaternion<dqScalar_>>(std::move(primary), dual) {}

/**
 * @brief Construct a DualQuaternion from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Quaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual)
    : DualNumber<Quaternion<dqScalar_>>(std::move(primary), std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a Vec8.
 * @param vec A Vec8 representing the dual quaternion values.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Vec8<dqScalar_>& vec)
    : DualNumber<Quaternion<dqScalar_>>(Quaternion<dqScalar_>(vec.template head<4>()), Quaternion<dqScalar_>(vec.template tail<4>())) {}

/**
 * @brief Construct a DualQuaternion from an rvalue Vec8.
 * @param vec An rvalue Vec8 representing the dual quaternion values.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Vec8<dqScalar_>&& vec)
    : DualNumber<Quaternion<dqScalar_>>(Quaternion<dqScalar_>(std::move(vec.template head<4>())), Quaternion<dqScalar_>(std::move(vec.template tail<4>()))) {}

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
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const dqScalar_& h0, const dqScalar_& h1, const dqScalar_& h2, const dqScalar_& h3,
                                        const dqScalar_& h4, const dqScalar_& h5, const dqScalar_& h6, const dqScalar_& h7)     
    : DualNumber<Quaternion<dqScalar_>>(Quaternion<dqScalar_>(h0, h1, h2, h3), Quaternion<dqScalar_>(h4, h5, h6, h7)) {}

// Mutable operators

/**
 * @brief Normalize this DualQuaternion.
 * @return A reference to the normalized DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>& DualQuaternion<dqScalar_>::normalize() {
    const dqScalar_& norm = this->primary_.norm();
    this->primary_ /= norm;
    this->dual_ /= norm;
    return *this;
}

// Service functions

/**
 * @brief Compute the norm of this DualQuaternion.
 * @return The norm of the DualQuaternion.
 */
template<typename dqScalar_>
DualNumber<dqScalar_> DualQuaternion<dqScalar_>::norm() const noexcept {
    if (this->primary_.norm() == 0) 
        return DualNumber<dqScalar_>();
    const dqScalar_& primary_norm = this->primary_.norm();
    const dqScalar_& dual_norm = this->primary_.vec4().dot(this->dual_.vec4()) / this->primary_norm;

    return DualNumber<dqScalar_>(primary_norm, dual_norm);
}

/**
 * @brief Compute the conjugate of this DualQuaternion.
 * @return The conjugate of the DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::conj() const noexcept {
    return DualQuaternion<dqScalar_>(this->primary_.conj(), this->dual.conj());
}

/**
 * @brief Compute the inverse of this DualQuaternion.
 * @return The inverse of the DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::inv() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the natural logarithm of this DualQuaternion.
 * @return The natural logarithm of the DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::ln() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the exponential of this DualQuaternion.
 * @return The exponential of the DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::exp() const noexcept {
    // Implementation details here...
}

/**
 * @brief Return a normalized version of this DualQuaternion.
 * @return A normalized DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::normalized() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the Hamilton plus matrix of this DualQuaternion.
 * @return The Hamilton plus matrix of the DualQuaternion.
 */
template<typename dqScalar_>
Mat8<dqScalar_> DualQuaternion<dqScalar_>::hamiplus() const noexcept {
    return (Mat8<dqScalar_>() << this->primary_.hamiplus(), Mat4<dqScalar_>::Zero(), this->dual_.hamiplus(), this->primary_.haminus()).finished();
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename dqScalar_>
Mat8<dqScalar_> DualQuaternion<dqScalar_>::haminus() const noexcept {
    return (Mat8<dqScalar_>() << this->primary_.haminus(), this->primary_.hamiplus(), this->dual_.haminus(), Mat4<dqScalar_>::Zero()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec6 representation.
 * @return A Vec6 representation of the DualQuaternion.
 */
template<typename dqScalar_>
Vec6<dqScalar_> DualQuaternion<dqScalar_>::vec6() const noexcept {
    return (Vec6<dqScalar_>() << this->primary_.vec3(), this->dual_.vec3()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename dqScalar_>
Vec8<dqScalar_> DualQuaternion<dqScalar_>::vec8() const noexcept {
    return (Vec8<dqScalar_>() << this->primary_.vec4(), this->dual_.vec4()).finished();
}


    
}