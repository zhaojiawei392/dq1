#include "DualQuaternion.h"


namespace dq1
{

// Default constructor
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(): primary_(), dual_() {}

/**
 * @brief Construct a DualQuaternion from a Vec8.
 * @param vec A Vec8 representing the dual quaternion values.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(const Vec8<Scalar_>& vec): primary_(vec.template head<4>()), dual_(vec.template tail<4>()) {}

/**
 * @brief Construct a DualQuaternion from an rvalue Vec8.
 * @param vec An rvalue Vec8 representing the dual quaternion values.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(Vec8<Scalar_>&& vec): primary_(std::move(vec.template head<4>())), dual_(std::move(vec.template tail<4>())) {}

/**
 * @brief Construct a DualQuaternion from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(const Quaternion<Scalar_>& primary, const Quaternion<Scalar_>& dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(const Quaternion<Scalar_>& primary, Quaternion<Scalar_>&& dual)
    : primary_(primary), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(Quaternion<Scalar_>&& primary, const Quaternion<Scalar_>& dual)
    : primary_(std::move(primary)), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(Quaternion<Scalar_>&& primary, Quaternion<Scalar_>&& dual)
    : primary_(std::move(primary)), dual_(std::move(dual)) {}

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
template<typename Scalar_>
DualQuaternion<Scalar_>::DualQuaternion(const Scalar_& h0, const Scalar_& h1, const Scalar_& h2, const Scalar_& h3,
                                        const Scalar_& h4, const Scalar_& h5, const Scalar_& h6, const Scalar_& h7)     
                                        : primary_(h0, h1, h2, h3), dual_(h4, h5, h6, h7) {}

// Mutable operators

/**
 * @brief Add another DualQuaternion to this one.
 * @param other The other DualQuaternion to add.
 * @return A reference to the updated DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::operator+=(const DualQuaternion& other) noexcept {
    primary_ += other.primary_;
    dual_ += other.dual_;
    return *this;
}

/**
 * @brief Subtract another DualQuaternion from this one.
 * @param other The other DualQuaternion to subtract.
 * @return A reference to the updated DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::operator-=(const DualQuaternion& other) noexcept {
    primary_ -= other.primary_;
    dual_ -= other.dual_;
    return *this;
}

/**
 * @brief Multiply this DualQuaternion by another DualQuaternion.
 * @param other The other DualQuaternion to multiply by.
 * @return A reference to the updated DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::operator*=(const DualQuaternion& other) noexcept {
    primary_ *= other.primary_;
    dual_ = primary_ * other.dual_ + dual_ * other.primary_;
    return *this;
}

/**
 * @brief Multiply this DualQuaternion by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A reference to the updated DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::operator*=(const Scalar_& scalar) noexcept {
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

/**
 * @brief Divide this DualQuaternion by a scalar.
 * @param scalar The scalar to divide by.
 * @return A reference to the updated DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::operator/=(const Scalar_& scalar) noexcept {
    primary_ /= scalar;
    dual_ /= scalar;
    return *this;
}

/**
 * @brief Normalize this DualQuaternion.
 * @return A reference to the normalized DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>& DualQuaternion<Scalar_>::normalize() {
    // Normalize both the primary and dual parts
    primary_.normalize();
    dual_.normalize();
    return *this;
}

// Const operators

/**
 * @brief Add this DualQuaternion to another.
 * @param other The other DualQuaternion to add.
 * @return A new DualQuaternion resulting from the addition.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator+(const DualQuaternion& other) const noexcept {
    return DualQuaternion(primary_ + other.primary_, dual_ + other.dual_);
}

/**
 * @brief Add this DualQuaternion to an rvalue DualQuaternion.
 * @param other The rvalue DualQuaternion to add.
 * @return A new DualQuaternion resulting from the addition.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator+(DualQuaternion&& other) const noexcept {
    return DualQuaternion(primary_ + std::move(other.primary_), dual_ + std::move(other.dual_));
}

/**
 * @brief Subtract another DualQuaternion from this one.
 * @param other The other DualQuaternion to subtract.
 * @return A new DualQuaternion resulting from the subtraction.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator-(const DualQuaternion& other) const noexcept {
    return DualQuaternion(primary_ - other.primary_, dual_ - other.dual_);
}

/**
 * @brief Subtract an rvalue DualQuaternion from this one.
 * @param other The rvalue DualQuaternion to subtract.
 * @return A new DualQuaternion resulting from the subtraction.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator-(DualQuaternion&& other) const noexcept {
    return DualQuaternion(primary_ - std::move(other.primary_), dual_ - std::move(other.dual_));
}

/**
 * @brief Multiply this DualQuaternion by another DualQuaternion.
 * @param other The other DualQuaternion to multiply by.
 * @return A new DualQuaternion resulting from the multiplication.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator*(const DualQuaternion& other) const noexcept {
    // Implementation details here...
}

/**
 * @brief Multiply this DualQuaternion by an rvalue DualQuaternion.
 * @param other The rvalue DualQuaternion to multiply by.
 * @return A new DualQuaternion resulting from the multiplication.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator*(DualQuaternion&& other) const noexcept {
    // Implementation details here...
}

/**
 * @brief Multiply this DualQuaternion by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A new DualQuaternion resulting from the multiplication.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator*(const Scalar_& scalar) const noexcept {
    return DualQuaternion(primary_ * scalar, dual_ * scalar);
}

/**
 * @brief Divide this DualQuaternion by a scalar.
 * @param scalar The scalar to divide by.
 * @return A new DualQuaternion resulting from the division.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator/(const Scalar_& scalar) const noexcept {
    return DualQuaternion(primary_ / scalar, dual_ / scalar);
}

/**
 * @brief Negate this DualQuaternion.
 * @return A new DualQuaternion resulting from the negation.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::operator-() const noexcept {
    return DualQuaternion(-primary_, -dual_);
}

/**
 * @brief Check if this DualQuaternion is equal to another.
 * @param other The other DualQuaternion to compare.
 * @return True if the DualQuaternions are equal, false otherwise.
 */
template<typename Scalar_>
bool DualQuaternion<Scalar_>::operator==(const DualQuaternion& other) const noexcept {
    return primary_ == other.primary_ && dual_ == other.dual_;
}

/**
 * @brief Check if this DualQuaternion is not equal to another.
 * @param other The other DualQuaternion to compare.
 * @return True if the DualQuaternions are not equal, false otherwise.
 */
template<typename Scalar_>
bool DualQuaternion<Scalar_>::operator!=(const DualQuaternion& other) const noexcept {
    return !(*this == other);
}

/**
 * @brief Convert this DualQuaternion to a string representation.
 * @return A string representation of the DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_>::operator std::string() const {
    // Implementation details here...
}

// Service functions

/**
 * @brief Compute the norm of this DualQuaternion.
 * @return The norm of the DualQuaternion.
 */
template<typename Scalar_>
Scalar_ DualQuaternion<Scalar_>::norm() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the conjugate of this DualQuaternion.
 * @return The conjugate of the DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::conj() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the inverse of this DualQuaternion.
 * @return The inverse of the DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::inv() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the natural logarithm of this DualQuaternion.
 * @return The natural logarithm of the DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::ln() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the exponential of this DualQuaternion.
 * @return The exponential of the DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::exp() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the power of this DualQuaternion raised to a given index.
 * @param index The index to raise the DualQuaternion to.
 * @return The resulting DualQuaternion after the power operation.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::pow(const Scalar_& index) const noexcept {
    // Implementation details here...
}

/**
 * @brief Return a normalized version of this DualQuaternion.
 * @return A normalized DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::normalized() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the Hamilton plus matrix of this DualQuaternion.
 * @return The Hamilton plus matrix of the DualQuaternion.
 */
template<typename Scalar_>
Mat8<Scalar_> DualQuaternion<Scalar_>::hamiplus() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename Scalar_>
Mat8<Scalar_> DualQuaternion<Scalar_>::haminus() const noexcept {
    // Implementation details here...
}

/**
 * @brief Convert this DualQuaternion to a string.
 * @return A string representation of the DualQuaternion.
 */
template<typename Scalar_>
std::string DualQuaternion<Scalar_>::to_string() const {
    // Implementation details here...
}

/**
 * @brief Extract the primary part of the dual quaternion.
 * @return The primary part as a new DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::P() const noexcept {
    // Implementation details here...
}

/**
 * @brief Extract the dual part of the dual quaternion.
 * @return The dual part as a new DualQuaternion.
 */
template<typename Scalar_>
DualQuaternion<Scalar_> DualQuaternion<Scalar_>::D() const noexcept {
    // Implementation details here...
}

/**
 * @brief Convert this DualQuaternion to a Vec6 representation.
 * @return A Vec6 representation of the DualQuaternion.
 */
template<typename Scalar_>
Vec6<Scalar_> DualQuaternion<Scalar_>::vec6() const noexcept {
    // Implementation details here...
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename Scalar_>
Vec8<Scalar_> DualQuaternion<Scalar_>::vec8() const noexcept {
    // Implementation details here...
}

// Friend functions

/**
 * @brief Multiply a scalar with a DualQuaternion.
 * @param scalar The scalar to multiply.
 * @param quaternion The DualQuaternion to multiply with.
 * @return A new DualQuaternion resulting from the multiplication.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> operator*(const qScalar_& scalar, const DualQuaternion<qScalar_>& quaternion) {
    // Implementation details here...
}

/**
 * @brief Multiply a scalar with a rvalue DualQuaternion.
 * @param scalar The scalar to multiply.
 * @param quaternion The rvalue DualQuaternion to multiply with.
 * @return A new DualQuaternion resulting from the multiplication.
 */
template<typename qScalar_>
DualQuaternion<qScalar_> operator*(const qScalar_& scalar, DualQuaternion<qScalar_>&& quaternion) {
    // Implementation details here...
}

/**
 * @brief Output a DualQuaternion to a stream.
 * @param os The output stream.
 * @param q The DualQuaternion to output.
 * @return The output stream with the DualQuaternion.
 */
template<typename qScalar_>
std::ostream& operator<<(std::ostream& os, const DualQuaternion<qScalar_>& q) {
    // Implementation details here...
}
    
}