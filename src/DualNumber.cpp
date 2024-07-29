#include "DualNumber.h"


namespace dq1
{

// Default constructor
template<typename Number_>
DualNumber<Number_>::DualNumber(): primary_(), dual_() {}

/**
 * @brief Construct a DualNumber from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Number_>
DualNumber<Number_>::DualNumber(const Number_& primary, const Number_& dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualNumber from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Number_>
DualNumber<Number_>::DualNumber(const Number_& primary, Number_&& dual)
    : primary_(primary), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualNumber from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Number_>
DualNumber<Number_>::DualNumber(Number_&& primary, const Number_& dual)
    : primary_(std::move(primary)), dual_(dual) {}

/**
 * @brief Construct a DualNumber from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Number_>
DualNumber<Number_>::DualNumber(Number_&& primary, Number_&& dual)
    : primary_(std::move(primary)), dual_(std::move(dual)) {}

// Mutable operators

/**
 * @brief Add another DualNumber to this one.
 * @param other The other DualNumber to add.
 * @return A reference to the updated DualNumber.
 */
template<typename Number_>
DualNumber<Number_>& DualNumber<Number_>::operator+=(const DualNumber& other) noexcept {
    primary_ += other.primary_;
    dual_ += other.dual_;
    return *this;
}

/**
 * @brief Subtract another DualNumber from this one.
 * @param other The other DualNumber to subtract.
 * @return A reference to the updated DualNumber.
 */
template<typename Number_>
DualNumber<Number_>& DualNumber<Number_>::operator-=(const DualNumber& other) noexcept {
    primary_ -= other.primary_;
    dual_ -= other.dual_;
    return *this;
}

/**
 * @brief Multiply this DualNumber by another DualNumber.
 * @param other The other DualNumber to multiply by.
 * @return A reference to the updated DualNumber.
 */
template<typename Number_>
DualNumber<Number_>& DualNumber<Number_>::operator*=(const DualNumber& other) noexcept {
    primary_ *= other.primary_;
    dual_ = primary_ * other.dual_ + dual_ * other.primary_;
    return *this;
}

/**
 * @brief Multiply this DualNumber by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A reference to the updated DualNumber.
 */
template<typename Number_>
template<typename Scalar_>
DualNumber<Number_>& DualNumber<Number_>::operator*=(const Scalar_& scalar) noexcept {
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

/**
 * @brief Divide this DualNumber by a scalar.
 * @param scalar The scalar to divide by.
 * @return A reference to the updated DualNumber.
 */
template<typename Number_>
template<typename Scalar_>
DualNumber<Number_>& DualNumber<Number_>::operator/=(const Scalar_& scalar) noexcept {
    primary_ /= scalar;
    dual_ /= scalar;
    return *this;
}

/**
 * @brief Normalize this DualNumber.
 * @return A reference to the normalized DualNumber.
 */
template<typename Number_>
DualNumber<Number_>& DualNumber<Number_>::normalize() {
    // Normalize both the primary and dual parts
    primary_.normalize();
    dual_.normalize();
    return *this;
}

// Const operators

/**
 * @brief Add this DualNumber to another.
 * @param other The other DualNumber to add.
 * @return A new DualNumber resulting from the addition.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator+(const DualNumber& other) const noexcept {
    return DualNumber(primary_ + other.primary_, dual_ + other.dual_);
}

/**
 * @brief Add this DualNumber to an rvalue DualNumber.
 * @param other The rvalue DualNumber to add.
 * @return A new DualNumber resulting from the addition.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator+(DualNumber&& other) const noexcept {
    return DualNumber(primary_ + std::move(other.primary_), dual_ + std::move(other.dual_));
}

/**
 * @brief Subtract another DualNumber from this one.
 * @param other The other DualNumber to subtract.
 * @return A new DualNumber resulting from the subtraction.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator-(const DualNumber& other) const noexcept {
    return DualNumber(primary_ - other.primary_, dual_ - other.dual_);
}

/**
 * @brief Subtract an rvalue DualNumber from this one.
 * @param other The rvalue DualNumber to subtract.
 * @return A new DualNumber resulting from the subtraction.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator-(DualNumber&& other) const noexcept {
    return DualNumber(primary_ - std::move(other.primary_), dual_ - std::move(other.dual_));
}

/**
 * @brief Multiply this DualNumber by another DualNumber.
 * @param other The other DualNumber to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator*(const DualNumber& other) const noexcept {
    // Implementation details here...
}

/**
 * @brief Multiply this DualNumber by an rvalue DualNumber.
 * @param other The rvalue DualNumber to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator*(DualNumber&& other) const noexcept {
    // Implementation details here...
}

/**
 * @brief Multiply this DualNumber by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Number_>
template<typename Scalar_>
DualNumber<Number_> DualNumber<Number_>::operator*(const Scalar_& scalar) const noexcept {
    return DualNumber(primary_ * scalar, dual_ * scalar);
}

/**
 * @brief Divide this DualNumber by a scalar.
 * @param scalar The scalar to divide by.
 * @return A new DualNumber resulting from the division.
 */
template<typename Number_>
template<typename Scalar_>
DualNumber<Number_> DualNumber<Number_>::operator/(const Scalar_& scalar) const noexcept {
    return DualNumber(primary_ / scalar, dual_ / scalar);
}

/**
 * @brief Negate this DualNumber.
 * @return A new DualNumber resulting from the negation.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::operator-() const noexcept {
    return DualNumber(-primary_, -dual_);
}

/**
 * @brief Check if this DualNumber is equal to another.
 * @param other The other DualNumber to compare.
 * @return True if the DualNumbers are equal, false otherwise.
 */
template<typename Number_>
bool DualNumber<Number_>::operator==(const DualNumber& other) const noexcept {
    return primary_ == other.primary_ && dual_ == other.dual_;
}

/**
 * @brief Check if this DualNumber is not equal to another.
 * @param other The other DualNumber to compare.
 * @return True if the DualNumbers are not equal, false otherwise.
 */
template<typename Number_>
bool DualNumber<Number_>::operator!=(const DualNumber& other) const noexcept {
    return !(*this == other);
}

/**
 * @brief Convert this DualNumber to a string representation.
 * @return A string representation of the DualNumber.
 */
template<typename Number_>
DualNumber<Number_>::operator std::string() const {
    // Implementation details here...
}

// Service functions

/**
 * @brief Compute the norm of this DualNumber.
 * @return The norm of the DualNumber.
 */
template<typename Number_>
template<typename Scalar_>
Scalar_ DualNumber<Number_>::norm() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the conjugate of this DualNumber.
 * @return The conjugate of the DualNumber.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::conj() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the inverse of this DualNumber.
 * @return The inverse of the DualNumber.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::inv() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the natural logarithm of this DualNumber.
 * @return The natural logarithm of the DualNumber.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::ln() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the exponential of this DualNumber.
 * @return The exponential of the DualNumber.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::exp() const noexcept {
    // Implementation details here...
}

/**
 * @brief Compute the power of this DualNumber raised to a given index.
 * @param index The index to raise the DualNumber to.
 * @return The resulting DualNumber after the power operation.
 */
template<typename Number_>
template<typename Scalar_>
DualNumber<Number_> DualNumber<Number_>::pow(const Scalar_& index) const noexcept {
    // Implementation details here...
}

/**
 * @brief Return a normalized version of this DualNumber.
 * @return A normalized DualNumber.
 */
template<typename Number_>
DualNumber<Number_> DualNumber<Number_>::normalized() const noexcept {
    // Implementation details here...
}

/**
 * @brief Convert this DualNumber to a string.
 * @return A string representation of the DualNumber.
 */
template<typename Number_>
std::string DualNumber<Number_>::to_string() const {
    // Implementation details here...
}

/**
 * @brief Extract the primary part of the dual quaternion.
 * @return The primary part as a new DualNumber.
 */
template<typename Number_>
Number_ DualNumber<Number_>::P() const noexcept {
    // Implementation details here...
}

/**
 * @brief Extract the dual part of the dual quaternion.
 * @return The dual part as a new DualNumber.
 */
template<typename Number_>
Number_ DualNumber<Number_>::D() const noexcept {
    // Implementation details here...
}

// Friend functions

/**
 * @brief Multiply a scalar with a DualNumber.
 * @param scalar The scalar to multiply.
 * @param quaternion The DualNumber to multiply with.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Scalar_, typename fNumber_>
DualNumber<fNumber_> operator*(const Scalar_& scalar, const DualNumber<fNumber_>& quaternion) {
    // Implementation details here...
}

/**
 * @brief Multiply a scalar with a rvalue DualNumber.
 * @param scalar The scalar to multiply.
 * @param quaternion The rvalue DualNumber to multiply with.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Scalar_, typename fNumber_>
DualNumber<fNumber_> operator*(const Scalar_& scalar, DualNumber<fNumber_>&& quaternion) {
    // Implementation details here...
}

/**
 * @brief Output a DualNumber to a stream.
 * @param os The output stream.
 * @param q The DualNumber to output.
 * @return The output stream with the DualNumber.
 */
template<typename fNumber_>
std::ostream& operator<<(std::ostream& os, const DualNumber<fNumber_>& q) {
    // Implementation details here...
}
    
}