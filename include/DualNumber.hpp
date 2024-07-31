#pragma once
#include "Macro.hpp"
#include <iostream>

namespace dq1
{

template<typename Data_>
class DualNumber{
protected:
    Data_ primary_; //primary part
    Data_ dual_; // dual part
public:

    // Constructors and Assignments

    DualNumber();
    DualNumber(const Data_& primary, const Data_& dual);
    DualNumber(const Data_& primary, Data_&& dual);
    DualNumber(Data_&& primary, const Data_& dual);
    DualNumber(Data_&& primary, Data_&& dual);

    DualNumber(const DualNumber& dn)=default;
    DualNumber(DualNumber&& dn)=default;
    virtual ~DualNumber()=default;
    DualNumber& operator=(const DualNumber& dn)=default;
    DualNumber& operator=(DualNumber&& dn)=default;

    // mutable operators

    DualNumber& operator+=(const DualNumber& other) noexcept;
    DualNumber& operator-=(const DualNumber& other) noexcept;
    DualNumber& operator*=(const DualNumber& other) noexcept;
    template<typename Scalar_>
    DualNumber& operator*=(const Scalar_& scalar) noexcept;
    template<typename Scalar_>
    DualNumber& operator/=(const Scalar_& scalar) noexcept;
    DualNumber& normalize() noexcept=delete;

    // const operators

    DualNumber operator+(const DualNumber& other) const noexcept;
    DualNumber operator+(DualNumber&& other) const noexcept;
    DualNumber operator-(const DualNumber& other) const noexcept;
    DualNumber operator-(DualNumber&& other) const noexcept;
    DualNumber operator*(const DualNumber& other) const noexcept;
    DualNumber operator*(DualNumber&& other) const noexcept;
    template<typename Scalar_>
    DualNumber operator*(const Scalar_& scalar) const noexcept;
    template<typename Scalar_>
    DualNumber operator/(const Scalar_& scalar) const noexcept;
    DualNumber operator-() const noexcept;
    bool operator==(const DualNumber& other) const noexcept;
    bool operator!=(const DualNumber& other) const noexcept; 
    operator std::string() const;
    explicit operator Data_() const;

    // service functions
    template<typename Scalar_>
    Scalar_ norm() const noexcept=delete;
    DualNumber conj() const noexcept;
    DualNumber inv() const noexcept=delete;
    DualNumber ln() const noexcept=delete;
    DualNumber exp() const noexcept=delete;
    template<typename Scalar_>
    DualNumber pow(const Scalar_& index) const noexcept;
    DualNumber normalized() const noexcept=delete;
    std::string to_string() const;

    Data_ P() const noexcept;
    Data_ D() const noexcept;

    // Friends "const"
    
    template<typename Scalar_>
    friend DualNumber<Scalar_> operator*(const Scalar_& scalar, const DualNumber<Scalar_>& dual_number);
    template<typename Scalar_>
    friend DualNumber<Scalar_> operator*(const Scalar_& scalar, DualNumber<Scalar_>&& dual_number);    
    template<typename fData_>
    friend std::ostream& operator<<(std::ostream& os, const DualNumber<fData_>& dual_number);
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

/**
 * @brief Multiply a scalar with a DualNumber.
 * @param scalar The scalar to multiply.
 * @param quaternion The DualNumber to multiply with.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Scalar_>
DualNumber<Scalar_> operator*(const Scalar_& scalar, const DualNumber<Scalar_>& dual_number) {
    return dual_number * scalar;
}

/**
 * @brief Multiply a scalar with a rvalue DualNumber.
 * @param scalar The scalar to multiply.
 * @param quaternion The rvalue DualNumber to multiply with.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Scalar_>
DualNumber<Scalar_> operator*(const Scalar_& scalar, DualNumber<Scalar_>&& dual_number) {
    return std::move(dual_number *= scalar);
}

/**
 * @brief Output a DualNumber to a stream.
 * @param os The output stream.
 * @param q The DualNumber to output.
 * @return The output stream with the DualNumber.
 */
template<typename fData_>
std::ostream& operator<<(std::ostream& os, const DualNumber<fData_>& q) {
    os << q.operator std::string();  
    return os;
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class DualNumber *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

// Default constructor
/**
 * @brief Default constructor that initializes a DualNumber to zero.
 */
template<typename Data_>
DualNumber<Data_>::DualNumber(): primary_(), dual_() {}

/**
 * @brief Construct a DualNumber from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Data_>
DualNumber<Data_>::DualNumber(const Data_& primary, const Data_& dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualNumber from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Data_>
DualNumber<Data_>::DualNumber(const Data_& primary, Data_&& dual)
    : primary_(primary), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualNumber from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Data_>
DualNumber<Data_>::DualNumber(Data_&& primary, const Data_& dual)
    : primary_(std::move(primary)), dual_(dual) {}

/**
 * @brief Construct a DualNumber from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename Data_>
DualNumber<Data_>::DualNumber(Data_&& primary, Data_&& dual)
    : primary_(std::move(primary)), dual_(std::move(dual)) {}

// Mutable operators

/**
 * @brief Add another DualNumber to this one.
 * @param other The other DualNumber to add.
 * @return A reference to the updated DualNumber.
 */
template<typename Data_>
DualNumber<Data_>& DualNumber<Data_>::operator+=(const DualNumber& other) noexcept {
    primary_ += other.primary_;
    dual_ += other.dual_;
    return *this;
}

/**
 * @brief Subtract another DualNumber from this one.
 * @param other The other DualNumber to subtract.
 * @return A reference to the updated DualNumber.
 */
template<typename Data_>
DualNumber<Data_>& DualNumber<Data_>::operator-=(const DualNumber& other) noexcept {
    primary_ -= other.primary_;
    dual_ -= other.dual_;
    return *this;
}

/**
 * @brief Multiply this DualNumber by another DualNumber.
 * @param other The other DualNumber to multiply by.
 * @return A reference to the updated DualNumber.
 */
template<typename Data_>
DualNumber<Data_>& DualNumber<Data_>::operator*=(const DualNumber& other) noexcept {
    primary_ *= other.primary_;
    dual_ = primary_ * other.dual_ + dual_ * other.primary_;
    return *this;
}

/**
 * @brief Multiply this DualNumber by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A reference to the updated DualNumber.
 */
template<typename Data_>
template<typename Scalar_>
DualNumber<Data_>& DualNumber<Data_>::operator*=(const Scalar_& scalar) noexcept {
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

/**
 * @brief Divide this DualNumber by a scalar.
 * @param scalar The scalar to divide by.
 * @return A reference to the updated DualNumber.
 */
template<typename Data_>
template<typename Scalar_>
DualNumber<Data_>& DualNumber<Data_>::operator/=(const Scalar_& scalar) noexcept {
    primary_ /= scalar;
    dual_ /= scalar;
    return *this;
}

// Const operators

/**
 * @brief Add this DualNumber to another.
 * @param other The other DualNumber to add.
 * @return A new DualNumber resulting from the addition.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator+(const DualNumber& other) const noexcept {
    return DualNumber(primary_ + other.primary_, dual_ + other.dual_);
}

/**
 * @brief Add this DualNumber to an rvalue DualNumber.
 * the rvalue DualNumber will be moved to return.
 * @param other The rvalue DualNumber to add.
 * @return the rvalue DualNumber after the addition.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator+(DualNumber&& other) const noexcept {
    return std::move(other += *this);
}

/**
 * @brief Subtract another DualNumber from this one.
 * @param other The other DualNumber to subtract.
 * @return A new DualNumber resulting from the subtraction.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator-(const DualNumber& other) const noexcept {
    return DualNumber(primary_ - other.primary_, dual_ - other.dual_);
}

/**
 * @brief Subtract an rvalue DualNumber from this one.
 * the rvalue DualNumber will be moved to return.
 * @param other The rvalue DualNumber to subtract.
 * @return the rvalue DualNumber after the subtraction.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator-(DualNumber&& other) const noexcept {
    return std::move(-other += *this);
}

/**
 * @brief Multiply this DualNumber by another DualNumber.
 * @param other The other DualNumber to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator*(const DualNumber& other) const noexcept {
    return *this * other;
}

/**
 * @brief Multiply this DualNumber by an rvalue DualNumber.
 * @param other The rvalue DualNumber to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator*(DualNumber&& other) const noexcept {
    return std::move(other * (*this));
}

/**
 * @brief Multiply this DualNumber by a scalar.
 * @param scalar The scalar to multiply by.
 * @return A new DualNumber resulting from the multiplication.
 */
template<typename Data_>
template<typename Scalar_>
DualNumber<Data_> DualNumber<Data_>::operator*(const Scalar_& scalar) const noexcept {
    return DualNumber(primary_ * scalar, dual_ * scalar);
}

/**
 * @brief Divide this DualNumber by a scalar.
 * @param scalar The scalar to divide by.
 * @return A new DualNumber resulting from the division.
 */
template<typename Data_>
template<typename Scalar_>
DualNumber<Data_> DualNumber<Data_>::operator/(const Scalar_& scalar) const noexcept {
    return DualNumber(primary_ / scalar, dual_ / scalar);
}

/**
 * @brief Negate this DualNumber.
 * @return A new DualNumber resulting from the negation.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::operator-() const noexcept {
    return DualNumber(-primary_, -dual_);
}

/**
 * @brief Check if this DualNumber is equal to another.
 * @param other The other DualNumber to compare.
 * @return True if the DualNumbers are equal, false otherwise.
 */
template<typename Data_>
bool DualNumber<Data_>::operator==(const DualNumber& other) const noexcept {
    return primary_ == other.primary_ && dual_ == other.dual_;
}

/**
 * @brief Check if this DualNumber is not equal to another.
 * @param other The other DualNumber to compare.
 * @return True if the DualNumbers are not equal, false otherwise.
 */
template<typename Data_>
bool DualNumber<Data_>::operator!=(const DualNumber& other) const noexcept {
    return primary_ != other.primary_ || dual_ != other.dual_;
}

/**
 * @brief Convert this DualNumber to a string representation.
 * @return A string representation of the DualNumber.
 */
template<typename Data_>
DualNumber<Data_>::operator std::string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(PRINT_PRECISION); // Set fixed-point notation and precision
    oss << primary_ << " + " << " ϵ ( " << dual_ << " )";
    return oss.str();
}

// Service functions

/**
 * @brief Compute the conjugate of this DualNumber.
 * @return The conjugate of the DualNumber.
 */
template<typename Data_>
DualNumber<Data_> DualNumber<Data_>::conj() const noexcept {
    return DualNumber(primary_, -dual_);
}

/**
 * @brief Convert this DualNumber to a string.
 * @return A string representation of the DualNumber.
 */
template<typename Data_>
std::string DualNumber<Data_>::to_string() const {
    return operator std::string();
}

/**
 * @brief Extract the primary part of the dual quaternion.
 * @return The primary part as a new DualNumber.
 */
template<typename Data_>
Data_ DualNumber<Data_>::P() const noexcept {
    return primary_;
}

/**
 * @brief Extract the dual part of the dual quaternion.
 * @return The dual part as a new DualNumber.
 */
template<typename Data_>
Data_ DualNumber<Data_>::D() const noexcept {
    return dual_;
}

    
}