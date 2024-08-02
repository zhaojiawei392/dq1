#pragma once
#include "Quaternion.hpp"


namespace dq1
{

template<typename dqScalar_>
class DualQuaternion{
protected:
    Quaternion<dqScalar_> primary_;
    Quaternion<dqScalar_> dual_;
public:

    // Constructors and Assignments

    DualQuaternion();
    DualQuaternion(const Quaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual);
    DualQuaternion(const Quaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual=Quaternion<dqScalar_>());
    DualQuaternion(Quaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual);
    DualQuaternion(Quaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual=Quaternion<dqScalar_>());
    explicit DualQuaternion(const Vec8<dqScalar_>& vec8);
    explicit DualQuaternion(Vec8<dqScalar_>&& vec8);
    DualQuaternion(const dqScalar_& h0, const dqScalar_& h1=0, const dqScalar_& h2=0, const dqScalar_& h3=0, const dqScalar_& h4=0, const dqScalar_& h5=0, const dqScalar_& h6=0, const dqScalar_& h7=0);
    
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
    dqScalar_ operator[](int index) const;
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
    Mat8<dqScalar_> hamiplus() const noexcept;
    Mat8<dqScalar_> haminus() const noexcept;
    Quaternion<dqScalar_> primary() const noexcept;
    Quaternion<dqScalar_> dual() const noexcept;
    Vec6<dqScalar_> vec6() const noexcept;
    Vec8<dqScalar_> vec8() const noexcept;
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

};


template<typename dqScalar_>
class UnitDualQuaternion: public DualQuaternion<dqScalar_>{
public:

    // Constructors and Assignments

    UnitDualQuaternion()=delete;
    UnitDualQuaternion(const UnitQuaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual);
    UnitDualQuaternion(const UnitQuaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual);
    UnitDualQuaternion(UnitQuaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual);
    UnitDualQuaternion(UnitQuaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual);
    explicit UnitDualQuaternion(const Vec8<dqScalar_>& vec8); 
    explicit UnitDualQuaternion(Vec8<dqScalar_>&& vec8);
    UnitDualQuaternion(const dqScalar_& h0, const dqScalar_& h1=0, const dqScalar_& h2=0, const dqScalar_& h3=0, const dqScalar_& h4=0, const dqScalar_& h5=0, const dqScalar_& h6=0, const dqScalar_& h7=0);
    
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


// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class DualQuaternion *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************

// Default constructor
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(): primary_(), dual_() {}

/**
 * @brief Construct a DualQuaternion from two quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Quaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual)
    : primary_(primary), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from a lvalue primary and rvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Quaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual)
    : primary_(primary), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a rvalue primary and lvalue dual quaternion.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Quaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual)
    : primary_(std::move(primary)), dual_(dual) {}

/**
 * @brief Construct a DualQuaternion from two rvalue quaternions.
 * @param primary The primary part of the dual quaternion.
 * @param dual The dual part of the dual quaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Quaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual)
    : primary_(std::move(primary)), dual_(std::move(dual)) {}

/**
 * @brief Construct a DualQuaternion from a Vec8.
 * @param vec8 A Vec8 representing the dual quaternion values.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const Vec8<dqScalar_>& vec8)
    : primary_(vec8.template head<4>()), dual_(vec8.template tail<4>()) {}

/**
 * @brief Construct a DualQuaternion from an rvalue Vec8.
 * @param vec8 An rvalue Vec8 representing the dual quaternion values.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(Vec8<dqScalar_>&& vec8)
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
template<typename dqScalar_>
DualQuaternion<dqScalar_>::DualQuaternion(const dqScalar_& h0, const dqScalar_& h1, const dqScalar_& h2, const dqScalar_& h3,
                                        const dqScalar_& h4, const dqScalar_& h5, const dqScalar_& h6, const dqScalar_& h7)     
                                        : primary_(h0, h1, h2, h3), dual_(h4, h5, h6, h7) {}

// Mutable operators    

template<typename dqScalar_>
DualQuaternion<dqScalar_>& DualQuaternion<dqScalar_>::operator+=(const DualQuaternion& other) noexcept{
    primary_ += other.primary_;
    dual_ += other.dual_;
    return *this;
}

template<typename dqScalar_>
DualQuaternion<dqScalar_>& DualQuaternion<dqScalar_>::operator-=(const DualQuaternion& other) noexcept{
    primary_ -= other.primary_;
    dual_ -= other.dual_;
    return *this;
}

template<typename dqScalar_>
DualQuaternion<dqScalar_>& DualQuaternion<dqScalar_>::operator*=(const DualQuaternion& other) noexcept{
    primary_ *= other.primary_;
    dual_ = primary_ * other.dual_ + dual_ * other.primary_;
    return *this;
}

template<typename dqScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<dqScalar_>&> 
DualQuaternion<dqScalar_>::operator*=(const Scalar_& scalar) noexcept{
    primary_ *= scalar;
    dual_ *= scalar;
    return *this;
}

template<typename dqScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<dqScalar_>&> 
DualQuaternion<dqScalar_>::operator/=(const Scalar_& scalar) noexcept{
    primary_ /= scalar;
    dual_ /= scalar;
    return *this;
}

/**
 * @brief Normalize this DualQuaternion.
 * @return A reference to the normalized DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_>& DualQuaternion<dqScalar_>::normalize() noexcept {
    const dqScalar_& norm = primary_.norm();
    primary_ /= norm;
    dual_ /= norm;
    return *this;
}

// Const operator    

template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::operator+(const DualQuaternion<dqScalar_>& other) const noexcept{
    return DualQuaternion<dqScalar_>(primary_ + other.primary_, dual_ + other.dual_);
}
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::operator-(const DualQuaternion<dqScalar_>& other) const noexcept{
    return DualQuaternion<dqScalar_>(primary_ - other.primary_, dual_ - other.dual_);
}
template<typename dqScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<dqScalar_>> 
DualQuaternion<dqScalar_>::operator/(const Scalar_& scalar) const noexcept{
    return DualQuaternion<dqScalar_>(primary_ / scalar, dual_ / scalar);
}
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::operator-() const noexcept{
    return DualQuaternion<dqScalar_>(-primary_, -dual_);
}

template<typename dqScalar_>
dqScalar_ DualQuaternion<dqScalar_>::operator[](int index) const{
    return vec8()[index];
}
template<typename dqScalar_>
bool DualQuaternion<dqScalar_>::operator==(const DualQuaternion<dqScalar_>& other) const noexcept{
    return primary_ == other.primary_ && dual_ =+ other.dual_;
}
template<typename dqScalar_>
bool DualQuaternion<dqScalar_>::operator!=(const DualQuaternion<dqScalar_>& other) const noexcept{
    return primary_ != other.primary_ || dual_ != other.dual_;
}
template<typename dqScalar_>
DualQuaternion<dqScalar_>::operator std::string() const{
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
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::norm() const noexcept {
    if (primary_.norm() == 0) 
        return DualQuaternion<dqScalar_>();
    const dqScalar_& primary_norm = primary_.norm();
    const dqScalar_& dual_norm = primary_.vec4().dot(dual_.vec4()) / primary_norm;

    return DualQuaternion<dqScalar_>(Quaternion<dqScalar_>(primary_norm), Quaternion<dqScalar_>(dual_norm));
}

/**
 * @brief Compute the conjugate of this DualQuaternion.
 * @return The conjugate of the DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::conj() const noexcept {
    return DualQuaternion<dqScalar_>(primary_.conj(), dual_.conj());
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

template<typename dqScalar_>
template<typename Scalar_>
std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuaternion<dqScalar_>>
DualQuaternion<dqScalar_>::pow(const Scalar_& index) const noexcept{

}

/**
 * @brief Return a normalized version of this DualQuaternion.
 * @return A normalized DualQuaternion.
 */
template<typename dqScalar_>
DualQuaternion<dqScalar_> DualQuaternion<dqScalar_>::normalized() const noexcept {
    return operator/=(primary_.norm());
}

/**
 * @brief Compute the Hamilton plus matrix of this DualQuaternion.
 * @return The Hamilton plus matrix of the DualQuaternion.
 */
template<typename dqScalar_>
Mat8<dqScalar_> DualQuaternion<dqScalar_>::hamiplus() const noexcept {
    return (Mat8<dqScalar_>() << primary_.hamiplus(), Mat4<dqScalar_>::Zero(), dual_.hamiplus(), primary_.haminus()).finished();
}

/**
 * @brief Compute the Hamilton minus matrix of this DualQuaternion.
 * @return The Hamilton minus matrix of the DualQuaternion.
 */
template<typename dqScalar_>
Mat8<dqScalar_> DualQuaternion<dqScalar_>::haminus() const noexcept {
    return (Mat8<dqScalar_>() << primary_.haminus(), primary_.hamiplus(), dual_.haminus(), Mat4<dqScalar_>::Zero()).finished();
}

template<typename dqScalar_>
Quaternion<dqScalar_> DualQuaternion<dqScalar_>::primary() const noexcept{
    return primary_;
}

template<typename dqScalar_>
Quaternion<dqScalar_> DualQuaternion<dqScalar_>::dual() const noexcept{
    return dual_;
}

/**
 * @brief Convert this DualQuaternion to a Vec6 representation.
 * @return A Vec6 representation of the DualQuaternion.
 */
template<typename dqScalar_>
Vec6<dqScalar_> DualQuaternion<dqScalar_>::vec6() const noexcept {
    return (Vec6<dqScalar_>() << primary_.vec3(), dual_.vec3()).finished();
}

/**
 * @brief Convert this DualQuaternion to a Vec8 representation.
 * @return A Vec8 representation of the DualQuaternion.
 */
template<typename dqScalar_>
Vec8<dqScalar_> DualQuaternion<dqScalar_>::vec8() const noexcept {
    return (Vec8<dqScalar_>() << primary_.vec4(), dual_.vec4()).finished();
}

template<typename dqScalar_>
std::string DualQuaternion<dqScalar_>::to_string() const{
    return operator std::string();
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Class UnitDualQuaternion *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************



// Constructors and Assignments

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(const UnitQuaternion<dqScalar_>& primary, const Quaternion<dqScalar_>& dual)
    : DualQuaternion<dqScalar_>(primary, dual){ } 

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(const UnitQuaternion<dqScalar_>& primary, Quaternion<dqScalar_>&& dual)
    : DualQuaternion<dqScalar_>(primary, std::move(dual)){ }

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(UnitQuaternion<dqScalar_>&& primary, const Quaternion<dqScalar_>& dual)
    : DualQuaternion<dqScalar_>(std::move(primary), dual){ }

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(UnitQuaternion<dqScalar_>&& primary, Quaternion<dqScalar_>&& dual)
    : DualQuaternion<dqScalar_>(std::move(primary), std::move(dual)){ }

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(const Vec8<dqScalar_>& vec8)
    : DualQuaternion<dqScalar_>( UnitQuaternion<dqScalar_>(vec8.template head<4>()), Quaternion<dqScalar_>(vec8.template tail<4>()) ){ }

template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(Vec8<dqScalar_>&& vec8) 
    : DualQuaternion<dqScalar_>( UnitQuaternion<dqScalar_>(std::move(vec8.template head<4>())), Quaternion<dqScalar_>(std::move(vec8.template tail<4>())) ){ }
template<typename dqScalar_>
UnitDualQuaternion<dqScalar_>::UnitDualQuaternion(const dqScalar_& h0, const dqScalar_& h1, const dqScalar_& h2, const dqScalar_& h3, const dqScalar_& h4, const dqScalar_& h5, const dqScalar_& h6, const dqScalar_& h7)
    : DualQuaternion<dqScalar_>( UnitQuaternion<dqScalar_>(h0, h1, h2, h3), Quaternion<dqScalar_>(h4, h5, h6, h7) ){ }
    
    
}