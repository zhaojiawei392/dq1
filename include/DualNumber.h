#pragma once
#include <iostream>

namespace dq1
{

template<typename Number_>
class DualNumber{
protected:
    Number_ primary_; //primary part
    Number_ dual_; // dual part
public:

    // Constructors and Assignments

    DualNumber();
    DualNumber(const Number_& primary, const Number_& dual);
    DualNumber(const Number_& primary, Number_&& dual);
    DualNumber(Number_&& primary, const Number_& dual);
    DualNumber(Number_&& primary, Number_&& dual);

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
    DualNumber& normalize();

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

    // service functions
    template<typename Scalar_>
    Scalar_ norm() const noexcept;
    DualNumber conj() const noexcept;
    DualNumber inv() const noexcept;
    DualNumber ln() const noexcept;
    DualNumber exp() const noexcept;
    template<typename Scalar_>
    template<typename Scalar_>
    DualNumber pow(const Scalar_& index) const noexcept;
    DualNumber normalized() const noexcept;
    std::string to_string() const;

    Number_ P() const noexcept;
    Number_ D() const noexcept;

    // Friends "const"
    
    template<typename Scalar_, typename fNumber_>
    friend DualNumber<fNumber_> operator*(const Scalar_& scalar, const DualNumber<fNumber_>& dual_number);
    template<typename Scalar_, typename fNumber_>
    friend DualNumber<fNumber_> operator*(const Scalar_& scalar, DualNumber<fNumber_>&& dual_number);    
    template<typename fNumber_>
    friend std::ostream& operator<<(std::ostream& os, const DualNumber<fNumber_>& q);
};

}