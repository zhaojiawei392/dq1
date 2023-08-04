#pragma once
#include <string>

namespace dq1
{
template<typename Scalar_> class AbstractComplex;
template<typename Scalar_> class AbstractReal;
template<typename Scalar_> class AbstractImaginary;
template<typename Scalar_> class Re;
template<typename Scalar_> class Im;
template<typename Scalar_> class Imx;
template<typename Scalar_> class Imy;
template<typename Scalar_> class Imz;

template<typename Scalar_> class AbstractComplex
{
public:
    AbstractComplex() noexcept;
    AbstractComplex(const Scalar_& sca) noexcept;
    AbstractComplex(Scalar_&& sca) noexcept;
    virtual ~AbstractComplex() noexcept=0;

    AbstractComplex& operator=(const AbstractComplex& ac) noexcept;
    AbstractComplex& operator=(AbstractComplex&& ac) noexcept;

    operator Scalar_() const noexcept;
    operator std::string() const noexcept;

protected:
    Scalar_ scalar_;
};

template<typename Scalar_> class AbstractReal : public AbstractComplex<Scalar_>
{
public:
    AbstractReal() noexcept;
    AbstractReal(const Scalar_& sca) noexcept;
    AbstractReal(Scalar_&& sca) noexcept;
    virtual ~AbstractReal() noexcept=0;

    AbstractReal<Scalar_>& operator=(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractReal<Scalar_>& operator=(AbstractReal<Scalar_>&& ar) noexcept;

    AbstractReal<Scalar_>      operator+(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractReal<Scalar_>      operator-(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractReal<Scalar_>      operator*(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractImaginary<Scalar_> operator*(const AbstractImaginary<Scalar_>& ai) noexcept;
    AbstractReal<Scalar_>      operator/(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractImaginary<Scalar_> operator/(const AbstractImaginary<Scalar_>& ai) noexcept;
};

template<typename Scalar_> class AbstractImaginary: public AbstractComplex<Scalar_>
{
public:
    AbstractImaginary() noexcept;
    AbstractImaginary(const Scalar_& sca) noexcept;
    AbstractImaginary(Scalar_&& sca) noexcept;
    virtual ~AbstractImaginary() noexcept=0;

    AbstractImaginary<Scalar_>& operator=(const AbstractImaginary<Scalar_>& ai) noexcept;
    AbstractImaginary<Scalar_>& operator=(AbstractImaginary<Scalar_>&& ai) noexcept;


    AbstractImaginary<Scalar_> operator+(const AbstractImaginary<Scalar_>& ai) noexcept;
    AbstractImaginary<Scalar_> operator-(const AbstractImaginary<Scalar_>& ai) noexcept;
    AbstractReal<Scalar_>      operator*(const AbstractImaginary<Scalar_>& ai) noexcept;
    AbstractImaginary<Scalar_> operator*(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractImaginary<Scalar_> operator/(const AbstractReal<Scalar_>& ar) noexcept;
    AbstractReal<Scalar_>      operator/(const AbstractImaginary<Scalar_>& ai) noexcept;

};

template<typename Scalar_> class Re : public AbstractReal<Scalar_>
{
public:
    Re() noexcept;
    Re(const Scalar_& sca) noexcept;
    Re(Scalar_&& sca) noexcept;
    ~Re() noexcept;

    Re& operator=(const Re& re) noexcept;
    Re& operator=(Re&& re) noexcept;
};

template<typename Scalar_> class Im : public AbstractImaginary<Scalar_>
{
public:
    Im() noexcept;
    Im(const Scalar_& sca) noexcept;
    Im(Scalar_&& sca) noexcept;
    ~Im() noexcept;

    Im& operator=(const Im& im) noexcept;
    Im& operator=(Im&& im) noexcept;
};

template<typename Scalar_> class Imx : public Im<Scalar_>
{
public:
    Imx() noexcept;
    Imx(const Scalar_& sca) noexcept;
    Imx(Scalar_&& sca) noexcept;
    ~Imx() noexcept;

    Imx& operator=(const Imx& imx) noexcept;
    Imx& operator=(Imx&& imx) noexcept;

    Imy operator*(const Imz& imz) noexcept;
    Imz operator*(const Imy& imy) noexcept;

};

template<typename Scalar_> class Imy : public Im<Scalar_>
{
public:
    Imy() noexcept;
    Imy(const Scalar_& sca) noexcept;
    Imy(Scalar_&& sca) noexcept;
    ~Imy() noexcept;

    Imy& operator=(const Imy& imy) noexcept;
    Imy& operator=(Imy&& imy) noexcept;

    Imz operator*(const Imx& imx) noexcept;
    Imx operator*(const Imz& imz) noexcept;

};

template<typename Scalar_> class Imz : public Im<Scalar_>
{
public:
    Imz() noexcept;
    Imz(const Scalar_& sca) noexcept;
    Imz(Scalar_&& sca) noexcept;
    ~Imz() noexcept;

    Imz& operator=(const Imz& imz) noexcept;
    Imz& operator=(Imz&& imz) noexcept;

    Imx operator*(const Imy& imy) noexcept;
    Imy operator*(const Imx& imx) noexcept;

};

}