#pragma once
#include <string>

namespace dq1
{
template<typename Scalar_> class Re;
template<typename Scalar_> class Imx;
template<typename Scalar_> class Imy;
template<typename Scalar_> class Imz;

template<typename Scalar_> class AbstractComplex
{
public:
    AbstractComplex() noexcept;
    AbstractComplex(const Scalar_& sca) noexcept;
    AbstractComplex(Scalar_&& sca) noexcept;

    AbstractComplex& operator=(const AbstractComplex& ac) noexcept;
    AbstractComplex& operator=(AbstractComplex&& ac) noexcept;

    virtual operator Scalar_() const noexcept =0;

    virtual ~AbstractComplex() noexcept;
protected:
    Scalar_ scalar_;
};

template<typename Scalar_> class AbstractReal : public AbstractComplex
{
public:
    AbstractReal() noexcept;
    AbstractReal(const Scalar_& sca) noexcept;
    AbstractReal(Scalar_&& sca) noexcept;

    AbstractReal& operator=(const AbstractReal& ar) noexcept;
    AbstractReal& operator=(AbstractReal&& ar) noexcept;

    virtual operator Scalar_() const noexcept=0;

    virtual ~AbstractReal() noexcept;
};

template<typename Scalar_> class AbstractImaginary: public AbstractComplex
{
public:
    AbstractImaginary() noexcept;
    AbstractImaginary(const Scalar_& sca) noexcept;
    AbstractImaginary(Scalar_&& sca) noexcept;

    AbstractImaginary& operator=(const AbstractImaginary& ai) noexcept;
    AbstractImaginary& operator=(AbstractImaginary&& ai) noexcept;

    virtual operator Scalar_() const noexcept=0;

    virtual ~AbstractImaginary() noexcept;
};

template<typename Scalar_> class Re : public AbstractReal
{
public:
    Re() noexcept;
    Re(const Scalar_& sca) noexcept;
    Re(Scalar_&& sca) noexcept;

    Re& operator=(const Re& re) noexcept;
    Re& operator=(Re&& re) noexcept;

    Re operator+(const Re& re) noexcept;
    Re operator-(const Re& re) noexcept;
    Re operator*(const Re& re) noexcept;
    Re operator/(const Re& re) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Re() noexcept;
};

template<typename Scalar_> class Im : public AbstractImaginary
{
public:
    Im() noexcept;
    Im(const Scalar_& sca) noexcept;
    Im(Scalar_&& sca) noexcept;

    Im& operator=(const Im& im) noexcept;
    Im& operator=(Im&& im) noexcept;

    Im operator+(const Im& im) noexcept;
    Im operator-(const Im& im) noexcept;
    Re operator*(const Im& im) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Im() noexcept;
};

template<typename Scalar_> class Imx : public Im
{
public:
    Imx() noexcept;
    Imx(const Scalar_& sca) noexcept;
    Imx(Scalar_&& sca) noexcept;

    Imx& operator=(const Imx& imx) noexcept;
    Imx& operator=(Imx&& imx) noexcept;

    Imy operator*(const Imz& imz) noexcept;
    Imz operator*(const Imy& imy) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imx() noexcept;
};

template<typename Scalar_> class Imy : public Im
{
public:
    Imy() noexcept;
    Imy(const Scalar_& sca) noexcept;
    Imy(Scalar_&& sca) noexcept;

    Imy& operator=(const Imy& imy) noexcept;
    Imy& operator=(Imy&& imy) noexcept;

    Imz operator*(const Imx& imx) noexcept;
    Imx operator*(const Imz& imz) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imy() noexcept;
};

template<typename Scalar_> class Imz : public Im
{
public:
    Imz() noexcept;
    Imz(const Scalar_& sca) noexcept;
    Imz(Scalar_&& sca) noexcept;

    Imz& operator=(const Imz& imz) noexcept;
    Imz& operator=(Imz&& imz) noexcept;

    Imx operator*(const Imy& imy) noexcept;
    Imy operator*(const Imx& imx) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imz() noexcept;
};

}