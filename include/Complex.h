#pragma once
#include <string>

namespace dq1
{
template<typename Scalar_> class Re;
template<typename Scalar_> class Imx;
template<typename Scalar_> class Imy;
template<typename Scalar_> class Imz;

template<typename Scalar_>
class AbstractComplex
{
public:
    explicit AbstractComplex() noexcept;
    explicit AbstractComplex(const Scalar_& sca)=default noexcept;
    explicit AbstractComplex(Scalar_&& sca)=default noexcept;

    AbstractComplex& operator=(const AbstractComplex& ac)=default noexcept;
    AbstractComplex& operator=(AbstractComplex&& ac)=default noexcept;

    virtual operator Scalar_() const noexcept =0;

    virtual ~AbstractComplex()=default noexcept;
protected:
    Scalar_ scalar_;
};

template<typename Scalar_>
class AbstractReal : public AbstractComplex
{
public:
    explicit AbstractReal() noexcept;
    explicit AbstractReal(const Scalar_& sca)=default noexcept;
    explicit AbstractReal(Scalar_&& sca)=default noexcept;

    AbstractReal& operator=(const AbstractReal& ar)=default noexcept;
    AbstractReal& operator=(AbstractReal&& ar)=default noexcept;

    virtual operator Scalar_() const noexcept=0;

    virtual ~AbstractReal()=default noexcept;
};

template<typename Scalar_>
class AbstractImaginary: public AbstractComplex
{
public:
    explicit AbstractImaginary() noexcept;
    explicit AbstractImaginary(const Scalar_& sca)=default noexcept;
    explicit AbstractImaginary(Scalar_&& sca)=default noexcept;

    AbstractImaginary& operator=(const AbstractImaginary& ai)=default noexcept;
    AbstractImaginary& operator=(AbstractImaginary&& ai)=default noexcept;

    virtual operator Scalar_() const noexcept=0;

    virtual ~AbstractImaginary()=default noexcept;
};

template<typename Scalar_>
class Re : public AbstractReal
{
public:
    explicit Re() noexcept;
    explicit Re(const Scalar_& sca)=default noexcept;
    explicit Re(Scalar_&& sca)=default noexcept;

    Re& operator=(const Re& re)=default noexcept;
    Re& operator=(Re&& re)=default noexcept;

    Re operator+(const Re& re) noexcept;
    Re operator-(const Re& re) noexcept;
    Re operator*(const Re& re) noexcept;
    Re operator/(const Re& re) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Re()=default noexcept;
};

template<typename Scalar_>
class Im : public AbstractImaginary
{
public:
    explicit Im() noexcept;
    explicit Im(const Scalar_& sca)=default noexcept;
    explicit Im(Scalar_&& sca)=default noexcept;

    Im& operator=(const Im& im)=default noexcept;
    Im& operator=(Im&& im)=default noexcept;

    Im operator+(const Im& im) noexcept;
    Im operator-(const Im& im) noexcept;
    Im operator*(const Im& im) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Im()=default noexcept;
};

template<typename Scalar_>
class Imx : public Im
{
public:
    explicit Imx() noexcept;
    explicit Imx(const Scalar_& sca)=default noexcept;
    explicit Imx(Scalar_&& sca)=default noexcept;

    Imx& operator=(const Imx& imx)=default noexcept;
    Imx& operator=(Imx&& imx)=default noexcept;

    Imy operator*(const Imz& imz) noexcept;
    Imz operator*(const Imy& imy) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imx()=default noexcept;
};

template<typename Scalar_>
class Imy : public Im
{
public:
    explicit Imy() noexcept;
    explicit Imy(const Scalar_& sca)=default noexcept;
    explicit Imy(Scalar_&& sca)=default noexcept;

    Imy& operator=(const Imy& imy)=default noexcept;
    Imy& operator=(Imy&& imy)=default noexcept;

    Imz operator*(const Imx& imx) noexcept;
    Imx operator*(const Imz& imz) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imy()=default noexcept;
};

template<typename Scalar_>
class Imz : public Im
{
public:
    explicit Imz() noexcept;
    explicit Imz(const Scalar_& sca)=default noexcept;
    explicit Imz(Scalar_&& sca)=default noexcept;

    Imz& operator=(const Imz& imz)=default noexcept;
    Imz& operator=(Imz&& imz)=default noexcept;

    Imx operator*(const Imy& imy) noexcept;
    Imy operator*(const Imx& imx) noexcept;

    virtual operator Scalar_() const noexcept;

    virtual ~Imz()=default noexcept;
};

}