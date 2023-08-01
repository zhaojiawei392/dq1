

namespace dq1
{

template<typename Scalar_>
class AbstractComplex
{
public:
    AbstractComplex() noexcept;
    AbstractComplex(const Scalar_& ac)=default noexcept;
    AbstractComplex(Scalar_&& ac)=default noexcept;

    AbstractComplex& operator=(const AbstractComplex& ac)=0 noexcept;
    AbstractComplex& operator=(AbstractComplex&& ac)=0 noexcept;

    AbstractComplex operator+(const AbstractComplex& ac)=0 const noexcept;
    AbstractComplex operator-(const AbstractComplex& ac)=0 const noexcept;
    AbstractComplex operator*(const AbstractComplex& ac)=0 const noexcept;
    AbstractComplex operator/(const Scalar_& s)=0 const noexcept;

    virtual ~AbstractComplex()=default noexcept;
private:
    Scalar_ scalar_;
};

template<typename Scalar_>
class AbstractReal : public AbstractComplex
{
public:
    AbstractReal(Scalar_&& ac);
};

template<typename Scalar_>
class AbstractImaginary: public AbstractComplex
{
public:
    AbstractImaginary();
};

}