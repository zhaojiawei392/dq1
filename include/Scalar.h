

namespace dq1
{

template<typename Scalar_>
class AbstractComplex
{
public:
    AbstractComplex(Scalar_&& ac);

    Virtual AbstractComplex operator+(const AbstractComplex& ac)=0 const;
    Virtual AbstractComplex operator-(const AbstractComplex& ac)=0 const;
    Virtual AbstractComplex operator*(const AbstractComplex& ac)=0 const;
    Virtual AbstractComplex operator/(const Scalar_& s)=0 const;
private:
    Scalar_ scalar_;
};

template<typename Scalar_>
class Re : public AbstractComplex
{
public:
    Re()
};

template<typename Scalar_>
class Imx : public AbstractComplex
{

};

template<typename Scalar_>
class Imy : public AbstractComplex
{

};

template<typename Scalar_>
class Imz : public AbstractComplex
{

};

}