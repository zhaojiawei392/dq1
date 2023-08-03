#include "Complex.h"

namespace dq1
{

template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex() noexcept: scalar_(0) {}
template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex(const Scalar_& scalar) noexcept: scalar_(scalar) {}
template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex(Scalar_&& scalar) noexcept: scalar_(std::move(scalar)) {}

template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal() noexcept: AbstractComplex() {}
template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal(const Scalar_& scalar) noexcept: AbstractComplex(scalar) {}
template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal(Scalar_&& scalar) noexcept: AbstractComplex(std::move(scalar)) {}

template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary() noexcept: AbstractComplex() {}
template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary(const Scalar_& scalar) noexcept: AbstractComplex(scalar) {}
template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary(Scalar_&& scalar) noexcept: AbstractComplex(std::move(scalar)) {}

template<typename Scalar_> Re<Scalar_>::Re() noexcept: AbstractReal() {}
template<typename Scalar_> Im<Scalar_>::Im() noexcept: AbstractImaginary() {}
template<typename Scalar_> Imx<Scalar_>::Imx() noexcept: Im() {}
template<typename Scalar_> Imy<Scalar_>::Imy() noexcept: Im() {}
template<typename Scalar_> Imz<Scalar_>::Imz() noexcept: Im() {}

template<typename Scalar_> Re<Scalar_> Re<Scalar_>::operator+(const Re<Scalar_>& re) noexcept{
    return Re<Scalar_>(scalar_ + re.scalar_);
}

}