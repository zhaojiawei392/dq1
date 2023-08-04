#include "Complex.h"

namespace dq1
{

template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex() noexcept: scalar_(0) {}
template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex(const Scalar_& scalar) noexcept: scalar_(scalar) {}
template<typename Scalar_> AbstractComplex<Scalar_>::AbstractComplex(Scalar_&& scalar) noexcept: scalar_(std::move(scalar)) {}
template<typename Scalar_> AbstractComplex<Scalar_>::~AbstractComplex<Scalar_>() noexcept {}
template<typename Scalar_> AbstractComplex<Scalar_>& AbstractComplex<Scalar_>::operator=(const AbstractComplex<Scalar_>& ac) noexcept {
    if (this != &ac){
        scalar_ = ac.scalar_;
    }
    return *this;
}
template<typename Scalar_> AbstractComplex<Scalar_>& AbstractComplex<Scalar_>::operator=(AbstractComplex<Scalar_>&& ac) noexcept {
    if (this != &ac){
        scalar_ = std::move(ac.scalar_);
    }
    return *this;
}
template<typename Scalar_> AbstractComplex<Scalar_>::operator Scalar_() const noexcept {
    return scalar_;
}
template<typename Scalar_> AbstractComplex<Scalar_>::operator std::string() const noexcept {
    return std::to_string(scalar_);
}

template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal() noexcept: AbstractComplex<Scalar_>() {}
template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal(const Scalar_& scalar) noexcept: AbstractComplex<Scalar_>(scalar) {}
template<typename Scalar_> AbstractReal<Scalar_>::AbstractReal(Scalar_&& scalar) noexcept: AbstractComplex<Scalar_>(std::move(scalar)) {}
template<typename Scalar_> AbstractReal<Scalar_>::~AbstractReal<Scalar_>() noexcept {}
template<typename Scalar_> AbstractReal<Scalar_>& AbstractReal<Scalar_>::operator=(const AbstractReal<Scalar_>& ar) noexcept {
    AbstractComplex<Scalar_>::operator=(ar);
    return *this;
}
template<typename Scalar_> AbstractReal<Scalar_>& AbstractReal<Scalar_>::operator=(AbstractReal<Scalar_>&& ar) noexcept {
    AbstractComplex<Scalar_>::operator=(std::move(ar));
    return *this;
}

template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary() noexcept: AbstractComplex() {}
template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary(const Scalar_& scalar) noexcept: AbstractComplex(scalar) {}
template<typename Scalar_> AbstractImaginary<Scalar_>::AbstractImaginary(Scalar_&& scalar) noexcept: AbstractComplex(std::move(scalar)) {}
template<typename Scalar_> AbstractImaginary<Scalar_>::~AbstractImaginary<Scalar_>() noexcept {}
template<typename Scalar_> AbstractImaginary<Scalar_>& AbstractImaginary<Scalar_>::operator=(const AbstractImaginary<Scalar_>& ai) noexcept {
    AbstractComplex<Scalar_>::operator=(ai);
    return *this;
}
template<typename Scalar_> AbstractImaginary<Scalar_>& AbstractImaginary<Scalar_>::operator=(AbstractImaginary<Scalar_>&& ai) noexcept {
    AbstractComplex<Scalar_>::operator=(std::move(ai));
    return *this;
}

template<typename Scalar_> Re<Scalar_>::Re() noexcept: AbstractReal() {}
template<typename Scalar_> Im<Scalar_>::Im() noexcept: AbstractImaginary() {}
template<typename Scalar_> Imx<Scalar_>::Imx() noexcept: Im() {}
template<typename Scalar_> Imy<Scalar_>::Imy() noexcept: Im() {}
template<typename Scalar_> Imz<Scalar_>::Imz() noexcept: Im() {}

}