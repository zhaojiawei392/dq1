#include <iostream>
#include <eigen3/Eigen/Dense>

namespace dq1
{
template<typename Scalar_, int size>
using Evec=Eigen::Matrix<Scalar_, size, 1>;

template<typename Scalar_, int rows_, int cols_>
using Emtx=Eigen::Matrix<Scalar_, rows_, cols_>;

template<typename Scalar_>
class Quaternion{
public:
    Quaternion(const Evec<Scalar_, 3>& xyz);
    Quaternion(const Evec<Scalar_, 4>& wxyz);
    Quaternion(Evec<Scalar_, 3>&& xyz);
    Quaternion(Evec<Scalar_, 4>&& wxyz);

    // Constructor lvalue
    Quaternion(const Scalar_& w=Scalar_(), const Scalar_& x=Scalar_(), const Scalar_& y=Scalar_(), const Scalar_& z=Scalar_());
    // Constructor rvalue
    Quaternion(Scalar_&& w, Scalar_&& x, Scalar_&& y, Scalar_&& z);
    Quaternion(Scalar_&& w, Scalar_&& x, Scalar_&& y);
    Quaternion(Scalar_&& w, Scalar_&& x);
    Quaternion(Scalar_&& w);

    // Destructor
    ~Quaternion();

    Scalar_ w() const;
    Scalar_ x() const;
    Scalar_ y() const;
    Scalar_ z() const;

    /**
     * @brief Calculates the sum of this quaternion and argument quaternion.
     * This function takes one quaternion and returns the sum of this quaternion and argument quaternion.
     * @param quaternion The argument quaternion.
     * @return the sum of this quaternion and argument quaternion.
     */
    Quaternion operator+(Quaternion&& quaternion);
    Quaternion operator-(Quaternion&& quaternion);
    Quaternion operator*(Quaternion&& quaternion);
    Quaternion operator*(Scalar_&& scalar);
    Quaternion operator/(Scalar_&& scalar);



protected:
    Scalar_ w_, x_, y_, z_;

};

}