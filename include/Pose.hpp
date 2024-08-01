#pragma once
#include "DualQuaternion.hpp"

namespace dq1{

template<typename rScalar_>
using Rotation = UnitQuaternion<rScalar_>;
template<typename tScalar_>
using Translation = PureQuaternion<tScalar_>;
template<typename aScalar_>
using UnitAxis = UnitPureQuaternion<aScalar_>;


template<typename pScalar_>
class Pose: public UnitDualQuaternion<pScalar_>{
public:

    // Constructors and Assignments

    Pose(const UnitQuaternion<pScalar_>& primary, const Quaternion<pScalar_>& dual);
    Pose(const UnitQuaternion<pScalar_>& primary, Quaternion<pScalar_>&& dual=Quaternion<pScalar_>());
    Pose(UnitQuaternion<pScalar_>&& primary, const Quaternion<pScalar_>& dual);
    Pose(UnitQuaternion<pScalar_>&& primary, Quaternion<pScalar_>&& dual=Quaternion<pScalar_>());
    explicit Pose(const Vec8<pScalar_>& vec8); 
    explicit Pose(Vec8<pScalar_>&& vec8);
    Pose(const pScalar_& h0, const pScalar_& h1=0, const pScalar_& h2=0, const pScalar_& h3=0, const pScalar_& h4=0, const pScalar_& h5=0, const pScalar_& h6=0, const pScalar_& h7=0);
    
            Pose()=delete;
            Pose(const Pose& other)=default;
            Pose(Pose& other)=default;
    virtual ~Pose()=default;
    Pose& operator=(const Pose& other)=default;
    Pose& operator=(Pose&& other)=default;


    template<typename First_, typename... Args_>
    static Pose build_from(First_ first, Args_... args){
        return build_from(first) * build_from(args...);
    }
    static Pose build_from(Rotation<pScalar_> rotation){
        return Pose(rotation);
    }
    static Pose build_from(Translation<pScalar_>  translation){
        return Pose(Rotation<pScalar_> (1), translation / 2);
    }
    static Pose build_from(Pose pose){
        return pose;
    }
    





};


}




// ***********************************************************************************************************************
// ***********************************************************************************************************************
// Implementations *******************************************************************************************************
// ***********************************************************************************************************************
// ***********************************************************************************************************************


namespace dq1{

template<typename pScalar_>
Pose<pScalar_>::Pose(const UnitQuaternion<pScalar_>& primary, const Quaternion<pScalar_>& dual)
    : UnitDualQuaternion<pScalar_>(primary, dual){}

template<typename pScalar_>
Pose<pScalar_>::Pose(const UnitQuaternion<pScalar_>& primary, Quaternion<pScalar_>&& dual)
    : UnitDualQuaternion<pScalar_>(primary, std::move(dual)){}

template<typename pScalar_>
Pose<pScalar_>::Pose(UnitQuaternion<pScalar_>&& primary, const Quaternion<pScalar_>& dual)
    : UnitDualQuaternion<pScalar_>(std::move(primary), dual){}

template<typename pScalar_>
Pose<pScalar_>::Pose(UnitQuaternion<pScalar_>&& primary, Quaternion<pScalar_>&& dual)
    : UnitDualQuaternion<pScalar_>(std::move(primary), std::move(dual)){}

template<typename pScalar_>
Pose<pScalar_>::Pose(const Vec8<pScalar_>& vec8)
    : UnitDualQuaternion<pScalar_>( vec8 ){ }

template<typename pScalar_> 
Pose<pScalar_>::Pose(Vec8<pScalar_>&& vec8)
    : UnitDualQuaternion<pScalar_>( std::move(vec8) ){ }

template<typename pScalar_>
Pose<pScalar_>::Pose(const pScalar_& h0, const pScalar_& h1, const pScalar_& h2, const pScalar_& h3, const pScalar_& h4, const pScalar_& h5, const pScalar_& h6, const pScalar_& h7)
    : UnitDualQuaternion<pScalar_>( h0, h1, h2, h3, h4, h5, h6, h7 ){ }

}
