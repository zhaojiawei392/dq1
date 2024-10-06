/** 
 *     This file is part of dq1.
 *  
 *     dq1 is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dq1 is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dq1. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file include/dq1/Geometry.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include "Pose.hpp"
#include "Macro.hpp"

namespace Geometry
{
using namespace dq1::Macro;

class Geo{
protected:
    
public:
    virtual ~Geo()=default;

    virtual scalar_t distance(const Geo& other) const noexcept = 0;

};

class Point: public Geo{
protected:
    Vec3 _position;
public:
    Point(const Vec3 vec3);
    Point(scalar_t x=0, scalar_t y=0, scalar_t z=0);

    virtual scalar_t distance(const Geo& other) const noexcept override;
    

};

class Line: public Geo{
protected:

public:

};

class LineSegment: public Geo{

};

class Plane: public Geo{

};

class PlaneSegment: public Geo{
    
};

class Sphere: public Geo{

};

class Cuboid: public Geo{

};

class Cone: public Geo{

};

class Cylinder: public Geo{

};

class Capsule: public Geo{

};

}