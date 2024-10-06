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
 *     \file examples/example2.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include <iostream>
#include <stdlib.h>
#include "../include/dq1.hpp"

int main()
{

    using namespace dq1::Macro;
    Vec6 pos;
    pos << 0,0,0,0,0,0;
    
    // Construct a robot
    dq1::kinematics::SerialManipulator robot("robot1.json", pos);

    // Set base and effector as default Pose which is 1.
    robot.set_base(Pose());
    robot.set_effector(Pose());

    Pose x_init = robot.end_pose();
    scalar_t radius = 0.01;
    scalar_t rad_speed = 0.001;
    size_t i = 0;
    while (true)
    {   
        Tran offset = x_init.translation() + Tran(0, radius * cos(rad_speed*i), radius * sin(rad_speed*i));
        ++i;
        Pose xd = Pose::build_from(offset);
        robot.update(xd);
        std::cout << "translation error: " << robot.end_pose().translation() - xd.translation() << "\n";
    }

}

