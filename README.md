 
    This file is part of dq1.
 
    dq1 is free software: you can redistribute it and/or modify 
    it under the terms of the GNU General Public License as published 
    by the Free Software Foundation, either version 3 of the License, 
    or (at your option) any later version.
 
    dq1 is distributed in the hope that it will be useful, 
    but WITHOUT ANY WARRANTY; without even the implied warranty of 
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with dq1. If not, see <https://www.gnu.org/licenses/>.



    Filename README.md
    Author Jiawei ZHAO
	Version 1.0
	Date 2023-2024


# DualQuaternion
This project is the implementation of dual quaternion based robot control platform. About dual quaternion, please refer to https://en.wikipedia.org/wiki/Dual_quaternion. This project is solely intended for personal practice and experimentation. It does not come with any warranty or guarantee of safety.

# How to install and use this library
This project has several dependencies. Make sure these dependencies are already installed within your environment, otherwise install them first. Here are some steps you might need to finish the install:
1. Quadratic programming solver - qpOASES | Share lib is perfered for avoiding unnecessary re-build
```
    cd ~/Downloads
    git clone https://github.com/coin-or/qpOASES.git
    cd qpOASES
    mkdir build
    cd build
    sudo cmake .. -DBUILD_SHARED_LIBS=ON
    sudo make install
```

