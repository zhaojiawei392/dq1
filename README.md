 
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
## 1. Quadratic programming solver - qpOASES
```bash
cd ~/Downloads
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build
cd build
sudo cmake .. -DBUILD_SHARED_LIBS=ON
sudo make install
```
Here the build variable `BUILD_SHARED_LIBS` is set to `ON` so the compiler will build qpOASES as shared object. The default install path is `/usr/local/lib`, which is not in the dynamic linker's search path. this results in runtime error while the program tries to access the libqpOASES.so, complaining no such file or directory. So to fix this issue, we need to add the path `/usr/local/lib` to the dynamic linker's search path. Here is a permanent solution:
### 1. create a new conf file for storing the path `/usr/local/lib`
```bash
sudo nano /etc/ld.so.conf.d/local.conf
```
### 2. Add the following line to include `/usr/local/lib`
```bash
/usr/local/lib
```
### 3. Save the file by pressing crtl + x, and then press Enter to exit the editor
### 4. Update the dynamic linker by running:
```bash
sudo ldconfig
```


