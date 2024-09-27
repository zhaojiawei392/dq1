 ## 
 ##    This file is part of dq1.
 ## 
 ##    dq1 is free software: you can redistribute it and/or modify 
 ##    it under the terms of the GNU General Public License as published 
 ##    by the Free Software Foundation, either version 3 of the License, 
 ##    or (at your option) any later version.
 ## 
 ##    dq1 is distributed in the hope that it will be useful, 
 ##    but WITHOUT ANY WARRANTY; without even the implied warranty of 
 ##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 ##    See the GNU General Public License for more details.
 ## 
 ##    You should have received a copy of the GNU General Public License
 ##    along with dq1. If not, see <https://www.gnu.org/licenses/>.
 ##

 ##
 ##    Filename examples/CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2023-2024
 ##

# Examples
message(STATUS "dq1_BUILD_EXAMPLES is set to ${dq1_BUILD_EXAMPLES}")

if(dq1_BUILD_EXAMPLES)
    # Compile dq1 examples
    set(EXAMPLE_NAMES
        example1
    )

    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
        target_link_libraries(${EXAMPLE} dq1)
    endforeach()
endif()