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
 ##    Filename src/CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2023-2024
 ##

add_library(dq1 SHARED Modeling.cpp)

set_property(TARGET dq1 PROPERTY VERSION ${PACKAGE_VERSION})
set_property(TARGET dq1 PROPERTY SOVERSION ${PACKAGE_SO_VERSION})

target_include_directories(dq1
                           PUBLIC
                            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/dq1>
                            $<INSTALL_INTERFACE:include/dq1>
                           )
# state that SqrtLibrary need PIC when the default is shared libraries
set_target_properties(dq1 PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
                    )

find_package(
    qpOASES REQUIRED
    nlohmann_json 3.11.3 REQUIRED
    Eigen3 3.3 REQUIRED
    Boost
)


include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(dq1 PUBLIC dq1_compiler_flags)
target_link_libraries(dq1 PRIVATE
    qpOASES
)

set(installable_libs dq1 dq1_compiler_flags)
install(TARGETS ${installable_libs}
    EXPORT dq1Targets
    DESTINATION lib
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")

