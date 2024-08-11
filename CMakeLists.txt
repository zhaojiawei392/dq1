cmake_minimum_required(VERSION 3.28.3)

project(dq1 VERSION 1.0 DESCRIPTION "An implementation of the dual quaternion" LANGUAGES CXX)

# Package metadata
set(PACKAGE_NAME "dq1")
set(PACKAGE_VERSION "1.0.0")
set(PACKAGE_SO_VERSION "1.0")
set(PACKAGE_AUTHOR "ZHAO JIAWEI.")
set(PACKAGE_MAINTAINER "ZHAO JIAWEI.")
set(PACKAGE_URL "https://github.com/zhaojiawei392/DualQuaternion")

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(dq1_LIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Installation directories
include(GNUInstallDirs)
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Check for unsupported configurations
if(BUILD_SHARED_LIBS AND WIN32)
    message(FATAL_ERROR "Compiling dq1 as a shared library on Windows is not supported.")
endif()

# Compiler flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")

# Source files
file(GLOB SRC src/*.cpp)

# RPATH settings
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()


# **********************************************************************************************************************
# **********************************************************************************************************************
# *********************************************Build and Install********************************************************
# **********************************************************************************************************************
# **********************************************************************************************************************

find_package(qpOASES REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

#Create library
add_library(dq1 INTERFACE)
target_include_directories(dq1 INTERFACE 
    $<BUILD_INTERFACE:${MY_LIBRARY_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dq1 INTERFACE
    qpOASES
)

# install(TARGETS dq1
#     EXPORT qpOASES 
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

set_target_properties(dq1 PROPERTIES
    VERSION ${PACKAGE_VERSION}
    SOVERSION ${PACKAGE_SO_VERSION}
)

# Install headers
install(FILES dq1.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dq1
FILES_MATCHING PATTERN "*.hpp" PATTERN "*.ipp"
PATTERN ".svn" EXCLUDE)




# **********************************************************************************************************************
# **********************************************************************************************************************
# *********************************************Package config***********************************************************
# **********************************************************************************************************************
# **********************************************************************************************************************

include(CMakePackageConfigHelpers)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})

# Generate the config file
configure_package_config_file(dq1Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dq1Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dq1
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dq1ConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install the config file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dq1Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dq1ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dq1
    COMPONENT dq1)




# Examples
option(dq1_BUILD_EXAMPLES OFF)
# Alternative manual setting, without the 'option' keyword
set(dq1_BUILD_EXAMPLES ON CACHE BOOL "Build example programs")

if(dq1_BUILD_EXAMPLES)
    # Compile examples
    set(EXAMPLE_NAMES
        example1
    )
    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
        target_link_libraries(${EXAMPLE} qpOASES)
    endforeach()
endif()













