# PROJECT CONFIGURATION
cmake_minimum_required(VERSION 3.1)

project(NViewsTrian LANGUAGES C CXX VERSION 1.0.0)

set(LIBRARY_TARGET_NAME "NViewsTrian")
set(LIBRARY_TARGET_NAME_EXPORT "${LIBRARY_TARGET_NAME}Export")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # We require C++ 14


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-O3 -DNDEBUG")

message(STATUS "Building in ${CMAKE_BUILD_TYPE} mode\n")



# build the examples
set(BUILD_${LIBRARY_TARGET_NAME}_EXAMPLE ON)
# build the tests
set(BUILD_${LIBRARY_TARGET_NAME}_TESTS ON)



# Build type

# Directory for built libraries
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib CACHE PATH "The directory in which to place libraries built by this project")
# Directory for built executables
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin CACHE PATH "The directory in which to place executables built by this project")

# BUILD CONFIGURATIONS
option(CMAKE_VERBOSE_MAKEFILE "Generate verbose makefiles?" OFF)

set(CODE_PROFILING OFF CACHE BOOL "Turn on code profiling?")


# Add the .cmake files that ship with Eigen3 to the CMake module path (useful for finding other stuff)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" CACHE STRING "The CMake module path used for this project")

# FIND EIGEN3
set( ENV{EIGEN3_ROOT_DIR} ${CMAKE_SOURCE_DIR}/eigen)
find_package(Eigen3 3.3 REQUIRED)
if(EIGEN3_FOUND)
message(STATUS "Found Eigen3 library (version ${EIGEN3_VERSION_STRING})")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}\n")
else()
message(STATUS "Eigen library not found!")
endif()


# FIND ADDITIONAL LIBRARIES

ADD_DEFINITIONS ( -DPQXX_HIDE_EXP_OPTIONAL )

# IMPORT OpenGV library
add_definitions(-march=native)


set(${LIBRARY_TARGET_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(${LIBRARY_TARGET_NAME}_SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(${LIBRARY_TARGET_NAME}_EXAMPLES_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/examples)
set(${LIBRARY_TARGET_NAME}_TESTS_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/tests)


# Expose the include directories for this project
set(${LIBRARY_TARGET_NAME}_ADD_INCLUDES ${EIGEN3_INCLUDE_DIR} )
set(${LIBRARY_TARGET_NAME}_CERT_INCLUDES ${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS} ${${LIBRARY_TARGET_NAME}_ADD_INCLUDES})


# Get the set of Essential header and source files
set(${LIBRARY_TARGET_NAME}_HDRS
${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}/NViewsTypes.h
${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}/NViewsUtils.h
${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}/NViewsClass.h
)


set(${LIBRARY_TARGET_NAME}_SRCS
${${LIBRARY_TARGET_NAME}_SOURCE_DIRS}/NViewsUtils.cpp
${${LIBRARY_TARGET_NAME}_SOURCE_DIRS}/NViewsClass.cpp
)





# Build the Essential library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDRS} ${${LIBRARY_TARGET_NAME}_SRCS}  )


target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
        # only when building from the source tree
        $<BUILD_INTERFACE:${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}>
        $<INSTALL_INTERFACE:include>
        ${${LIBRARY_TARGET_NAME}_ADD_INCLUDES}
)


target_link_libraries(${LIBRARY_TARGET_NAME})




if(${CODE_PROFILING})
  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES COMPILE_FLAGS "-pg -g" LINK_FLAGS "-pg -g")
endif()


set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${LIBRARY_TARGET_NAME}
  EXPORT  ${LIBRARY_TARGET_NAME_EXPORT}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX}
  INCLUDES DESTINATION "include"
  PUBLIC_HEADER DESTINATION "include/${LIBRARY_TARGET_NAME}"
 )



# Build the example executable
IF(BUILD_${LIBRARY_TARGET_NAME}_EXAMPLE)
message(STATUS "Adding examples to build")
add_subdirectory(examples)
endif()


# building the tests
if(BUILD_${LIBRARY_TARGET_NAME}_TESTS)
message(STATUS "Adding tests to build")
add_subdirectory(tests)
endif()


# Install
include(cmake/setup_installation.cmake)

