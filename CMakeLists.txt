cmake_minimum_required(VERSION 3.2)

# wrat stands for Weidong's Robotic Application Toolkit
project(wrat)

if(UNIX)
  message("UNIX platform found.")
elseif(MSVC)
  message("MSVC platform found.")
elseif(LINUX)
  message("LINUX platform found.")
else()
  message("Unkown platform.")
endif()

# Fix behavior of CMAKE_CXX_STANDARD when targeting macOS.
# https://stackoverflow.com/questions/10851247/how-do-i-activate-c-11-in-cmake
if (POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif ()
set(CMAKE_CXX_STANDARD 11)

# https://stackoverflow.com/questions/41361631/optimize-in-cmake-by-default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Enable all warnings to promote bug-free code.
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(ccd REQUIRED)
#find_package(Doxygen REQUIRED)


if(Eigen3_FOUND)
  message("Eigen3 Found")
endif()

if(fcl_FOUND)
  message("FCL Found")
  message("FCL include dir is " ${FCL_INCLUDE_DIRS})
  message("FCL lib is " ${FCL_LIBRARIES})
endif()

if(ccd_FOUND)
  message("CCD Found")
  message("CCD include dir is " ${CCD_INCLUDE_DIRS})
  message("CCD lib is " ${CCD_LIBRARIES})
endif()

# if(Doxygen_FOUND)
#   message("Doxygen Found")
# endif()

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} )
#set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${FCL_INCLUDE_DIR})
include_directories(${CCD_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

