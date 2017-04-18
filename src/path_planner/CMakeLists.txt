#
# Project settings
#

# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( MY_COOL_PROJECT CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_DEFINITIONS( -std=c++11 )
ADD_EXECUTABLE(        path_planner path_gamma_chi.cpp )
TARGET_LINK_LIBRARIES( path_planner ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( path_planner PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

ADD_EXECUTABLE(        path_follower path_follower.cpp )
TARGET_LINK_LIBRARIES( path_follower ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( path_follower PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
