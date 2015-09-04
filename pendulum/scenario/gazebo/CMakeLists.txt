cmake_minimum_required(VERSION 2.8.7)

set( COMPONENT_NAME "controller" )
project( "reveal-package-pendulum-gazebo-${COMPONENT_NAME}" )

#------------------------------------------------------------------------------

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g ")

include_directories( .. )

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO REQUIRED gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

#------------------------------------------------------------------------------
set( LIBS 
  ${GAZEBO_LIBRARIES}
) 

#------------------------------------------------------------------------------
add_library( gz-plugin SHARED plugin.cpp )
target_link_libraries( gz-plugin ${LIBS} )

#------------------------------------------------------------------------------
# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/gazebo.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/reveal.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
