cmake_minimum_required(VERSION 2.8.7)

set( COMPONENT_NAME "controller" )
project( "reveal-package-industrial_arm-gazebo-${COMPONENT_NAME}" )

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g ")

include_directories( ${REVEAL_SIM_INCLUDE} )
link_directories( ${REVEAL_SIM_LIBRARY} )

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO REQUIRED gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

find_package( reveal REQUIRED )
include_directories( ${REVEAL_INCLUDE_DIRS} )
link_directories( ${REVEAL_LIBRARY_DIRS} )

find_package( reveal_sim_gazebo REQUIRED )
include_directories( ${REVEAL_SIM_GAZEBO_INCLUDE_DIRS} )
link_directories( ${REVEAL_SIM_GAZEBO_LIBRARY_DIRS} )

set( LIBS 
  ${GAZEBO_LIBRARIES}
  ${REVEAL_LIBRARIES}
  ${REVEAL_SIM_GAZEBO_LIBRARIES}
) 

if( DB_DIRECT_INSERT ) 
  set( LIBS 
    ${LIBS}
    reveal_db
  ) 
endif( DB_DIRECT_INSERT )

set( HDRS 
  arm_controller.h
)

add_library( gz-arm-plugin SHARED gazebo_arm_plugin.cpp ${HDRS} )
target_link_libraries( gz-arm-plugin ${LIBS} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/gazebo.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/reveal.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
