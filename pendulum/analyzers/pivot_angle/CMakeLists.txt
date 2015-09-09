cmake_minimum_required( VERSION 2.8.7 )
project( reveal-analyzer-pendulum-pivot_angle )

find_package( Reveal REQUIRED )
include_directories( ${REVEAL_INCLUDE_DIRS} )
link_directories( ${REVEAL_LIBRARY_DIRS} )

set( LIBS 
  ${REVEAL_LIBRARIES}
) 

add_library( analyzer "MODULE" analyzer.cpp )
target_link_libraries( analyzer ${LIBS} )

