cmake_minimum_required(VERSION 2.8.7)

set( COMPONENT_NAME "generator" )
project( "reveal-package-pendulum-data-${COMPONENT_NAME}" )

#------------------------------------------------------------------------------
#option( DATA_GENERATION "compile package for data generation" OFF )
#if( DATA_GENERATION ) 
#  add_definitions( -DDATA_GENERATION )
  option( DB_DIRECT_INSERT "compile package to directly insert into db" OFF )
  if( DB_DIRECT_INSERT ) 
    add_definitions( -DDB_DIRECT_INSERT )
  endif( DB_DIRECT_INSERT )
  #server only
  ## Note: hackish. We can't necessarily guarantee where this will be based on
  #  relative path, but good enough for now.

#  set( ANALYZER_SOURCE_PATH "${PACKAGE_NAME}/${ANALYZER_DIR}/energy" )
#  set( ANALYZER_BUILD_PATH "${ANALYZER_SOURCE_PATH}/build" )
#  set( ANALYZER_BUILD_TARGET "libanalyzer.so" )

#  add_definitions( -DANALYZER_SOURCE_PATH="${ANALYZER_SOURCE_PATH}" )
#  add_definitions( -DANALYZER_BUILD_PATH="${ANALYZER_BUILD_PATH}" )
#  add_definitions( -DANALYZER_BUILD_TARGET="${ANALYZER_BUILD_TARGET}" )
#  add_definitions( -DANALYZER_PATH="${ANALYZER_BUILD_PATH}/${ANALYZER_BUILD_TARGET}" )
#endif( DATA_GENERATION )
#------------------------------------------------------------------------------


include_directories( .. )

find_package( Boost REQUIRED )

#include_directories( ${REVEAL_SIM_INCLUDE} )
#link_directories( ${REVEAL_SIM_LIBRARY} )

find_package( reveal REQUIRED )
include_directories( ${REVEAL_INCLUDE_DIRS} )
link_directories( ${REVEAL_LIBRARY_DIRS} )

#------------------------------------------------------------------------------
set( HDRS 
  pendulum.h
) 

set( LIBS 
  ${BOOST_LIBRARIES}
  ${REVEAL_LIBRARIES}
#  ${REVEAL_SIM_LIBRARY}
) 

if( DB_DIRECT_INSERT ) 
  set( LIBS 
    ${LIBS}
    reveal_db
  ) 
endif( DB_DIRECT_INSERT )

#------------------------------------------------------------------------------
add_executable( data-generator generator.cpp ${HDRS} )
target_link_libraries( data-generator ${LIBS} )

#------------------------------------------------------------------------------
