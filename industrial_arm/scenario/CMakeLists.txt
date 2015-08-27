cmake_minimum_required(VERSION 2.8.7)

#------------------------------------------------------------------------------
set( PACKAGE_NAME "industrial_arm" )
set( PACKAGE_ROOT_PATH "${PACKAGE_NAME}" )
set( ANALYZER_DIR "analyzers" )
set( EXPERIMENT_DIR "scenario" )

#------------------------------------------------------------------------------
option( GAZEBO "compile package with Gazebo support" ON )

#------------------------------------------------------------------------------
option( DATA_GENERATION "compile package for data generation" OFF )
if( DATA_GENERATION ) 
  add_definitions( -DDATA_GENERATION )
  option( DB_DIRECT_INSERT "compile package to directly insert into db" OFF )
  if( DB_DIRECT_INSERT ) 
    add_definitions( -DDB_DIRECT_INSERT )
  endif( DB_DIRECT_INSERT )
  #server only
  ## Note: hackish. We can't necessarily guarantee where this will be based on
  #  relative path, but good enough for now.

  set( ANALYZER_SOURCE_PATH "${PACKAGE_NAME}/${ANALYZER_DIR}/block_energy" )
  set( ANALYZER_BUILD_PATH "${ANALYZER_SOURCE_PATH}/build" )
  set( ANALYZER_BUILD_TARGET "libanalyzer.so" )

  add_definitions( -DANALYZER_SOURCE_PATH="${ANALYZER_SOURCE_PATH}" )
  add_definitions( -DANALYZER_BUILD_PATH="${ANALYZER_BUILD_PATH}" )
  add_definitions( -DANALYZER_BUILD_TARGET="${ANALYZER_BUILD_TARGET}" )
  add_definitions( -DANALYZER_PATH="${ANALYZER_BUILD_PATH}/${ANALYZER_BUILD_TARGET}" )
endif( DATA_GENERATION )

#------------------------------------------------------------------------------

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR} )

if( GAZEBO )
  add_subdirectory( "gazebo" )
endif()

