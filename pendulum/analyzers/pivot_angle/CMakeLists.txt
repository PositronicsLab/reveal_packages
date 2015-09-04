cmake_minimum_required( VERSION 2.8.7 )
project( Reveal-Analyzer-Pendulum-Energy )

set( PACKAGE_NAME "pendulum" )
set( PACKAGE_ROOT_PATH "${PACKAGE_NAME}" )
set( ANALYZER_DIR "analyzers" )

set( ANALYZER_NAME "pivot_angle" )

set( ANALYZER_SOURCE_PATH "${PACKAGE_NAME}/${ANALYZER_DIR}/${ANALYZER_NAME}" )
set( ANALYZER_BUILD_PATH "${PACKAGE_NAME}/${ANALYZER_DIR}/build/${ANALYZER_NAME}" )
set( ANALYZER_BUILD_TARGET "libanalyzer.so" )

add_definitions( -DPACKAGE_NAME="${PACKAGE_NAME}" )
add_definitions( -DANALYZER_NAME="${ANALYZER_NAME}" )
add_definitions( -DANALYZER_SOURCE_PATH="${ANALYZER_SOURCE_PATH}" )
add_definitions( -DANALYZER_BUILD_PATH="${ANALYZER_BUILD_PATH}" )
add_definitions( -DANALYZER_BUILD_TARGET="${ANALYZER_BUILD_TARGET}" )
add_definitions( -DANALYZER_PATH="${ANALYZER_BUILD_PATH}/${ANALYZER_BUILD_TARGET}" )

#message( WARNING "${ANALYZER_SOURCE_PATH}" )
#message( WARNING "${ANALYZER_BUILD_PATH}" )
#message( WARNING "${ANALYZER_BUILD_TARGET}" )

set( CMAKE_MODULES_DIR ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "CMake scripts" )
set( CMAKE_MODULE_PATH ${CMAKE_MODULES_DIR} ${CMAKE_MODULE_PATH} )

find_package( Boost REQUIRED )

find_package( Reveal REQUIRED )
include_directories( ${REVEAL_INCLUDE_DIRS} )
link_directories( ${REVEAL_LIBRARY_DIRS} )

set( LIBS 
  ${REVEAL_LIBRARIES}
) 

add_library( analyzer "MODULE" analyzer.cpp )
target_link_libraries( analyzer ${LIBS} )

