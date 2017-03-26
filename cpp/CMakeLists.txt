cmake_minimum_required(VERSION 2.8)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

add_subdirectory (src/IPPE)
add_subdirectory (src/IPPE_demo_generic_pointset)
add_subdirectory (src/IPPE_demo_square_pointset)

