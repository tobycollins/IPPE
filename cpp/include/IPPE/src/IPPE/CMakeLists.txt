project( IPPE )
find_package( OpenCV REQUIRED )
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include)
add_library( IPPE
    ippe.cpp
)
target_link_libraries( IPPE ${OpenCV_LIBS} )
