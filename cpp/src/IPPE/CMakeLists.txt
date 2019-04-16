project( IPPE )
find_package( OpenCV REQUIRED )

option(BUILD_SHARED_LIBS "Build as shared lib" ON)
if(BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include)
add_library( IPPE
    ippe.cpp
)
target_link_libraries( IPPE ${OpenCV_LIBS} )
