project(FastPPF)

set(DATATYPE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(dataType SHARED STATIC dataType.h dataType.cpp)
target_link_libraries (dataType ${PCL_LIBRARIES})
target_include_directories(dataType PUBLIC ${PCL_INCLUDE_DIRS})
