project(FastPPF)

set(HPR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(HPR SHARED STATIC HPR.cpp HPR.h)
target_link_libraries (HPR ${PCL_LIBRARIES})
target_include_directories(HPR PUBLIC ${PCL_INCLUDE_DIRS})
