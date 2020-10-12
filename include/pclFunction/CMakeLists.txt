project(FastPPF)

set(PCLFUNCTION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pclFunction SHARED STATIC pclFunction.cpp pclFunction.h)

target_link_libraries (pclFunction ${PCL_LIBRARIES} dataType)

target_include_directories(pclFunction PUBLIC ${PCL_INCLUDE_DIRS} ${DATATYPE_INCLUDE_DIR})