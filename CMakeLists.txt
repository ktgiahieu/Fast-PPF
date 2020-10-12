cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(FastPPF)

add_subdirectory(include)

add_executable (FastPPF demo.cpp)
target_link_libraries (FastPPF PPF)
target_include_directories(FastPPF PUBLIC ${PCL_INCLUDE_DIRS} ${PCLFUNCTION_INCLUDE_DIR} ${PPF_INCLUDE_DIR})
