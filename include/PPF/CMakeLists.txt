project(FastPPF)

set(PPF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

add_library(PPF SHARED STATIC PPF.cpp PPF.h MyPPFRegistration.cpp MyPPFRegistration.hpp)
target_link_libraries(PPF pclFunction meshSampling HPR ${OpenCV_LIBS})
target_include_directories(PPF PUBLIC ${PCL_INCLUDE_DIRS} ${PCLFUNCTION_INCLUDE_DIR} ${MESHSAMPLING_INCLUDE_DIR} ${HPR_INCLUDE_DIR})




