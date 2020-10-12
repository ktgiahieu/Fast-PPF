project(FastPPF)

find_package(PCL 1.2 REQUIRED)

add_subdirectory(dataType)
add_subdirectory(pclFunction)
add_subdirectory(meshSampling)
add_subdirectory(HPR)
add_subdirectory(PPF)