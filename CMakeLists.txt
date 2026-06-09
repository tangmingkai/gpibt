cmake_minimum_required(VERSION 3.16)
project(gpibt CXX)

add_subdirectory(third_party/grid-pathfinding/graph ./graph)
add_subdirectory(third_party/argparse ./argparse)

find_package(OpenMP REQUIRED)

add_compile_options(-fopenmp)
add_link_options(-fopenmp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -mtune=native -march=native)


find_package(ortools CONFIG REQUIRED)

include_directories(
   ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE MAPF_LIB_SOURCES
    "mapf-svg/algorithm/*.cpp"
)
add_library(lib-mapf STATIC ${MAPF_LIB_SOURCES})
target_compile_features(lib-mapf PUBLIC cxx_std_17)

target_link_libraries(lib-mapf PUBLIC ortools::ortools)
target_link_libraries(lib-mapf PUBLIC OpenMP::OpenMP_CXX)


target_link_libraries(lib-mapf
    PUBLIC 
      lib-graph
)

add_executable(mapf-svg-main "mapf-svg/mapf.cpp")
target_compile_features(mapf-svg-main  PUBLIC cxx_std_17)
target_link_libraries(mapf-svg-main PUBLIC lib-mapf)

