# Author: Jeroen de Haas <jah.dehaas@avans.nl>
cmake_minimum_required(VERSION 3.1.3)
project(pigisland)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wpedantic)#  -Werror)
    add_compile_options("$<$<CONFIG:DEBUG>:-g>")
endif()

  
add_executable(pigisland
  src/pigisland/main.cpp
  src/pigisland/node_algorithm.cpp
  src/pigisland/resources.cpp
  src/pigisland/shark.cpp
  src/pigisland/boat.cpp
  src/pigisland/pig.cpp
  src/stud_lib_kmint/a_star/a_star.cpp
  src/stud_lib_kmint/a_star/heuristics.cpp)

target_compile_features(pigisland PUBLIC cxx_std_17)
target_link_libraries(pigisland PRIVATE kmint)
target_include_directories(pigisland PRIVATE 
  src
  include)

add_custom_target(pigisland-resources ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/resources" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources")
