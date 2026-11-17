
cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED SKBUILD_PROJECT_NAME)
  set(SKBUILD_PROJECT_NAME "thunder_Rizon4s")
endif()

project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

# Set TRUE to build python bindings
option(BUILD_PYTHON_BINDINGS "Build python bindings" ON)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

if(BUILD_PYTHON_BINDINGS)
  set(Python_FIND_STRATEGY LOCATION)
  set(Python_FIND_VIRTUALENV FIRST)
  find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 REQUIRED)

  add_compile_definitions(ENABLE_PYBIND11)

  pybind11_add_module(thunder_Rizon4s thunder_Rizon4s.cpp Rizon4s_gen.cpp)
  target_link_libraries(thunder_Rizon4s PRIVATE yaml-cpp::yaml-cpp pybind11::module Eigen3::Eigen)
  install(TARGETS thunder_Rizon4s LIBRARY DESTINATION thunder_Rizon4s_py)

else()
  add_library(thunder_Rizon4s SHARED thunder_Rizon4s.cpp Rizon4s_gen.cpp)
  target_link_libraries(thunder_Rizon4s PUBLIC yaml-cpp::yaml-cpp Eigen3::Eigen)
  target_include_directories(thunder_Rizon4s PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install(TARGETS thunder_Rizon4s DESTINATION lib)
  install(FILES thunder_Rizon4s.h Rizon4s_gen.h DESTINATION include)
endif()
