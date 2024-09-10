cmake_minimum_required(VERSION 3.10)
project(boids LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/install" ${CMAKE_PREFIX_PATH})
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

# Code Coverage option
option(CODE_COVERAGE "Enable code coverage" ON)

if(CODE_COVERAGE)
  # Enable coverage flags for GCC
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -fno-inline -g")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  endif()
endif()

add_subdirectory(src/libboids)
add_subdirectory(src/gui)
add_subdirectory(src/app)
add_subdirectory(test)