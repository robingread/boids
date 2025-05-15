cmake_minimum_required(VERSION 3.10)
project(boids LANGUAGES CXX)

# Compile options
option(BUILD_TESTS "Build Tests" ON)
option(CODE_COVERAGE "Enable code coverage" ON)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/install" ${CMAKE_PREFIX_PATH})

# Set the install path for development purposes.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path" FORCE)
endif()

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

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
