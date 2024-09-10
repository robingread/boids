cmake_minimum_required(VERSION 3.10)
project(boids LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/install" ${CMAKE_PREFIX_PATH})
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

add_subdirectory(src/libboids)
add_subdirectory(src/gui)
add_subdirectory(src/app)
add_subdirectory(test)