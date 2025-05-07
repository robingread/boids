cmake_minimum_required(VERSION 3.10)
project(libboids LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})

file(GLOB HEADERS "*.h")
add_library(libboids SHARED
    boids.cpp
    flock.cpp
    utils.cpp
)
target_link_libraries(libboids
    Eigen3::Eigen
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui)

# Specify where the public headers are located
target_include_directories(libboids PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>  # For consumers after install
)

# Installation rules for the library and headers
install(TARGETS libboids
    EXPORT libboidsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

# Export the library target for use by other projects
install(EXPORT libboidsTargets
    FILE libboidsTargets.cmake
    NAMESPACE libboids::
    DESTINATION lib/cmake/libboids
)

# Generate a libboidsConfig.cmake file for downstream use
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/libboidsConfigVersion.cmake"
    VERSION 1.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/libboidsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/libboidsConfig.cmake"
    INSTALL_DESTINATION lib/cmake/libboids
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/libboidsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/libboidsConfigVersion.cmake"
    DESTINATION lib/cmake/libboids
)
