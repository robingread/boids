cmake_minimum_required(VERSION 3.10)
project(libboids_gui LANGUAGES CXX)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)   # For signals and slots
set(CMAKE_AUTORCC ON)   # For resource files

# Add the source files and UI files
file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.h")

add_library(libboids_gui SHARED ${SOURCES} ${HEADERS})
target_link_libraries(libboids_gui Qt5::Core Qt5::Widgets Qt5::Gui libboids)

# Specify where the public headers are located
target_include_directories(libboids_gui PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>  # For consumers after install
)

# Installation rules for the library and headers
install(TARGETS libboids_gui
    EXPORT libboids_guiTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

# Export the library target for use by other projects
install(EXPORT libboids_guiTargets
    FILE libboids_guiTargets.cmake
    NAMESPACE libboids::
    DESTINATION lib/cmake/libboids_gui
)

# Generate a libboidsConfig.cmake file for downstream use
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/libboids_guiConfigVersion.cmake"
    VERSION 1.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/libboids_guiConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/libboids_guiConfig.cmake"
    INSTALL_DESTINATION lib/cmake/libboids_gui
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/libboids_guiConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/libboids_guiConfigVersion.cmake"
    DESTINATION lib/cmake/libboids_gui
)
