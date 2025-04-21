cmake_minimum_required(VERSION 3.10)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(boidsim main.cpp)
target_link_libraries(boidsim Qt5::Core Qt5::Widgets Qt5::Gui libboids libboids_gui)
install(TARGETS boidsim RUNTIME DESTINATION bin)
