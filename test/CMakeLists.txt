find_package(Catch2 REQUIRED)
find_package(GTest REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui Test)

file(GLOB_RECURSE SOURCES "*.cpp")

add_executable(test_example
    gui/test_slider.cpp
    libboids/test_boids.cpp
    libboids/test_flock.cpp
    libboids/test_utils.cpp
    main.cpp
)
target_link_libraries(test_example
    Catch2::Catch2
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    Qt5::Test
    GTest::GTest
    GTest::Main
    libboids
    libboids_gui
)

add_test(NAME test_example COMMAND test_example)
