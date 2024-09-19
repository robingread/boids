find_package(GTest REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui Test)

file(GLOB_RECURSE SOURCES "*.cpp")
add_executable(test_example ${SOURCES})
target_link_libraries(test_example Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Test GTest::GTest GTest::Main libboids)

add_test(NAME test_example COMMAND test_example)