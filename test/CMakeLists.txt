cmake_minimum_required(VERSION 3.16)
project(ARCUnitTests LANGUAGES CXX)
enable_testing()
add_executable(arc_unit_tests test_main.cpp)
target_compile_features(arc_unit_tests PRIVATE cxx_std_20)
# Test doubles never enter the firmware include path.
target_include_directories(arc_unit_tests PRIVATE
    stubs ../lib ../lib/RS485 ../lib/Sensor ../lib/i2cComm)
if(MSVC)
    target_compile_options(arc_unit_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(arc_unit_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME arc_unit_tests COMMAND arc_unit_tests)
