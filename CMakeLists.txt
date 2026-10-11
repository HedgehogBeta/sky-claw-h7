cmake_minimum_required(VERSION 3.16)
project(sky_claw_logic C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()
add_library(mechanism APL/mechanism/mechanism.c APL/mechanism/mechanism_config.c)
target_include_directories(mechanism PUBLIC APL/mechanism)
target_compile_options(mechanism PRIVATE -Wall -Wextra -Werror -pedantic)
add_executable(test_mechanism tests/test_mechanism.c)
target_compile_options(test_mechanism PRIVATE -Wall -Wextra -Werror -pedantic -UNDEBUG)
target_link_libraries(test_mechanism PRIVATE mechanism m)
add_test(NAME mechanism COMMAND test_mechanism)

add_executable(test_djmotor
    tests/test_djmotor.c
    FML/motor/src/DJmotor.c
    FML/algorithm/src/pid.c
    FML/algorithm/src/mathFunc.c
    FML/algorithm/src/vector.c)
target_include_directories(test_djmotor PRIVATE
    tests/stubs
    FML/motor/inc
    FML/motor
    FML/algorithm/inc
    APL/mechanism)
target_compile_options(test_djmotor PRIVATE -Wall -Wextra -pedantic -UNDEBUG)
target_link_libraries(test_djmotor PRIVATE m)
add_test(NAME djmotor COMMAND test_djmotor)
