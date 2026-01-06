cmake_minimum_required(VERSION 3.20)

project(libts)

set(CMAKE_C_STANDARD 90)    # Use ANSI C

add_library(ts STATIC
    "src/ts_interpolation.c")
target_include_directories(ts PUBLIC include)

add_library(tssyslibc STATIC
    "src/ts_sys_libc.c")
target_include_directories(tssyslibc PUBLIC include)

# -- tests --
enable_testing()

add_executable(test_interpolation test/test_interpolation.c)
target_include_directories(test_interpolation PRIVATE include)
target_link_libraries(test_interpolation ts)
add_test(test_interpolation test_interpolation)
