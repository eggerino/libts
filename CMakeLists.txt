cmake_minimum_required(VERSION 3.20)

project(ts)

# -- libts --
set(lib_src_files
    "src/ts_interpolation.c")
add_library(ts STATIC ${lib_src_files})
target_include_directories(ts PUBLIC include)

# -- libtssyslibc --
set(lib_sys_src_files
    "src/ts_sys_libc.c")
add_library(libtssyslibc STATIC ${lib_sys_src_files})
target_include_directories(libtssyslibc PUBLIC include)

# -- tests --
set(tests_src_files
    "test/main.c"
    "test/test_interpolation.c")
add_executable(tstests ${tests_src_files})
target_include_directories(tstests PRIVATE include)
target_link_libraries(tstests ts)
enable_testing()
add_test(tstests tstests)