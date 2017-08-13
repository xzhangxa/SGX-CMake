cmake_minimum_required(VERSION 2.8)
project(sgx-cmake)

option(BUILD_SAMPLE "Build sample" OFF)
if(BUILD_SAMPLE)
    add_subdirectory(sample)
endif()
