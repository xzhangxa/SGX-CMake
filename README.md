# SGX-CMake

CMake file for compiling with Intel SGX.

To make compiling code using SGX with CMake easier, a single FindSGX.cmake file is provided for the purpose.

This code example is from official Intel SGX sample [Cxx11SGXDemo](https://github.com/01org/linux-sgx/tree/master/SampleCode/Cxx11SGXDemo).

Please refer to the CMakeLists.txt file for how to use FindSGX.cmake file.

## How to use

```
list(APPEND CMAKE_MODULE_PATH ${PATH_TO_FindSGX.cmake_FILE})
find_package(SGX REQUIRED)
```

The following variables are provided:
- SGX_HW # ON for hardware, OFF for simulation.
- SGX_MODE # Debug; PreRelease; Release.
- SGX_INCLUDE_DIR
- SGX_TLIBC_INCLUDE_DIR
- SGX_LIBCXX_INCLUDE_DIR
- SGX_INCLUDE_DIRS # all 3 dirs above
- SGX_LIBRARY_DIR

The following functions are provided:
```
# build enclave library
add_enclave_library(target
                    [USE_PREFIX]
                    SRCS src_file1 src_file2 ...
                    EDL edl_file
                    EDL_SEARCH_PATHS path1 path2 ...
                    [TRUSTED_LIBS lib1 lib2 ...]
                    [LDSCRIPT ld_script_file])

# build trusted static library to be linked into enclave library
add_trusted_library(target
                    [USE_PREFIX]
                    SRCS src_file1 src_file2 ...
                    EDL edl_file
                    EDL_SEARCH_PATHS path1 path2 ...
                    [LDSCRIPT ld_script_file])

# sign the enclave, according to configurations one-step or two-step signing will be performed.
# default one-step signing output enclave name is target.signed.so, change it with OUTPUT option.
enclave_sign(target
             KEY key
	     [IGNORE_INIT]
	     [IGNORE_REL]
             [CONFIG config]
             [OUTPUT file_name])

# build untrusted executable to run with enclave
add_untrusted_executable(target
                         [USE_PREFIX]
                         SRCS src1 src2 ...
                         EDL edl_file1 edl_file2 ...
                         EDL_SEARCH_PATHS path1 path2 ...)

# build untrusted library to be run with enclave
add_untrusted_library(target
                      STATIC | SHARED | MODULE
                      [USE_PREFIX]
                      SRCS src1 src2 ...
                      EDL edl_file1 edl_file2 ...
                      EDL_SEARCH_PATHS path1 path2 ...)
```

## ROS

A ROS package sgx_cmake is provided for using inside ROS.

To use it, clone the repo to catkin_ws/src, and in the projects which need to use it:
```
# add in package.xml
<build_depend>sgx_cmake</build_depend>

# add in CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS sgx_cmake)
find_package(SGX)
```
