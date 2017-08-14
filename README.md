# SGX-CMake

CMake file for compiling with Intel SGX.

This is still WIP...

To make compiling code using SGX with CMake easier, a single FindSGX.cmake file is provided for the purpose.

This code example is from official Intel SGX sample [Cxx11SGXDemo](https://github.com/01org/linux-sgx/tree/master/SampleCode/Cxx11SGXDemo).

Please refer to the CMakeLists.txt file for how to use FindSGX.cmake file.

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
