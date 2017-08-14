cmake_minimum_required(VERSION 2.8)
project(sgx_cmake)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS sgx_cmake-extras.cmake)

install(FILES ../cmake/FindSGX.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
