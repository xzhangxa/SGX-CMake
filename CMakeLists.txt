cmake_minimum_required(VERSION 2.8)
project(sgx-cmake)

include(cmake/SGX.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(EDL_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/Enclave ${CMAKE_SOURCE_DIR}/Enclave/TrustedLibrary)

set(E_SRCS Enclave/Enclave.cpp Enclave/TrustedLibrary/Libcxx.cpp)
if(SGX_HW AND SGX_MODE STREQUAL "Release")
    set(LDS Enclave/Enclave.lds)
else()
    set(LDS Enclave/Enclave_debug.lds)
endif()
add_enclave_library(enclave "${E_SRCS}" EDL Enclave/Enclave.edl EDL_SEARCH_PATHS "${EDL_SEARCH_PATHS}" LDSCRIPT ${LDS}
                    KEY Enclave/Enclave_private.pem CONFIG Enclave/Enclave.config.xml)

set(SRCS App/App.cpp App/TrustedLibrary/Libcxx.cpp)
add_untrusted_executable(app "${SRCS}" EDL Enclave/Enclave.edl EDL_SEARCH_PATHS "${EDL_SEARCH_PATHS}")
