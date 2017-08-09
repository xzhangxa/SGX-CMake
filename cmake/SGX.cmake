# FindPackage cmake file for Intel SGX SDK

cmake_minimum_required(VERSION 2.8.8)

set(SGX_PATH "$ENV{SGX_SDK}" CACHE PATH "Intel SGX SDK path.")
if(SGX_PATH STREQUAL "")
    message(ERROR "Intel SGXSDK environment file is not sourced and SGX SDK path is not set.")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(ERROR "Only x64 build is supported by Intel SGX.")
endif()

find_path(SGXSDK_INCLUDE_DIRS sgx.h "${SGX_PATH}/include")
find_library(SGXSDK_LIBRARIES libsgx_urts.so "${SGX_PATH}/lib64")

set(SGX_COMMON_CFLAGS -m64)
set(SGX_LIBRARY_PATH ${SGX_PATH}/lib64)
set(SGX_ENCLAVE_SIGNER ${SGX_PATH}/bin/x64/sgx_sign)
set(SGX_EDGER8R ${SGX_PATH}/bin/x64/sgx_edger8r)

set(SGX_HW ON CACHE BOOL "Run SGX on hardware, OFF for simulation.")
set(SGX_MODE Debug CACHE STRING "SGX build mode: Debug; PreRelease; Release.")

if(SGX_HW)
    set(SGX_URTS_LIB sgx_urts)
    set(SGX_USVC_LIB sgx_uae_service)
    set(SGX_TRTS_LIB sgx_trts)
    set(SGX_TSVC_LIB sgx_tservice)
else()
    set(SGX_URTS_LIB sgx_urts_sim)
    set(SGX_USVC_LIB sgx_uae_service_sim)
    set(SGX_TRTS_LIB sgx_trts_sim)
    set(SGX_TSVC_LIB sgx_tservice_sim)
endif()

if(SGX_MODE STREQUAL "Debug")
    set(SGX_COMMON_CFLAGS "${SGX_COMMON_CFLAGS} -O0 -g -DDEBUG -UNDEBUG -UEDEBUG")
elseif(SGX_MODE STREQUAL "PreRelease")
    set(SGX_COMMON_CFLAGS "${SGX_COMMON_CFLAGS} -O2 -UDEBUG -DNDEBUG -DEDEBUG")
elseif(SGX_MODE STREQUAL "Release")
    set(SGX_COMMON_CFLAGS "${SGX_COMMON_CFLAGS} -O2 -UDEBUG -DNDEBUG -UEDEBUG")
else()
    message(FATAL_ERROR "SGX_MODE ${SGX_MODE} is not Debug, PreRelease or Release.")
endif()

set(ENCLAVE_INC_FLAGS "-I${SGX_PATH}/include \
                       -I${SGX_PATH}/include/tlibc \
                       -I${SGX_PATH}/include/libcxx")
                       #-I${SGX_PATH}/include/stdc++ \
                       #-I${SGX_PATH}/include/stlport")
set(ENCLAVE_C_FLAGS "${SGX_COMMON_CFLAGS} -nostdinc -fvisibility=hidden -fpie -fstack-protector ${ENCLAVE_INC_FLAGS}")
set(ENCLAVE_CXX_FLAGS "${ENCLAVE_C_FLAGS} -nostdinc++")

set(APP_INC_FLAGS "-I${SGX_PATH}/include")
set(APP_C_FLAGS "${SGX_COMMON_CFLAGS} -fPIC -Wno-attributes ${APP_INC_FLAGS}")
set(APP_CXX_FLAGS "${APP_C_FLAGS}")

function(add_enclave_library target edl_file edl_search_paths version_script key config srcs)
    get_filename_component(EDL_NAME ${edl_file} NAME_WE)
    get_filename_component(EDL_ABSPATH ${edl_file} ABSOLUTE)
    set(EDL_T_C "${CMAKE_CURRENT_BINARY_DIR}/${EDL_NAME}_t.c")
    list(APPEND edl_search_paths "${SGX_PATH}/include")
    string(REPLACE ";" ":" SEARCH_PATHS "${edl_search_paths}")
    add_custom_command(OUTPUT ${EDL_T_C}
                       COMMAND ${SGX_EDGER8R} --trusted ${EDL_ABSPATH} --search-path ${SEARCH_PATHS}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    add_library(EDL_T_O OBJECT ${EDL_T_C})
    set_target_properties(EDL_T_O PROPERTIES COMPILE_FLAGS ${ENCLAVE_C_FLAGS})
    target_include_directories(EDL_T_O PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    add_library(${target} SHARED ${srcs} $<TARGET_OBJECTS:EDL_T_O>)
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${ENCLAVE_CXX_FLAGS})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    get_filename_component(LDS_ABSPATH ${version_script} ABSOLUTE)
    target_link_libraries(${target} "${SGX_COMMON_CFLAGS} \
        -Wl,--no-undefined -nostdlib -nodefaultlibs -nostartfiles -L${SGX_LIBRARY_PATH} \
        -Wl,--whole-archive -l${SGX_TRTS_LIB} -Wl,--no-whole-archive \
        -Wl,--start-group -lsgx_tstdc -lsgx_tcxx -lsgx_tcrypto -l${SGX_TSVC_LIB} -Wl,--end-group \
        -Wl,-Bstatic -Wl,-Bsymbolic -Wl,--no-undefined \
        -Wl,-pie,-eenclave_entry -Wl,--export-dynamic \
        -Wl,--defsym,__ImageBase=0 \
        -Wl,--version-script=${LDS_ABSPATH}")

    get_filename_component(KEY_ABSPATH ${key} ABSOLUTE)
    get_filename_component(CONFIG_ABSPATH ${config} ABSOLUTE)
    add_custom_target(${target}-sign ALL ${SGX_ENCLAVE_SIGNER} sign -key ${KEY_ABSPATH} -config ${CONFIG_ABSPATH}
                      -enclave $<TARGET_FILE_NAME:${target}> -out "${target}.signed.so"
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

endfunction()

function(add_untrusted_executable target edl_file edl_search_paths srcs)
    get_filename_component(EDL_NAME ${edl_file} NAME_WE)
    get_filename_component(EDL_ABSPATH ${edl_file} ABSOLUTE)
    set(EDL_U_C "${CMAKE_CURRENT_BINARY_DIR}/${EDL_NAME}_u.c")
    list(APPEND edl_search_paths "${SGX_PATH}/include")
    string(REPLACE ";" ":" SEARCH_PATHS "${edl_search_paths}")
    add_custom_command(OUTPUT ${EDL_U_C}
                       COMMAND ${SGX_EDGER8R} --untrusted ${EDL_ABSPATH} --search-path ${SEARCH_PATHS}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    add_library(EDL_U_O OBJECT ${EDL_U_C})
    set_target_properties(EDL_U_O PROPERTIES COMPILE_FLAGS ${APP_C_FLAGS})
    target_include_directories(EDL_U_O PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    add_executable(${target} ${srcs} $<TARGET_OBJECTS:EDL_U_O>)
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${APP_CXX_FLAGS})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(${target} "${SGX_COMMON_CFLAGS} \
                                     -L${SGX_LIBRARY_PATH} \
                                     -l${SGX_URTS_LIB} \
                                     -l${SGX_USVC_LIB} \
                                     -lpthread")
endfunction()
