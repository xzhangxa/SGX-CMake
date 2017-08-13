# FindPackage cmake file for Intel SGX SDK

cmake_minimum_required(VERSION 2.8.8)
include(CMakeParseArguments)

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
set(SGX_MODE PreRelease CACHE STRING "SGX build mode: Debug; PreRelease; Release.")

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

function(_build_edl_obj edl edl_search_paths)
    get_filename_component(EDL_NAME ${edl} NAME_WE)
    get_filename_component(EDL_ABSPATH ${edl} ABSOLUTE)
    set(EDL_T_C "${CMAKE_CURRENT_BINARY_DIR}/${EDL_NAME}_t.c")
    set(SEARCH_PATHS "")
    foreach(path ${edl_search_paths})
        get_filename_component(ABSPATH ${path} ABSOLUTE)
        list(APPEND SEARCH_PATHS "${ABSPATH}")
    endforeach()
    list(APPEND SEARCH_PATHS "${SGX_PATH}/include")
    string(REPLACE ";" ":" SEARCH_PATHS "${SEARCH_PATHS}")
    add_custom_command(OUTPUT ${EDL_T_C}
                       COMMAND ${SGX_EDGER8R} --trusted ${EDL_ABSPATH} --search-path ${SEARCH_PATHS}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    add_library(${target}-edlobj OBJECT ${EDL_T_C})
    set_target_properties(${target}-edlobj PROPERTIES COMPILE_FLAGS ${ENCLAVE_C_FLAGS})
    target_include_directories(${target}-edlobj PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

# build trusted static library to be linked into enclave library
# 'srcs' is list of source files
function(add_trusted_library target srcs)
    set(oneValueArgs EDL EDL_SEARCH_PATHS LDSCRIPT)
    cmake_parse_arguments("SGX" "" "${oneValueArgs}" "" ${ARGN})
    if("${SGX_EDL}" STREQUAL "")
        message(FATAL_ERROR "${target}: SGX enclave edl file is not provided!")
    endif()
    if("${SGX_EDL_SEARCH_PATHS}" STREQUAL "")
        message(FATAL_ERROR "${target}: SGX enclave edl file search paths are not provided!")
    endif()
    if(NOT "${SGX_LDSCRIPT}" STREQUAL "")
        get_filename_component(LDS_ABSPATH ${SGX_LDSCRIPT} ABSOLUTE)
        set(LDSCRIPT_FLAG "-Wl,--version-script=${LDS_ABSPATH}")
    endif()

    _build_edl_obj(${SGX_EDL} ${SGX_EDL_SEARCH_PATHS})

    add_library(${target} STATIC ${srcs} $<TARGET_OBJECTS:${target}-edlobj>)
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${ENCLAVE_CXX_FLAGS})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    target_link_libraries(${target} "${SGX_COMMON_CFLAGS} \
        -Wl,--no-undefined -nostdlib -nodefaultlibs -nostartfiles -L${SGX_LIBRARY_PATH} \
        -Wl,--whole-archive -l${SGX_TRTS_LIB} -Wl,--no-whole-archive \
        -Wl,--start-group -lsgx_tstdc -lsgx_tcxx -lsgx_tkey_exchange -lsgx_tcrypto -l${SGX_TSVC_LIB} -Wl,--end-group \
        -Wl,-Bstatic -Wl,-Bsymbolic -Wl,--no-undefined \
        -Wl,-pie,-eenclave_entry -Wl,--export-dynamic \
        ${LDSCRIPT_FLAG} \
        -Wl,--defsym,__ImageBase=0")
endfunction()

# build enclave shared library
# 'srcs' is list of source files
# 'trusted_libs' is list of trusted libraries to be built in enclave
function(add_enclave_library target srcs trusted_libs)
    set(oneValueArgs EDL EDL_SEARCH_PATHS LDSCRIPT)
    cmake_parse_arguments("SGX" "" "${oneValueArgs}" "" ${ARGN})
    if("${SGX_EDL}" STREQUAL "")
        message(FATAL_ERROR "${target}: SGX enclave edl file is not provided!")
    endif()
    if("${SGX_EDL_SEARCH_PATHS}" STREQUAL "")
        message(FATAL_ERROR "${target}: SGX enclave edl file search paths are not provided!")
    endif()
    if(NOT "${SGX_LDSCRIPT}" STREQUAL "")
        get_filename_component(LDS_ABSPATH ${SGX_LDSCRIPT} ABSOLUTE)
        set(LDSCRIPT_FLAG "-Wl,--version-script=${LDS_ABSPATH}")
    endif()

    _build_edl_obj(${SGX_EDL} ${SGX_EDL_SEARCH_PATHS})

    add_library(${target} SHARED ${srcs} $<TARGET_OBJECTS:${target}-edlobj>)
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${ENCLAVE_CXX_FLAGS})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    set(TLIB_LIST "")
    foreach(TLIB ${trusted_libs})
        string(APPEND TLIB_LIST "$<TARGET_FILE:${TLIB}> ")
        add_dependencies(${target} ${TLIB})
    endforeach()

    target_link_libraries(${target} "${SGX_COMMON_CFLAGS} \
        -Wl,--no-undefined -nostdlib -nodefaultlibs -nostartfiles -L${SGX_LIBRARY_PATH} \
        -Wl,--whole-archive -l${SGX_TRTS_LIB} -Wl,--no-whole-archive \
        -Wl,--start-group ${TLIB_LIST} -lsgx_tstdc -lsgx_tcxx -lsgx_tkey_exchange -lsgx_tcrypto -l${SGX_TSVC_LIB} -Wl,--end-group \
        -Wl,-Bstatic -Wl,-Bsymbolic -Wl,--no-undefined \
        -Wl,-pie,-eenclave_entry -Wl,--export-dynamic \
        ${LDSCRIPT_FLAG} \
        -Wl,--defsym,__ImageBase=0")
endfunction()

function(enclave_sign target)
    set(oneValueArgs KEY CONFIG)
    cmake_parse_arguments("SGX" "" "${oneValueArgs}" "" ${ARGN})
    if("${SGX_CONFIG}" STREQUAL "")
        message(FATAL_ERROR "${target}: SGX enclave config is not provided!")
    endif()
    if("${SGX_KEY}" STREQUAL "")
        if (NOT SGX_HW OR NOT SGX_MODE STREQUAL "Release")
            message(FATAL_ERROR "Private key used to sign enclave is not provided!")
        endif()
    else()
        get_filename_component(KEY_ABSPATH ${SGX_KEY} ABSOLUTE)
    endif()

    get_filename_component(CONFIG_ABSPATH ${SGX_CONFIG} ABSOLUTE)

    if(SGX_HW AND SGX_MODE STREQUAL "Release")
        add_custom_target(${target}-sign ALL
                          COMMAND ${SGX_ENCLAVE_SIGNER} gendata -config ${CONFIG_ABSPATH}
                              -enclave $<TARGET_FILE_NAME:${target}> -out "${target}_hash.hex"
                          COMMAND ${CMAKE_COMMAND} -E cmake_echo_color
                              --cyan "SGX production enclave first step signing finished, \
use ${CMAKE_CURRENT_BINARY_DIR}/${target}_hash.hex for second step"
                          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    else()
        add_custom_target(${target}-sign ALL ${SGX_ENCLAVE_SIGNER} sign -key ${KEY_ABSPATH} -config ${CONFIG_ABSPATH}
                          -enclave $<TARGET_FILE_NAME:${target}> -out "${target}.signed.so"
                          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endfunction()

function(add_untrusted_executable target srcs)
    set(oneValueArgs EDL EDL_SEARCH_PATHS)
    cmake_parse_arguments("SGX" "" "${oneValueArgs}" "" ${ARGN})
    if("${SGX_EDL}" STREQUAL "")
        message(FATAL_ERROR "SGX enclave edl file is not provided!")
    endif()
    if("${SGX_EDL_SEARCH_PATHS}" STREQUAL "")
        message(FATAL_ERROR "SGX enclave edl file search paths are not provided!")
    endif()

    get_filename_component(EDL_NAME ${SGX_EDL} NAME_WE)
    get_filename_component(EDL_ABSPATH ${SGX_EDL} ABSOLUTE)
    set(EDL_U_C "${CMAKE_CURRENT_BINARY_DIR}/${EDL_NAME}_u.c")
    set(SEARCH_PATHS "")
    foreach(path ${SGX_EDL_SEARCH_PATHS})
        get_filename_component(ABSPATH ${path} ABSOLUTE)
        list(APPEND SEARCH_PATHS "${ABSPATH}")
    endforeach()
    list(APPEND SEARCH_PATHS "${SGX_PATH}/include")
    string(REPLACE ";" ":" SEARCH_PATHS "${SEARCH_PATHS}")
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
                                     -lsgx_ukey_exchange \
                                     -lpthread")
endfunction()
