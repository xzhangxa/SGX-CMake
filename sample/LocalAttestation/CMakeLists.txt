set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(/opt/intel/sgxsdk/include Include Enclave1 Enclave2 Enclave3 LocalAttestationCode Untrusted_LocalAttestation)
set(EDL_SEARCH_PATHS LocalAttestationCode Enclave1 Enclave2 Enclave3)

file(GLOB U_SRCS Untrusted_LocalAttestation/*.cpp)
add_library(UntrustedLocalAttestation STATIC ${U_SRCS})

file(GLOB T_SRCS LocalAttestationCode/*.c*)
add_trusted_library(LocalAttestationLib SRCS "${T_SRCS}" EDL LocalAttestationCode/LocalAttestationCode.edl EDL_SEARCH_PATHS ${EDL_SEARCH_PATHS})

add_enclave_library(Enclave1
                    SRCS Enclave1/Enclave1.cpp Enclave1/Utility_E1.cpp
                    TRUSTED_LIBS LocalAttestationLib
                    EDL Enclave1/Enclave1.edl
                    EDL_SEARCH_PATHS ${EDL_SEARCH_PATHS}
                    USE_PREFIX
                    LDSCRIPT Enclave1/Enclave1.lds)
add_enclave_library(Enclave2
                    SRCS Enclave2/Enclave2.cpp Enclave2/Utility_E2.cpp
                    TRUSTED_LIBS LocalAttestationLib
                    EDL Enclave2/Enclave2.edl
                    EDL_SEARCH_PATHS ${EDL_SEARCH_PATHS}
                    USE_PREFIX
                    LDSCRIPT Enclave2/Enclave2.lds)
add_enclave_library(Enclave3
                    SRCS Enclave3/Enclave3.cpp Enclave3/Utility_E3.cpp
                    TRUSTED_LIBS LocalAttestationLib
                    EDL Enclave3/Enclave3.edl
                    EDL_SEARCH_PATHS ${EDL_SEARCH_PATHS}
                    USE_PREFIX
                    LDSCRIPT Enclave3/Enclave3.lds)

enclave_sign(Enclave1 OUTPUT libenclave1.so KEY Enclave1/Enclave1_private.pem CONFIG Enclave1/Enclave1.config.xml)
enclave_sign(Enclave2 OUTPUT libenclave2.so KEY Enclave2/Enclave2_private.pem CONFIG Enclave2/Enclave2.config.xml)
enclave_sign(Enclave3 OUTPUT libenclave3.so KEY Enclave3/Enclave3_private.pem CONFIG Enclave3/Enclave3.config.xml)

add_untrusted_executable(LocalAttestation
                         SRCS App/App.cpp
                         USE_PREFIX
                         EDL Enclave1/Enclave1.edl Enclave2/Enclave2.edl Enclave3/Enclave3.edl
                         EDL_SEARCH_PATHS ${EDL_SEARCH_PATHS})
target_link_libraries(LocalAttestation UntrustedLocalAttestation)
