##################################
#    MATLAB Wrapper using mex    #
##################################

find_package(Matlab QUIET COMPONENTS MAIN_PROGRAM MEX_COMPILER)

if(NOT Matlab_FOUND)
    message(STATUS "${RED}+++${RESET} MATLAB installation not found, check your PATH environment variable")
    return()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # Check to see if the selected GNU compiler is "officially" supported
    if(NOT     ((${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "4.9.0.0") AND
                (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS    "5.0.0.0") AND
                (${CMAKE_C_COMPILER_VERSION}   VERSION_GREATER "4.9.0.0") AND
                (${CMAKE_C_COMPILER_VERSION}   VERSION_LESS    "5.0.0.0")))
        
        # The compiler being may not be technically not supported by MATLAB mex. There are two solutions to this
        # problem: 1) LD_PRELOAD or 2) use mex to statically link to the current GCC's libstdc++.
        # By default, we choose option 2. See mex_compile.sh.in for more info.
        # Note that when MATLAB supports GCC 6.3, this code can be removed.
        set(UNOFFICIAL_MEX_SUPPORT TRUE)

        # this gets passed to mex_compile.sh.in
        set(ADD_STATIC_LIBCXX true)
    endif()
endif()

# Display Matlab location and compiler version
message(STATUS "${BLUE}+++${RESET} Matlab installation found: ${BLUE}${Matlab_ROOT_DIR}${RESET}")
message(STATUS "${BLUE}+++${RESET} GCC C++ Compiler (g++) Version: ${BLUE}${CMAKE_CXX_COMPILER_VERSION}${RESET}")
if(UNOFFICIAL_MEX_SUPPORT)
    message(STATUS "${BLUE}+++${RESET} Compiler version is ${YELLOW}(technically not)${RESET} supported by Matlab")
else()
    message(STATUS "${BLUE}+++${RESET} Compiler version is supported by Matlab")
endif()

##################################
# Build and setup the MEX module #
##################################

set(MEX_DEFINES "")
foreach(def ${ROSFLIGHT_GIT})
    string(APPEND MEX_DEFINES "-D${def} ")
endforeach(def)

# Create a list of sources necessary for the MEX module and prepend with appropriate path
set(MEX_BINDING_SOURCE src/rosflight_api.cpp)
string(REGEX REPLACE "([^;]+)" "${CMAKE_CURRENT_SOURCE_DIR}/\\1" MEX_BINDING_SOURCE "${MEX_BINDING_SOURCE}")

# Configure the mex_compile.sh script that will build and link to create a mex
set(MEX_COMPILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/mex_compile.sh.in)
set(MEX_COMPILE ${CMAKE_CURRENT_BINARY_DIR}/mex_compile.sh)
configure_file(${MEX_COMPILE_IN} ${MEX_COMPILE} @ONLY)

set(MEX_PREFIX rosflight_api)
set(MEX_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MEX_PREFIX}.${Matlab_MEX_EXTENSION}")

add_custom_target(matlab_bindings ALL DEPENDS ${MEX_OUTPUT})
add_custom_command(OUTPUT ${MEX_OUTPUT}
                    COMMAND ${CMAKE_COMMAND} -E env bash ${MEX_COMPILE} ${MEX_PREFIX} $<TARGET_FILE:${STATIC_LIB}>
                    DEPENDS ${MEX_BINDING_SOURCE} ${STATIC_LIB} ${MEX_COMPILE})

# Place the compiled .mexa64 and .m files in the Matlab path
# install(TARGETS matlab_bindings DESTINATION "$ENV{HOME}/Documents/MATLAB")
install(DIRECTORY "+rosflight" DESTINATION "$ENV{HOME}/Documents/MATLAB")
install(FILES ${MEX_OUTPUT} DESTINATION "$ENV{HOME}/Documents/MATLAB/+rosflight")
