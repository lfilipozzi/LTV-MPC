cmake_minimum_required(VERSION 3.0.0)
project(libltvmpc CXX)
set (libltvmpc_VERSION_MAJOR 0)
set (libltvmpc_VERSION_MINOR 1)
include(CheckCXXCompilerFlag)

# Use C++ 14
set(CMAKE_CXX_STANDARD 14)
# Add -fPIC compiler option
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Option to build shared libraries
option (BUILD_SHARED_LIBS "Build shared libraries." ON)

# Detect compiler and act accordingly
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR 
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    option(CMAKE_CXX_VERBOSE "Verbose compilation output?" OFF)
    option(CMAKE_LD_VERBOSE "Verbose linking output?" OFF)

    if(CMAKE_CXX_VERBOSE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --verbose")
    endif()
    if(CMAKE_LD_VERBOSE)
        set(
            CMAKE_EXE_LINKER_FLAGS 
            "-Wl,--as-needed -Wl,--trace -Wl,--threads -Wl,--stats 
            ${CMAKE_EXE_LINKER_FLAGS}")
    endif()
    check_cxx_compiler_flag(-Wall HAVE_WALL)
    if (HAVE_WALL)
        # Enable all warnings
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
    endif()

    # First check for C++17; if not check for C++14 as fallback
    check_cxx_compiler_flag(-std=c++17 COMPILER_SUPPORTS_CXX17)
    if(COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    elseif(NOT COMPILER_SUPPORTS_CXX17)
        check_cxx_compiler_flag(-std=c++14 COMPILER_SUPPORTS_CXX14)
        if(COMPILER_SUPPORTS_CXX14)
              set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
        endif()
    else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 or 
            14 support. Please use a different C++ compiler.")
    endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    # Using Intel C++
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # Using Visual Studio C++
endif()

# Enable unit testing
enable_testing()
add_subdirectory(test)

# Add subdirectories to the build
add_subdirectory(src)
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    add_subdirectory(src/Linux/)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    add_subdirectory(src/Windows/)
endif()



