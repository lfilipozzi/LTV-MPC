cmake_minimum_required(VERSION 3.0.0)

# Define build directory
set(CMAKE_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BUILD_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BUILD_DIR}/lib)
message(STATUS "EXECUTABLE_OUTPUT_PATH: " ${EXECUTABLE_OUTPUT_PATH})
message(STATUS "LIBRARY_OUTPUT_PATH: " ${LIBRARY_OUTPUT_PATH})
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})

# Rename the source files that will create the library, to a proper name!
set(libltvmpc_SRC
   mpcProblem.cpp
   mpcController.cpp
   qpSolver.cpp
)

# Add any other header files
set(ltvmpc_INCLUDES
    ../include/mpcProblem.h
    ../include/mpcController.h
    ../include/qpSolver.h
)

# Git
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
	execute_process(
		COMMAND git rev-parse HEAD
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE GIT_BRANCH
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
else()
	set(GIT_BRANCH "Unknown")
endif()
add_definitions(-DGIT_BRANCH=${GIT_BRANCH})

# Create the ltvmpc library
add_library(libltvmpc STATIC ${libltvmpc_SRC})

# Add Eigen header files
include_directories(
    ${CMAKE_SOURCE_DIR}/lib/eigen/Eigen
    ${CMAKE_SOURCE_DIR}/lib/eigen
)

# Link qpOASES library
file(GLOB qpOASES_SRC ${CMAKE_SOURCE_DIR}/lib/qpoases/src/*.cpp)
add_library(qpOASES STATIC ${qpOASES_SRC})
include_directories( 
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES/extras
)
target_link_libraries(libltvmpc qpOASES)

# Install library
install(TARGETS libltvmpc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

# Create the ltvmpc executable, add dependencies, and install
add_executable(ltvmpc main.cpp)
target_link_libraries(ltvmpc libltvmpc)
install(TARGETS ltvmpc DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
