cmake_minimum_required(VERSION 3.0.0)

# Add Eigen header files to unit test source files
include_directories(${CMAKE_SOURCE_DIR}/lib/eigen/Eigen)
include_directories( 
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES/extras
)

# Set up unit testing
add_executable(testMpcProblem_EXE ../test/testMpcProblem.cpp)
target_link_libraries(testMpcProblem_EXE ltvmpc_LIB)
add_test(testMpcProblem testMpcProblem_EXE)

add_executable(testQpOasesSolver_EXE ../test/testQpOasesSolver.cpp)
target_link_libraries(testQpOasesSolver_EXE ltvmpc_LIB)
add_test(testQpOasesSolver testQpOasesSolver_EXE)
