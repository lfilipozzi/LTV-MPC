cmake_minimum_required(VERSION 3.0.0)

# Add Eigen header files to unit test source files
include_directories(${CMAKE_SOURCE_DIR}/lib/eigen/Eigen)
include_directories( 
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES
    ${CMAKE_SOURCE_DIR}/lib/qpoases/include/qpOASES/extras
)

# Set up unit testing
add_executable(testQpOasesSolver ../test/testQpOasesSolver.cpp)
target_link_libraries(testQpOasesSolver libltvmpc)
add_test(testQpOasesSolver testQpOasesSolver)

add_executable(testMpcProblem ../test/testMpcProblem.cpp)
target_link_libraries(testMpcProblem libltvmpc)
add_test(testMpcProblem testMpcProblem)

add_executable(testMpcController ../test/testMpcController.cpp)
target_link_libraries(testMpcController libltvmpc)
add_test(testMpcController testMpcController)
