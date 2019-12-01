#include <assert.h>
#include <Eigen>
#include "../include/qpSolver.h"

/**
 * Check the implementation of the qpOASES solver can solve a QP problem.
 */
void testSolveQp() {
    // Set up the solver
    unsigned int NxQP(2);
    unsigned int NcQP(1);
    QpOasesSolver solver(NxQP, NcQP);
    
    // Set up the QP problem to solve
    QpProblem problem;
    problem.resize(NxQP,NcQP);
    
    
    Vector solution;
    Vector expectedSolution;
    bool status;
    expectedSolution.resize(NxQP);
    
    // Solve the problem
    // Check the solver can find the solution
    problem.H << 1, 0, 0, 2;
    problem.f << 1, 2;
    problem.A << 1, 0;
    problem.b << 2;
    problem.lb << 1, 2;
    problem.ub << 3, 4;
    expectedSolution << 1, 2;
    status = solver.solve(problem, solution);
    
    assert(solution == expectedSolution && status);
    
    // Solve the problem again to check hot-start
    // Check hot start and check return false when the problem is infeasible
    problem.H << 1, 0, 0, 2;
    problem.f << 1, 2;
    problem.A << 1, 1;
    problem.b << 2;
    problem.lb << 1, 2;
    problem.ub << 3, 4;
    status = solver.solve(problem, solution);
    assert(!status);
}


int main() {
    testSolveQp();
    
    return 0;
}
