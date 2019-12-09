#include <qpOASES.hpp>
#include <ctime>
#include <iostream>
#include <memory>
#include "../include/mpcProblem.h"
#include "../include/mpcController.h"

#include <unsupported/Eigen/MatrixFunctions>


/** Example for qpOASES main function using the QProblem class. */
int main() {
    {
        unsigned int Nx = 3;
        unsigned int Nu = 3;
        Matrix A, B;
        A.resize(Nx, Nx);
        B.resize(Nx, Nu);
        A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        B << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        float Ts = 0.1;
        
        std::clock_t c_start = std::clock();
        A = (A*Ts).exp();
        B *= A;
        std::clock_t c_end = std::clock();
        std::cout << (c_end - c_start) / (double) CLOCKS_PER_SEC << std::endl;
        
        std::cout << "A:" << A << std::endl << std::endl;
        std::cout << "B:" << B << std::endl << std::endl;
    }
    
    
    // Define cost function
    Matrix Q, R, T;
    Vector fx, fu;
    
    // Define plant
    Matrix A, B;
    Vector x0;
    float Ts = 0.01;
    
    // Define actuator bounds
    Vector ulb, uub;
    
    // Polytopic constraints
    Matrix Ax, Au;
    Vector b;
    
    // Define sizes
    unsigned int Nt, Np, Nx, Nu, Nc, Ns;
    Np = 10;
    Nt = 5;
    Nx = 3;
    Nu = 2;
    Nc = 1;
    Ns = 0;
    
    Q.resize(Nx, Nx);
    R.resize(Nu, Nu);
    T.resize(Nx, Nu);
    fx.resize(Nx);
    fu.resize(Nu);
    A.resize(Nx, Nx);
    B.resize(Nx, Nu);
    x0.resize(Nx);
    ulb.resize(Nu);
    uub.resize(Nu);
    Ax.resize(Nc, Nx);
    Au.resize(Nc, Nu);
    b.resize(Nc);
    
    Q = Matrix::Identity(Nx, Nx);
    R = Matrix::Identity(Nu, Nu);
    T << 1, 0, 0, 1, 0, 0;
    fx << 1, 1, 1;
    fu << 1, 1;
    
    A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
    B << 1, 0, 0, 1, 0, 0;
    x0 << 0, 1, 2;
    
    Ax << 1, 0, 0;
    Au << 1, 1;
    b << 1000;
    
    ulb << -10, -10;
    uub <<  10,  10;
    
    std::cout << "Q:"   << std::endl << Q   << std::endl << std::endl;
    std::cout << "R:"   << std::endl << R   << std::endl << std::endl;
    std::cout << "T:"   << std::endl << T   << std::endl << std::endl;
    std::cout << "fx:"  << std::endl << fx  << std::endl << std::endl;
    std::cout << "fu:"  << std::endl << fu  << std::endl << std::endl;
    std::cout << "A:"   << std::endl << A   << std::endl << std::endl;
    std::cout << "B:"   << std::endl << B   << std::endl << std::endl;
    std::cout << "x0:"  << std::endl << x0  << std::endl << std::endl;
    std::cout << "ulb:" << std::endl << ulb << std::endl << std::endl;
    std::cout << "uub:" << std::endl << uub << std::endl << std::endl;
    std::cout << "Ax:"  << std::endl << Ax << std::endl << std::endl;
    std::cout << "Au:"  << std::endl << Au  << std::endl << std::endl;
    std::cout << "b:"   << std::endl << b   << std::endl << std::endl;
    
    std::clock_t c_start = std::clock();
    
    // Create QP solver
    std::unique_ptr<IQpSolver> solver;
    int nWSR = 100;
    solver = std::make_unique<QpOasesSolver>(Nu*Nt, Nc*Np, nWSR);
    
    // Create MPC controller 
    MpcController controller(std::move(solver), Nt, Np, Nx, Nu, Nc, Ns);
    controller.initialize();
    
    // Set MPC formulation
    controller.setCostFunction(
        Q.data(), R.data(), T.data(), fx.data(), fu.data()
    );
    controller.setInitialCondition(x0.data());
    controller.setPlantModel(A.data(), B.data(), Ts);
    controller.setActuatorBounds(ulb.data(), uub.data());
    controller.setConstraints(Ax.data(), Au.data(), b.data());
    
    controller.update();
    
    MatrixType * control;
    bool status = controller.output(&control);
    
    std::clock_t c_end = std::clock();
    
    std::cout << "status: " << status << std::endl;
    std::cout << "u: " << 
        *control << ", " << 
        *(control+1) << ", " << 
        *(control+2) << ", " << 
        *(control+3) << ", " << 
        *(control+4) << ", " << 
        *(control+5) << ", " << 
        *(control+6) << ", " << 
        *(control+7) << ", " << 
        *(control+8) << ", " << 
        *(control+9) << std::endl << std::endl;
        
    std::cout << (c_end - c_start) / (double) CLOCKS_PER_SEC << std::endl;
    
    c_start = std::clock();
    controller.update();
    status = controller.output(&control);
    c_end = std::clock();
    std::cout << (c_end - c_start) / (double) CLOCKS_PER_SEC << std::endl;
    
    c_start = std::clock();
    controller.update();
    status = controller.output(&control);
    c_end = std::clock();
    std::cout << (c_end - c_start) / (double) CLOCKS_PER_SEC << std::endl;
    
    c_start = std::clock();
    controller.update();
    status = controller.output(&control);
    c_end = std::clock();
    std::cout << (c_end - c_start) / (double) CLOCKS_PER_SEC << std::endl;
    
    float tol = 1e-3;
    assert(abs(*control     - -0.5)    < tol);
    assert(abs(*(control+1) - -1.2093) < tol);
    assert(abs(*(control+2) -  0)      < tol);
    assert(abs(*(control+3) - -1)      < tol);
    assert(abs(*(control+4) -  0)      < tol);
    assert(abs(*(control+5) - -0.7907) < tol);
    assert(abs(*(control+6) -  0)      < tol);
    assert(abs(*(control+7) - -0.5815) < tol);
    assert(abs(*(control+8) -  0)      < tol);
    assert(abs(*(control+9) - -0.3722) < tol);
    
    
    
    
    
    
//     // Define the MPC Problem
//     unsigned int Np = 10;
//     unsigned int Nt = 5;
//     unsigned int Nx = 3;
//     unsigned int Nu = 2;
//     unsigned int Nc = 1;
//     unsigned int Ns = 1;
//     
//     float Ts = 0.1;
//     
//     MpcProblem mpcProblem(Nt, Np, Nx, Nu, Nc, Ns);
//     
//     Matrix Q, R, T;
//     Vector fx, fu;
//     
//     Matrix A, B;
//     Vector x0;
//     
//     Matrix Ax, Au;
//     Vector b;
//     
//     Vector lb, ub;
//     
//     Q.resize(Nx,Nx);
//     R.resize(Nu,Nu);
//     T.resize(Nx,Nu);
//     fx.resize(Nx);
//     fu.resize(Nu);
//     
//     A.resize(Nx,Nx);
//     B.resize(Nx,Nu);
//     x0.resize(Nx);
//     
//     Ax.resize(Nc,Nx);
//     Au.resize(Nc,Nu);
//     b.resize(Nc);
//     
//     lb.resize(Nu);
//     ub.resize(Nu);
//     
//     Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//     R << 1, 0, 0, 1;
//     T << 1, 0, 0, 1, 0, 0;
//     fx << 1, 1, 1;
//     fu << 1, 1;
//     
//     A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
//     B << 1, 0, 0, 1, 0, 0;
//     x0 << 0, 1, 2;
//     
//     Ax << 1, 1, 1;
//     Au << 1, 1;
//     b << 1000;
//     
//     lb << -10, -10;
//     ub <<  10,  10;
//     
//     mpcProblem.setCostFunction(
//         Q.data(), R.data(), T.data(), fx.data(), fu.data()
//     );
//     mpcProblem.setPlantModel(A.data(), B.data(), Ts);
//     mpcProblem.setInitialCondition(x0.data());
//     mpcProblem.setConstraints(Ax.data(), Au.data(), b.data());
//     mpcProblem.setActuatorBounds(lb.data(), ub.data());
//     mpcProblem.setSoftConstraints(0, {0}, 10000);
//     
//     // Convert MPC problem to QP problem
//     QpProblem qp = mpcProblem.toQp();
//     
//     std::cout << "H:"  << std::endl << qp.H << std::endl << std::endl;
//     std::cout << "f:"  << std::endl << qp.f << std::endl << std::endl;
//     std::cout << "A:"  << std::endl << qp.A << std::endl << std::endl;
//     std::cout << "b:"  << std::endl << qp.b << std::endl << std::endl;
//     std::cout << "lb:" << std::endl << qp.lb << std::endl << std::endl;
//     std::cout << "ub:" << std::endl << qp.ub << std::endl << std::endl;
    
    
//     Matrix test;
//     test.resize(3,3);
//     test << 1, 2, 3, 4, 5, 6, 7, 8, 9;
//     // Column major
//     for (unsigned int i = 0; i < 9; i++) {
//         std::cout << *(test.data() + i);
//     }
//     std::cout << std::endl << std::endl;
//     // Row major? Yes when defining new variable.
//     Matrix testT;
//     testT.resize(3,3);
//     testT = test.transpose();
//     for (unsigned int i = 0; i < 9; i++) {
//         std::cout << *(testT.data() + i);
//     }
//     std::cout << std::endl << std::endl;
//     
//     unsigned int Np = 1;
//     unsigned int Nt = 1;
//     unsigned int Nx = 1;
//     unsigned int Nu = 1;
//     unsigned int Nc = 1;
//     unsigned int Ns = 1;
//     std::unique_ptr<IQpSolver> solver;
//     solver = std::make_unique<QpOasesSolver>(Nu*Np, Nc);
//     MpcController * controller = new MpcController(std::move(solver), Nt, Np, Nx, Nu, Nc, Ns);
//     
//     unsigned int slackIdx;
//     std::vector<unsigned int> constraintsIdx;
//     double weight;
//     controller->setSoftConstraints(slackIdx, constraintsIdx, weight);
//     std::cout << "OK" << std::endl;
//     
//     delete(controller);
    
    
//     // Set up the solver
//     unsigned int NxQP(2);
//     unsigned int NcQP(1);
//     QpOasesSolver solver(NxQP, NcQP);
//     
//     // Set up the QP problem to solve
//     QpProblem problem;
//     problem.resize(NxQP,NcQP);
//     
//     
//     Vector solution;
//     Vector expectedSolution;
//     bool status;
//     expectedSolution.resize(NxQP);
//     
//     // Solve the problem
//     // Check the solver can find the solution
//     problem.H << 1, 0, 0, 2;
//     problem.f << 1, 2;
//     problem.A << 1, 0;
//     problem.b << 2;
//     problem.lb << 1, 2;
//     problem.ub << 3, 4;
//     expectedSolution << 1, 2;
//     status = solver.solve(problem, solution);
//     
//     assert(solution == expectedSolution && status);
//     
//     // Solve the problem again to check hot-start
//     // Check hot start and check return false when the problem is infeasible
//     problem.H << 1, 0, 0, 2;
//     problem.f << 1, 2;
//     problem.A << 1, 1;
//     problem.b << 2;
//     problem.lb << 1, 2;
//     problem.ub << 3, 4;
//     status = solver.solve(problem, solution);
//     assert(!status);
    
    
	return 0;
}
