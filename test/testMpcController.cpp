#include <Eigen>
#include "../include/mpcController.h"
#include <memory>

void testMpcController() {
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
    
    // Create QP solver
    std::unique_ptr<IQpSolver> solver;
    int nWSR = 100;
    solver = std::make_unique<QpOasesSolver>(Nu*Nt+Ns, Nc*Np, nWSR);
    
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
    
    float tol = 1e-3;
    assert(status);
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
}

int main() {
    testMpcController();
    
    return 0;
}
