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
    
    // Define sizes
    unsigned int Nt, Np, Nx, Nu, Nc, Ns;
    Np = 10;
    Nt = 5;
    Nx = x0.size();
    Nu = B.cols();
    Nc = 0;
    Ns = 0;
    
    // Create QP solver
    std::unique_ptr<IQpSolver> solver;
    solver = std::make_unique<QpOasesSolver>(Nu*Np, Nc);
    
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
    
}

int main() {
    testMpcController();
    
    return 0;
}
