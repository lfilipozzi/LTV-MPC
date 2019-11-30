#include "../include/mpcController.h"
#include "../include/qpSolver.h"


MpcController::MpcController(std::unique_ptr<IQpSolver> qpSolverPtr, 
                             const unsigned int Nt, const unsigned int Np, 
                             const unsigned int Nx, const unsigned int Nu, 
                             const unsigned int Nr, const unsigned int Nc, 
                             const unsigned int Ns) : 
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nr(Nr), m_Nc(Nc), m_Ns(Ns), 
    m_mpcFormulation(Nt, Np, Nx, Nu, Nr, Nc, Ns), 
    m_qpSolverPtr(std::move(qpSolverPtr)) {
        
}


void MpcController::initialize() {
    // TODO
}


bool MpcController::update(Vector & solution) {
    // State-space matrices TODO allocating these matrices needs to be done only at initalization
    Matrix A, B, C, D;
    // Cost functions matrices and vectors
    Matrix Q, R, T;
    Vector fx, fu;
    // Constraints matrices and vectors
    Matrix Ax, Au, As;
    Vector b;
    // Actuator bounds
    Vector lb, ub;
    
    // Resize matrices and vectors //TODO need to be done only at initialization of MPC
    A.resize(m_Nx,m_Nx);
    B.resize(m_Nx,m_Nu);
    C.resize(m_Nr,m_Nx);
    D.resize(m_Nr,m_Nu);
//     Q.resize(); TODO
//     R.resize(); TODO
//     T.resize(); TODO
//     fx.resize(); TODO
//     fu.resize(); TODO
//     Ax.resize(); TODO
//     Au.resize(); TODO
//     As.resize(); TODO
//     b.resize(); TODO
//     lb.resize(); TODO
//     ub.resize(); TODO
    
    // TODO Compute the matrices to set up the MPC problem
    
    // TODO the following code needs to be done only once as we are using pointers: Change the code to do that
    m_mpcFormulation.setPlantModel(&A, &B, &C, &D);
    m_mpcFormulation.setCostFunction(&Q, &R, &T, &fx, &fu);
    m_mpcFormulation.setActuatorBounds(&lb, &ub);
    m_mpcFormulation.setConstraints(&Ax, &Au, &As, &b);
    
    // Solve the MPC problem with the batch approach
    Vector sequenceSol(m_Nt*m_Nu+m_Ns);
    QpProblem qpFormulation(m_mpcFormulation.toQp());
    bool status = m_qpSolverPtr->solveProbem(qpFormulation, sequenceSol);
    
//     solution = TODO
    
    return status;
}

