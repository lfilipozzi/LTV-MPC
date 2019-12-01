#include "../include/mpcController.h"


MpcController::MpcController(
    std::unique_ptr<IQpSolver> qpSolverPtr, 
    const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
    const unsigned int Nu, const unsigned int Nc, const unsigned int Ns
) : 
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nc(Nc), m_Ns(Ns), 
    m_mpcProblem(Nt, Np, Nx, Nu, Nc, Ns), 
    m_qpSolverPtr(std::move(qpSolverPtr)) {
        
}


void MpcController::initialize() {
    
}


bool MpcController::update(Vector & solution) {
    // Solve the MPC problem with the batch approach
    solution.resize(m_Nt*m_Nu+m_Ns);
    QpProblem qpFormulation(m_mpcProblem.toQp());
    bool status = m_qpSolverPtr->solve(qpFormulation, solution);
    
    return status;
}

