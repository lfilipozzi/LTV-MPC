#include "../include/mpcController.h"


MpcController::MpcController(
    std::unique_ptr<IQpSolver> qpSolverPtr, 
    const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
    const unsigned int Nu, const unsigned int Nc, const unsigned int Ns
) : 
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nc(Nc), m_Ns(Ns), 
    m_mpcProblem(Nt, Np, Nx, Nu, Nc, Ns), 
    m_qpSolverPtr(std::move(qpSolverPtr)) {
    // Resize vectors
    m_controlSeq.resize(m_Np*m_Nu + m_Ns);
}


void MpcController::initialize() {}


bool MpcController::update() {
    // Solve the MPC problem with the batch approach
    QpProblem qpFormulation(m_mpcProblem.toQp());
    m_status = m_qpSolverPtr->solve(qpFormulation, m_controlSeq);
    return m_status;
}


bool MpcController::output(Vector control) {
    control.resize(m_Nu);
    control = m_controlSeq.segment(0,m_Nu);
    return m_status;
};


void MpcController::terminate() {}


