#include "../include/mpcController.h"
#include "../include/qpSolver.h"

MpcController::MpcController(
    std::unique_ptr<IQpSolver> qpSolverPtr, 
    const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
    const unsigned int Nu, const unsigned int Nc, const unsigned int Ns
) : 
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nc(Nc), m_Ns(Ns), 
    m_mpcProblem(Nt, Np, Nx, Nu, Nc, Ns), 
    p_qpSolver(std::move(qpSolverPtr)) {
    // Resize vectors
    m_controlSeq.resize(m_Np*m_Nu + m_Ns);
    m_scaling.input.resize(m_Nu);
    m_scaling.slack.resize(m_Ns);
    // Initialize matrices
    m_scaling.input.setOnes();
    m_scaling.slack.setOnes();
}


void MpcController::initialize() {}


bool MpcController::update() {
    // Compute the scaling
    DiagonalMatrix scaling;
    Vector scalingDiag;
    scaling.resize(m_Nu*m_Nt+m_Ns);
    scalingDiag.resize(m_Nu*m_Nt+m_Ns);
    for (unsigned int i = 0; i < m_Nt; i++) {
        scalingDiag.segment(i*m_Nu, m_Nu) = m_scaling.input;
    }
    scalingDiag.segment(m_Nu*m_Nt, m_Ns) = m_scaling.slack;
    scaling = scalingDiag.asDiagonal();
    
    /* Solve the MPC problem with the batch approach.
     * Transform the MPC problem to a QP problem, scale it, solve it, and 
     * unscale the results.
     */
    QpProblem qpFormulation(m_mpcProblem.toQp());
    qpFormulation.scale(scaling);
    m_status = p_qpSolver->solve(qpFormulation, m_controlSeq);
    m_controlSeq = scaling * m_controlSeq;
    return m_status;
}


bool MpcController::output(MatrixType ** control) {
    *control = m_controlSeq.data();
    return m_status;
};


void MpcController::terminate() {}


