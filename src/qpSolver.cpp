#include "../include/qpSolver.h"
#include <QProblem.hpp>
#include <limits>


/***
 *       ____         ____           _____ ______  _____ 
 *      / __ \       / __ \   /\    / ____|  ____|/ ____|
 *     | |  | |_ __ | |  | | /  \  | (___ | |__  | (___  
 *     | |  | | '_ \| |  | |/ /\ \  \___ \|  __|  \___ \ 
 *     | |__| | |_) | |__| / ____ \ ____) | |____ ____) |
 *      \___\_\ .__/_\____/_/   _\_\_____/|______|_____/ 
 *            | |  / ____|     | |                       
 *            |_| | (___   ___ | |_   _____ _ __         
 *                 \___ \ / _ \| \ \ / / _ \ '__|        
 *                 ____) | (_) | |\ V /  __/ |           
 *                |_____/ \___/|_| \_/ \___|_|           
 *                                                       
 *                                                       
 */

QpOasesSolver::QpOasesSolver(
    unsigned int NxQP, unsigned int NcQP, int nWSR, double * cpuTime
) : 
    m_coldStart(true), 
    m_NxQP(NxQP),
    m_NcQP(NcQP),
    m_nWSR(nWSR), 
    m_cpuTimePtr(cpuTime) {
    m_lbA.resize(m_NcQP);
    m_lbA.segment(0, m_NcQP) = 
        Matrix::Constant(m_NcQP, 1, std::numeric_limits<float>::min());
}


QpOasesSolver::~QpOasesSolver() {
    
}


bool QpOasesSolver::solveProbem(const QpProblem & qpProblem, 
                                Vector & solution) {
    // Check QP size
    unsigned int NxQP;
    unsigned int NcQP;
    bool sizeCheck = qpProblem.getSize(NxQP, NcQP);
    if(!(sizeCheck && NxQP == m_NxQP && NcQP == m_NcQP))
        return false;
    
    solution.resize(m_NxQP);
    
    // Create reference to the QP vectors and matrices
    const Matrix & H = qpProblem.H;
    const Vector & f = qpProblem.f;
    const Matrix & A = qpProblem.A;
    const Vector & b = qpProblem.b;
    const Vector & lb = qpProblem.lb;
    const Vector & ub = qpProblem.ub;
        
    if(m_coldStart) {
        // Initialize the problem
        m_qpOasesProblem = qpOASES::SQProblem(m_NxQP, m_NcQP);
        
        // Solve the problem
        m_qpOasesProblem.init(H.data(),
                              f.data(),
                              A.data(),
                              lb.data(),
                              ub.data(),
                              m_lbA.data(),
                              b.data(), 
                              m_nWSR,
                              m_cpuTimePtr
                             );
    }
    else {
        // Solve the problem
        m_qpOasesProblem.hotstart(H.data(),
                                  f.data(),
                                  A.data(),
                                  lb.data(),
                                  ub.data(),
                                  m_lbA.data(),
                                  b.data(), 
                                  m_nWSR,
                                  m_cpuTimePtr
                                 );
    }
    
    // Get solution
    qpOASES::returnValue status;
    qpOASES::real_t solutionData[m_NxQP];
    status = m_qpOasesProblem.getPrimalSolution(solutionData);
    solution = Eigen::Map<Vector>(solutionData, m_NxQP, 1);
    
    return (status == qpOASES::SUCCESSFUL_RETURN);
}
