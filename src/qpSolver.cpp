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
    m_NxQP(NxQP),
    m_NcQP(NcQP),
    m_nWSR(nWSR), 
    p_cpuTime(cpuTime) {
    // Set lower bound inequalities to -infinity
    m_lbA.resize(m_NcQP);
    m_lbA.segment(0, m_NcQP) = 
        Matrix::Constant(m_NcQP, 1, -std::numeric_limits<MatrixType>::max());
    // Create the QP problem
    m_qpOasesProblem = qpOASES::SQProblem(m_NxQP, m_NcQP);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    m_qpOasesProblem.setOptions(options);
}


QpOasesSolver::~QpOasesSolver() {}


bool QpOasesSolver::solve(
    const QpProblem & qpProblem, Vector & solution
) {
    // Check QP size
    unsigned int NxQP;
    unsigned int NcQP;
    bool sizeCheck = qpProblem.getSize(NxQP, NcQP);
    if(!(sizeCheck && NxQP == m_NxQP && NcQP == m_NcQP))
        return false;
    
    solution.resize(m_NxQP);
    
//     const Matrix & H = qpProblem.H;
    const Vector & f  = qpProblem.f;
//     const Matrix & A = qpProblem.A;
    const Vector & b  = qpProblem.b;
    const Vector & lb = qpProblem.lb;
    const Vector & ub = qpProblem.ub;
    
    // Convert matrices to row-major matrices
    // TODO: H is square and symmetric so the row-major and column major are the same. Try without this conversion once everything is working.
    const MatrixRowMajor H = Eigen::Map<const MatrixRowMajor> (
        qpProblem.H.data(), NxQP, NxQP
    );
    const MatrixRowMajor A = Eigen::Map<const MatrixRowMajor> (
        qpProblem.A.data(), NcQP, NxQP
    );
    
    // Set maximum of working set recalculation
    qpOASES::int_t nWSR = m_nWSR;
    
//     if(m_coldStart) {
//         // Initialize the problem
//         m_qpOasesProblem = qpOASES::SQProblem(m_NxQP, m_NcQP);
//         
//         // Solve the problem
//         m_qpOasesProblem.init(
//             H.data(),
//             f.data(),
//             A.data(),
//             lb.data(),
//             ub.data(),
//             m_lbA.data(),
//             b.data(), 
//             nWSR,
//             p_cpuTime
//         );
//         
//         if (m_qpOasesProblem.isInitialised() && m_qpOasesProblem.isSolved())
//             m_coldStart = !m_coldStart;
//     }
//     else {
//         // Solve the problem
//         m_qpOasesProblem.hotstart(
//             H.data(),
//             f.data(),
//             A.data(),
//             lb.data(),
//             ub.data(),
//             m_lbA.data(),
//             b.data(), 
//             nWSR,
//             p_cpuTime
//         );
//         
//         if (!m_qpOasesProblem.isSolved())
//             m_coldStart = !m_coldStart;
//     }
    
    qpOASES::returnValue status;
    if(m_qpOasesProblem.getCount() == 0) {
        // Cold start: initialize and solve the first problem
        status = m_qpOasesProblem.init(
            H.data(),
            f.data(),
            A.data(),
            lb.data(),
            ub.data(),
            m_lbA.data(),
            b.data(), 
            nWSR,
            p_cpuTime
        );
    }
    else {
        // Hot start: solve the problem
        status = m_qpOasesProblem.hotstart(
            H.data(),
            f.data(),
            A.data(),
            lb.data(),
            ub.data(),
            m_lbA.data(),
            b.data(), 
            nWSR,
            p_cpuTime
        );
        
        if ((status != qpOASES::SUCCESSFUL_RETURN) && 
            (status != qpOASES::RET_MAX_NWSR_REACHED)) {
            // If an error occurs, reset problem data structures.
            m_qpOasesProblem.reset( );
            
            // Initialize again the problem.
            qpOASES::int_t nWSR = m_nWSR;
            status = m_qpOasesProblem.init(
                H.data(),
                f.data(),
                A.data(),
                lb.data(),
                ub.data(),
                m_lbA.data(),
                b.data(), 
                nWSR,
                p_cpuTime
            );
        }
    }
    
    // Get solution
    qpOASES::real_t solutionData[m_NxQP];
    m_qpOasesProblem.getPrimalSolution(solutionData);
    solution = Eigen::Map<Vector>(solutionData, m_NxQP, 1);
    
    // Check feasibility
    if (m_qpOasesProblem.isInfeasible())
        return false;
    
    return (status == qpOASES::SUCCESSFUL_RETURN);
}
