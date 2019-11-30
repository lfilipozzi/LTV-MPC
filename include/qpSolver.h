#ifndef QPSOLVER_H_INCLUDED
#define QPSOLVER_H_INCLUDED

#include <Eigen>
#include <qpOASES.hpp>



typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;



/**
 * @brief Structure containing a standard Quadratic Programming (QP) problem.
 */
struct QpProblem {
    Matrix H;
    Vector f;
    Matrix A;
    Vector b;
    Vector lb;
    Vector ub;
    
    /**
     * @overload
     * @details Two QP problems are considered equals if they have the same 
     * solution. Thus, they are equal if their cost function and/or their 
     * constraints are the same to within a multiplicative constant.
     */
    bool operator==(const QpProblem & qp) const {
        // TODO change overload of operator== so that the cost function and 
        // polytopic constraints can differ by a multiplicative scalar so that 
        // two problems are equals as long as they have the same solutions
        return (H == qp.H && f == qp.f && 
                A == qp.A && b == qp.b &&
                lb == qp.lb && ub == qp.ub);
    }
    
    /**
     * @brief Resize the QP problem.
     * @param[in] NxQP Number of decision variables.
     * @param[out] NcQP Number of constraints.
     */
    void resize(unsigned int NxQP, unsigned int NcQP) {
        H.resize(NxQP,NxQP);
        f.resize(NxQP);
        A.resize(NcQP,NxQP);
        b.resize(NcQP);
        lb.resize(NxQP);
        ub.resize(NxQP);
    }
    
    /**
     * @brief Return the size of the QP problem. Return false if the matrices 
     * do not have compatible sizes.
     * @param[out] NxQP Number of decision variables.
     * @param[out] NcQP Number of constraints.
     */
    bool getSize(unsigned int & NxQP, unsigned int & NcQP) const {
        unsigned int Nx = H.rows();
        unsigned int Nc = A.rows();
        // Check the size is valid
        if(H.rows() != Nx && H.cols() != Nx)
            return false;
        if(f.rows() != Nx && f.cols() != 1)
            return false;
        if(A.rows() != Nc && A.cols() != Nx)
            return false;
        if(b.rows() != Nc && b.cols() != 1)
            return false;
        if(lb.rows() != Nx && lb.cols() != 1)
            return false;
        if(ub.rows() != Nx && lb.cols() != 1)
            return false;
        // Return sizes
        NxQP = H.rows();
        NcQP = A.rows();
        return true;
    }
};



/// Interface for a Quadratic Programming solver
/**
 * @author Louis Filipozzi
 * @brief Define an interface for a Quadratic Programming solver.
 * @details A quadratic programming solver must be able to solve the following
 * problem:
 * \f{eqnarray*}{
 *  \min && \frac{1}{2} x^T H x + f^T x\\
 *  s.t. && A x \leq b
 * \f}
 */
class IQpSolver {
public:
    /**
     * @brief Solve the QP problem qpProblem and give the solution. If the 
     * problem is solved, return true.
     * @param[in] qpProblem The QP problem.
     * @param[out] solution Vector used to return the solution if solved.
     */
    virtual bool solveProbem(
        const QpProblem & qpProblem, Vector & solution
    ) = 0;
    
};



/**
 * @brief Implementation of the QP solver with qpOASES.
 * @author Louis Filipozzi
 * @remark qpOASES requires that the size of the problem does not change.
 */
class QpOasesSolver : public IQpSolver {
public:
    /**
     * @brief Constructor.
     * @param[in] NxQP Number of decision variables of the QP problem.
     * @param[in] NcQP Number of constraints of the QP problem.
     * @param[in] nWSR Maximum number of working set recalculations.
     * @param[in] cpuTime Maximum allowed CPU time in second.
     */
    QpOasesSolver(
        unsigned int NxQP, unsigned int NcQP, int nWSR = 10, 
        double* cpuTime = nullptr
    );
    ~QpOasesSolver();
    
    virtual bool solveProbem(
        const QpProblem & qpProblem, Vector & solution
    );
    
private:
    /// Set cold-start or hot-start
    bool m_coldStart;
    
    /// The formulation of the QP problem in qpOASES
    qpOASES::SQProblem m_qpOasesProblem;
    
    /** 
     * Number of decision variables of the QP problem (qpOASES assumes a 
     * constant size problem).
     */
    const unsigned int m_NxQP;
    
    /** 
     * Number of constraints of the QP problem (qpOASES assumes a constant size
     * problem).
     */
    const unsigned int m_NcQP;
    
    /// Maximum number of working set recalculation.
    qpOASES::int_t m_nWSR;
    
    /// Maximum CPU time to solve the problem (in second).
    qpOASES::real_t * m_cpuTimePtr;
    
    /**
     * Lower bound constraints. qpOASES allows to use lower bound constraints
     * but this does not fit the implementation of the interface.
     */
    Vector m_lbA;
};


#endif // ABSTRACTSOLVER_H_INCLUDED
