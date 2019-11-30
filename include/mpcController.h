#ifndef MPCCONTROLLER_H_INCLUDED
#define MPCCONTROLLER_H_INCLUDED

#include"mpcProblem.h"
#include"qpSolver.h"
#include<memory>


/// Model Predictive Controller 
/**
 * @author Louis Filipozzi
 * @brief Solve and Mode Predictive Control problem using the batch approach.
 */
class MpcController {
public:
    /// Constructor
    /** 
     * @brief Constructor of he MpcController.
     * @remark After using this constructor, the MPC takes ownership of 
     * the QP solver qpSolver.
     * @param[in] qpSolver The solver to use.
     * @param[in] Nt The control horizon.
     * @param[in] Np The prediction horizon.
     * @param[in] Nx The number of states of the plant.
     * @param[in] Nu The number of inputs of the plant.
     * @param[in] Nr The number of reference signals (or output of the plant).
     * @param[in] Nc The number of polytopic constraints to enforce.
     * @param[in] Ns The number of slack variables.
     */
    MpcController(
        const std::unique_ptr<IQpSolver> qpSolver, 
        const unsigned int Nt, const unsigned int Np, 
        const unsigned int Nx, const unsigned int Nu, 
        const unsigned int Nr, const unsigned int Nc, 
        const unsigned int Ns = 0
    );
    
    /**
     * @brief Initialize the MPC controller.
     */
    void initialize();
    
    /**
     * @brief Solve the MPC problem and give the solution.
     * @param[out] solution The solution of the problem.
     */
    bool update(Vector & solution);
    
private:
    /// Control horizon
    const unsigned int m_Nt;
    
    /// Prediction horizon
    const unsigned int m_Np;
    
    /// Number of state
    const unsigned int m_Nx;
    
    /// Number of input
    const unsigned int m_Nu;
    
    /// Number of reference signal
    const unsigned int m_Nr;
    
    /// Number of polytopic constraints
    const unsigned int m_Nc;
    
    /// Number of slack variable
    const unsigned int m_Ns;
    
    /// MPC problem formulation
    MpcProblem m_mpcFormulation;
    
    /// QP solver used to solve the MPC problem
    std::unique_ptr<IQpSolver> m_qpSolverPtr;
    
};


#endif // MPCCONTROLLER_H_INCLUDED
