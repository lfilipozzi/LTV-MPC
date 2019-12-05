#ifndef MPCCONTROLLER_H_INCLUDED
#define MPCCONTROLLER_H_INCLUDED

#include"mpcProblem.h"
#include"qpSolver.h"
#include<memory>


/// Model Predictive Controller 
/**
 * @author Louis Filipozzi
 * @brief Solve and Mode Predictive Control problem using the batch approach.
 * @details Matrices and vector are passed as tables using column-major storage.
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
     * @param[in] Nc The number of polytopic constraints to enforce.
     * @param[in] Ns The number of slack variables.
     */
    MpcController(
        const std::unique_ptr<IQpSolver> qpSolver, 
        const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
        const unsigned int Nu, const unsigned int Nc, const unsigned int Ns = 0
    );
    
    /**
     * @brief Initialize the MPC controller.
     */
    void initialize();
    
    /**
     * @brief Solve the MPC problem and give the solution.
     * @return The status of the optimization.
     */
    bool update();
    
    /**
     * @brief Return the control output to apply according to the solution of 
     * MPC problem.
     * @param[out] control The control output.
     * @return The status of the optimization.
     */
    bool output(MatrixType * control[]);
    
    /**
     * @brief Terminate the MPC controller.
     */
    void terminate();
    
    /**
     * @brief Set the cost function in the problem formulation.
     * @param[in] Q The cost matrix on the states.
     * @param[in] R The cost matrix on the inputs.
     * @param[in] T The cost matrix on the states and inputs.
     * @param[in] fx The cost vector on the states.
     * @param[in] fu The cost vector on the inputs.
     */
    inline void setCostFunction(
        const MatrixType * Q, const MatrixType * R, const MatrixType * T, 
        const MatrixType * fx, const MatrixType * fu
    ) {m_mpcProblem.setCostFunction(Q, R, T, fx, fu);};
    
    /**
     * @brief Set the state initial condition.
     * @param[in] xInit The initial state.
     */
    inline void setInitialCondition(const MatrixType * xInit) {
        m_mpcProblem.setInitialCondition(xInit);
    };
    
        /**
     * @brief Set the A and B matrices defining the plant.
     * @param[in] A The A matrix of the discrete state-space.
     * @param[in] B The B matrix of the discrete state-space.
     * @param[in] Ts The sampling time of the model (-1 for continuous-time
     * system).
     */
    inline void setPlantModel(
        const MatrixType * A, const MatrixType * B, float Ts
    ) {
        m_mpcProblem.setPlantModel(A, B, Ts);
    };
    
//     /**
//      * @brief Discretize the state-space.
//      * @param[in] Ts The sampling time.
//      */
//     inline void discretizePlant(float Ts) {
//         m_mpcProblem.discretizePlant(Ts);
//     };
    
    /**
     * @brief Set the matrices and vectors defining the polytopic constraints.
     * @param[in] Ax The constraint matrix on the states.
     * @param[in] Au The constraint matrix on the inputs.
     * @param[in] As The constraint matrix on the slack variables.
     * @param[in] b  The bound vector
     */
    inline void setConstraints(
        const MatrixType * Ax, const MatrixType * Au, const MatrixType * b
    ) {m_mpcProblem.setConstraints(Ax, Au, b);};
    
    /**
     * @brief Set the vectors defining the actuator bounds.
     * @param[in] lb The lower bound input constraints.
     * @param[in] ub The upper bound input constraints.
     */
    void setActuatorBounds(const MatrixType * lb, const MatrixType * ub) {
        m_mpcProblem.setActuatorBounds(lb, ub);
    };
    
    /**
     * @brief Modify the inequality constraints to add soft constraints.
     * @param[in] slackIdx The index of the slack variable being set.
     * @param[in] constIdx The index (or indices) of the constraints which have 
     * to be soften.
     * @param[in] weight The weight used to penalize the soft constraint in the 
     * cost function.
     */
    inline void setSoftConstraints(
        unsigned int const & slackIdx, 
        std::vector<unsigned int> const & constIdx, const double & weight
    ) {
        m_mpcProblem.setSoftConstraints(slackIdx, constIdx, weight);
    };
    
    /**
     * @brief Reset the soft constraints from the MPC problem formulation. This 
     * does not reduce the size of the problem.
     */
    inline void resetSoftConsraints() {m_mpcProblem.resetSoftConsraints();};
    
private:
    /// Control horizon
    const unsigned int m_Nt;
    
    /// Prediction horizon
    const unsigned int m_Np;
    
    /// Number of state
    const unsigned int m_Nx;
    
    /// Number of input
    const unsigned int m_Nu;
    
    /// Number of polytopic constraints
    const unsigned int m_Nc;
    
    /// Number of slack variable
    const unsigned int m_Ns;
    
    /// MPC problem formulation
    MpcProblem m_mpcProblem;
    
    /// QP solver used to solve the MPC problem
    std::unique_ptr<IQpSolver> p_qpSolver;
    
    /// The status of the optimization
    bool m_status;
    
    /// Control sequence (solution of the MPC problem)
    Vector m_controlSeq;
};


#endif // MPCCONTROLLER_H_INCLUDED
