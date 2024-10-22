#ifndef MPCPROBLEM_H_INCLUDED
#define MPCPROBLEM_H_INCLUDED

#include <Eigen>
#include "qpSolver.h"


/// Model Predictive Control Problem
/**
 * @author Louis Filipozzi
 * 
 * @brief This class defines a Model Predictive Control. Its mathematical 
 * formulation is given as
 * \f{eqnarray*}{
 *  \min\limits_{(u_k)_{k\in [0,N-1]}} & 
 *      \sum\limits_{k=0}^{N_p-1} x_k^T Q x_k + 
 *      \sum\limits_{k=0}^{N_t-1} u_k^T T x_k + x_k^T T^T u_k + u_k^T R u_k\\
 *  s.t.
 *      & x_0 = x_{init}\\
 *      & x_{k+1} = A x_k +  B u_k\\
 *      & u_{min} \leq u_k \leq u_{max}\\
 *      & A_x x_k + A_u \delta u_k \leq b
 * \f}
 * where \f$ N_p, N_t \f$ are respectively the prediction and control horizons.
 */
class MpcProblem {
public: 
    MpcProblem(
        const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
        const unsigned int Nu, const unsigned int Nc, const unsigned int Ns
    );
    ~MpcProblem();
    
    /**
     * @brief Set the Q, R, T, fx, and fu matrices defining the cost function.
     * @param[in] Q The cost matrix on the states.
     * @param[in] R The cost matrix on the inputs.
     * @param[in] T The cost matrix on the states and inputs.
     * @param[in] fx The cost vector on the states.
     * @param[in] fu The cost vector on the inputs.
     */
    void setCostFunction(
        const MatrixType * Q, const MatrixType * R, const MatrixType * T, 
        const MatrixType * fx, const MatrixType * fu
    );
    
    /**
     * @brief Set the state initial condition.
     * @param[in] xInit The initial state.
     */
    void setInitialCondition(const MatrixType * xInit) {
        m_stateInit = Eigen::Map<const Vector>(xInit, m_Nx);
    };
    
        /**
     * @brief Set the A and B matrices defining the plant.
     * @param[in] A The A matrix of the state-space.
     * @param[in] B The B matrix of the state-space.
     * @param[in] Ts The sampling time of the model (-1 for continuous-time
     * system).
     */
    void setPlantModel(
        const MatrixType * A, const MatrixType * B, float Ts = -1.0
    );
    
    /**
     * @brief Discretize the state-space. Return true if successful.
     * @param[in] Ts The sampling time.
     */
    bool discretizePlant(float Ts);
    
    /**
     * @brief Set the matrices and vectors defining the polytopic constraints.
     * @param[in] Ax The constraint matrix on the states.
     * @param[in] Au The constraint matrix on the inputs.
     * @param[in] As The constraint matrix on the slack variables.
     * @param[in] b  The bound vector
     */
    void setConstraints(
        const MatrixType * Ax, const MatrixType * Au, const MatrixType * b
    );
    
    /**
     * @brief Set the vectors defining the actuator bounds.
     * @param[in] lb The lower bound input constraints.
     * @param[in] ub The upper bound input constraints.
     */
    void setActuatorBounds(const MatrixType * lb, const MatrixType * ub);
    
    /**
     * @brief Modify the inequality constraints to add soft constraints.
     * @param[in] slackIdx The index of the slack variable being set.
     * @param[in] constIdx The index (or indices) of the constraints which have 
     * to be soften.
     * @param[in] weight The weight used to penalize the soft constraint in the 
     * cost function.
     */
    void setSoftConstraints(
        unsigned int const & slackIdx, 
        std::vector<unsigned int> const & constIdx, const double & weight
    );
    
    /**
     * @brief Reset the soft constraints from the MPC problem formulation. This 
     * does not reduce the size of the problem.
     */
    void resetSoftConsraints();
    
    /**
     * @brief Set a scale factor for the input.
     * @param[in] inputIdx The index of the input.
     * @param[in] actor The scaling factor.
     */
    void setInputScaleFactor(
        unsigned int const & inputIdx, double const & factor
    ) {
        if (!(inputIdx >= 0 && inputIdx < m_Nu && factor > 0))
            return;
        
        m_scaling.input(inputIdx) = 1/factor;
    };
    
    /**
     * @brief Scale the MPC problem by modifying the matrices defining the MPC 
     * problem.
     */
    void scale();
    
    /**
     * @brief Unscale the solution of the problem.
     * @param[in,out] solution The solution.
     */
    void unscaleSol(Vector & solution);
    
    /**
     * @brief Transform the MPC Problem into a standard QP problem by using the 
     *  batch approach.
     */
    QpProblem toQp();
    
    /**
     * @brief Get the plant model.
     * @param[out] A The A state-space matrix.
     * @param[out] B The B state-space matrix.
     * @param[out] Ts The sampling time.
     */
    void getPlantModel(Matrix & A, Matrix & B, float & Ts) const {
        A.resize(m_plant.A.rows(), m_plant.A.cols());
        B.resize(m_plant.B.rows(), m_plant.B.cols());
        A  = m_plant.A;
        B  = m_plant.B;
        Ts = m_plant.Ts;
    };
    
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
    
    /// The cost function
    struct CostFunction {
        Matrix Q;       // Quadratic cost on the states
        Matrix R;       // Quadratic cost on the inputs
        Matrix T;       // Quadratic cost on the states-inputs
        Matrix S;       // Quadratic cost on the slack variables
        Vector fx;      // Linear cost on the states
        Vector fu;      // Linear cost on the inputs
        bool isScaled = false;
    } m_costFunction;
    
    /// Initial condition
    Vector m_stateInit;
    
    /// Representation of the plant using a state-space
    struct Plant {
        Matrix A;
        Matrix B;
        float Ts;
        bool isScaled = false;
    } m_plant;
    
    /// Polytopic constraints of the problem (aka constraints)
    struct Constraints {
        Matrix Ax;
        Matrix Au;
        Matrix As;
        Vector b;
        bool isScaled = false;
    } m_constraints;
    
    /// Plant input constraints (aka bounds)
    struct Bounds {
        Vector lb;
        Vector ub;
        bool isScaled = false;
    } m_bounds;
    
    /// Scaling factors
    struct ScalingFactors {
        Vector input;
        Vector slack;
    } m_scaling;
    
    /// QP formulation of the MPC problem
    QpProblem m_qp;
};







#endif // MPCPROBLEM_H_INCLUDED
