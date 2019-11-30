#ifndef MPCPROBLEM_H_INCLUDED
#define MPCPROBLEM_H_INCLUDED

#include <Eigen>
#include "qpSolver.h"


/// Model Predictive Control Problem
/**
 * @author Louis Filipozzi
 * 
 * @brief This class defines a Model Predictive Control problem used for 
 * reference tracking. Its mathematical formulation is given as
 * \f{eqnarray*}{
 *  \min\limits_{(\delta u_k)_{k\in [0,N-1]}} & 
 *      \sum\limits_{k=0}^{N_p-1} ||\delta y_k - \delta y_{ref}||_{Q_e}^2 + 
 *      \sum\limits_{k=0}^{N_t-1} ||\delta u_k||_{R_{\delta u}}^2\\
 *  s.t.
 *      & \delta x_0 = 0\\
 *      & \delta x_{k+1} = A \delta x_k +  B \delta u_k\\
 *      & \delta y_k = C \delta x_k + D \delta u_k\\
 *      & u_{min} \leq u_{-1} + \delta u_k \leq u_{max}\\
 *      & A_x (x_0 + \delta x_k) + A_u (u_{-1} + \delta u_k) \leq b
 * \f}
 * where \f$ N_p, N_t \f$ are respectively the prediction and control horizons.
 */
class MpcProblem {
public: 
    MpcProblem(
        const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
        const unsigned int Nu, const unsigned int Nr, const unsigned int Nc, 
        const unsigned int Ns
    );
    ~MpcProblem();
    
    /**
     * @brief Set the A, B, C, and D matrices defining the plant.
     * @param[in] A The A matrix of the discrete state-space.
     * @param[in] B The B matrix of the discrete state-space.
     * @param[in] C The C matrix of the discrete state-space.
     * @param[in] D The D matrix of the discrete state-space.
     */
    void setPlantModel(
        Matrix *const A, Matrix *const B, Matrix *const C, Matrix *const D
    );
    
    /**
     * @brief Set the Q, R, T, fx, and fu matrices defining the cost function.
     * @param[in] Q The cost matrix on the states.
     * @param[in] R The cost matrix on the inputs.
     * @param[in] T The cost matrix on the states and inputs.
     * @param[in] fx The cost vector on the states.
     * @param[in] fu The cost vector on the inputs.
     */
    void setCostFunction(
        Matrix *const Q, Matrix *const R, Matrix *const T, Vector *const fx, 
        Vector *const fu
    );
    
    /**
     * @brief Set the matrices and vectors defining the polytopic constraints.
     * @param[in] Ax The constraint matrix on the states.
     * @param[in] Au The constraint matrix on the inputs.
     * @param[in] As The constraint matrix on the slack variables.
     * @param[in] b  The bound vector
     */
    void setConstraints(
        Matrix *const Ax, Matrix *const Au, Matrix *const As, Vector *const b
    );
    
    /**
     * @brief Set the vectors defining the actuator bounds.
     * @param[in] lb The lower bound input constraints.
     * @param[in] ub The upper bound input constraints.
     */
    void setActuatorBounds(Vector *const lb, Vector *const ub);
    
    /**
     * @brief Transform the MPC Problem into a standard QP problem by using the 
     *  batch approach.
     */
    QpProblem toQp();
    
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
    
    /// Representation of the plant using a state-space
    struct Plant {
        Matrix * A;
        Matrix * B;
        Matrix * C;
        Matrix * D;
    } m_plant;
    
    /// The cost function
    struct CostFunction {
        Matrix * Q;
        Matrix * R;
        Matrix * T;
        Vector * fx;
        Vector * fu;
    } m_costFunction;
    
    /// Polytopic constraints of the problem (aka constraints)
    struct Constraints {
        Matrix * Ax;
        Matrix * Au;
        Matrix * As;
        Vector * b;
    } m_constraints;
    
    /// Plant input constraints (aka bounds)
    struct Bounds {
        Vector * lb;
        Vector * ub;
    } m_bounds;
    
    /// QP formulation of the MPC problem
    QpProblem m_qp;
};







#endif // MPCPROBLEM_H_INCLUDED
