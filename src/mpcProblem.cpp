#include "../include/mpcProblem.h"
#include <limits>

// Forbid Eigen automatic resizing with operator=
#define EIGEN_NO_AUTOMATIC_RESIZING

// Some conventions:
// * When performing manipulation on the matrices, index variables used are i, 
//   j, and k. Variable i is used for operations on rows (and for vectors), 
//   variable j is used for operations on column (and for row vectors), and 
//   variable k is used for combined operation on row and column.


MpcProblem::MpcProblem(
    const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
    const unsigned int Nu, const unsigned int Nr, const unsigned int Nc, 
    const unsigned int Ns
) :
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nr(Nr), m_Nc(Nc), m_Ns(Ns) {
    // Resize the batch QP problem matrices
    m_qp.H.resize(m_Nu*m_Nt+m_Ns, m_Nu*m_Nt+m_Ns);
    m_qp.f.resize(m_Nu*m_Nt+m_Ns);
    m_qp.A.resize(m_Np*m_Nc, m_Nu*m_Nt+m_Ns);
    m_qp.b.resize(m_Np*m_Nc);
    m_qp.lb.resize(m_Nu*m_Nt+m_Ns);
    m_qp.ub.resize(m_Nu*m_Nt+m_Ns);
}


MpcProblem::~MpcProblem() {
    
}


void MpcProblem::setPlantModel(
    Matrix * const A, Matrix * const B, Matrix * const C, Matrix * const D
) {
    m_plant.A = A;
    m_plant.B = B;
    m_plant.C = C;
    m_plant.D = D;
}


void MpcProblem::setCostFunction(
    Matrix * const Q, Matrix * const R, Matrix * const T, Vector * const fx, 
    Vector * const fu
) {
    m_costFunction.Q = Q;
    m_costFunction.R = R;
    m_costFunction.T = T;
    m_costFunction.fx = fx;
    m_costFunction.fu = fu;
}


void MpcProblem::setConstraints(
    Matrix * const Ax, Matrix * const Au, Matrix * const As, Vector * const b
) {
    m_constraints.Ax = Ax;
    m_constraints.Au = Au;
    m_constraints.As = As;
    m_constraints.b  = b;
}


void MpcProblem::setActuatorBounds(Vector * const lb, Vector * const ub) {
    m_bounds.lb = lb;
    m_bounds.ub = ub;
}


QpProblem MpcProblem::toQp() {
    /*
     * Compute batch matrices of the cost function
     */
    Matrix Su, Qbar, Rbar, Tbar;
    Vector fubar, fxbar;
    
    Su.resize(m_Nx*m_Np, m_Nu*m_Nt);
    Qbar.resize(m_Nx*m_Np, m_Nx*m_Np);
    Rbar.resize(m_Nu*m_Nt, m_Nu*m_Nt);
    Tbar.resize(m_Nx*m_Np, m_Nu*m_Nt);
    fxbar.resize(m_Nx*m_Np);
    fubar.resize(m_Nu*m_Nt);
    
    Su.block(m_Nx, 0, m_Nx, m_Nu) = *(m_plant.B);
    // Fill the first column
    for(unsigned int i = 2; i < m_Np; i++) {
        Su.block(m_Nx*i, 0, m_Nx, m_Nu) = 
            *(m_plant.A) * Su.block(m_Nx*(i-1), 0, m_Nx, m_Nu);
    }
    // Copy first column to other column progressively
    for(unsigned int j = 1; j < m_Nt; j++) {
        Su.block(m_Nx*(j+1), m_Nu*j, m_Nx*(m_Np-j-1), m_Nu) = 
            Su.block(m_Nx, 0, m_Nx*(m_Np-j-1), m_Nu);
    }
    
    // Write cost function for batch approach
    for(unsigned int k = 0; k < m_Np; k++) {
        Qbar.block(m_Nx*k, m_Nx*k, m_Nx, m_Nx) = *(m_costFunction.Q);
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        Rbar.block(m_Nu*k, m_Nu*k, m_Nu, m_Nu) = *(m_costFunction.R);
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        Tbar.block(m_Nx*k, m_Nu*k, m_Nx, m_Nu) = *(m_costFunction.T);
    }
    
    for(unsigned int i = 0; i < m_Np; i++) {
        fxbar.segment(m_Nx*i, m_Nx) = *(m_costFunction.fx);
    }
    
    for(unsigned int i = 0; i < m_Nt; i++) {
        fubar.segment(m_Nu*i, m_Nu) = *(m_costFunction.fu);
    }
    
    Matrix tmp;
    tmp.resize(m_Nt*m_Nu, m_Nt*m_Nu);
    tmp = Tbar.transpose() * Su;
    m_qp.H.block(0, 0, m_Nu*m_Nt, m_Nu*m_Nt) = 2 * (
            Su.transpose() * Qbar * Su + 
            tmp + 
            tmp.transpose() + 
            Rbar
            );
    m_qp.f.segment(0, m_Nu*m_Nt) = Su.transpose() * fxbar + fubar;
    
    
    /*
     * Compute batch vectors for decision variable constraints
     */
    for(unsigned int i = 0; i < m_Nt; i++) {
        m_qp.lb.segment(m_Nu*i, m_Nu) = *(m_bounds.lb);
        m_qp.ub.segment(m_Nu*i, m_Nu) = *(m_bounds.ub);
    }
    
    
    /*
     * Compute batch matrices for polytopic constraints
     */
    Matrix Axbar, Aubar;
    Axbar.resize(m_Nc*m_Np, m_Nx*m_Np);
    Aubar.resize(m_Nc*m_Np, m_Nu*m_Nt);
    for(unsigned int k = 0; k < m_Np; k++) {
        Axbar.block(m_Nc*k, m_Nx*k, m_Nc, m_Nx) = *(m_constraints.Ax);
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        Aubar.block(m_Nc*k, m_Nu*k, m_Nc, m_Nu) = *(m_constraints.Au);
    }
    
    m_qp.A.block(0, 0, m_Np*m_Nc, m_Nu*m_Nt) = Axbar * Su + Aubar;
    for(unsigned int i = 0; i < m_Np; i++) {
        m_qp.b.segment(m_Nc*i, m_Nc) = *(m_constraints.b);
    }
    
    
    /*
     * Modify the problem to add soft constraints
     */
    Matrix As;
    As.resize(m_Nc, m_Ns);
    // TODO As = ...
    
    m_qp.H.block(m_Nu*m_Nt, m_Nu*m_Nt, m_Ns, m_Ns) = 
        Matrix::Identity(m_Ns, m_Ns); // TODO Add slack variable penalty
    m_qp.f.segment(m_Nu*m_Nt, m_Ns) = Matrix::Zero(m_Ns, 1);
    for(unsigned int i = 0; i < m_Np; i++) {
        m_qp.A.block(m_Nc*i, m_Nu*m_Nt, m_Nc, m_Ns) =  As;
    }
    m_qp.lb.segment(m_Nc*m_Np, m_Ns) = 
        Matrix::Constant(m_Ns, 1, std::numeric_limits<float>::min());
    m_qp.ub.segment(m_Nc*m_Np, m_Ns) = 
        Matrix::Constant(m_Ns, 1, std::numeric_limits<float>::max());
    
    return m_qp;
}






