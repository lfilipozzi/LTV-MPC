#include "../include/mpcProblem.h"
#include <limits>
#include <iostream>

// Forbid Eigen automatic resizing with operator=
#define EIGEN_NO_AUTOMATIC_RESIZING

// Some conventions:
// * When performing manipulation on the matrices, index variables used are i, 
//   j, and k. Variable i is used for operations on rows (and for vectors), 
//   variable j is used for operations on column (and for row vectors), and 
//   variable k is used for combined operation on row and column.


MpcProblem::MpcProblem(
    const unsigned int Nt, const unsigned int Np, const unsigned int Nx, 
    const unsigned int Nu, const unsigned int Nc, const unsigned int Ns
) :
    m_Nt(Nt), m_Np(Np), m_Nx(Nx), m_Nu(Nu), m_Nc(Nc), m_Ns(Ns) {
    // Resize the matrices used to formulate the MPC 
    m_costFunction.Q.resize(m_Nx, m_Nx);
    m_costFunction.R.resize(m_Nu, m_Nu);
    m_costFunction.T.resize(m_Nx, m_Nu);
    m_costFunction.S.resize(m_Ns, m_Ns);
    m_costFunction.fx.resize(m_Nx);
    m_costFunction.fu.resize(m_Nu);
    m_stateInit.resize(m_Nx);
    m_plant.A.resize(m_Nx, m_Nx);
    m_plant.B.resize(m_Nx, m_Nu);
    m_constraints.Ax.resize(m_Nc, m_Nx);
    m_constraints.Au.resize(m_Nc, m_Nu);
    m_constraints.As.resize(m_Nc, m_Ns);
    m_constraints.b.resize(m_Nc);
    // Resize the batch QP problem matrices
//     m_qp.H.resize(m_Nu*m_Nt+m_Ns, m_Nu*m_Nt+m_Ns);
//     m_qp.f.resize(m_Nu*m_Nt+m_Ns);
//     m_qp.A.resize(m_Np*m_Nc, m_Nu*m_Nt+m_Ns);
//     m_qp.b.resize(m_Np*m_Nc);
//     m_qp.lb.resize(m_Nu*m_Nt+m_Ns);
//     m_qp.ub.resize(m_Nu*m_Nt+m_Ns);
    m_qp.resize(m_Nu*m_Nt+m_Ns, m_Np*m_Nc);
    // Initialize matrices whose value is set by entry
    m_constraints.As.Zero(m_Nc, m_Ns);
}


MpcProblem::~MpcProblem() {
    
}


void MpcProblem::setPlantModel(
    const MatrixType * A, const MatrixType * B, float Ts
) {
    m_plant.A = Eigen::Map<const Matrix>(A, m_Nx, m_Nx);
    m_plant.B = Eigen::Map<const Matrix>(B, m_Nx, m_Nu);
    m_plant.Ts = Ts;
}


// void MpcProblem::discretizePlant(float Ts) {
//     if (m_plant.Ts == -1.0) {
//         Matrix Atmp;
//         Atmp = m_plant.A*Ts;
//         m_plant.A = (m_plant.A*Ts).exp(m_Nx, m_Nx).eval();
//         m_plant.B *= m_plant.A;
//         m_plant.Ts = Ts;
//     }
//     // TODO else
// }



void MpcProblem::setCostFunction(
    const MatrixType * Q, const MatrixType * R, const MatrixType * T, 
    const MatrixType * fx, const MatrixType * fu
) {
    m_costFunction.Q = Eigen::Map<const Matrix>(Q, m_Nx, m_Nx);
    m_costFunction.R = Eigen::Map<const Matrix>(R, m_Nu, m_Nu);
    m_costFunction.T = Eigen::Map<const Matrix>(T, m_Nx, m_Nu);
    m_costFunction.fx = Eigen::Map<const Vector>(fx, m_Nx);
    m_costFunction.fu = Eigen::Map<const Vector>(fu, m_Nu);
}


void MpcProblem::setConstraints(
    const MatrixType * Ax, const MatrixType * Au, const MatrixType * b
) {
    // Update matrices
    m_constraints.Ax = Eigen::Map<const Matrix>(Ax, m_Nc, m_Nx);
    m_constraints.Au = Eigen::Map<const Matrix>(Au, m_Nc, m_Nu);
    m_constraints.b  = Eigen::Map<const Vector>(b, m_Nc);
}


void MpcProblem::setActuatorBounds(
    const MatrixType * lb, const MatrixType * ub
) {
    m_bounds.lb = Eigen::Map<const Vector>(lb, m_Nu);
    m_bounds.ub = Eigen::Map<const Vector>(ub, m_Nu);
}


void MpcProblem::setSoftConstraints(
    unsigned int const & slackIdx, std::vector<unsigned int> const & constIdx, 
    const double & weight
) {
    if (!(slackIdx >= 0 && slackIdx < m_Ns && weight > 0 && !constIdx.empty()))
        return;
    
    // Set the penalty associated to the slack variable
    m_costFunction.S(slackIdx,slackIdx) = weight;
    // Reset the column of the matrix As to zero and update the column
    m_constraints.As.block(0, slackIdx, m_Nc, 1).Zero(m_Nc, 1);
    for (auto it = constIdx.begin(); it != constIdx.end(); it++) {
        if (*it >= 0 && *it < m_Nc)
            m_constraints.As(*it, slackIdx) = -1.0;
    }
}


void MpcProblem::resetSoftConsraints() {
    // Set the penalty on the slack variables to zero
    for (unsigned int k = 0; k < m_Ns; k++)
        m_costFunction.S(k,k) = 0.0;
    // Set the inequality matrix on slack variable to zero
    m_constraints.As.Zero(m_Nc, m_Ns);
}



QpProblem MpcProblem::toQp() {
    /*
     * Compute batch matrices of the cost function
     */
    Matrix Sx, Su, Qbar, Rbar, Tbar;
    Vector fubar, fxbar;
    
    Sx.resize(m_Nx*m_Np, m_Nx);
    Su.resize(m_Nx*m_Np, m_Nu*m_Nt);
    Qbar.resize(m_Nx*m_Np, m_Nx*m_Np);
    Rbar.resize(m_Nu*m_Nt, m_Nu*m_Nt);
    Tbar.resize(m_Nx*m_Np, m_Nu*m_Nt);
    fxbar.resize(m_Nx*m_Np);
    fubar.resize(m_Nu*m_Nt);
    
    Sx.block(0, 0, m_Nx, m_Nx).setIdentity();
    for (unsigned int i = 1; i < m_Np; i++) {
        Sx.block(m_Nx*i, 0, m_Nx, m_Nx) = 
        m_plant.A * Sx.block(m_Nx*(i-1), 0, m_Nx, m_Nx);
    }
    
    Su.setZero();
    if (m_Np > 1)
        Su.block(m_Nx, 0, m_Nx, m_Nu) = m_plant.B;
    // Fill the first column
    for(unsigned int i = 2; i < m_Np; i++) {
        Su.block(m_Nx*i, 0, m_Nx, m_Nu) = 
            m_plant.A * Su.block(m_Nx*(i-1), 0, m_Nx, m_Nu);
    }
    // Copy first column to other column progressively
    for(unsigned int j = 1; j < m_Nt; j++) {
        Su.block(m_Nx*(j+1), m_Nu*j, m_Nx*(m_Np-j-1), m_Nu) = 
            Su.block(m_Nx, 0, m_Nx*(m_Np-j-1), m_Nu);
    }
    
    // Write cost function for batch approach
    Qbar.setZero();
    for(unsigned int k = 0; k < m_Np; k++) {
        Qbar.block(m_Nx*k, m_Nx*k, m_Nx, m_Nx) = m_costFunction.Q;
    }
    
    Rbar.setZero();
    for(unsigned int k = 0; k < m_Nt; k++) {
        Rbar.block(m_Nu*k, m_Nu*k, m_Nu, m_Nu) = m_costFunction.R;
    }
    
    Tbar.setZero();
    for(unsigned int k = 0; k < m_Nt; k++) {
        Tbar.block(m_Nx*k, m_Nu*k, m_Nx, m_Nu) = m_costFunction.T;
    }
    
    for(unsigned int i = 0; i < m_Np; i++) {
        fxbar.segment(m_Nx*i, m_Nx) = m_costFunction.fx;
    }
    
    for(unsigned int i = 0; i < m_Nt; i++) {
        fubar.segment(m_Nu*i, m_Nu) = m_costFunction.fu;
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
    m_qp.f.segment(0, m_Nu*m_Nt) = Su.transpose() * fxbar + fubar + 
        2 * (Qbar * Su + Tbar).transpose() * Sx * m_stateInit;
    
    
    /*
     * Compute batch vectors for decision variable constraints
     */
    for(unsigned int i = 0; i < m_Nt; i++) {
        m_qp.lb.segment(m_Nu*i, m_Nu) = m_bounds.lb;
        m_qp.ub.segment(m_Nu*i, m_Nu) = m_bounds.ub;
    }
    
    
    /*
     * Compute batch matrices for polytopic constraints
     */
    Matrix Axbar, Aubar;
    Axbar.resize(m_Nc*m_Np, m_Nx*m_Np);
    Aubar.resize(m_Nc*m_Np, m_Nu*m_Nt);
    Axbar.setZero();
    Aubar.setZero();
    for(unsigned int k = 0; k < m_Np; k++) {
        Axbar.block(m_Nc*k, m_Nx*k, m_Nc, m_Nx) = m_constraints.Ax;
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        Aubar.block(m_Nc*k, m_Nu*k, m_Nc, m_Nu) = m_constraints.Au;
    }
    m_qp.A.block(0, 0, m_Np*m_Nc, m_Nu*m_Nt) = Axbar * Su + Aubar;
    for(unsigned int i = 0; i < m_Np; i++) {
        m_qp.b.segment(m_Nc*i, m_Nc) = m_constraints.b;
    }
    m_qp.b -= Axbar * Sx * m_stateInit;
    
    
    /*
     * Modify the problem to add soft constraints
     */    
    m_qp.H.block(m_Nu*m_Nt, m_Nu*m_Nt, m_Ns, m_Ns) = m_costFunction.S;
    m_qp.f.segment(m_Nu*m_Nt, m_Ns) = Matrix::Zero(m_Ns, 1);
    for(unsigned int i = 0; i < m_Np; i++) {
        m_qp.A.block(m_Nc*i, m_Nu*m_Nt, m_Nc, m_Ns) =  m_constraints.As;
    }
    m_qp.lb.segment(m_Nu*m_Nt, m_Ns) = 
        Matrix::Constant(m_Ns, 1, 0);
    m_qp.ub.segment(m_Nu*m_Nt, m_Ns) = 
        Matrix::Constant(m_Ns, 1, std::numeric_limits<MatrixType>::max());
    
    return m_qp;
}






