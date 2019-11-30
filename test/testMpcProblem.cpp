#include <assert.h>
#include "../include/mpcProblem.h"


/**
 * Test the conversion from an MPC problem to a QP problem.
 */
void testConversionToQp() {
    // Define the MPC Problem
    unsigned int Np = 10;
    unsigned int Nt = 5;
    unsigned int Nx = 3;
    unsigned int Nu = 2;
    unsigned int Nr = 1;
    unsigned int Nc = 1;
    unsigned int Ns = 1;
    
    MpcProblem mpcProblem(Nt, Np, Nx, Nu, Nr, Nc, Ns);
    
    Matrix A, B, C, D;
    
    Matrix Q, R, T;
    Vector fx, fu;
    
    Matrix Ax, Au;
    Vector lbA, ubA;
    
    Vector lb, ub;
    
    A.resize(Nx,Nx);
    B.resize(Nx,Nu);
    C.resize(Nr,Nx);
    D.resize(Nr,Nu);
    
    Q.resize(Nx,Nx);
    R.resize(Nu,Nu);
    T.resize(Nx,Nu);
    fx.resize(Nx);
    fu.resize(Nu);
    
    Ax.resize(Nc,Nx);
    Au.resize(Nc,Nu);
    lbA.resize(Nc);
    ubA.resize(Nc);
    
    lb.resize(Nu);
    ub.resize(Nu);
    
    A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
    B << 1, 0, 0, 1, 1, 0;
//     C << TODO
//     D << TODO
    
    Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R << 1, 0, 0, 1;
    T << 1, 0, 0, 1, 0, 0;
//     fx << 1, 1, 1;
//     fu << 1, 1;
    
//     Ax << TODO
//     Au << TODO
//     lbA << TODO
//     ubA << TODO
    
//     lb << TODO
//     ub << TODO
    
//     mpcProblem.setPlantModel(&A, &B, &C, &D);
//     mpcProblem.setCostFunction(&Q, &R, &T, &fx, &fu);
//     mpcProblem.setConstraints(&Ax, &Au, &lbA, &ubA);
//     mpcProblem.setActuatorBounds(&lb, &ub);
    
    // Convert MPC problem to QP problem
//     QpProblem qp = mpcProblem.toQp(); TODO: uncomment once the problem is formulated
    
    // Define the expected QP Problem
    QpProblem expectedQp;
//     expectedQp.H.resize(TODO);
//     expectedQp.f.resize(TODO);
//     expectedQp.A.resize(TODO);
//     expectedQp.lbA.resize(TODO);
//     expectedQp.ubA.resize(TODO);
//     expectedQp.lb.resize(TODO);
//     expectedQp.ub.resize(TODO);
    
//     expectedQp.H << TODO;
//     expectedQp.f << TODO;
//     expectedQp.A << TODO;
//     expectedQp.lbA << TODO;
//     expectedQp.ubA << TODO;
//     expectedQp.lb << TODO;
//     expectedQp.ub << TODO;
    
    // Compare the two QPs
//     assert(qp == expectedQp) TODO: uncomment
}


int main() {
    testConversionToQp();
    
    return 0;
}
