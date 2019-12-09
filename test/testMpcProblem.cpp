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
    unsigned int Nc = 1;
    unsigned int Ns = 1;
    
    float Ts = 0.1;
    
    MpcProblem mpcProblem(Nt, Np, Nx, Nu, Nc, Ns);
    
    Matrix Q, R, T;
    Vector fx, fu;
    
    Matrix A, B;
    Vector x0;
    
    Matrix Ax, Au;
    Vector b;
    
    Vector lb, ub;
    
    Q.resize(Nx,Nx);
    R.resize(Nu,Nu);
    T.resize(Nx,Nu);
    fx.resize(Nx);
    fu.resize(Nu);
    
    A.resize(Nx,Nx);
    B.resize(Nx,Nu);
    x0.resize(Nx);
    
    Ax.resize(Nc,Nx);
    Au.resize(Nc,Nu);
    b.resize(Nc);
    
    lb.resize(Nu);
    ub.resize(Nu);
    
    Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R << 1, 0, 0, 1;
    T << 1, 0, 0, 1, 0, 0;
    fx << 1, 1, 1;
    fu << 1, 1;
    
    A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
    B << 1, 0, 0, 1, 0, 0;
    x0 << 0, 1, 2;
    
    Ax << 1, 1, 1;
    Au << 1, 1;
    b << 1000;
    
    lb << -10, -10;
    ub <<  10,  10;
    
    mpcProblem.setCostFunction(
        Q.data(), R.data(), T.data(), fx.data(), fu.data()
    );
    mpcProblem.setPlantModel(A.data(), B.data(), Ts);
    mpcProblem.setInitialCondition(x0.data());
    mpcProblem.setConstraints(Ax.data(), Au.data(), b.data());
    mpcProblem.setActuatorBounds(lb.data(), ub.data());
    mpcProblem.setSoftConstraints(0, {0}, 10000);
    
    // Convert MPC problem to QP problem
    QpProblem qp = mpcProblem.toQp();
    
    // Define the expected QP Problem
    QpProblem expectedQp;
    expectedQp.H.resize(Nt*Nu+Ns, Nt*Nu+Ns);
    expectedQp.f.resize(Nt*Nu+Ns);
    expectedQp.A.resize(Np*Nc, Nt*Nu+Ns);
    expectedQp.b.resize(Np*Nc);
    expectedQp.lb.resize(Nt*Nu+Ns);
    expectedQp.ub.resize(Nt*Nu+Ns);
    
    expectedQp.H << 
        20 ,      0, 18,     0, 16,     0, 14,     0, 12,     0,     0,
         0 , 174764,  0, 87382,  0, 43692,  0, 21848,  0, 10928,     0,
         18,      0, 18,     0, 16,     0, 14,     0, 12,     0,     0,
         0 ,  87382,  0, 43692,  0, 21846,  0, 10924,  0,  5464,     0,
         16,      0, 16,     0, 16,     0, 14,     0, 12,     0,     0,
         0 ,  43692,  0, 21846,  0, 10924,  0,  5462,  0,  2732,     0,
         14,      0, 14,     0, 14,     0, 14,     0, 12,     0,     0,
         0 ,  21848,  0, 10924,  0,  5462,  0,  2732,  0,  1366,     0,
         12,      0, 12,     0, 12,     0, 12,     0, 12,     0,     0,
         0 ,  10928,  0,  5464,  0,  2732,  0,  1366,  0,   684,     0,
         0 ,      0,  0,     0,  0,     0,  0,     0,  0,     0, 10000;

    expectedQp.f <<  10, 350038, 9, 175020, 8, 87512, 7, 43760, 6, 21888, 0;
    expectedQp.A << 
         1,    1,    0,    0,    0,    0,    0,    0,    0,    0,   -1,
         1,    1,    1,    1,    0,    0,    0,    0,    0,    0,   -1,
         1,    2,    1,    1,    1,    1,    0,    0,    0,    0,   -1,
         1,    4,    1,    2,    1,    1,    1,    1,    0,    0,   -1,
         1,    8,    1,    4,    1,    2,    1,    1,    1,    1,   -1,
         1,   16,    1,    8,    1,    4,    1,    2,    1,    1,   -1,
         1,   32,    1,   16,    1,    8,    1,    4,    1,    2,   -1,
         1,   64,    1,   32,    1,   16,    1,    8,    1,    4,   -1,
         1,  128,    1,   64,    1,   32,    1,   16,    1,    8,   -1,
         1,  256,    1,  128,    1,   64,    1,   32,    1,   16,   -1;
    expectedQp.b << 997, 992, 978, 938, 822, 482, -522, -3502, -12378, -38878; 
    MatrixType inf = std::numeric_limits<MatrixType>::max();
    expectedQp.lb << -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, 0;
    expectedQp.ub << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, inf;
    
    // Compare the two QPs
    assert(qp == expectedQp);
}


/**
 * Test discretization of the system.
 */
void testDiscretization() {
    unsigned int Nx = 3;
    unsigned int Nu = 3;
    Matrix A, B;
    A.resize(Nx, Nx);
    B.resize(Nx, Nu);
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    B << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    float Ts = 0.1;
    
    unsigned int Np = 0;
    unsigned int Nt = 0;
    unsigned int Nc = 0;
    unsigned int Ns = 0;
    MpcProblem mpcProblem(Nt, Np, Nx, Nu, Nc, Ns);
    
    // Discretize and check results
    mpcProblem.setPlantModel(A.data(), B.data());
    bool status = mpcProblem.discretizePlant(Ts);
    
    Matrix AExpected;
    float TsExpected = Ts;
    mpcProblem.getPlantModel(A, B, Ts);
    
    AExpected.resize(Nx, Nx);
    AExpected << 
        1.3732,   0.5315,   0.6898,
        1.0093,   2.2481,   1.4870,
        1.6454,   1.9648,   3.2843; 
    float tol = 1e-3;
    assert((A - AExpected).cwiseAbs().minCoeff() < tol);
    assert(Ts == TsExpected);
    assert(status);
    
    // Check no discretization if the model is already discrete
    status = mpcProblem.discretizePlant(Ts);
    assert(!status);
}


int main() {
    testConversionToQp();
    testDiscretization();
    
    return 0;
}
