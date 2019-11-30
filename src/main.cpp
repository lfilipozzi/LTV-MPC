#include <qpOASES.hpp>
#include <ctime>
#include <iostream>
#include "../include/mpcProblem.h"

/** Example for qpOASES main function using the QProblem class. */
int main() {
    // Test block construction
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
    
    Matrix m_Su;
    Matrix m_Qbar;
    Matrix m_Rbar;
    Matrix m_Tbar;
    Vector m_fubar;
    Vector m_fxbar;
    
    Matrix A;
    Matrix B;
    Matrix Q;
    Matrix R;
    Matrix T;
    Vector fx;
    Vector fu;
    A.resize(3,3);
    B.resize(3,2);
    Q.resize(3,3);
    R.resize(2,2);
    T.resize(3,2);
    fx.resize(3,1);
    fu.resize(2,1);
    A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
    B << 1, 0, 0, 1, 1, 0;
    Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R << 1, 0, 0, 1;
    T << 1, 0, 0, 1, 0, 0;
    fx << 1, 1, 1;
    fu << 1, 1;
    
    unsigned int m_Nx = A.rows();
    unsigned int m_Nu = B.cols();
    unsigned int m_Np = 10;
    unsigned int m_Nt = 5;
    
    Matrix H;
    Matrix f;
    
    m_Su.resize(m_Nx*m_Np, m_Nu*m_Nt);
    m_Qbar.resize(m_Nx*m_Np, m_Nx*m_Np);
    m_Rbar.resize(m_Nu*m_Nt, m_Nu*m_Nt);
    m_Tbar.resize(m_Nx*m_Np, m_Nu*m_Nt);
    m_fxbar.resize(m_Nx*m_Np);
    m_fubar.resize(m_Nu*m_Nt);
    
    Matrix tmp;
    tmp.resize(m_Nt*m_Nu, m_Nt*m_Nu);
    
//     std::cout << "A = " << std::endl << A << std::endl << std::endl;
//     std::cout << "B = " << std::endl << B << std::endl << std::endl;
    
    std::clock_t c_start = std::clock();
    m_Su.block(m_Nx, 0, m_Nx, m_Nu) = B;
    // Fill the first column
    for(unsigned int i = 2; i < m_Np; i++) {
        m_Su.block(m_Nx*i, 0, m_Nx, m_Nu) = 
            A * m_Su.block(m_Nx*(i-1), 0, m_Nx, m_Nu);
    }
    // Copy first column to other column progressively
    for(unsigned int j = 1; j < m_Nt; j++) {
        m_Su.block(m_Nx*(j+1), m_Nu*j, m_Nx*(m_Np-j-1), m_Nu) = 
            m_Su.block(m_Nx, 0, m_Nx*(m_Np-j-1), m_Nu);
    }
    
    for(unsigned int k = 0; k < m_Np; k++) {
        m_Qbar.block(m_Nx*k, m_Nx*k, m_Nx, m_Nx) = Q;
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        m_Rbar.block(m_Nu*k, m_Nu*k, m_Nu, m_Nu) = R;
    }
    
    for(unsigned int k = 0; k < m_Nt; k++) {
        m_Tbar.block(m_Nx*k, m_Nu*k, m_Nx, m_Nu) = T;
    }
    
    for(unsigned int i = 0; i < m_Np; i++) {
        m_fxbar.segment(m_Nx*i, m_Nx) = fx;
    }
    
    for(unsigned int i = 0; i < m_Nt; i++) {
        m_fubar.segment(m_Nu*i, m_Nu) = fu;
    }
    
    tmp = m_Tbar.transpose() * m_Su;
    H = 2 * (
            m_Su.transpose() * m_Qbar * m_Su + 
            tmp + 
            tmp.transpose() + 
            m_Rbar
            );
    
//     H = 2 * (
//             m_Su.transpose() * m_Qbar * m_Su + 
//             m_Tbar.transpose() * m_Su + 
//             m_Su.transpose() * m_Tbar + 
//             m_Rbar
//             );
    f = m_Su.transpose() * m_fxbar + m_fubar;
    
    std::clock_t c_end = std::clock();
    
    std::cout << "m_Su = " << std::endl << m_Su << std::endl << std::endl;
    std::cout << "m_Qbar = " << std::endl << m_Qbar << std::endl << std::endl;
    std::cout << "m_Rbar = " << std::endl << m_Rbar << std::endl << std::endl;
    std::cout << "m_Tbar = " << std::endl << m_Tbar << std::endl << std::endl;
    std::cout << "m_fxbar = " << std::endl << m_fxbar << std::endl << std::endl;
    std::cout << "m_fubar = " << std::endl << m_fubar << std::endl << std::endl;
    std::cout << "H = " << std::endl << H << std::endl << std::endl;
    std::cout << "f = " << std::endl << f << std::endl << std::endl;
    
    long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";
    
//     qpOASES::real_t table[] = {*(f.data())};
    
//     qpOASES::real_t & table[] = (f.data());
    
    qpOASES::real_t table[f.size()];
    for (int k = 0; k < f.size(); k++)
        table[k] = *(f.data() + k);
    
//     std::cout << *(f.data()) << std::endl;
//     
    std::cout << table[0] << std::endl;
    std::cout << table[1] << std::endl;
    std::cout << table[2] << std::endl;
    std::cout << table[3] << std::endl;
    std::cout << table[4] << std::endl;
    
    assert(table[0] == 9851.0f);
    
//     // Check if eigen is working
//     using Eigen::MatrixXd;
//     Eigen::MatrixXd m(2,2);
//     m(0,0) = 3;
//     m(1,0) = 2.5;
//     m(0,1) = -1;
//     m(1,1) = m(1,0) + m(0,1);
//     std::cout << m << std::endl;
//   
// 	USING_NAMESPACE_QPOASES
// 
// 	/* Setup data of first QP. */
// 	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
// 	real_t A[1*2] = { 1.0, 1.0 };
// 	real_t g[2] = { 1.5, 1.0 };
// 	real_t lb[2] = { 0.5, -2.0 };
// 	real_t ub[2] = { 5.0, 2.0 };
// 	real_t lbA[1] = { -1.0 };
// 	real_t ubA[1] = { 2.0 };
// // 	real_t H[4*4] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5 };
// // 	real_t A[1*4] = { 1.0, 1.0, 1.0, 1.0 };
// // 	real_t g[4] = { 1.5, 1.0, 1.5, 1.0 };
// // 	real_t lb[4] = { 0.5, -2.0, 0.5, -2.0 };
// // 	real_t ub[4] = { 5.0, 2.0, 5.0, 2.0 };
// // 	real_t lbA[1] = { -1.0 };
// // 	real_t ubA[1] = { 2.0 };
// // 
// 	/* Setup data of second QP. */
// 	real_t g_new[2] = { 1.0, 1.5 };
// 	real_t lb_new[2] = { 0.0, -1.0 };
// 	real_t ub_new[2] = { 5.0, -0.5 };
// 	real_t lbA_new[1] = { -2.0 };
// 	real_t ubA_new[1] = { 1.0 };
// 
// 
// 	/* Setting up QProblem object. */
// 	QProblem example( 2,1 );
// 
// 	Options options;
// 	example.setOptions( options );
// 
// 	/* Solve first QP. */
// 	int_t nWSR = 10;
//     std::clock_t c_start = std::clock();
// 	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
//     
//     
// 	/* Get and print solution of first QP. */
// 	real_t xOpt[2];
// 	real_t yOpt[2+1];
// 	example.getPrimalSolution( xOpt );
// 	example.getDualSolution( yOpt );
//     std::clock_t c_end = std::clock();
// 	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
// 			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
// 	
// 	/* Solve second QP. */
// 	nWSR = 10;
//     std::clock_t c_start1 = std::clock();
// 	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );
// 
// 	/* Get and print solution of second QP. */
// 	example.getPrimalSolution( xOpt );
// 	example.getDualSolution( yOpt );
//     std::clock_t c_end1 = std::clock();
// 	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
// 			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
// 
// 	example.printOptions();
// 	/*example.printProperties();*/
// 
// 	/*getGlobalMessageHandler()->listAllMessages();*/
//     
//     long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
//     std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";
//     
//     long double time_elapsed_ms1 = 1000.0 * (c_end1-c_start1) / CLOCKS_PER_SEC;
//     std::cout << "CPU time used: " << time_elapsed_ms1 << " ms\n";

    
//     std::cout << std::endl << std::endl;
//     
//     int a = 1;
//     int & b = a;
//     std::cout << a << ',' << b << std::endl;
//     {
//         int c = 2;
//         b = c;
//     }
//     std::cout << a << ',' << b << std::endl;
//     a = 0;
//     std::cout << a << ',' << b << std::endl;
    
    
	return 0;
}
