Solve a standard Model Predictive Control (MPC) problem written as
\begin{eqnarray*}
    \min\limits_{(u_k)_{k\in [0,N-1]}} & 
        \sum\limits_{k=0}^{N_p-1} x_k^T Q x_k + 
        \sum\limits_{k=0}^{N_t-1} u_k^T T x_k + x_k^T T^T u_k + u_k^T R u_k\\
    \text{s.t.}
        & x_0 = x_{init}\\
        & x_{k+1} = A x_k +  B u_k\\
        & u_{min} \leq u_k \leq u_{max}\\
        & A_x x_k + A_u \delta u_k \leq b
\end{enarray*}

where $Q$, $R$, $T$, $f_x$, $f_u$ define the cost function. $A$ and $B$ is a
state-space representation of the system. $x_{init}$ is the initial state. The
prediction horizon is denoted as $N_p$ and the control horizon is denoted as
$N_t$.

The MPC is implemented to allow for Linear Time-Varying (LTV) systems. However, 
the size of the problem must not change over time.

Installation
============
The library is build with the CMake build system.

    cd buid
    cmake ..
    make

MATLAB S-Function
--------------
An C++\MEX S-function is defined to use the MPC in MATLAB\Simulink. The 
S-function is defined in the `sfunc` directory. Run the MATLAB script
`makeSFunction.m` to compile the MEX executable.


