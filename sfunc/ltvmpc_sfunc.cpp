#include "../include/mpcController.h"
#include <memory>
#include <vector>
#include <ctime>

// #define MATLAB_MEX_FILE // TODO comment when compiling with MEX
// #include "matrix.h"
// #include "mex.h"
// #include <tmwtypes.h>

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME ltvmpc_sfunc
#include "simstruc.h"

// Indices of inputs
#define IN_Q_IDX        0
#define IN_R_IDX        1
#define IN_T_IDX        2
#define IN_FX_IDX       3
#define IN_FU_IDX       4
#define IN_X0_IDX       5
#define IN_A_IDX        6
#define IN_B_IDX        7
#define IN_ULB_IDX      8
#define IN_UUB_IDX      9
#define IN_AINEQX_IDX   10
#define IN_AINEQU_IDX   11
#define IN_BINEQ_IDX    12
#define NUM_INPUTS      13

// Indices of outputs 
#define OUT_U_IDX       0
#define OUT_STATUS_IDX  1
#define OUT_TIME_IDX    2
#define NUM_OUTPUTS     3

// Indices of parameters
#define PARAM_NP_IDX            0
#define PARAM_NT_IDX            1
#define PARAM_NX_IDX            2
#define PARAM_NU_IDX            3
#define PARAM_NC_IDX            4
#define PARAM_NS_IDX            5
#define PARAM_SLACKIDX_IDX      6
#define PARAM_SLACKWEIGHT_IDX   7
#define PARAM_INPUTSCALING_IDX  8
#define PARAM_TS_IDX            9
#define NUM_PARAMS              10

// TODO set macro with param name for easier debugging

// S-Function discrete states
#define NUM_DISC_STATES     0

// S-Function continuous states
#define NUM_CONT_STATES     0



#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define IS_PARAM_UINT8(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsUint8(pVal))

#define IS_PARAM_CELLARRAY(pVal) (mxIsCell(pVal)) //





#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/**
 * @brief Verify parameter definitions and types.
 */
static void mdlCheckParameters(SimStruct *S)
{
    int paramIndex  = 0;
    bool invalidParam = false;

    // Np: prediction horizon
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NP_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NP_IDX;
            goto EXIT_TYPE_POINT;
        }
    }
    
    // Nt: control horizon
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NT_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NT_IDX;
            goto EXIT_TYPE_POINT;
        }
    }

    // Nx: number of states
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NX_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NX_IDX;
            goto EXIT_TYPE_POINT;
        }
    }

    // Nu: number of inputs
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NU_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NU_IDX;
            goto EXIT_TYPE_POINT;
        }
    }

    // Nc: number of polytopic constraints
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NC_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NC_IDX;
            goto EXIT_TYPE_POINT;
        }
    }

    // Ns: Number of slack variables
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NS_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NS_IDX;
            goto EXIT_TYPE_POINT;
        }
    }

    // Indices of soft constraints
    {
        // Get number of constraints and slack variable
        unsigned int * pNc, * pNs;
        pNc = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NC_IDX));
        pNs = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NS_IDX));
        const unsigned int Nc = *pNc;
        const unsigned int Ns = *pNs;
        
        // TODO need to check:
        // X param is a cell
        // - size of the cell: as many element than slack variable
        // - for each component:
        //   - value is acceptable: not bigger than Nc-1, not smaller than 0
        //   - value is uint8, not empty ...
        
        // Check the parameter is a cell array
        const mxArray *pCellArray = ssGetSFcnParam(S, PARAM_SLACKIDX_IDX);
        if (!IS_PARAM_CELLARRAY(pCellArray)) {
            invalidParam = true;
            paramIndex = PARAM_SLACKIDX_IDX;
            goto EXIT_TYPE_POINT;
        }
        
        // The cell can be empty if there is no soft constraint. Check the 
        // parameter if it is not empty
        if (!mxIsEmpty(pCellArray)) {
            // Check the dimension of the cell array
            const mwSize * dims = mxGetDimensions(pCellArray);
            if (dims[0] != Ns) {
                invalidParam = true;
                paramIndex = PARAM_SLACKIDX_IDX;
                goto EXIT_CELL_SIZE_POINT;
            }
            
            // Check each element of the cell array
            for (mwIndex idxCell = 0; idxCell < dims[0]; idxCell++) {
                // Get cell element of the array
                mxArray * pCell = mxGetCell(pCellArray, idxCell);
                
                // Check type of the element
                if (!IS_PARAM_UINT8(pCell)) {
                    invalidParam = true;
                    paramIndex = PARAM_SLACKIDX_IDX;
                    goto EXIT_CELL_TYPE_POINT;
                }
                
                // Check value of the element
                const mwSize * dim = mxGetDimensions(pCell);
                uint8_T * p = (uint8_T *) mxGetData(pCell);
                for (int rowIdx = 0; rowIdx < dim[0]; rowIdx++) {
                    for (int colIdx = 0; colIdx < dim[1]; colIdx++) {
                        int subs[] = {rowIdx, colIdx};
                        int index = mxCalcSingleSubscript(pCell, 2, subs);
                        int value = (*(p+index)-1);
                        if (!(value >= 0 && value < Nc)) {
                            invalidParam = true;
                        paramIndex = PARAM_SLACKIDX_IDX;
                        goto EXIT_CELL_VALUE_POINT;
                        }
                    }
                }
            }
        }
    }

    // Penalty weight of slack variables associated to soft constraints
    // TODO check parameter
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_SLACKWEIGHT_IDX);
        if (!IS_PARAM_CELLARRAY(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_SLACKWEIGHT_IDX;
            goto EXIT_TYPE_POINT;
        }
    }
    
    // Scaling used to normalize the inputs
    // TODO check parameter
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_INPUTSCALING_IDX);
        if (!IS_PARAM_DOUBLE(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_INPUTSCALING_IDX;
            goto EXIT_TYPE_POINT;
        }
    }
    
    // Ts: sampling time
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_TS_IDX);
        if (!IS_PARAM_DOUBLE(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_TS_IDX;
            goto EXIT_TYPE_POINT;
        }
    }
    
    
    EXIT_TYPE_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The data type and or complexity of "
            "parameter %d does not match the requirement.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
    
    EXIT_CELL_TYPE_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The data type and or complexity of the "
            "content of the cell in parameter %d does not match the "
            "requirement.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
    
    EXIT_CELL_SIZE_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The data size of the cell in parameter "
            "%d does not match the requirement.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
    
    EXIT_CELL_VALUE_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The value of the cell in parameter "
            "%d does not match the requirement.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
}
#endif // MDL_CHECK_PARAMETERS





/**
 * @brief Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /*
     * Specify the number of parameters expected by the S-function.
     */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        // Parameter mismatch will be reported by Simulink 
        return;
    }
    #endif
    
    /*
     * Specify the parameters are not tunable.
     */
    ssSetSFcnParamNotTunable(S, PARAM_NP_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_NT_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_NX_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_NU_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_NC_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_NS_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_SLACKIDX_IDX);
    ssSetSFcnParamNotTunable(S, PARAM_SLACKWEIGHT_IDX);

    ssSetArrayLayoutForCodeGen(S, SS_COLUMN_MAJOR);

    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    /*
     * Specify the number of states that the function has.
     */
    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);

    /*
     * Configure the block inputs.
     */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) 
        return;
    
    // Get the size of the MPC problem
    unsigned int * pNx, * pNu, * pNc;
    
    pNx = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NX_IDX));
    pNu = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NU_IDX));
    pNc = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NC_IDX));
    
    const unsigned int Nx = *pNx;
    const unsigned int Nu = *pNu;
    const unsigned int Nc = *pNc;
    
    // Q matrix input
    ssSetInputPortMatrixDimensions(S, IN_Q_IDX, Nx, Nx);
    ssSetInputPortDataType(S, IN_Q_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_Q_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_Q_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_Q_IDX, 1);

    // R matrix input
    ssSetInputPortMatrixDimensions(S, IN_R_IDX, Nu, Nu);
    ssSetInputPortDataType(S, IN_R_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_R_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_R_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_R_IDX, 1);

    // T matrix input
    ssSetInputPortMatrixDimensions(S, IN_T_IDX, Nx, Nu);
    ssSetInputPortDataType(S, IN_T_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_T_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_T_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_T_IDX, 1);

    // fx vector input
    ssSetInputPortWidth(S, IN_FX_IDX, Nx);
    ssSetInputPortDataType(S, IN_FX_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_FX_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_FX_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_FX_IDX, 1);

    // fu vector input
    ssSetInputPortWidth(S, IN_FU_IDX, Nu);
    ssSetInputPortDataType(S, IN_FU_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_FU_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_FU_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_FU_IDX, 1);

    // x0 vector input
    ssSetInputPortWidth(S, IN_X0_IDX, Nx);
    ssSetInputPortDataType(S, IN_X0_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_X0_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_X0_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_X0_IDX, 1);

    // A state-space matrix
    ssSetInputPortMatrixDimensions(S, IN_A_IDX, Nx, Nx);
    ssSetInputPortDataType(S, IN_A_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_A_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_A_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_A_IDX, 1);

    // B state-space matrix
    ssSetInputPortMatrixDimensions(S, IN_B_IDX, Nx, Nu);
    ssSetInputPortDataType(S, IN_B_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_B_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_B_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_B_IDX, 1);

    // Lower-bound actuator saturation
    ssSetInputPortWidth(S, IN_ULB_IDX, Nu);
    ssSetInputPortDataType(S, IN_ULB_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_ULB_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_ULB_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_ULB_IDX, 1);

    // Upper-bound actuator saturation
    ssSetInputPortWidth(S, IN_UUB_IDX, Nu);
    ssSetInputPortDataType(S, IN_UUB_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_UUB_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_UUB_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_UUB_IDX, 1);

    // Inequality matrix on the states
    ssSetInputPortMatrixDimensions(S, IN_AINEQX_IDX, Nc, Nx);
    ssSetInputPortDataType(S, IN_AINEQX_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_AINEQX_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_AINEQX_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_AINEQX_IDX, 1);

    // Inequality matrix on the inputs
    ssSetInputPortMatrixDimensions(S, IN_AINEQU_IDX, Nc, Nu);
    ssSetInputPortDataType(S, IN_AINEQU_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_AINEQU_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_AINEQU_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_AINEQU_IDX, 1);

    // Polytopic constraints bound
    ssSetInputPortWidth(S, IN_BINEQ_IDX, Nc);
    ssSetInputPortDataType(S, IN_BINEQ_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_BINEQ_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_BINEQ_IDX, 1);
    ssSetInputPortRequiredContiguous(S, IN_BINEQ_IDX, 1);
    
    /*
     * Configure the block outputs.
     */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) 
        return;
    
    // Control output of the MPC
    ssSetOutputPortWidth(S, OUT_U_IDX, Nu);
    ssSetOutputPortDataType(S, OUT_U_IDX, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, OUT_U_IDX, COMPLEX_NO);
    
    // Status of the optimization
    ssSetOutputPortWidth(S, OUT_STATUS_IDX, 1);
    ssSetOutputPortDataType(S, OUT_STATUS_IDX, SS_BOOLEAN);
    ssSetOutputPortComplexSignal(S, OUT_STATUS_IDX, COMPLEX_NO);
    
    // Computation time
    ssSetOutputPortWidth(S, OUT_TIME_IDX, 1);
    ssSetOutputPortDataType(S, OUT_TIME_IDX, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, OUT_TIME_IDX, COMPLEX_NO);

    /*
     * Set the number of sample times and S-function states
     */
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimulinkVersionGeneratedIn(S, "10.0");

    // Take care when specifying exception free code - see sfuntmpl_doc.c
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                     SS_OPTION_WORKS_WITH_CODE_REUSE));
}





/**
 * @brief Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    time_T Ts = mxGetScalar(ssGetSFcnParam(S, PARAM_TS_IDX));
    ssSetSampleTime(S, 0, Ts);
    ssSetModelReferenceSampleTimeDisallowInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}





#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
/**
 * @brief This function is called once at start of model execution. If you have
 * states that should be initialized once, this is the place to do it.
 */
static void mdlStart(SimStruct *S)
{
    // Get the size of the MPC problem
    unsigned int * pNp, * pNt, * pNx, * pNu, * pNc, * pNs;
    
    pNp = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NP_IDX));
    pNt = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NT_IDX));
    pNx = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NX_IDX));
    pNu = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NU_IDX));
    pNc = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NC_IDX));
    pNs = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NS_IDX));
    
    const unsigned int Np = *pNp;
    const unsigned int Nt = *pNt;
    const unsigned int Nx = *pNx;
    const unsigned int Nu = *pNu;
    const unsigned int Nc = *pNc;
    const unsigned int Ns = *pNs;
    
    // Create solver and MPC controller
    std::unique_ptr<IQpSolver> solver;
    int nWSR = 100;
    solver = std::make_unique<QpOasesSolver>(Nu*Nt+Ns, Nc*Np, nWSR);
    MpcController * controller;
    controller =  new MpcController(std::move(solver), Nt, Np, Nx, Nu, Nc, Ns);
    ssGetPWork(S)[0] = (void *) controller;
    
    // Define soft constraints
    // Get parameters (cell array)
    const mxArray * pSlackIdxCellArray    = ssGetSFcnParam(S, PARAM_SLACKIDX_IDX);
    const mxArray * pSlackWeightCellArray = ssGetSFcnParam(S, PARAM_SLACKWEIGHT_IDX);
    for (int slackIdx = 0; slackIdx < Ns; slackIdx++) {
        std::vector<unsigned int> constraintsIdx;
        double weight;
        
        // Get element of the cell
        mxArray * pSlackIdxCell = mxGetCell(pSlackIdxCellArray, slackIdx);
        mxArray * pSlackWeightCell = mxGetCell(pSlackWeightCellArray, slackIdx);
        
        // Get the weight
        weight = mxGetScalar(pSlackWeightCell);
        // Get the number of element per dimension of the matrix
        const mwSize * dim = mxGetDimensions(pSlackIdxCell);
        // Get pointer to first element
        uint8_T * p = (uint8_T *) mxGetData(pSlackIdxCell);
        
        for (int rowIdx = 0; rowIdx < dim[0]; rowIdx++) {
            for (int colIdx = 0; colIdx < dim[1]; colIdx++) {
                int subs[] = {rowIdx, colIdx};
                int index = mxCalcSingleSubscript(pSlackIdxCell, 2, subs);
                constraintsIdx.push_back(*(p+index)-1);
            }
        }
        
        // Set soft constraint in the MPC controller
        if (controller) {
            controller->initialize();
            controller->setSoftConstraints(slackIdx, constraintsIdx, weight);
        }
        else {
            char msg[256];
            sprintf(msg,"Error due to null pointer");
            ssSetErrorStatus(S, msg);
            return;
        }
    }
    
    // Set input scaling factors
    double * pInputScaling = mxGetPr(ssGetSFcnParam(S, PARAM_INPUTSCALING_IDX));
    for (int idx = 0; idx < Nu; idx++) {
        double factor = *(pInputScaling + idx);
        controller->setInputScaleFactor(idx, factor);
    }
}
#endif // MDL_START





#define MDL_OUTPUTS
/**
 * @brief Update the block's states and compute its output signals.
 */
static void mdlOutputs(SimStruct *S, int_T /*tid*/)
{
    MpcController * controller = (MpcController *) ssGetPWork(S)[0];
    
    /*
     * Compute the solution of the MPC problem.
     */
    // Start clock
    std::clock_t c_start = std::clock();
    
    // Get input signal pointer
    const real_T * Q   = (real_T *) ssGetInputPortRealSignal(S, IN_Q_IDX);
    const real_T * R   = (real_T *) ssGetInputPortRealSignal(S, IN_R_IDX);
    const real_T * T   = (real_T *) ssGetInputPortRealSignal(S, IN_T_IDX);
    const real_T * fx  = (real_T *) ssGetInputPortRealSignal(S, IN_FX_IDX);
    const real_T * fu  = (real_T *) ssGetInputPortRealSignal(S, IN_FU_IDX);
    const real_T * x0  = (real_T *) ssGetInputPortRealSignal(S, IN_X0_IDX);
    const real_T * A   = (real_T *) ssGetInputPortRealSignal(S, IN_A_IDX);
    const real_T * B   = (real_T *) ssGetInputPortRealSignal(S, IN_B_IDX);
    const real_T * ulb = (real_T *) ssGetInputPortRealSignal(S, IN_ULB_IDX);
    const real_T * uub = (real_T *) ssGetInputPortRealSignal(S, IN_UUB_IDX);
    const real_T * Ax  = (real_T *) ssGetInputPortRealSignal(S, IN_AINEQX_IDX);
    const real_T * Au  = (real_T *) ssGetInputPortRealSignal(S, IN_AINEQU_IDX);
    const real_T * b   = (real_T *) ssGetInputPortRealSignal(S, IN_BINEQ_IDX);
    
    if (controller) {
        // Set LTV-MPC formulation
        controller->setCostFunction(Q, R, T, fx, fu);
        controller->setInitialCondition(x0);
        controller->setPlantModel(A, B);
        controller->setActuatorBounds(ulb, uub);
        controller->setConstraints(Ax, Au, b);
        
        // Discretize the plant 
        // TODO
        
        // Solve the MPC problem
        controller->update();
    }
    else {
        char msg[256];
        sprintf(msg,"Error due to null pointer");
        ssSetErrorStatus(S, msg);
        return;
    }
    
    // Get timer
    std::clock_t c_end = std::clock();
    
    /*
     * Output the signals.
     */
    unsigned int * pNu;
    pNu = (unsigned int *) mxGetData(ssGetSFcnParam(S, PARAM_NU_IDX));
    const unsigned int Nu = *pNu;
    
    // Output solution and optimization status
    real_T * yControl = ssGetOutputPortRealSignal(S, OUT_U_IDX);
    boolean_T * yStatus = (boolean_T *) ssGetOutputPortRealSignal(S, OUT_STATUS_IDX);
    real_T * yTime = ssGetOutputPortRealSignal(S, OUT_TIME_IDX);
    
    if (controller) {
        // Set output
        double * control;
        *yStatus = (boolean_T) controller->output(&control);
        for (unsigned int i = 0; i < Nu; i++) {
            yControl[i] = *(control + i);
        }
    }
    else {
        char msg[256];
        sprintf(msg,"Error due to null pointer");
        ssSetErrorStatus(S, msg);
        return;
    }
    
    // Output computation time
    *yTime = (c_end - c_start) / (double) CLOCKS_PER_SEC;
}





/**
 * @brief In this function, you should perform any actions that are necessary at
 * the termination of a simulation.  For example, if memory was allocated in
 * mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    MpcController  * controller;
    controller = (MpcController *) ssGetPWork(S)[0];
    controller->terminate();
    delete controller;
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif







