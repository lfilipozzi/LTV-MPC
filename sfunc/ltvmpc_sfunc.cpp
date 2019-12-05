#include "../include/mpcController.h"
#include <memory>

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
#define NUM_OUTPUTS     1

// Indices of parameters
#define PARAM_NP_IDX            0
#define PARAM_NT_IDX            1
#define PARAM_NX_IDX            2
#define PARAM_NU_IDX            3
#define PARAM_NC_IDX            4
#define PARAM_NS_IDX            5
#define PARAM_SLACKIDX_IDX      6
#define PARAM_SLACKWEIGHT_IDX   7
#define NUM_PARAMS              8

// Sampling time
#define SAMPLE_TIME_0         INHERITED_SAMPLE_TIME

// S-Function states
#define NUM_DISC_STATES       0
#define NUM_CONT_STATES       0



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
            goto EXIT_POINT;
        }
    }
    
    // Nt: control horizon
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NT_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NT_IDX;
            goto EXIT_POINT;
        }
    }

    // Nx: number of states
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NX_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NX_IDX;
            goto EXIT_POINT;
        }
    }

    // Nu: number of inputs
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NU_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NU_IDX;
            goto EXIT_POINT;
        }
    }

    // Nc: number of polytopic constraints
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NC_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NC_IDX;
            goto EXIT_POINT;
        }
    }

    // Ns: Number of slack variables
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_NS_IDX);
        if (!IS_PARAM_UINT8(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_NS_IDX;
            goto EXIT_POINT;
        }
    }

    // Indices of soft constraints
    {
        const mxArray *pCellArray = ssGetSFcnParam(S, PARAM_SLACKIDX_IDX);
        if (!IS_PARAM_CELLARRAY(pCellArray)) {
            invalidParam = true;
            paramIndex = PARAM_SLACKIDX_IDX;
            goto EXIT_POINT;
        }
        else {
            if (!mxIsEmpty(pCellArray)) {
                const mwSize * dims = mxGetDimensions(pCellArray);
                for (mwIndex iCell = 0; iCell < dims[0]; iCell++) {
                    mxArray * pCellElement = mxGetCell(pCellArray, iCell);
                    unsigned int * p = (unsigned int *) mxGetPr(pCellElement);
                    mexPrintf("The content at %d is %d\n", iCell, (unsigned int)*p);
                }
            }
        }
//         else {
//             if (!mxIsEmpty(pCell)) {
//                 const mwSize * pDims;
//                 const mxArray * pCell;
//                 const mxArray * pCellArray;
//                 double *output;
//                 const int nbElem = mxGetNumberOfElements(pCell);
//                 for (mwIndex i = 0; i < nbElem; i++) {
//                     cellElement = mxGetCell(cell,jcell);
//                     p = mxGetPr(cellElement)
//                     mexPrintf("The content at %d is %g\n", jcell, *p);
// //                     if (!IS_PARAM_UINT8(pCell)) {
// //                         invalidParam = true;
// //                         paramIndex = PARAM_SLACKIDX_IDX;
// //                         goto CELL_CONTENT_POINT;
// //                     }
//                 }
//             }
//         }
    }

    // Penalty weight of slack variables associated to soft constraints
    {
        const mxArray *pVal = ssGetSFcnParam(S, PARAM_SLACKWEIGHT_IDX);
        if (!IS_PARAM_CELLARRAY(pVal)) {
            invalidParam = true;
            paramIndex = PARAM_SLACKWEIGHT_IDX;
            goto EXIT_POINT;
        }
    }
    
    
    EXIT_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The data type and or complexity of "
            "parameter %d does not match the information specified in the "
            "S-function Builder dialog. For non-double parameters you will "
            "need to cast them using int8, int16, int32, uint8, uint16, uint32 "
            "or boolean.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
    
    CELL_CONTENT_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The content of the cell not legal", 
                paramIndex + 1);
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
    
    // Q matrix input
    ssSetInputPortMatrixDimensions(S, IN_Q_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_Q_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_Q_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_Q_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_Q_IDX, 1);

    // R matrix input
    ssSetInputPortMatrixDimensions(S, IN_R_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_R_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_R_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_R_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_R_IDX, 1);

    // T matrix input
    ssSetInputPortMatrixDimensions(S, IN_T_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_T_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_T_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_T_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_T_IDX, 1);

    // fx vector input
    ssSetInputPortWidth(S, IN_FX_IDX, 1);
    ssSetInputPortDataType(S, IN_FX_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_FX_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_FX_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_FX_IDX, 1);

    // fu vector input
    ssSetInputPortWidth(S, IN_FU_IDX, 1);
    ssSetInputPortDataType(S, IN_FU_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_FU_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_FU_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_FU_IDX, 1);

    // x0 vector input
    ssSetInputPortWidth(S, IN_X0_IDX, 1);
    ssSetInputPortDataType(S, IN_X0_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_X0_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_X0_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_X0_IDX, 1);

    // A state-space matrix
    ssSetInputPortMatrixDimensions(S, IN_A_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_A_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_A_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_A_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_A_IDX, 1);

    // B state-space matrix
    ssSetInputPortMatrixDimensions(S, IN_B_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_B_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_B_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_B_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_B_IDX, 1);

    // Lower-bound actuator saturation
    ssSetInputPortWidth(S, IN_ULB_IDX, 1);
    ssSetInputPortDataType(S, IN_ULB_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_ULB_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_ULB_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_ULB_IDX, 1);

    // Upper-bound actuator saturation
    ssSetInputPortWidth(S, IN_UUB_IDX, 1);
    ssSetInputPortDataType(S, IN_UUB_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_UUB_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_UUB_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_UUB_IDX, 1);

    // Inequality matrix on the states
    ssSetInputPortMatrixDimensions(S, IN_AINEQX_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_AINEQX_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_AINEQX_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_AINEQX_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_AINEQX_IDX, 1);

    // Inequality matrix on the inputs
    ssSetInputPortMatrixDimensions(S, IN_AINEQU_IDX, 2, 2);
    ssSetInputPortDataType(S, IN_AINEQU_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_AINEQU_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_AINEQU_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_AINEQU_IDX, 1);

    // Polytopic constraints bound
    ssSetInputPortWidth(S, IN_BINEQ_IDX, 1);
    ssSetInputPortDataType(S, IN_BINEQ_IDX, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, IN_BINEQ_IDX, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, IN_BINEQ_IDX, 0);
    ssSetInputPortRequiredContiguous(S, IN_BINEQ_IDX, 1);
    
    /*
     * Configure the block inputs.
     */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) 
        return;
    
    // Control output of the MPC
    ssSetOutputPortWidth(S, OUT_U_IDX, 1);
    ssSetOutputPortDataType(S, OUT_U_IDX, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, OUT_U_IDX, COMPLEX_NO);

    /*
     * Set the number of sample times.
     */
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimulinkVersionGeneratedIn(S, "10.0");

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                     SS_OPTION_WORKS_WITH_CODE_REUSE));
}





/**
 * @brief Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
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
    const mxArray * pNp = ssGetSFcnParam(S, PARAM_NP_IDX);
    const mxArray * pNt = ssGetSFcnParam(S, PARAM_NT_IDX);
    const mxArray * pNx = ssGetSFcnParam(S, PARAM_NX_IDX);
    const mxArray * pNu = ssGetSFcnParam(S, PARAM_NU_IDX);
    const mxArray * pNc = ssGetSFcnParam(S, PARAM_NC_IDX);
    const mxArray * pNs = ssGetSFcnParam(S, PARAM_NS_IDX);
    
    const unsigned int Np = (unsigned int) *mxGetPr(pNp);
    const unsigned int Nt = (unsigned int) *mxGetPr(pNt);
    const unsigned int Nx = (unsigned int) *mxGetPr(pNx);
    const unsigned int Nu = (unsigned int) *mxGetPr(pNu);
    const unsigned int Nc = (unsigned int) *mxGetPr(pNc);
    const unsigned int Ns = (unsigned int) *mxGetPr(pNs);
    
    std::unique_ptr<IQpSolver> solver;
    solver = std::make_unique<QpOasesSolver>(Nu*Np, Nc);
    MpcController * controller;
    controller =  new MpcController(std::move(solver), Nt, Np, Nx, Nu, Nc, Ns);
    ssGetPWork(S)[0] = (void *) controller;
    
    // Define soft constraints
    const mxArray * slackIdxCell    = mxGetCell(ssGetSFcnParam(S, PARAM_SLACKIDX_IDX), 0);
    const mxArray * slackWeightCell = mxGetCell(ssGetSFcnParam(S, PARAM_SLACKWEIGHT_IDX), 0);
    for (int i = 0; i < Ns; i++)
    {
        unsigned int slackIdx;
        std::vector<unsigned int> constraintsIdx;
        double weight;
         
//         // TODO assign value to slackIdx, constraintsIdx, and weight
       
         if (controller)
            controller->setSoftConstraints(slackIdx, constraintsIdx, weight);
        else {
            char msg[256];
            sprintf(msg,"Error due to null pointer");
            ssSetErrorStatus(S, msg);
            return;
        }
    }
}
#endif // MDL_START





#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
/**
 *  @brief This function is called at each major step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
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
    
    // Get output signal pointer
    real_T *y0u = (real_T *) ssGetOutputPortRealSignal(S, 0);
    
    // TODO
}
#endif // MDL_UPDATE





#define MDL_OUTPUTS
/**
 * @brief This function is executed at each minor step.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    MpcController * controller = (MpcController *) ssGetPWork(S)[0];
    
    // TODO
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
    delete controller;
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



