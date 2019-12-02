#include "../include/mpcController.h"
#include <memory>

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME ltvmpc_sfunc
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS            14
/* Input Port  0 */
#define IN_PORT_0_NAME        Q
#define INPUT_0_WIDTH         DYNAMICALLY_SIZED
#define INPUT_DIMS_0_COL      DYNAMICALLY_SIZED
#define INPUT_0_DTYPE         real_T
#define INPUT_0_COMPLEX       COMPLEX_NO
#define IN_0_FRAME_BASED      FRAME_NO
#define IN_0_BUS_BASED        0
#define IN_0_BUS_NAME         
#define IN_0_DIMS             2-D
#define INPUT_0_FEEDTHROUGH   1
#define IN_0_ISSIGNED         0
#define IN_0_WORDLENGTH       8
#define IN_0_FIXPOINTSCALING  1
#define IN_0_FRACTIONLENGTH   9
#define IN_0_BIAS             0
#define IN_0_SLOPE            0.125
/* Input Port  1 */
#define IN_PORT_1_NAME        R
#define INPUT_1_WIDTH         1
#define INPUT_DIMS_1_COL      1
#define INPUT_1_DTYPE         real_T
#define INPUT_1_COMPLEX       COMPLEX_NO
#define IN_1_FRAME_BASED      FRAME_NO
#define IN_1_BUS_BASED        0
#define IN_1_BUS_NAME         
#define IN_1_DIMS             2-D
#define INPUT_1_FEEDTHROUGH   1
#define IN_1_ISSIGNED         0
#define IN_1_WORDLENGTH       8
#define IN_1_FIXPOINTSCALING  1
#define IN_1_FRACTIONLENGTH   9
#define IN_1_BIAS             0
#define IN_1_SLOPE            0.125
/* Input Port  2 */
#define IN_PORT_2_NAME        T
#define INPUT_2_WIDTH         1
#define INPUT_DIMS_2_COL      1
#define INPUT_2_DTYPE         real_T
#define INPUT_2_COMPLEX       COMPLEX_NO
#define IN_2_FRAME_BASED      FRAME_NO
#define IN_2_BUS_BASED        0
#define IN_2_BUS_NAME         
#define IN_2_DIMS             2-D
#define INPUT_2_FEEDTHROUGH   1
#define IN_2_ISSIGNED         0
#define IN_2_WORDLENGTH       8
#define IN_2_FIXPOINTSCALING  1
#define IN_2_FRACTIONLENGTH   9
#define IN_2_BIAS             0
#define IN_2_SLOPE            0.125
/* Input Port  3 */
#define IN_PORT_3_NAME        fx
#define INPUT_3_WIDTH         1
#define INPUT_DIMS_3_COL      1
#define INPUT_3_DTYPE         real_T
#define INPUT_3_COMPLEX       COMPLEX_NO
#define IN_3_FRAME_BASED      FRAME_NO
#define IN_3_BUS_BASED        0
#define IN_3_BUS_NAME         
#define IN_3_DIMS             1-D
#define INPUT_3_FEEDTHROUGH   1
#define IN_3_ISSIGNED         0
#define IN_3_WORDLENGTH       8
#define IN_3_FIXPOINTSCALING  1
#define IN_3_FRACTIONLENGTH   9
#define IN_3_BIAS             0
#define IN_3_SLOPE            0.125
/* Input Port  4 */
#define IN_PORT_4_NAME        fu
#define INPUT_4_WIDTH         1
#define INPUT_DIMS_4_COL      1
#define INPUT_4_DTYPE         real_T
#define INPUT_4_COMPLEX       COMPLEX_NO
#define IN_4_FRAME_BASED      FRAME_NO
#define IN_4_BUS_BASED        0
#define IN_4_BUS_NAME         
#define IN_4_DIMS             1-D
#define INPUT_4_FEEDTHROUGH   1
#define IN_4_ISSIGNED         0
#define IN_4_WORDLENGTH       8
#define IN_4_FIXPOINTSCALING  1
#define IN_4_FRACTIONLENGTH   9
#define IN_4_BIAS             0
#define IN_4_SLOPE            0.125
/* Input Port  5 */
#define IN_PORT_5_NAME        x0
#define INPUT_5_WIDTH         1
#define INPUT_DIMS_5_COL      1
#define INPUT_5_DTYPE         real_T
#define INPUT_5_COMPLEX       COMPLEX_NO
#define IN_5_FRAME_BASED      FRAME_NO
#define IN_5_BUS_BASED        0
#define IN_5_BUS_NAME         
#define IN_5_DIMS             1-D
#define INPUT_5_FEEDTHROUGH   1
#define IN_5_ISSIGNED         0
#define IN_5_WORDLENGTH       8
#define IN_5_FIXPOINTSCALING  1
#define IN_5_FRACTIONLENGTH   9
#define IN_5_BIAS             0
#define IN_5_SLOPE            0.125
/* Input Port  6 */
#define IN_PORT_6_NAME        A
#define INPUT_6_WIDTH         1
#define INPUT_DIMS_6_COL      1
#define INPUT_6_DTYPE         real_T
#define INPUT_6_COMPLEX       COMPLEX_NO
#define IN_6_FRAME_BASED      FRAME_NO
#define IN_6_BUS_BASED        0
#define IN_6_BUS_NAME         
#define IN_6_DIMS             2-D
#define INPUT_6_FEEDTHROUGH   1
#define IN_6_ISSIGNED         0
#define IN_6_WORDLENGTH       8
#define IN_6_FIXPOINTSCALING  1
#define IN_6_FRACTIONLENGTH   9
#define IN_6_BIAS             0
#define IN_6_SLOPE            0.125
/* Input Port  7 */
#define IN_PORT_7_NAME        B
#define INPUT_7_WIDTH         1
#define INPUT_DIMS_7_COL      1
#define INPUT_7_DTYPE         real_T
#define INPUT_7_COMPLEX       COMPLEX_NO
#define IN_7_FRAME_BASED      FRAME_NO
#define IN_7_BUS_BASED        0
#define IN_7_BUS_NAME         
#define IN_7_DIMS             2-D
#define INPUT_7_FEEDTHROUGH   1
#define IN_7_ISSIGNED         0
#define IN_7_WORDLENGTH       8
#define IN_7_FIXPOINTSCALING  1
#define IN_7_FRACTIONLENGTH   9
#define IN_7_BIAS             0
#define IN_7_SLOPE            0.125
/* Input Port  8 */
#define IN_PORT_8_NAME        ulb
#define INPUT_8_WIDTH         1
#define INPUT_DIMS_8_COL      1
#define INPUT_8_DTYPE         real_T
#define INPUT_8_COMPLEX       COMPLEX_NO
#define IN_8_FRAME_BASED      FRAME_NO
#define IN_8_BUS_BASED        0
#define IN_8_BUS_NAME         
#define IN_8_DIMS             1-D
#define INPUT_8_FEEDTHROUGH   1
#define IN_8_ISSIGNED         0
#define IN_8_WORDLENGTH       8
#define IN_8_FIXPOINTSCALING  1
#define IN_8_FRACTIONLENGTH   9
#define IN_8_BIAS             0
#define IN_8_SLOPE            0.125
/* Input Port  9 */
#define IN_PORT_9_NAME        uub
#define INPUT_9_WIDTH         1
#define INPUT_DIMS_9_COL      1
#define INPUT_9_DTYPE         real_T
#define INPUT_9_COMPLEX       COMPLEX_NO
#define IN_9_FRAME_BASED      FRAME_NO
#define IN_9_BUS_BASED        0
#define IN_9_BUS_NAME         
#define IN_9_DIMS             1-D
#define INPUT_9_FEEDTHROUGH   1
#define IN_9_ISSIGNED         0
#define IN_9_WORDLENGTH       8
#define IN_9_FIXPOINTSCALING  1
#define IN_9_FRACTIONLENGTH   9
#define IN_9_BIAS             0
#define IN_9_SLOPE            0.125
/* Input Port  10 */
#define IN_PORT_10_NAME        Ax
#define INPUT_10_WIDTH         1
#define INPUT_DIMS_10_COL      1
#define INPUT_10_DTYPE         real_T
#define INPUT_10_COMPLEX       COMPLEX_NO
#define IN_10_FRAME_BASED      FRAME_NO
#define IN_10_BUS_BASED        0
#define IN_10_BUS_NAME         
#define IN_10_DIMS             2-D
#define INPUT_10_FEEDTHROUGH   1
#define IN_10_ISSIGNED         0
#define IN_10_WORDLENGTH       8
#define IN_10_FIXPOINTSCALING  1
#define IN_10_FRACTIONLENGTH   9
#define IN_10_BIAS             0
#define IN_10_SLOPE            0.125
/* Input Port  11 */
#define IN_PORT_11_NAME        Au
#define INPUT_11_WIDTH         1
#define INPUT_DIMS_11_COL      1
#define INPUT_11_DTYPE         real_T
#define INPUT_11_COMPLEX       COMPLEX_NO
#define IN_11_FRAME_BASED      FRAME_NO
#define IN_11_BUS_BASED        0
#define IN_11_BUS_NAME         
#define IN_11_DIMS             2-D
#define INPUT_11_FEEDTHROUGH   1
#define IN_11_ISSIGNED         0
#define IN_11_WORDLENGTH       8
#define IN_11_FIXPOINTSCALING  1
#define IN_11_FRACTIONLENGTH   9
#define IN_11_BIAS             0
#define IN_11_SLOPE            0.125
/* Input Port  12 */
#define IN_PORT_12_NAME        As
#define INPUT_12_WIDTH         1
#define INPUT_DIMS_12_COL      1
#define INPUT_12_DTYPE         real_T
#define INPUT_12_COMPLEX       COMPLEX_NO
#define IN_12_FRAME_BASED      FRAME_NO
#define IN_12_BUS_BASED        0
#define IN_12_BUS_NAME         
#define IN_12_DIMS             2-D
#define INPUT_12_FEEDTHROUGH   1
#define IN_12_ISSIGNED         0
#define IN_12_WORDLENGTH       8
#define IN_12_FIXPOINTSCALING  1
#define IN_12_FRACTIONLENGTH   9
#define IN_12_BIAS             0
#define IN_12_SLOPE            0.125
/* Input Port  13 */
#define IN_PORT_13_NAME        b
#define INPUT_13_WIDTH         1
#define INPUT_DIMS_13_COL      1
#define INPUT_13_DTYPE         real_T
#define INPUT_13_COMPLEX       COMPLEX_NO
#define IN_13_FRAME_BASED      FRAME_NO
#define IN_13_BUS_BASED        0
#define IN_13_BUS_NAME         
#define IN_13_DIMS             1-D
#define INPUT_13_FEEDTHROUGH   1
#define IN_13_ISSIGNED         0
#define IN_13_WORDLENGTH       8
#define IN_13_FIXPOINTSCALING  1
#define IN_13_FRACTIONLENGTH   9
#define IN_13_BIAS             0
#define IN_13_SLOPE            0.125

#define NUM_OUTPUTS           1
/* Output Port  0 */
#define OUT_PORT_0_NAME       y0u
#define OUTPUT_0_WIDTH        1
#define OUTPUT_DIMS_0_COL     1
#define OUTPUT_0_DTYPE        real_T
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUT_0_FRAME_BASED     FRAME_NO
#define OUT_0_BUS_BASED       0
#define OUT_0_BUS_NAME        
#define OUT_0_DIMS            1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125

#define NPARAMS               8
/* Parameter 0 */
#define PARAMETER_0_NAME      Np
#define PARAMETER_0_DTYPE     real_T
#define PARAMETER_0_COMPLEX   COMPLEX_NO
/* Parameter 1 */
#define PARAMETER_1_NAME      Nt
#define PARAMETER_1_DTYPE     real_T
#define PARAMETER_1_COMPLEX   COMPLEX_NO
/* Parameter 2 */
#define PARAMETER_2_NAME      Nx
#define PARAMETER_2_DTYPE     real_T
#define PARAMETER_2_COMPLEX   COMPLEX_NO
/* Parameter 3 */
#define PARAMETER_3_NAME      Nu
#define PARAMETER_3_DTYPE     real_T
#define PARAMETER_3_COMPLEX   COMPLEX_NO
/* Parameter 4 */
#define PARAMETER_4_NAME      Nc
#define PARAMETER_4_DTYPE     real_T
#define PARAMETER_4_COMPLEX   COMPLEX_NO
/* Parameter 5 */
#define PARAMETER_5_NAME      Ns
#define PARAMETER_5_DTYPE     real_T
#define PARAMETER_5_COMPLEX   COMPLEX_NO
/* Parameter 6 */
#define PARAMETER_6_NAME      softConstIdx
#define PARAMETER_6_DTYPE     real_T
#define PARAMETER_6_COMPLEX   COMPLEX_NO
/* Parameter 7 */
#define PARAMETER_7_NAME      slackWeight
#define PARAMETER_7_DTYPE     real_T
#define PARAMETER_7_COMPLEX   COMPLEX_NO

#define SAMPLE_TIME_0         INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES       0
#define DISC_STATES_IC        [0]
#define NUM_CONT_STATES       0
#define CONT_STATES_IC        [0]

#define SFUNWIZ_GENERATE_TLC  1
#define SOURCEFILES           "__SFB__../../libltvmpc/lib/eigen/"
#define PANELINDEX            8
#define USE_SIMSTRUCT         0
#define SHOW_COMPILE_STEPS    0
#define CREATE_DEBUG_MEXFILE  0
#define SAVE_CODE_ONLY        0
#define SFUNWIZ_REVISION      3.0
/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"

#define PARAM_DEF0(S) ssGetSFcnParam(S, 0)
#define PARAM_DEF1(S) ssGetSFcnParam(S, 1)
#define PARAM_DEF2(S) ssGetSFcnParam(S, 2)
#define PARAM_DEF3(S) ssGetSFcnParam(S, 3)
#define PARAM_DEF4(S) ssGetSFcnParam(S, 4)
#define PARAM_DEF5(S) ssGetSFcnParam(S, 5)
#define PARAM_DEF6(S) ssGetSFcnParam(S, 6)
#define PARAM_DEF7(S) ssGetSFcnParam(S, 7)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

/*====================*
 * S-function methods *
 *====================*/
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *     Verify parameter definitions and types.
 */
static void mdlCheckParameters(SimStruct *S)
{
    int paramIndex  = 0;
    bool invalidParam = false;
    /* All parameters must match the S-function Builder Dialog */

    {
        const mxArray *pVal0 = ssGetSFcnParam(S, 0);
        if (!IS_PARAM_DOUBLE(pVal0)) {
            invalidParam = true;
            paramIndex = 0;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal1 = ssGetSFcnParam(S, 1);
        if (!IS_PARAM_DOUBLE(pVal1)) {
            invalidParam = true;
            paramIndex = 1;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal2 = ssGetSFcnParam(S, 2);
        if (!IS_PARAM_DOUBLE(pVal2)) {
            invalidParam = true;
            paramIndex = 2;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal3 = ssGetSFcnParam(S, 3);
        if (!IS_PARAM_DOUBLE(pVal3)) {
            invalidParam = true;
            paramIndex = 3;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal4 = ssGetSFcnParam(S, 4);
        if (!IS_PARAM_DOUBLE(pVal4)) {
            invalidParam = true;
            paramIndex = 4;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal5 = ssGetSFcnParam(S, 5);
        if (!IS_PARAM_DOUBLE(pVal5)) {
            invalidParam = true;
            paramIndex = 5;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal6 = ssGetSFcnParam(S, 6);
        if (!IS_PARAM_DOUBLE(pVal6)) {
            invalidParam = true;
            paramIndex = 6;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal7 = ssGetSFcnParam(S, 7);
        if (!IS_PARAM_DOUBLE(pVal7)) {
            invalidParam = true;
            paramIndex = 7;
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
}
#endif /* MDL_CHECK_PARAMETERS */
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS); /* Number of expected parameters */
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    #endif

    ssSetArrayLayoutForCodeGen(S, SS_COLUMN_MAJOR);

    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);


    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    /* Input Port 0 */
    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/

    /* Input Port 1 */
    ssSetInputPortWidth(S, 1, INPUT_1_WIDTH);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, INPUT_1_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/

    /* Input Port 2 */
    ssSetInputPortWidth(S, 2, INPUT_2_WIDTH);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, INPUT_2_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 2, INPUT_2_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/

    /* Input Port 3 */
    ssSetInputPortWidth(S, 3, INPUT_3_WIDTH);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, INPUT_3_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 3, INPUT_3_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/

    /* Input Port 4 */
    ssSetInputPortWidth(S, 4, INPUT_4_WIDTH);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, INPUT_4_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 4, INPUT_4_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/

    /* Input Port 5 */
    ssSetInputPortWidth(S, 5, INPUT_5_WIDTH);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, INPUT_5_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 5, INPUT_5_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/

    /* Input Port 6 */
    ssSetInputPortWidth(S, 6, INPUT_6_WIDTH);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, INPUT_6_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 6, INPUT_6_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/

    /* Input Port 7 */
    ssSetInputPortWidth(S, 7, INPUT_7_WIDTH);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, INPUT_7_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 7, INPUT_7_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/

    /* Input Port 8 */
    ssSetInputPortWidth(S, 8, INPUT_8_WIDTH);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, INPUT_8_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 8, INPUT_8_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/

    /* Input Port 9 */
    ssSetInputPortWidth(S, 9, INPUT_9_WIDTH);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, INPUT_9_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 9, INPUT_9_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/

    /* Input Port 10 */
    ssSetInputPortWidth(S, 10, INPUT_10_WIDTH);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, INPUT_10_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 10, INPUT_10_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/

    /* Input Port 11 */
    ssSetInputPortWidth(S, 11, INPUT_11_WIDTH);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, INPUT_11_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 11, INPUT_11_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/

    /* Input Port 12 */
    ssSetInputPortWidth(S, 12, INPUT_12_WIDTH);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, INPUT_12_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 12, INPUT_12_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/

    /* Input Port 13 */
    ssSetInputPortWidth(S, 13, INPUT_13_WIDTH);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, INPUT_13_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 13, INPUT_13_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/


    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    /* Output Port 0 */
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetNumPWork(S, 0);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimulinkVersionGeneratedIn(S, "10.0");

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                     SS_OPTION_WORKS_WITH_CODE_REUSE));
}


#define MDL_SET_INPUT_PORT_DIMENSION_INFO
void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                  int              portIndex, 
                                  const DimsInfo_T *dimsInfo)
{
    DECL_AND_INIT_DIMSINFO(portDimsInfo);
    int_T dims[2] = { OUTPUT_0_WIDTH, 1 };
    bool  frameIn = (ssGetInputPortFrameData(S, 0) == FRAME_YES);

    ssSetInputPortDimensionInfo(S, 0, dimsInfo);

    if (ssGetOutputPortNumDimensions(S, 0) == (-1)) {
        /* the output port has not been set */

        portDimsInfo.width   = 1;
        portDimsInfo.numDims = frameIn ? 2 : 1;
        portDimsInfo.dims    = frameIn ? dims : &portDimsInfo.width;

        ssSetOutputPortDimensionInfo(S, 0, &portDimsInfo);
    }
}


#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
void mdlSetOutputPortDimensionInfo(SimStruct        *S,         
                                   int_T            portIndex,
                                   const DimsInfo_T *dimsInfo)
{
    DECL_AND_INIT_DIMSINFO(portDimsInfo);
    int_T dims[2] = { OUTPUT_0_WIDTH, 1 };
    bool  frameOut = (ssGetOutputPortFrameData(S, 0) == FRAME_YES);

    ssSetOutputPortDimensionInfo(S, 0, dimsInfo);

    if (ssGetInputPortNumDimensions(S, 0) == (-1)) {
      /* the input port has not been set */

        portDimsInfo.width   = 1;
        portDimsInfo.numDims = frameOut ? 2 : 1;
        portDimsInfo.dims    = frameOut ? dims : &portDimsInfo.width;

        ssSetInputPortDimensionInfo(S, 0, &portDimsInfo);
    }
}


#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    DECL_AND_INIT_DIMSINFO(portDimsInfo);
    int_T dims[2] = { 1, 1 };
    bool  frame = (ssGetInputPortFrameData(S, 0) == FRAME_YES) ||
                  (ssGetOutputPortFrameData(S, 0) == FRAME_YES);

    /* Neither the input nor the output ports have been set */

    portDimsInfo.width   = 1;
    portDimsInfo.numDims = frame ? 2 : 1;
    portDimsInfo.dims    = frame ? dims : &portDimsInfo.width;

    if (ssGetInputPortNumDimensions(S, 0) == (-1)) {  
        ssSetInputPortDimensionInfo(S, 0, &portDimsInfo);
    }

    if (ssGetOutputPortNumDimensions(S, 0) == (-1)) {
        ssSetInputPortDimensionInfo(S, 0, &portDimsInfo);
    }
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType(S, 0, dType);
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

static void mdlSetWorkWidths(SimStruct *S)
{

    const char_T *rtParamNames[] = {"P1","P2","P3","P4","P5","P6","P7","P8"};
    ssRegAllTunableParamsAsRunTimeParams(S, rtParamNames);

}

#endif

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    unsigned int Np, Nt, Nx, Nu, Nc, Ns;
    Np = 10;
    Nt = 5;
    Nx = 1;
    Nu = 1;
    Nc = 1;
    Ns = 1;
    std::unique_ptr<QpOasesSolver> solver;
    solver = std::make_unique<QpOasesSolver>(Nu*Np, Nc);
    
    MpcController controller(std::move(solver), Nt, Np, Nx, Nu, Nc, Ns);
}
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 *
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *Q = (real_T *) ssGetInputPortRealSignal(S, 0);
    const real_T *R = (real_T *) ssGetInputPortRealSignal(S, 1);
    const real_T *T = (real_T *) ssGetInputPortRealSignal(S, 2);
    const real_T *fx = (real_T *) ssGetInputPortRealSignal(S, 3);
    const real_T *fu = (real_T *) ssGetInputPortRealSignal(S, 4);
    const real_T *x0 = (real_T *) ssGetInputPortRealSignal(S, 5);
    const real_T *A = (real_T *) ssGetInputPortRealSignal(S, 6);
    const real_T *B = (real_T *) ssGetInputPortRealSignal(S, 7);
    const real_T *ulb = (real_T *) ssGetInputPortRealSignal(S, 8);
    const real_T *uub = (real_T *) ssGetInputPortRealSignal(S, 9);
    const real_T *Ax = (real_T *) ssGetInputPortRealSignal(S, 10);
    const real_T *Au = (real_T *) ssGetInputPortRealSignal(S, 11);
    const real_T *As = (real_T *) ssGetInputPortRealSignal(S, 12);
    const real_T *b = (real_T *) ssGetInputPortRealSignal(S, 13);
    real_T *y0u = (real_T *) ssGetOutputPortRealSignal(S, 0);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const int_T   p_width1  = mxGetNumberOfElements(PARAM_DEF1(S));
    const int_T   p_width2  = mxGetNumberOfElements(PARAM_DEF2(S));
    const int_T   p_width3  = mxGetNumberOfElements(PARAM_DEF3(S));
    const int_T   p_width4  = mxGetNumberOfElements(PARAM_DEF4(S));
    const int_T   p_width5  = mxGetNumberOfElements(PARAM_DEF5(S));
    const int_T   p_width6  = mxGetNumberOfElements(PARAM_DEF6(S));
    const int_T   p_width7  = mxGetNumberOfElements(PARAM_DEF7(S));
    const real_T *Np = (const real_T *) mxGetData(PARAM_DEF0(S));
    const real_T *Nt = (const real_T *) mxGetData(PARAM_DEF1(S));
    const real_T *Nx = (const real_T *) mxGetData(PARAM_DEF2(S));
    const real_T *Nu = (const real_T *) mxGetData(PARAM_DEF3(S));
    const real_T *Nc = (const real_T *) mxGetData(PARAM_DEF4(S));
    const real_T *Ns = (const real_T *) mxGetData(PARAM_DEF5(S));
    const real_T *softConstIdx = (const real_T *) mxGetData(PARAM_DEF6(S));
    const real_T *slackWeight = (const real_T *) mxGetData(PARAM_DEF7(S));
        const int_T u_width = ssGetInputPortWidth(S, 0);

    
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{

}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



