/*
 * File    : buck_2pi.c
 * Abstract:
 *   C MEX S-function for a buck converter with dual-loop PI control
 *   (voltage outer loop + current inner loop).
 *
 *   Input ports (8):
 *     0: Vout       - output voltage
 *     1: IL         - inductor current
 *     2: Vdc        - input DC voltage (not used in this simple PI, kept for future)
 *     3: Vref       - voltage reference
 *     4: Kp_u       - voltage loop proportional gain
 *     5: Ki_u       - voltage loop integral gain
 *     6: Kp_i       - current loop proportional gain
 *     7: Ki_i       - current loop integral gain
 *
 *   Output port:
 *     0: duty cycle d (range 0..1)
 *
 *   Discrete states:
 *     0: integral of voltage error  (integral_v)
 *     1: integral of current error  (integral_i)
 *
 *   Sample time: inherited from the driving block (INHERITED_SAMPLE_TIME)
 *
 * Copyright 2025
 */

#define S_FUNCTION_NAME  ctrl
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Set up numbers of inputs, outputs, states, etc.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* No dialog parameters – everything comes through input ports */
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    /* Two discrete states: voltage integral & current integral */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 2);

    /* Eight input ports, all scalar */
    if (!ssSetNumInputPorts(S, 8)) return;
    int_T i;
    for (i = 0; i < 8; i++) {
        ssSetInputPortWidth(S, i, 1);
        /* All inputs have direct feedthrough because they are used in Outputs */
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    /* One output port (duty cycle) */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    /* One sample time (inherited) */
    ssSetNumSampleTimes(S, 1);

    /* No extra work vectors */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Inherit sample time from the model's discrete solver step.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* Function: mdlInitializeConditions ==========================================
 * Abstract:
 *    Initialize both integrators to zero.
 */
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x = ssGetRealDiscStates(S);
    x[0] = 0.0;   /* integral_v */
    x[1] = 0.0;   /* integral_i */
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Compute duty cycle using dual-loop PI:
 *      Iref = Kp_u*(Vref - Vout) + Ki_u * integral_v   (saturated)
 *      d    = Kp_i*(Iref - IL)   + Ki_i * integral_i     (saturated 0..1)
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Get input signals */
    const real_T *Vout  = ssGetInputPortRealSignal(S, 0);
    const real_T *IL    = ssGetInputPortRealSignal(S, 1);
    /* Vdc = ssGetInputPortRealSignal(S,2) – not used */
    const real_T *Vref  = ssGetInputPortRealSignal(S, 3);
    const real_T *Kp_u  = ssGetInputPortRealSignal(S, 4);
    const real_T *Ki_u  = ssGetInputPortRealSignal(S, 5);
    const real_T *Kp_i  = ssGetInputPortRealSignal(S, 6);
    const real_T *Ki_i  = ssGetInputPortRealSignal(S, 7);

    /* Output pointer */
    real_T *d = ssGetOutputPortRealSignal(S, 0);

    /* State pointers */
    real_T *x = ssGetRealDiscStates(S);
    real_T integral_v = x[0];
    real_T integral_i = x[1];

    /* Voltage loop */
    real_T ev = Vref[0] - Vout[0];
    real_T Iref = Kp_u[0] * ev + Ki_u[0] * integral_v;

    /* Clamp Iref to a reasonable range (example 0 .. 10 A) */
    if (Iref > 10.0) Iref = 10.0;
    else if (Iref < 0.0) Iref = 0.0;

    /* Current loop */
    real_T ei = Iref - IL[0];
    real_T d_raw = Kp_i[0] * ei + Ki_i[0] * integral_i;

    /* Saturate duty cycle to [0, 1] */
    if (d_raw > 1.0) d_raw = 1.0;
    else if (d_raw < 0.0) d_raw = 0.0;

    d[0] = d_raw;

    UNUSED_ARG(tid);
}

/* Function: mdlUpdate ========================================================
 * Abstract:
 *    Update the two integrators:
 *      integral_v += ev * Ts
 *      integral_i += ei * Ts
 *    where Ts is the actual task sample time (inherited).
 */
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    real_T *x = ssGetRealDiscStates(S);

    const real_T *Vout = ssGetInputPortRealSignal(S, 0);
    const real_T *IL   = ssGetInputPortRealSignal(S, 1);
    const real_T *Vref = ssGetInputPortRealSignal(S, 3);
    const real_T *Kp_u = ssGetInputPortRealSignal(S, 4);
    const real_T *Ki_u = ssGetInputPortRealSignal(S, 5);
    /* Kp_i, Ki_i not needed for update, only error */

    /* Get actual sample time from solver */
    real_T Ts = ssGetSampleTime(S, 0);
    if (Ts <= 0.0) {
        /* Fallback in case sample time is not yet determined (should not happen) */
        Ts = 1e-4;
    }

    /* Calculate same intermediate Iref as in Outputs to obtain ei */
    real_T ev = Vref[0] - Vout[0];
    real_T integral_v = x[0];
    real_T Iref = Kp_u[0] * ev + Ki_u[0] * integral_v;
    if (Iref > 10.0) Iref = 10.0;
    else if (Iref < 0.0) Iref = 0.0;

    real_T ei = Iref - IL[0];

    /* Integrate */
    x[0] += ev * Ts;
    x[1] += ei * Ts;

    UNUSED_ARG(tid);
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No cleanup required.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

/*=============================*
 * Required S-function trailer *
 *=============================*/
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif