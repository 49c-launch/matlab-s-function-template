/*
 * File    : ctrl.c
 * Abstract:
 *   C MEX S-function for a buck converter with dual-loop PI control
 *   and internal PWM generation (center-aligned / triangle carrier).
 *
 *   Input ports (8):
 *     0: Vout       - output voltage
 *     1: IL         - inductor current
 *     2: Vref       - voltage reference
 *     3: Kp_u       - voltage loop proportional gain
 *     4: Ki_u       - voltage loop integral gain
 *     5: Kp_i       - current loop proportional gain
 *     6: Ki_i       - current loop integral gain
 *     7: deadtime   - dead time in seconds
 *
 *   Output ports:
 *     0: PWMA signal (0 or 1)
 *     1: PWMB signal (0 or 1, complementary with dead time)
 *
 *   Timing:
 *     Sample rate: 100 MHz (10 ns per step)
 *     PWM frequency: 20 kHz (5000 steps per period)
 *     PI update rate: 20 kHz (once per PWM period)
 *
 * Copyright 2025
 */

#define S_FUNCTION_NAME  ctrl
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "pi_ctrl.h"

#include <stdlib.h>

/* PWM and timing parameters */
#define PWM_FREQ      20000.0
#define SAMPLE_FREQ   50000000.0
#define STEPS_PER_PWM ((int_T)(SAMPLE_FREQ / PWM_FREQ))
#define HALF_PERIOD   (STEPS_PER_PWM / 2)

/* Saturation limits */
#define IREF_MAX  10.0
#define IREF_MIN   0.0
#define DUTY_MAX   1.0
#define DUTY_MIN   0.0

/* Number of input ports */
#define NUM_INPUTS 8

/* Input port indices */
#define PORT_VOUT     0
#define PORT_IL       1
#define PORT_VREF     2
#define PORT_KP_U     3
#define PORT_KI_U     4
#define PORT_KP_I     5
#define PORT_KI_I     6
#define PORT_DEADTIME 7

/* PWork indices */
#define PWORK_PI_V  0
#define PWORK_PI_I  1

/* IWork indices */
#define IWORK_CNT   0

/* RWork indices */
#define RWORK_DUTY  0

/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    for (i = 0; i < NUM_INPUTS; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 1);

    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, 1);
    ssSetNumIWork(S, 1);
    ssSetNumPWork(S, 2);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 1.0 / SAMPLE_FREQ);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    void **pwork = ssGetPWork(S);
    PI_Ctrl *pi_v = (PI_Ctrl *)malloc(sizeof(PI_Ctrl));
    PI_Ctrl *pi_i = (PI_Ctrl *)malloc(sizeof(PI_Ctrl));

    PI_Init(pi_v, IREF_MIN, IREF_MAX);
    PI_Init(pi_i, DUTY_MIN, DUTY_MAX);

    pwork[PWORK_PI_V] = pi_v;
    pwork[PWORK_PI_I] = pi_i;

    ssGetIWork(S)[IWORK_CNT] = 0;
    ssGetRWork(S)[RWORK_DUTY] = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *Vout     = ssGetInputPortRealSignal(S, PORT_VOUT);
    const real_T *IL       = ssGetInputPortRealSignal(S, PORT_IL);
    const real_T *Vref     = ssGetInputPortRealSignal(S, PORT_VREF);
    const real_T *Kp_u     = ssGetInputPortRealSignal(S, PORT_KP_U);
    const real_T *Ki_u     = ssGetInputPortRealSignal(S, PORT_KI_U);
    const real_T *Kp_i     = ssGetInputPortRealSignal(S, PORT_KP_I);
    const real_T *Ki_i     = ssGetInputPortRealSignal(S, PORT_KI_I);
    const real_T *deadtime = ssGetInputPortRealSignal(S, PORT_DEADTIME);
    real_T *pwma = ssGetOutputPortRealSignal(S, 0);
    real_T *pwmb = ssGetOutputPortRealSignal(S, 1);
    int_T *iwork = ssGetIWork(S);
    real_T *rwork = ssGetRWork(S);
    void **pwork = ssGetPWork(S);
    PI_Ctrl *pi_v = (PI_Ctrl *)pwork[PWORK_PI_V];
    PI_Ctrl *pi_i = (PI_Ctrl *)pwork[PWORK_PI_I];
    int_T cnt, dt_steps;
    real_T triangle, duty, ev, Iref, ei;
    real_T duty_upper, duty_lower;

    cnt = iwork[IWORK_CNT];

    if (cnt == 0) {
        ev   = Vref[0] - Vout[0];
        Iref = PI_Update(pi_v, Kp_u[0], Ki_u[0], ev);
        ei   = Iref - IL[0];
        rwork[RWORK_DUTY] = PI_Update(pi_i, Kp_i[0], Ki_i[0], ei);
    }

    duty = rwork[RWORK_DUTY];

    /* Dead time in steps */
    dt_steps = (int_T)(deadtime[0] * SAMPLE_FREQ);

    /* Effective duty thresholds with dead time offset (in normalized carrier units) */
    duty_upper = duty - (real_T)dt_steps / (real_T)HALF_PERIOD;
    duty_lower = duty + (real_T)dt_steps / (real_T)HALF_PERIOD;
    if (duty_upper < 0.0) duty_upper = 0.0;
    if (duty_lower > 1.0) duty_lower = 1.0;

    /* Triangle carrier */
    if (cnt < HALF_PERIOD) {
        triangle = (real_T)cnt / (real_T)HALF_PERIOD;
    } else {
        triangle = (real_T)(STEPS_PER_PWM - cnt) / (real_T)HALF_PERIOD;
    }

    /* PWMA: on when duty > triangle (shrunk inward by dead time) */
    pwma[0] = (duty_upper >= triangle) ? 1.0 : 0.0;

    /* PWMB: complementary, on when triangle > duty (shrunk inward by dead time) */
    pwmb[0] = (triangle >= duty_lower) ? 1.0 : 0.0;

    cnt++;
    if (cnt >= STEPS_PER_PWM) {
        cnt = 0;
    }
    iwork[IWORK_CNT] = cnt;

    UNUSED_ARG(tid);
}

static void mdlTerminate(SimStruct *S)
{
    void **pwork = ssGetPWork(S);
    if (pwork[PWORK_PI_V] != NULL) {
        free(pwork[PWORK_PI_V]);
        pwork[PWORK_PI_V] = NULL;
    }
    if (pwork[PWORK_PI_I] != NULL) {
        free(pwork[PWORK_PI_I]);
        pwork[PWORK_PI_I] = NULL;
    }
}

/*=============================*
 * Required S-function trailer *
 *=============================*/
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
