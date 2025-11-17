#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: codegenall_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "codegenall_initialize.h"
#include "kf3d.h"
#include "ekfEuler_atti.h"
#include "EKF_LC_PROCESS.h"
#include "EKF_LC.h"
#include "SUKF.h"
#include "mass_estimate.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void codegenall_initialize(void)
{
    d_Xe_not_empty_init();
    d_P_not_empty_init();
    c_Xe_not_empty_init();
    c_P_not_empty_init();
    b_P_not_empty_init();
    b_Xe_not_empty_init();
    Xe_not_empty_init();
    P_not_empty_init();
    c_deming_window_count_not_empty();
    buffer_z_not_empty_init();
    EKF_LC_PROCESS_init();
    ekfEuler_atti_init();
    kf3d_init();
    mass_estimate_init();
}

/*
 * File trailer for codegenall_initialize.c
 *
 * [EOF]
 */
