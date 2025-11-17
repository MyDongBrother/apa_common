/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: EKF_LC_PROCESS.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef EKF_LC_PROCESS_H
#define EKF_LC_PROCESS_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "PK_LocationImpl.h"
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void b_EKF_LC_PROCESS(EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg);
extern void EKF_LC_PROCESS(const EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg,
                           EKF_LC_PROCESS_arg_Type *b_EKF_LC_PROCESS_arg);
extern void EKF_LC_PROCESS_init(void);
extern void buffer_z_not_empty_init(void);
extern void c_deming_window_count_not_empty(void);

#endif

/*
 * File trailer for EKF_LC_PROCESS.h
 *
 * [EOF]
 */
