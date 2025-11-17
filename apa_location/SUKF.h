/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SUKF.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef SUKF_H
#define SUKF_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void SUKF(float z[2], float wz, float v, float t, const float q[2],
                 const float r[2], unsigned char measurement_valid, unsigned int reset,
                 unsigned char set_origin_heading, float psi0,
                 unsigned char reset_UKF_LC_origin, unsigned char car_moving, float y[4],
                 float Pdiag[4]);
extern void b_P_not_empty_init(void);
extern void b_Xe_not_empty_init(void);

#endif

/*
 * File trailer for SUKF.h
 *
 * [EOF]
 */
