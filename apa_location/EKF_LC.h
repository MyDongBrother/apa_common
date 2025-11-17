/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: EKF_LC.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef EKF_LC_H
#define EKF_LC_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void EKF_LC(float z[2], float wz, float v, float t, const float q[2],
                   const float r[2], unsigned char measurement_valid, unsigned int reset,
                   unsigned char set_origin_heading, float psi0,
                   unsigned char reset_EKF_LC_origin, unsigned char car_moving,
                   float y[4], float Pdiag[4]);
extern void P_not_empty_init(void);
extern void Xe_not_empty_init(void);

#endif

/*
 * File trailer for EKF_LC.h
 *
 * [EOF]
 */
