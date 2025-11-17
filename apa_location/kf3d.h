/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kf3d.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef KF3D_H
#define KF3D_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void d_P_not_empty_init(void);
extern void d_Xe_not_empty_init(void);
extern void kf3d(float veh_speed_motor, float t, float q, const float r[2], float ds,
                 unsigned int reset, float y[3]);
extern void kf3d_init(void);

#endif

/*
 * File trailer for kf3d.h
 *
 * [EOF]
 */
