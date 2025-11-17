/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mass_estimate.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 09-Nov-2018 11:13:15
 */

#ifndef MASS_ESTIMATE_H
#define MASS_ESTIMATE_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"

/* Function Declarations */
extern void accel_array_not_empty_init(void);
extern void mass_estimate(float R, float r, float accel, float pitch, float torq,
                          float reset, float y[2]);
extern void mass_estimate_init(void);

#endif

/*
 * File trailer for mass_estimate.h
 *
 * [EOF]
 */
