/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xger.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

#ifndef XGER_H
#define XGER_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void b_xger(int m, int n, float alpha1, int ix0, const float y[2], float A[20],
                   int ia0);
extern void xger(int m, int n, float alpha1, int ix0, const float y[4], float A[48],
                 int ia0);

#endif

/*
 * File trailer for xger.h
 *
 * [EOF]
 */
