/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern void b_xgemv(int m, int n, const float A[20], int ia0, const float x[20], int ix0,
                    float y[2]);
extern void xgemv(int m, int n, const float A[48], int ia0, const float x[48], int ix0,
                  float y[4]);

#endif

/*
 * File trailer for xgemv.h
 *
 * [EOF]
 */
