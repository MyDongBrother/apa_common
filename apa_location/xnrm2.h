/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xnrm2.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef XNRM2_H
#define XNRM2_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"

/* Function Declarations */
extern float b_xnrm2(int n, const float x[20], int ix0);
extern float c_xnrm2(int n, const float x[16], int ix0);
extern float d_xnrm2(int n, const float x[3]);
extern float e_xnrm2(const float x[4]);
extern float xnrm2(int n, const float x[48], int ix0);

#endif

/*
 * File trailer for xnrm2.h
 *
 * [EOF]
 */
