/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekfEuler_atti.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

#ifndef EKFEULER_ATTI_H
#define EKFEULER_ATTI_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "codegenall_types.h"
#include "PK_LocationImpl.h"

/* Function Declarations */
extern void c_P_not_empty_init(void);
extern void c_Xe_not_empty_init(void);
extern void b_ekfEuler_atti(ekfEuler_atti_arg_Type *ekfEuler_atti_arg);
extern void ekfEuler_atti(const ekfEuler_atti_arg_Type *ekfEuler_atti_arg,
                          ekfEuler_atti_arg_Type *b_ekfEuler_atti_arg);
extern void ekfEuler_atti_init(void);

#endif

/*
 * File trailer for ekfEuler_atti.h
 *
 * [EOF]
 */
