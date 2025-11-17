#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: combineVectorElements.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "combineVectorElements.h"

/* Function Definitions */

/*
 * Arguments    : const float x[30]
 * Return Type  : float
 */
float combineVectorElements(const float x[30])
{
    float y;
    int k;
    y = x[0];
    for (k = 0; k < 29; k++)
    {
        y += x[k + 1];
    }

    return y;
}

/*
 * File trailer for combineVectorElements.c
 *
 * [EOF]
 */
