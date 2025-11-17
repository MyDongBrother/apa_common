#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xgemv.h"

/* Function Definitions */

/*
 * Arguments    : int m
 *                int n
 *                const float A[20]
 *                int ia0
 *                const float x[20]
 *                int ix0
 *                float y[2]
 * Return Type  : void
 */
void b_xgemv(int m, int n, const float A[20], int ia0, const float x[20], int ix0,
             float y[2])
{
    int iy;
    int i10;
    int iac;
    int ix;
    float c;
    int i11;
    int ia;
    if ((m == 0) || (n == 0))
    {
    }
    else
    {
        for (iy = 1; iy <= n; iy++)
        {
            y[iy - 1] = 0.0F;
        }

        iy  = 0;
        i10 = ia0 + 10 * (n - 1);
        for (iac = ia0; iac <= i10; iac += 10)
        {
            ix  = ix0;
            c   = 0.0F;
            i11 = (iac + m) - 1;
            for (ia = iac; ia <= i11; ia++)
            {
                c += A[ia - 1] * x[ix - 1];
                ix++;
            }

            y[iy] += c;
            iy++;
        }
    }
}

/*
 * Arguments    : int m
 *                int n
 *                const float A[48]
 *                int ia0
 *                const float x[48]
 *                int ix0
 *                float y[4]
 * Return Type  : void
 */
void xgemv(int m, int n, const float A[48], int ia0, const float x[48], int ix0,
           float y[4])
{
    int iy;
    int i7;
    int iac;
    int ix;
    float c;
    int i8;
    int ia;
    if ((m == 0) || (n == 0))
    {
    }
    else
    {
        for (iy = 1; iy <= n; iy++)
        {
            y[iy - 1] = 0.0F;
        }

        iy = 0;
        i7 = ia0 + 12 * (n - 1);
        for (iac = ia0; iac <= i7; iac += 12)
        {
            ix = ix0;
            c  = 0.0F;
            i8 = (iac + m) - 1;
            for (ia = iac; ia <= i8; ia++)
            {
                c += A[ia - 1] * x[ix - 1];
                ix++;
            }

            y[iy] += c;
            iy++;
        }
    }
}

/*
 * File trailer for xgemv.c
 *
 * [EOF]
 */
