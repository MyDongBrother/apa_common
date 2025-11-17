#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlanhs.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "xzlanhs.h"

/* Function Definitions */

/*
 * Arguments    : const creal32_T A[16]
 *                int ilo
 *                int ihi
 * Return Type  : float
 */
float xzlanhs(const creal32_T A[16], int ilo, int ihi)
{
    float f;
    float scale;
    float sumsq;
    boolean_T firstNonZero;
    int j;
    int i5;
    int i;
    float reAij;
    float imAij;
    float temp2;
    f = 0.0F;
    if (!(ilo > ihi))
    {
        scale        = 0.0F;
        sumsq        = 0.0F;
        firstNonZero = true;
        for (j = ilo; j <= ihi; j++)
        {
            i5 = j + 1;
            if (ihi < j + 1)
            {
                i5 = ihi;
            }

            for (i = ilo; i <= i5; i++)
            {
                reAij = A[(i + ((j - 1) << 2)) - 1].re;
                imAij = A[(i + ((j - 1) << 2)) - 1].im;
                if (reAij != 0.0F)
                {
                    reAij = fabsf(reAij);
                    if (firstNonZero)
                    {
                        sumsq        = 1.0F;
                        scale        = reAij;
                        firstNonZero = false;
                    }
                    else if (scale < reAij)
                    {
                        temp2 = scale / reAij;
                        sumsq = 1.0F + sumsq * temp2 * temp2;
                        scale = reAij;
                    }
                    else
                    {
                        temp2 = reAij / scale;
                        sumsq += temp2 * temp2;
                    }
                }

                if (imAij != 0.0F)
                {
                    reAij = fabsf(imAij);
                    if (firstNonZero)
                    {
                        sumsq        = 1.0F;
                        scale        = reAij;
                        firstNonZero = false;
                    }
                    else if (scale < reAij)
                    {
                        temp2 = scale / reAij;
                        sumsq = 1.0F + sumsq * temp2 * temp2;
                        scale = reAij;
                    }
                    else
                    {
                        temp2 = reAij / scale;
                        sumsq += temp2 * temp2;
                    }
                }
            }
        }

        f = scale * sqrtf(sumsq);
    }

    return f;
}

/*
 * File trailer for xzlanhs.c
 *
 * [EOF]
 */
