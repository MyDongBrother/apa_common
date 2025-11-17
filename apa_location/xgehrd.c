#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgehrd.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "xgehrd.h"
#include "SUKF.h"
#include "xnrm2.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : float a[16]
 * Return Type  : void
 */
void xgehrd(float a[16])
{
    int i;
    float work[4];
    int im1n;
    int in;
    int ia0;
    float alpha1;
    float f2;
    float xnorm;
    float tau[3];
    int jy;
    int knt;
    int lastv;
    int i13;
    int lastc;
    int k;
    boolean_T exitg2;
    int ix;
    int ia;
    int exitg1;
    int i14;
    for (i = 0; i < 4; i++)
    {
        work[i] = 0.0F;
    }

    for (i = 0; i < 3; i++)
    {
        im1n = (i << 2) + 2;
        in   = (i + 1) << 2;
        ia0  = i + 3;
        if (ia0 < 4)
        {
            ia0 = 3;
        }
        else
        {
            ia0 = 4;
        }

        ia0 += i << 2;
        alpha1 = a[(i + (i << 2)) + 1];
        f2     = 0.0F;
        xnorm  = c_xnrm2(2 - i, a, ia0);
        if (xnorm != 0.0F)
        {
            xnorm = rt_hypotf(a[(i + (i << 2)) + 1], xnorm);
            if (a[(i + (i << 2)) + 1] >= 0.0F)
            {
                xnorm = -xnorm;
            }

            if (fabsf(xnorm) < 9.86076132E-32F)
            {
                knt = 0;
                i13 = (ia0 - i) + 1;
                do
                {
                    knt++;
                    for (k = ia0; k <= i13; k++)
                    {
                        a[k - 1] *= 1.01412048E+31F;
                    }

                    xnorm *= 1.01412048E+31F;
                    alpha1 *= 1.01412048E+31F;
                } while (!(fabsf(xnorm) >= 9.86076132E-32F));

                xnorm = rt_hypotf(alpha1, c_xnrm2(2 - i, a, ia0));
                if (alpha1 >= 0.0F)
                {
                    xnorm = -xnorm;
                }

                f2     = (xnorm - alpha1) / xnorm;
                alpha1 = 1.0F / (alpha1 - xnorm);
                i13    = (ia0 - i) + 1;
                while (ia0 <= i13)
                {
                    a[ia0 - 1] *= alpha1;
                    ia0++;
                }

                for (k = 1; k <= knt; k++)
                {
                    xnorm *= 9.86076132E-32F;
                }

                alpha1 = xnorm;
            }
            else
            {
                f2     = (xnorm - a[(i + (i << 2)) + 1]) / xnorm;
                alpha1 = 1.0F / (a[(i + (i << 2)) + 1] - xnorm);
                i13    = (ia0 - i) + 1;
                while (ia0 <= i13)
                {
                    a[ia0 - 1] *= alpha1;
                    ia0++;
                }

                alpha1 = xnorm;
            }
        }

        tau[i]                = f2;
        a[(i + (i << 2)) + 1] = 1.0F;
        jy                    = (i + im1n) - 1;
        if (tau[i] != 0.0F)
        {
            lastv = 3 - i;
            ia0   = (jy - i) + 2;
            while ((lastv > 0) && (a[ia0] == 0.0F))
            {
                lastv--;
                ia0--;
            }

            lastc  = 4;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0))
            {
                ia0 = in + lastc;
                ia  = ia0;
                do
                {
                    exitg1 = 0;
                    if (ia <= ia0 + ((lastv - 1) << 2))
                    {
                        if (a[ia - 1] != 0.0F)
                        {
                            exitg1 = 1;
                        }
                        else
                        {
                            ia += 4;
                        }
                    }
                    else
                    {
                        lastc--;
                        exitg1 = 2;
                    }
                } while (exitg1 == 0);

                if (exitg1 == 1)
                {
                    exitg2 = true;
                }
            }
        }
        else
        {
            lastv = 0;
            lastc = 0;
        }

        if (lastv > 0)
        {
            if (lastc != 0)
            {
                for (ia0 = 1; ia0 <= lastc; ia0++)
                {
                    work[ia0 - 1] = 0.0F;
                }

                ix  = jy;
                i13 = (in + ((lastv - 1) << 2)) + 1;
                for (k = in + 1; k <= i13; k += 4)
                {
                    ia0 = 0;
                    i14 = (k + lastc) - 1;
                    for (ia = k; ia <= i14; ia++)
                    {
                        work[ia0] += a[ia - 1] * a[ix];
                        ia0++;
                    }

                    ix++;
                }
            }

            if (!(-tau[i] == 0.0F))
            {
                ia0 = in;
                for (knt = 1; knt <= lastv; knt++)
                {
                    if (a[jy] != 0.0F)
                    {
                        xnorm = a[jy] * -tau[i];
                        ix    = 0;
                        i13   = lastc + ia0;
                        for (k = ia0; k + 1 <= i13; k++)
                        {
                            a[k] += work[ix] * xnorm;
                            ix++;
                        }
                    }

                    jy++;
                    ia0 += 4;
                }
            }
        }

        im1n += i;
        knt = (i + in) + 2;
        if (tau[i] != 0.0F)
        {
            lastv = 3 - i;
            ia0   = (im1n - i) + 1;
            while ((lastv > 0) && (a[ia0] == 0.0F))
            {
                lastv--;
                ia0--;
            }

            lastc  = 3 - i;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0))
            {
                ia0 = knt + ((lastc - 1) << 2);
                ia  = ia0;
                do
                {
                    exitg1 = 0;
                    if (ia <= (ia0 + lastv) - 1)
                    {
                        if (a[ia - 1] != 0.0F)
                        {
                            exitg1 = 1;
                        }
                        else
                        {
                            ia++;
                        }
                    }
                    else
                    {
                        lastc--;
                        exitg1 = 2;
                    }
                } while (exitg1 == 0);

                if (exitg1 == 1)
                {
                    exitg2 = true;
                }
            }
        }
        else
        {
            lastv = 0;
            lastc = 0;
        }

        if (lastv > 0)
        {
            if (lastc != 0)
            {
                for (ia0 = 1; ia0 <= lastc; ia0++)
                {
                    work[ia0 - 1] = 0.0F;
                }

                ia0 = 0;
                i13 = knt + ((lastc - 1) << 2);
                for (k = knt; k <= i13; k += 4)
                {
                    ix    = im1n;
                    xnorm = 0.0F;
                    i14   = (k + lastv) - 1;
                    for (ia = k; ia <= i14; ia++)
                    {
                        xnorm += a[ia - 1] * a[ix - 1];
                        ix++;
                    }

                    work[ia0] += xnorm;
                    ia0++;
                }
            }

            if (!(-tau[i] == 0.0F))
            {
                ia0 = knt - 1;
                jy  = 0;
                for (knt = 1; knt <= lastc; knt++)
                {
                    if (work[jy] != 0.0F)
                    {
                        xnorm = work[jy] * -tau[i];
                        ix    = im1n;
                        i13   = lastv + ia0;
                        for (k = ia0; k + 1 <= i13; k++)
                        {
                            a[k] += a[ix - 1] * xnorm;
                            ix++;
                        }
                    }

                    jy++;
                    ia0 += 4;
                }
            }
        }

        a[(i + (i << 2)) + 1] = alpha1;
    }
}

/*
 * File trailer for xgehrd.c
 *
 * [EOF]
 */
