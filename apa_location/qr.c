#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: qr.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "qr.h"
#include "xger.h"
#include "xgemv.h"
#include "SUKF.h"
#include "xnrm2.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : float A[20]
 *                float R[4]
 * Return Type  : void
 */
void b_qr(float A[20], float R[4])
{
    int i;
    float work[2];
    int i_i;
    int knt;
    float atmp;
    float f1;
    float xnorm;
    int iaii;
    float tau[2];
    int ia;
    int lastv;
    boolean_T exitg2;
    int exitg1;
    for (i = 0; i < 2; i++)
    {
        work[i] = 0.0F;
    }

    for (i = 0; i < 2; i++)
    {
        i_i   = i + i * 10;
        atmp  = A[i_i];
        f1    = 0.0F;
        xnorm = b_xnrm2(9 - i, A, i_i + 2);
        if (xnorm != 0.0F)
        {
            xnorm = rt_hypotf(A[i_i], xnorm);
            if (A[i_i] >= 0.0F)
            {
                xnorm = -xnorm;
            }

            if (fabsf(xnorm) < 9.86076132E-32F)
            {
                knt = 0;
                ia  = (i_i - i) + 10;
                do
                {
                    knt++;
                    for (lastv = i_i + 1; lastv + 1 <= ia; lastv++)
                    {
                        A[lastv] *= 1.01412048E+31F;
                    }

                    xnorm *= 1.01412048E+31F;
                    atmp *= 1.01412048E+31F;
                } while (!(fabsf(xnorm) >= 9.86076132E-32F));

                xnorm = rt_hypotf(atmp, b_xnrm2(9 - i, A, i_i + 2));
                if (atmp >= 0.0F)
                {
                    xnorm = -xnorm;
                }

                f1   = (xnorm - atmp) / xnorm;
                atmp = 1.0F / (atmp - xnorm);
                ia   = (i_i - i) + 10;
                for (lastv = i_i + 1; lastv + 1 <= ia; lastv++)
                {
                    A[lastv] *= atmp;
                }

                for (lastv = 1; lastv <= knt; lastv++)
                {
                    xnorm *= 9.86076132E-32F;
                }

                atmp = xnorm;
            }
            else
            {
                f1   = (xnorm - A[i_i]) / xnorm;
                atmp = 1.0F / (A[i_i] - xnorm);
                ia   = (i_i - i) + 10;
                for (lastv = i_i + 1; lastv + 1 <= ia; lastv++)
                {
                    A[lastv] *= atmp;
                }

                atmp = xnorm;
            }
        }

        tau[i] = f1;
        A[i_i] = atmp;
        if (i + 1 < 2)
        {
            atmp   = A[i_i];
            A[i_i] = 1.0F;
            if (tau[0] != 0.0F)
            {
                lastv = 10;
                knt   = i_i + 9;
                while ((lastv > 0) && (A[knt] == 0.0F))
                {
                    lastv--;
                    knt--;
                }

                knt    = 1;
                exitg2 = false;
                while ((!exitg2) && (knt > 0))
                {
                    ia = 11;
                    do
                    {
                        exitg1 = 0;
                        if (ia <= lastv + 10)
                        {
                            if (A[ia - 1] != 0.0F)
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
                            knt    = 0;
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
                knt   = 0;
            }

            if (lastv > 0)
            {
                b_xgemv(lastv, knt, A, 11, A, i_i + 1, work);
                b_xger(lastv, knt, -tau[0], i_i + 1, work, A, 11);
            }

            A[i_i] = atmp;
        }
    }

    i_i = 1;
    for (knt = 0; knt < 2; knt++)
    {
        for (i = 0; i + 1 <= knt + 1; i++)
        {
            R[i + (knt << 1)] = A[i + 10 * knt];
        }

        if (knt + 2 < 3)
        {
            R[1 + (knt << 1)] = 0.0F;
        }

        work[knt] = 0.0F;
    }

    for (i = 1; i >= 0; i--)
    {
        iaii = (i + i * 10) + 11;
        if (i + 1 < 2)
        {
            A[iaii - 11] = 1.0F;
            if (tau[i_i] != 0.0F)
            {
                lastv = 10;
                knt   = iaii;
                while ((lastv > 0) && (A[knt - 2] == 0.0F))
                {
                    lastv--;
                    knt--;
                }

                knt    = 1;
                exitg2 = false;
                while ((!exitg2) && (knt > 0))
                {
                    ia = iaii;
                    do
                    {
                        exitg1 = 0;
                        if (ia <= (iaii + lastv) - 1)
                        {
                            if (A[ia - 1] != 0.0F)
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
                            knt    = 0;
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
                knt   = 0;
            }

            if (lastv > 0)
            {
                b_xgemv(lastv, knt, A, iaii, A, iaii - 10, work);
                b_xger(lastv, knt, -tau[i_i], iaii - 10, work, A, iaii);
            }
        }

        ia = (iaii - i) - 1;
        for (lastv = iaii - 10; lastv + 1 <= ia; lastv++)
        {
            A[lastv] *= -tau[i_i];
        }

        A[iaii - 11] = 1.0F - tau[i_i];
        knt          = 1;
        while (knt <= i)
        {
            A[iaii - 12] = 0.0F;
            knt          = 2;
        }

        i_i--;
    }
}

/*
 * Arguments    : float A[48]
 *                float R[16]
 * Return Type  : void
 */
void qr(float A[48], float R[16])
{
    int i;
    float work[4];
    int i_i;
    int knt;
    float atmp;
    float f0;
    float xnorm;
    int iaii;
    float tau[4];
    int i_ip1;
    int coltop;
    int lastv;
    boolean_T exitg2;
    int ia;
    int exitg1;
    for (i = 0; i < 4; i++)
    {
        work[i] = 0.0F;
    }

    for (i = 0; i < 4; i++)
    {
        i_i   = i + i * 12;
        atmp  = A[i_i];
        f0    = 0.0F;
        xnorm = xnrm2(11 - i, A, i_i + 2);
        if (xnorm != 0.0F)
        {
            xnorm = rt_hypotf(A[i_i], xnorm);
            if (A[i_i] >= 0.0F)
            {
                xnorm = -xnorm;
            }

            if (fabsf(xnorm) < 9.86076132E-32F)
            {
                knt   = 0;
                i_ip1 = (i_i - i) + 12;
                do
                {
                    knt++;
                    for (coltop = i_i + 1; coltop + 1 <= i_ip1; coltop++)
                    {
                        A[coltop] *= 1.01412048E+31F;
                    }

                    xnorm *= 1.01412048E+31F;
                    atmp *= 1.01412048E+31F;
                } while (!(fabsf(xnorm) >= 9.86076132E-32F));

                xnorm = rt_hypotf(atmp, xnrm2(11 - i, A, i_i + 2));
                if (atmp >= 0.0F)
                {
                    xnorm = -xnorm;
                }

                f0    = (xnorm - atmp) / xnorm;
                atmp  = 1.0F / (atmp - xnorm);
                i_ip1 = (i_i - i) + 12;
                for (coltop = i_i + 1; coltop + 1 <= i_ip1; coltop++)
                {
                    A[coltop] *= atmp;
                }

                for (coltop = 1; coltop <= knt; coltop++)
                {
                    xnorm *= 9.86076132E-32F;
                }

                atmp = xnorm;
            }
            else
            {
                f0    = (xnorm - A[i_i]) / xnorm;
                atmp  = 1.0F / (A[i_i] - xnorm);
                i_ip1 = (i_i - i) + 12;
                for (coltop = i_i + 1; coltop + 1 <= i_ip1; coltop++)
                {
                    A[coltop] *= atmp;
                }

                atmp = xnorm;
            }
        }

        tau[i] = f0;
        A[i_i] = atmp;
        if (i + 1 < 4)
        {
            atmp   = A[i_i];
            A[i_i] = 1.0F;
            i_ip1  = (i + (i + 1) * 12) + 1;
            if (tau[i] != 0.0F)
            {
                lastv = 12 - i;
                knt   = (i_i - i) + 11;
                while ((lastv > 0) && (A[knt] == 0.0F))
                {
                    lastv--;
                    knt--;
                }

                knt    = 3 - i;
                exitg2 = false;
                while ((!exitg2) && (knt > 0))
                {
                    coltop = i_ip1 + (knt - 1) * 12;
                    ia     = coltop;
                    do
                    {
                        exitg1 = 0;
                        if (ia <= (coltop + lastv) - 1)
                        {
                            if (A[ia - 1] != 0.0F)
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
                            knt--;
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
                knt   = 0;
            }

            if (lastv > 0)
            {
                xgemv(lastv, knt, A, i_ip1, A, i_i + 1, work);
                xger(lastv, knt, -tau[i], i_i + 1, work, A, i_ip1);
            }

            A[i_i] = atmp;
        }
    }

    i_i = 3;
    for (knt = 0; knt < 4; knt++)
    {
        for (i = 0; i + 1 <= knt + 1; i++)
        {
            R[i + (knt << 2)] = A[i + 12 * knt];
        }

        for (i = knt + 1; i + 1 < 5; i++)
        {
            R[i + (knt << 2)] = 0.0F;
        }

        work[knt] = 0.0F;
    }

    for (i = 3; i >= 0; i--)
    {
        iaii = (i + i * 12) + 1;
        if (i + 1 < 4)
        {
            A[iaii - 1] = 1.0F;
            if (tau[i_i] != 0.0F)
            {
                lastv = 12 - i;
                knt   = (iaii - i) + 10;
                while ((lastv > 0) && (A[knt] == 0.0F))
                {
                    lastv--;
                    knt--;
                }

                knt    = 3 - i;
                exitg2 = false;
                while ((!exitg2) && (knt > 0))
                {
                    coltop = (iaii + (knt - 1) * 12) + 11;
                    ia     = coltop;
                    do
                    {
                        exitg1 = 0;
                        if (ia + 1 <= coltop + lastv)
                        {
                            if (A[ia] != 0.0F)
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
                            knt--;
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
                knt   = 0;
            }

            if (lastv > 0)
            {
                xgemv(lastv, knt, A, iaii + 12, A, iaii, work);
                xger(lastv, knt, -tau[i_i], iaii, work, A, iaii + 12);
            }
        }

        i_ip1 = (iaii - i) + 11;
        for (coltop = iaii; coltop + 1 <= i_ip1; coltop++)
        {
            A[coltop] *= -tau[i_i];
        }

        A[iaii - 1] = 1.0F - tau[i_i];
        for (knt = 1; knt <= i; knt++)
        {
            A[(iaii - knt) - 1] = 0.0F;
        }

        i_i--;
    }
}

/*
 * File trailer for qr.c
 *
 * [EOF]
 */
