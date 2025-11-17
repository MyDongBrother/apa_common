#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlarfg.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xzlarfg.h"
#include "SUKF.h"
#include "xnrm2.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                float *alpha1
 *                float x[3]
 * Return Type  : float
 */
float xzlarfg(int n, float *alpha1, float x[3])
{
    float tau;
    float xnorm;
    int knt;
    int k;
    tau = 0.0F;
    if (!(n <= 0))
    {
        xnorm = d_xnrm2(n - 1, x);
        if (xnorm != 0.0F)
        {
            xnorm = rt_hypotf(*alpha1, xnorm);
            if (*alpha1 >= 0.0F)
            {
                xnorm = -xnorm;
            }

            if (fabsf(xnorm) < 9.86076132E-32F)
            {
                knt = 0;
                do
                {
                    knt++;
                    for (k = 1; k + 1 <= n; k++)
                    {
                        x[k] *= 1.01412048E+31F;
                    }

                    xnorm *= 1.01412048E+31F;
                    *alpha1 *= 1.01412048E+31F;
                } while (!(fabsf(xnorm) >= 9.86076132E-32F));

                xnorm = rt_hypotf(*alpha1, d_xnrm2(n - 1, x));
                if (*alpha1 >= 0.0F)
                {
                    xnorm = -xnorm;
                }

                tau     = (xnorm - *alpha1) / xnorm;
                *alpha1 = 1.0F / (*alpha1 - xnorm);
                for (k = 1; k + 1 <= n; k++)
                {
                    x[k] *= *alpha1;
                }

                for (k = 1; k <= knt; k++)
                {
                    xnorm *= 9.86076132E-32F;
                }

                *alpha1 = xnorm;
            }
            else
            {
                tau     = (xnorm - *alpha1) / xnorm;
                *alpha1 = 1.0F / (*alpha1 - xnorm);
                for (k = 1; k + 1 <= n; k++)
                {
                    x[k] *= *alpha1;
                }

                *alpha1 = xnorm;
            }
        }
    }

    return tau;
}

/*
 * File trailer for xzlarfg.c
 *
 * [EOF]
 */
