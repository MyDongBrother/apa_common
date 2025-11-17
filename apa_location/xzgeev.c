#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgeev.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "xzgeev.h"
#include "xzlartg.h"
#include "xzhgeqz.h"
#include "SUKF.h"
#include "codegenall_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const float A[16]
 *                int *info
 *                creal32_T alpha1[4]
 *                creal32_T beta1[4]
 * Return Type  : void
 */
void xzgeev(const float A[16], int *info, creal32_T alpha1[4], creal32_T beta1[4])
{
    float anrm;
    int ii;
    boolean_T ilascl;
    creal32_T At[16];
    float anrmto;
    float absxk;
    int ilo;
    float ctoc;
    int ihi;
    boolean_T notdone;
    int exitg2;
    int i;
    float stemp_im;
    int j;
    float cto1;
    float mul;
    boolean_T exitg3;
    int nzcount;
    int jj;
    creal32_T atmp;
    boolean_T exitg4;
    int exitg1;
    anrm = 0.0F;
    for (ii = 0; ii < 16; ii++)
    {
        At[ii].re = A[ii];
        At[ii].im = 0.0F;
        absxk     = rt_hypotf(At[ii].re, At[ii].im);
        if (absxk > anrm)
        {
            anrm = absxk;
        }
    }

    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0F) && (anrm < 9.09494702E-13F))
    {
        anrmto = 9.09494702E-13F;
        ilascl = true;
    }
    else
    {
        if (anrm > 1.09951163E+12F)
        {
            anrmto = 1.09951163E+12F;
            ilascl = true;
        }
    }

    if (ilascl)
    {
        absxk   = anrm;
        ctoc    = anrmto;
        notdone = true;
        while (notdone)
        {
            stemp_im = absxk * 1.97215226E-31F;
            cto1     = ctoc / 5.0706024E+30F;
            if ((stemp_im > ctoc) && (ctoc != 0.0F))
            {
                mul   = 1.97215226E-31F;
                absxk = stemp_im;
            }
            else if (cto1 > absxk)
            {
                mul  = 5.0706024E+30F;
                ctoc = cto1;
            }
            else
            {
                mul     = ctoc / absxk;
                notdone = false;
            }

            for (ii = 0; ii < 16; ii++)
            {
                At[ii].re *= mul;
                At[ii].im *= mul;
            }
        }
    }

    ilo = 0;
    ihi = 4;
    do
    {
        exitg2  = 0;
        i       = 0;
        j       = 0;
        notdone = false;
        ii      = ihi;
        exitg3  = false;
        while ((!exitg3) && (ii > 0))
        {
            nzcount = 0;
            i       = ii;
            j       = ihi;
            jj      = 1;
            exitg4  = false;
            while ((!exitg4) && (jj <= ihi))
            {
                if ((At[(ii + ((jj - 1) << 2)) - 1].re != 0.0F) ||
                    (At[(ii + ((jj - 1) << 2)) - 1].im != 0.0F) || (ii == jj))
                {
                    if (nzcount == 0)
                    {
                        j       = jj;
                        nzcount = 1;
                        jj++;
                    }
                    else
                    {
                        nzcount = 2;
                        exitg4  = true;
                    }
                }
                else
                {
                    jj++;
                }
            }

            if (nzcount < 2)
            {
                notdone = true;
                exitg3  = true;
            }
            else
            {
                ii--;
            }
        }

        if (!notdone)
        {
            exitg2 = 2;
        }
        else
        {
            if (i != ihi)
            {
                for (ii = 0; ii < 4; ii++)
                {
                    atmp                      = At[(i + (ii << 2)) - 1];
                    At[(i + (ii << 2)) - 1]   = At[(ihi + (ii << 2)) - 1];
                    At[(ihi + (ii << 2)) - 1] = atmp;
                }
            }

            if (j != ihi)
            {
                for (ii = 0; ii + 1 <= ihi; ii++)
                {
                    atmp                      = At[ii + ((j - 1) << 2)];
                    At[ii + ((j - 1) << 2)]   = At[ii + ((ihi - 1) << 2)];
                    At[ii + ((ihi - 1) << 2)] = atmp;
                }
            }

            ihi--;
            if (ihi == 1)
            {
                exitg2 = 1;
            }
        }
    } while (exitg2 == 0);

    if (exitg2 == 1)
    {
    }
    else
    {
        do
        {
            exitg1  = 0;
            i       = 0;
            j       = 0;
            notdone = false;
            jj      = ilo + 1;
            exitg3  = false;
            while ((!exitg3) && (jj <= ihi))
            {
                nzcount = 0;
                i       = ihi;
                j       = jj;
                ii      = ilo + 1;
                exitg4  = false;
                while ((!exitg4) && (ii <= ihi))
                {
                    if ((At[(ii + ((jj - 1) << 2)) - 1].re != 0.0F) ||
                        (At[(ii + ((jj - 1) << 2)) - 1].im != 0.0F) || (ii == jj))
                    {
                        if (nzcount == 0)
                        {
                            i       = ii;
                            nzcount = 1;
                            ii++;
                        }
                        else
                        {
                            nzcount = 2;
                            exitg4  = true;
                        }
                    }
                    else
                    {
                        ii++;
                    }
                }

                if (nzcount < 2)
                {
                    notdone = true;
                    exitg3  = true;
                }
                else
                {
                    jj++;
                }
            }

            if (!notdone)
            {
                exitg1 = 1;
            }
            else
            {
                if (i != ilo + 1)
                {
                    for (ii = ilo; ii + 1 < 5; ii++)
                    {
                        atmp                    = At[(i + (ii << 2)) - 1];
                        At[(i + (ii << 2)) - 1] = At[ilo + (ii << 2)];
                        At[ilo + (ii << 2)]     = atmp;
                    }
                }

                if (j != ilo + 1)
                {
                    for (ii = 0; ii + 1 <= ihi; ii++)
                    {
                        atmp                    = At[ii + ((j - 1) << 2)];
                        At[ii + ((j - 1) << 2)] = At[ii + (ilo << 2)];
                        At[ii + (ilo << 2)]     = atmp;
                    }
                }

                ilo++;
                if (ilo + 1 == ihi)
                {
                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);
    }

    if (!(ihi < ilo + 3))
    {
        for (ii = ilo; ii + 1 < ihi - 1; ii++)
        {
            for (nzcount = ihi - 1; nzcount + 1 > ii + 2; nzcount--)
            {
                xzlartg(At[(nzcount + (ii << 2)) - 1], At[nzcount + (ii << 2)], &absxk,
                        &atmp, &At[(nzcount + (ii << 2)) - 1]);
                At[nzcount + (ii << 2)].re = 0.0F;
                At[nzcount + (ii << 2)].im = 0.0F;
                for (j = ii + 1; j + 1 < 5; j++)
                {
                    ctoc = absxk * At[(nzcount + (j << 2)) - 1].re +
                           (atmp.re * At[nzcount + (j << 2)].re -
                            atmp.im * At[nzcount + (j << 2)].im);
                    stemp_im = absxk * At[(nzcount + (j << 2)) - 1].im +
                               (atmp.re * At[nzcount + (j << 2)].im +
                                atmp.im * At[nzcount + (j << 2)].re);
                    cto1 = At[(nzcount + (j << 2)) - 1].re;
                    At[nzcount + (j << 2)].re =
                        absxk * At[nzcount + (j << 2)].re -
                        (atmp.re * At[(nzcount + (j << 2)) - 1].re +
                         atmp.im * At[(nzcount + (j << 2)) - 1].im);
                    At[nzcount + (j << 2)].im =
                        absxk * At[nzcount + (j << 2)].im -
                        (atmp.re * At[(nzcount + (j << 2)) - 1].im - atmp.im * cto1);
                    At[(nzcount + (j << 2)) - 1].re = ctoc;
                    At[(nzcount + (j << 2)) - 1].im = stemp_im;
                }

                atmp.re = -atmp.re;
                atmp.im = -atmp.im;
                for (i = 0; i + 1 <= ihi; i++)
                {
                    ctoc = absxk * At[i + (nzcount << 2)].re +
                           (atmp.re * At[i + ((nzcount - 1) << 2)].re -
                            atmp.im * At[i + ((nzcount - 1) << 2)].im);
                    stemp_im = absxk * At[i + (nzcount << 2)].im +
                               (atmp.re * At[i + ((nzcount - 1) << 2)].im +
                                atmp.im * At[i + ((nzcount - 1) << 2)].re);
                    cto1 = At[i + (nzcount << 2)].re;
                    At[i + ((nzcount - 1) << 2)].re =
                        absxk * At[i + ((nzcount - 1) << 2)].re -
                        (atmp.re * At[i + (nzcount << 2)].re +
                         atmp.im * At[i + (nzcount << 2)].im);
                    At[i + ((nzcount - 1) << 2)].im =
                        absxk * At[i + ((nzcount - 1) << 2)].im -
                        (atmp.re * At[i + (nzcount << 2)].im - atmp.im * cto1);
                    At[i + (nzcount << 2)].re = ctoc;
                    At[i + (nzcount << 2)].im = stemp_im;
                }
            }
        }
    }

    xzhgeqz(At, ilo + 1, ihi, info, alpha1, beta1);
    if ((*info == 0) && ilascl)
    {
        notdone = true;
        while (notdone)
        {
            stemp_im = anrmto * 1.97215226E-31F;
            cto1     = anrm / 5.0706024E+30F;
            if ((stemp_im > anrm) && (anrm != 0.0F))
            {
                mul    = 1.97215226E-31F;
                anrmto = stemp_im;
            }
            else if (cto1 > anrmto)
            {
                mul  = 5.0706024E+30F;
                anrm = cto1;
            }
            else
            {
                mul     = anrm / anrmto;
                notdone = false;
            }

            for (ii = 0; ii < 4; ii++)
            {
                alpha1[ii].re *= mul;
                alpha1[ii].im *= mul;
            }
        }
    }
}

/*
 * File trailer for xzgeev.c
 *
 * [EOF]
 */
