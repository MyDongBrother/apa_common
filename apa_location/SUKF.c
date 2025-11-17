#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SUKF.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "SUKF.h"
#include "xdlanv2.h"
#include "xdhseqr.h"
#include "xgehrd.h"
#include "xzgeev.h"
#include "qr.h"
#include "diag.h"
#include "powerf.h"
#include "abs.h"
#include "codegenall_rtwutil.h"

extern void RTE_PK_Location_Get_CurPos(float pa[4]);

/* Variable Definitions */
static float b_Xe[4];
static boolean_T b_Xe_not_empty;
static float b_P[16];
static boolean_T b_P_not_empty;
static float P2[16];

/* Function Declarations */
static void measurementupdate(const float Xkk1[36], const float Rn[4], const float wm[9],
                              const float wc[9], float Ykk1[18], float Y_k[2],
                              float P_yk[4]);
static void timeupdate(const float X[36], const float e_Xe[4], float v, float wz, float t,
                       const float Rv[16], const float wc[9], unsigned char car_moving,
                       float Xkk1[36], float X_k[4], float P_xk[16]);

/* Function Definitions */

/*
 * function [Ykk1,Y_k,P_yk]=measurementupdate(Xkk1,Rn,nRn,wm,wc,Lsigma)
 * Arguments    : const float Xkk1[36]
 *                const float Rn[4]
 *                const float wm[9]
 *                const float wc[9]
 *                float Ykk1[18]
 *                float Y_k[2]
 *                float P_yk[4]
 * Return Type  : void
 */
static void measurementupdate(const float Xkk1[36], const float Rn[4], const float wm[9],
                              const float wc[9], float Ykk1[18], float Y_k[2],
                              float P_yk[4])
{
    int j;
    int i;
    float ajj;
    float Y_kex[16];
    float x[4];
    float auto_gen_tmp_0[20];
    float b_wc[2];
    float b_Ykk1[2];
    int info;
    float c_wc[4];
    boolean_T exitg1;
    int jj;
    int b_info;
    int ix;
    int iy;

    /* 'SUKF:197' Ykk1=zeros(nRn,Lsigma*2 + 1 ); */
    /* 'SUKF:198' for i=1 : Lsigma*2 + 1 */
    for (j = 0; j < 9; j++)
    {
        for (i = 0; i < 2; i++)
        {
            Ykk1[i + (j << 1)] = Xkk1[i + (j << 2)];
        }
    }

    /*  % % % % % % % % % % % % % % % % %  */
    /* 'SUKF:202' Y_k=wm(1)*Ykk1(:,1); */
    for (j = 0; j < 2; j++)
    {
        Y_k[j] = wm[0] * Ykk1[j];
    }

    /* 'SUKF:203' for i=2 : Lsigma*2+1 */
    for (i = 0; i < 8; i++)
    {
        /* 'SUKF:204' Y_k=Y_k+wm(i)*Ykk1(:,i); */
        for (j = 0; j < 2; j++)
        {
            Y_k[j] += wm[i + 1] * Ykk1[j + ((i + 1) << 1)];
        }
    }

    /* 'SUKF:206' Y_kex=zeros(nRn,2*Lsigma); */
    /* 'SUKF:207' for i=2:2*Lsigma+1 */
    for (i = 0; i < 8; i++)
    {
        /* 'SUKF:208' Y_kex(:,i-1)=Y_k; */
        for (j = 0; j < 2; j++)
        {
            Y_kex[j + (i << 1)] = Y_k[j];
        }
    }

    /*  try */
    /* 'SUKF:211' [Syyy,P_yk]=qr([sqrt(wc(2))*(Ykk1(:,2:end)-Y_kex)  sqrt(Rn)]',0); */
    ajj = sqrtf(wc[1]);
    for (i = 0; i < 4; i++)
    {
        x[i] = sqrtf(Rn[i]);
    }

    for (j = 0; j < 2; j++)
    {
        for (i = 0; i < 8; i++)
        {
            auto_gen_tmp_0[i + 10 * j] =
                ajj * (Ykk1[j + ((1 + i) << 1)] - Y_kex[j + (i << 1)]);
        }

        for (i = 0; i < 2; i++)
        {
            auto_gen_tmp_0[(i + 10 * j) + 8] = x[j + (i << 1)];
        }
    }

    b_qr(auto_gen_tmp_0, P_yk);

    /*  catch error */
    /*      error; */
    /*  end */
    /*  P_yk = real(P_yk); */
    /*  try */
    /* 'SUKF:217' P_yk=chol(P_yk'*P_yk+wc(1)*(Ykk1(:,1)-Y_k)*(Ykk1(:,1)-Y_k)','lower'); */
    for (j = 0; j < 2; j++)
    {
        b_wc[j]   = wc[0] * (Ykk1[j] - Y_k[j]);
        b_Ykk1[j] = Ykk1[j] - Y_k[j];
        for (i = 0; i < 2; i++)
        {
            x[j + (i << 1)] = 0.0F;
            for (info = 0; info < 2; info++)
            {
                x[j + (i << 1)] += P_yk[info + (j << 1)] * P_yk[info + (i << 1)];
            }
        }
    }

    for (j = 0; j < 2; j++)
    {
        for (i = 0; i < 2; i++)
        {
            c_wc[j + (i << 1)] = b_wc[j] * b_Ykk1[i];
        }
    }

    for (j = 0; j < 2; j++)
    {
        for (i = 0; i < 2; i++)
        {
            P_yk[i + (j << 1)] = x[i + (j << 1)] + c_wc[i + (j << 1)];
        }
    }

    info   = 0;
    j      = 0;
    exitg1 = false;
    while ((!exitg1) && (j + 1 < 3))
    {
        jj  = j + (j << 1);
        ajj = 0.0F;
        if (!(j < 1))
        {
            ix = j;
            iy = j;
            i  = 1;
            while (i <= j)
            {
                ajj += P_yk[ix] * P_yk[iy];
                ix += 2;
                iy += 2;
                i = 2;
            }
        }

        ajj = P_yk[jj] - ajj;
        if (ajj > 0.0F)
        {
            ajj      = sqrtf(ajj);
            P_yk[jj] = ajj;
            if (j + 1 < 2)
            {
                ajj = 1.0F / ajj;
                for (i = jj + 1; i + 1 <= jj + 2; i++)
                {
                    P_yk[i] *= ajj;
                }
            }

            j++;
        }
        else
        {
            P_yk[jj] = ajj;
            info     = j + 1;
            exitg1   = true;
        }
    }

    if (info == 0)
    {
        b_info = 2;
    }
    else
    {
        b_info = info - 1;
    }

    if (2 <= b_info)
    {
        P_yk[2] = 0.0F;
    }

    /*  catch error */
    /*      error; */
    /*  end */
}

/*
 * function [Xkk1,X_k,P_xk]=timeupdate(X,Xe,v,wz,t,Rv,wm,wc,nXe,Lsigma,car_moving)
 * Arguments    : const float X[36]
 *                const float e_Xe[4]
 *                float v
 *                float wz
 *                float t
 *                const float Rv[16]
 *                const float wc[9]
 *                unsigned char car_moving
 *                float Xkk1[36]
 *                float X_k[4]
 *                float P_xk[16]
 * Return Type  : void
 */
static void timeupdate(const float X[36], const float e_Xe[4], float v, float wz, float t,
                       const float Rv[16], const float wc[9], unsigned char car_moving,
                       float Xkk1[36], float X_k[4], float P_xk[16])
{
    int i;
    int i3;
    float x;
    int jmax;
    float X_kex[32];
    float b_x[16];
    float auto_gen_tmp_0[48];
    int i4;
    float c[4];
    float b_Xkk1[4];
    int info;
    float b_c[16];
    int j;
    boolean_T exitg1;
    int jj;
    int ix;
    float ajj;
    int iy;

    /* 'SUKF:167' Xkk1=zeros(nXe,Lsigma*2+1); */
    /* time update */
    /* 'SUKF:169' for i=1 :Lsigma*2+1 */
    for (i = 0; i < 9; i++)
    {
        /* 'SUKF:170' Xkk1(:,i)=f_update(X(:,i),v,wz,t,car_moving); */
        /* 'f_update:2' y = [ x(1,:) + t*v*cos(x(3,:) - t*(x(4,:)/2 - wz/2)) */
        /* 'f_update:3'  x(2,:) + t*v*sin(x(3,:) - t*(x(4,:)/2 - wz/2)) */
        /* 'f_update:4'                      x(3,:) - t*(x(4,:) - wz)*single(car_moving)
         */
        /* 'f_update:5' x(4,:)/200*single(car_moving)+x(4,:)*
         * 199/200+1/200*wz*(1-single(car_moving))]; */
        Xkk1[i << 2] =
            X[i << 2] +
            t * v * cosf(X[2 + (i << 2)] - t * (X[3 + (i << 2)] / 2.0F - wz / 2.0F));
        Xkk1[1 + (i << 2)] =
            X[1 + (i << 2)] +
            t * v * sinf(X[2 + (i << 2)] - t * (X[3 + (i << 2)] / 2.0F - wz / 2.0F));
        Xkk1[2 + (i << 2)] =
            X[2 + (i << 2)] - t * (X[3 + (i << 2)] - wz) * (float)car_moving;
        Xkk1[3 + (i << 2)] = (X[3 + (i << 2)] / 200.0F * (float)car_moving +
                              X[3 + (i << 2)] * 199.0F / 200.0F) +
                             0.005F * wz * (1.0F - (float)car_moving);
    }

    /* 'SUKF:173' X_k=wm(1)*Xkk1(:,1); */
    /* 'SUKF:174' for i=2 : Lsigma*2+1 */
    /* 'SUKF:177' X_k = f_update(Xe,v,wz,t,car_moving); */
    /* 'f_update:2' y = [ x(1,:) + t*v*cos(x(3,:) - t*(x(4,:)/2 - wz/2)) */
    /* 'f_update:3'  x(2,:) + t*v*sin(x(3,:) - t*(x(4,:)/2 - wz/2)) */
    /* 'f_update:4'                      x(3,:) - t*(x(4,:) - wz)*single(car_moving) */
    /* 'f_update:5' x(4,:)/200*single(car_moving)+x(4,:)*
     * 199/200+1/200*wz*(1-single(car_moving))]; */
    X_k[0] = e_Xe[0] + t * v * cosf(e_Xe[2] - t * (e_Xe[3] / 2.0F - wz / 2.0F));
    X_k[1] = e_Xe[1] + t * v * sinf(e_Xe[2] - t * (e_Xe[3] / 2.0F - wz / 2.0F));
    X_k[2] = e_Xe[2] - t * (e_Xe[3] - wz) * (float)car_moving;
    X_k[3] = (e_Xe[3] / 200.0F * (float)car_moving + e_Xe[3] * 199.0F / 200.0F) +
             0.005F * wz * (1.0F - (float)car_moving);

    /* !!!!!!!!!!! */
    /* 'SUKF:178' X_kex=zeros(nXe,2*Lsigma); */
    /* 'SUKF:179' for i=2:2*Lsigma+1 */
    for (i = 0; i < 8; i++)
    {
        /* 'SUKF:180' X_kex(:,i-1)=X_k; */
        for (i3 = 0; i3 < 4; i3++)
        {
            X_kex[i3 + (i << 2)] = X_k[i3];
        }
    }

    /*  try */
    /* 'SUKF:183' [~,P_xk]=qr([sqrt(wc(2))*(Xkk1(:,2:end)-X_kex) sqrt(Rv)]',0); */
    x = sqrtf(wc[1]);
    for (jmax = 0; jmax < 16; jmax++)
    {
        b_x[jmax] = sqrtf(Rv[jmax]);
    }

    for (i3 = 0; i3 < 4; i3++)
    {
        for (i4 = 0; i4 < 8; i4++)
        {
            auto_gen_tmp_0[i4 + 12 * i3] =
                x * (Xkk1[i3 + ((1 + i4) << 2)] - X_kex[i3 + (i4 << 2)]);
        }

        for (i4 = 0; i4 < 4; i4++)
        {
            auto_gen_tmp_0[(i4 + 12 * i3) + 8] = b_x[i3 + (i4 << 2)];
        }
    }

    qr(auto_gen_tmp_0, P_xk);

    /* 'SUKF:183' ~ */
    /* memory fading? */
    /*  catch error */
    /*      a = 1; */
    /*  end */
    /*  try */
    /*  P_xk = real(P_xk); */
    /* 'SUKF:189'
     * P_xk=chol(P_xk'*P_xk+sign(wc(1))*(abs(wc(1)))*(Xkk1(:,1)-X_k)*(Xkk1(:,1)-X_k)','lower');
     */
    x = wc[0];
    if (wc[0] < 0.0F)
    {
        x = -1.0F;
    }
    else
    {
        if (wc[0] > 0.0F)
        {
            x = 1.0F;
        }
    }

    x *= fabsf(wc[0]);
    for (i3 = 0; i3 < 4; i3++)
    {
        c[i3]      = x * (Xkk1[i3] - X_k[i3]);
        b_Xkk1[i3] = Xkk1[i3] - X_k[i3];
        for (i4 = 0; i4 < 4; i4++)
        {
            b_x[i3 + (i4 << 2)] = 0.0F;
            for (jmax = 0; jmax < 4; jmax++)
            {
                b_x[i3 + (i4 << 2)] += P_xk[jmax + (i3 << 2)] * P_xk[jmax + (i4 << 2)];
            }
        }
    }

    for (i3 = 0; i3 < 4; i3++)
    {
        for (i4 = 0; i4 < 4; i4++)
        {
            b_c[i3 + (i4 << 2)] = c[i3] * b_Xkk1[i4];
        }
    }

    for (i3 = 0; i3 < 4; i3++)
    {
        for (i4 = 0; i4 < 4; i4++)
        {
            P_xk[i4 + (i3 << 2)] = b_x[i4 + (i3 << 2)] + b_c[i4 + (i3 << 2)];
        }
    }

    info   = 0;
    j      = 0;
    exitg1 = false;
    while ((!exitg1) && (j + 1 < 5))
    {
        jj = j + (j << 2);
        x  = 0.0F;
        if (!(j < 1))
        {
            ix = j;
            iy = j;
            for (jmax = 1; jmax <= j; jmax++)
            {
                x += P_xk[ix] * P_xk[iy];
                ix += 4;
                iy += 4;
            }
        }

        ajj = P_xk[jj] - x;
        if (ajj > 0.0F)
        {
            ajj      = sqrtf(ajj);
            P_xk[jj] = ajj;
            if (j + 1 < 4)
            {
                if (j != 0)
                {
                    ix = j;
                    i3 = (j + ((j - 1) << 2)) + 2;
                    for (jmax = j + 2; jmax <= i3; jmax += 4)
                    {
                        x  = -P_xk[ix];
                        iy = jj + 1;
                        i4 = (jmax - j) + 2;
                        for (i = jmax; i <= i4; i++)
                        {
                            P_xk[iy] += P_xk[i - 1] * x;
                            iy++;
                        }

                        ix += 4;
                    }
                }

                x  = 1.0F / ajj;
                i3 = (jj - j) + 4;
                for (jmax = jj + 1; jmax + 1 <= i3; jmax++)
                {
                    P_xk[jmax] *= x;
                }
            }

            j++;
        }
        else
        {
            P_xk[jj] = ajj;
            info     = j + 1;
            exitg1   = true;
        }
    }

    if (info == 0)
    {
        jmax = 4;
    }
    else
    {
        jmax = info - 1;
    }

    for (j = 1; j + 1 <= jmax; j++)
    {
        for (i = 1; i <= j; i++)
        {
            P_xk[(i + (j << 2)) - 1] = 0.0F;
        }
    }

    /* it is upper trangle here. but it should be lower  */
    /*  cholupdate(R,x,'+') */
    /*  catch error */
    /*      a=1; */
    /*  end */
}

/*
 * function
 * [y,Pdiag]=SUKF(z,wz,v,dS,t,q,r,measurement_valid,reset,set_origin_heading,psi0,reset_UKF_LC_origin,
 * car_moving) square root ukf for parameter estimation Arguments    : float z[2] float wz
 *                float v
 *                float t
 *                const float q[2]
 *                const float r[2]
 *                unsigned char measurement_valid
 *                unsigned int reset
 *                unsigned char set_origin_heading
 *                float psi0
 *                unsigned char reset_UKF_LC_origin
 *                unsigned char car_moving
 *                float y[4]
 *                float Pdiag[4]
 * Return Type  : void
 */
void SUKF(float z[2], float wz, float v, float t, const float q[2], const float r[2],
          unsigned char measurement_valid, unsigned int reset,
          unsigned char set_origin_heading, float psi0, unsigned char reset_UKF_LC_origin,
          unsigned char car_moving, float y[4], float Pdiag[4])
{
    static const float fv2[16] = {
        1.0E-5F,         0.0F, 0.0F, 0.0F, 0.0F, 1.0E-5F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.000999999931F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-7F};

    int info;
    int j;
    boolean_T exitg1;
    int i;
    int jj;
    int jmax;
    float d;
    int ix;
    float ajj;
    int iy;
    int k;
    int i2;
    float c;
    float Rv12;
    float Rv13;
    float mu1_im;
    float Rv23;
    float wm[9];
    float X[36];
    float wc[9];
    float b_q[16];
    float fv3[16];
    float gamma_P[4];
    float Xkk1[36];
    float X_k[4];
    float P_xk[16];
    float fv4[2];
    float b_wc[4];
    float Ykk1[18];
    float Y_k[2];
    float P_yk[4];
    float b_Ykk1[2];
    float Pxy[8];
    float b_c[8];
    float KK[8];
    float b_Pxy[16];
    boolean_T p;
    creal32_T alpha1[4];
    creal32_T beta1[4];
    int exitg2;
    boolean_T x[4];
    creal32_T T[16];
    float rt1i;
    float mu1_re;
    float sn;

    /* 'SUKF:2' coder.inline('never') */
    /*  persistent v_buffer; */
    /* 'SUKF:7' nXe=4; */
    /*  */
    /* 'SUKF:8' nRn=2; */
    /*  */
    /* 'SUKF:9' nPsub1=nXe; */
    /* %%%%%%%%% */
    /* 'SUKF:10' nPsub2=nRn; */
    /* 'SUKF:11' nPsub=nPsub1+nPsub2; */
    /* 'SUKF:12' assert(nRn <= 200); */
    /* 'SUKF:13' assert(nPsub <= 200); */
    /* 'SUKF:15' if isempty(Xe) || reset */
    if (!b_Xe_not_empty)
    {
        /* 'SUKF:16' Xe=single([z(1) z(2) psi0 0]'); */
        RTE_PK_Location_Get_CurPos(b_Xe);
        b_Xe_not_empty = true;

        /* PT6远永和隧道-2018-3-14 */
        /* 'SUKF:17' Xe(4) = single(0); */
        b_Xe[3] = 0.0F;

        /*      v_buffer = zeros(1,4); */
    }
    if (reset != 0U)
    {
        /* 'SUKF:16' Xe=single([z(1) z(2) psi0 0]'); */
        b_Xe[0] = z[0];
        b_Xe[1] = z[1];
        b_Xe[2] = psi0;
        b_Xe[3] = 0.0F;
    }

    /* 'SUKF:20' moving = single(1); */
    /* if vehicle is moving */
    /* 'SUKF:22' sPsub=eye(nPsub,nPsub); */
    /* 'SUKF:23' if isempty(P) || reset */
    if ((!b_P_not_empty) || (reset != 0U))
    {
        /* 'SUKF:24' P=(diag([.1 .1 10 0.001]))*0.0001; */
        memcpy(&b_P[0], &fv2[0], sizeof(float) << 4);
        b_P_not_empty = true;

        /* 'SUKF:25' P=chol(P,'lower'); */
        info   = 0;
        j      = 0;
        exitg1 = false;
        while ((!exitg1) && (j + 1 < 5))
        {
            jj = j + (j << 2);
            d  = 0.0F;
            if (!(j < 1))
            {
                ix = j;
                iy = j;
                for (k = 1; k <= j; k++)
                {
                    d += b_P[ix] * b_P[iy];
                    ix += 4;
                    iy += 4;
                }
            }

            ajj = b_P[jj] - d;
            if (ajj > 0.0F)
            {
                ajj     = sqrtf(ajj);
                b_P[jj] = ajj;
                if (j + 1 < 4)
                {
                    if (j != 0)
                    {
                        ix = j;
                        i2 = (j + ((j - 1) << 2)) + 2;
                        for (jmax = j + 2; jmax <= i2; jmax += 4)
                        {
                            c  = -b_P[ix];
                            iy = jj + 1;
                            k  = (jmax - j) + 2;
                            for (i = jmax; i <= k; i++)
                            {
                                b_P[iy] += b_P[i - 1] * c;
                                iy++;
                            }

                            ix += 4;
                        }
                    }

                    ajj = 1.0F / ajj;
                    i2  = (jj - j) + 4;
                    for (k = jj + 1; k + 1 <= i2; k++)
                    {
                        b_P[k] *= ajj;
                    }
                }

                j++;
            }
            else
            {
                b_P[jj] = ajj;
                info    = j + 1;
                exitg1  = true;
            }
        }

        if (info == 0)
        {
            jmax = 4;
        }
        else
        {
            jmax = info - 1;
        }

        for (j = 1; j + 1 <= jmax; j++)
        {
            for (i = 1; i <= j; i++)
            {
                b_P[(i + (j << 2)) - 1] = 0.0F;
            }
        }

        /* 'SUKF:26' P2 = P; */
        memcpy(&P2[0], &b_P[0], sizeof(float) << 4);
    }

    /* 'SUKF:28' if set_origin_heading == 1 */
    if (set_origin_heading == 1)
    {
        /* 'SUKF:29' Xe=single([z(1) z(2) psi0 0]'); */
        b_Xe[0] = z[0];
        b_Xe[1] = z[1];
        b_Xe[2] = psi0;
        b_Xe[3] = 0.0F;
    }

    /* 'SUKF:31' if reset_UKF_LC_origin == 1 */
    if (reset_UKF_LC_origin == 1)
    {
        /* 'SUKF:32' Xe(1:2) = z; */
        for (i = 0; i < 2; i++)
        {
            b_Xe[i] = z[i];
        }
    }

    /*  v_buffer(1:end-1) = v_buffer(2:end); */
    /*  v_buffer(end) = v; */
    /*  if abs(sum(abs(v_buffer)) -
     * max(v_buffer))<1e-4%判断为静止时不相信GPS测量!!!!!!!!!!!!重要 */
    /*  % if (min(abs(v_buffer)))==0%速度小时不相信GPS测量!!!!!!!!!!!!重要 */
    /*  %     measurement_valid = uint8(0); */
    /*  %     Xe(4) = Xe(4) * 199/200+1/200*wz; */
    /*      moving = single(0);%!!!!!!!为什么导致不准确的结果 */
    /*      z(1) = Xe(1); */
    /*      z(2) = Xe(2); */
    /*      dS = 0; */
    /*      % % %     Xe(4) = Xe(4);!!!!!!!!! */
    /*      % else */
    /*      measurement_valid = 1; */
    /*  end */
    /*  if abs(v)<0.00001 */
    /*      car_moving = uint8(0); */
    /*  else */
    /*      car_moving = uint8(1); */
    /*  end */
    /*  car_moving = car_moving_state(dS,t); */
    /* 'SUKF:55' if car_moving == 0 */
    if (car_moving == 0)
    {
        /* 'SUKF:56' z(1) = Xe(1); */
        z[0] = b_Xe[0];

        /* 'SUKF:57' z(2) = Xe(2); */
        z[1] = b_Xe[1];

        /*      Xe(4) = Xe(4) * 199/200+1/200*wz; */
        /* 'SUKF:59' dS = 0; */
        /* 'SUKF:60' measurement_valid = uint8(0); */
        measurement_valid = 0;
    }

    /*  v = dS / t;%added 2018-4-18 */
    /*  % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  */
    /* 'SUKF:65' alpha =0.5; */
    /* alpha =0.05;I found that alpha has a large influence on the result,in this case,
     * the smaller the better */
    /* 'SUKF:67' beta =2; */
    /* 'SUKF:68' Lsigma =nXe; */
    /* +nPsub; */
    /* 'SUKF:69' kappa =0; */
    /* 'SUKF:70' lambada =alpha*alpha*(Lsigma+kappa)-Lsigma; */
    /* 'SUKF:71' sqrt_gamma =alpha*(Lsigma+kappa)^0.5; */
    /* 'SUKF:73' Ym=z; */
    /* 'SUKF:75' q1 = q(1); */
    /* 'SUKF:76' q2 = q(2); */
    /*  Rv=([                              (q1^2*t^4*v^2*sin(Xe(3) - t*(Xe(4)/2 -
     * wz/2))^2)/4, -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) - t*(Xe(4)/2
     * - wz/2)))/4, -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2,        0 */
    /*   -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) - t*(Xe(4)/2 -
     * wz/2)))/4,                              (q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 -
     * wz/2))^2)/4,  (q1^2*t^3*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)))/2,        0 */
    /*                                   -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2,
     * (q1^2*t^3*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)))/2, q1^2*t^2,        0 */
    /*                                                                               0, 0,
     * 0, q2^2*t^2]); */
    /* 'SUKF:82' Rv11 = (q1^2*t^4*v^2*sin(Xe(3) - t*(Xe(4)/2 - wz/2))^2)/4; */
    ajj = sinf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F));

    /* 'SUKF:83' Rv12 = -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) -
     * t*(Xe(4)/2 - wz/2)))/4; */
    Rv12 = -(q[0] * q[0] * powf(t, 4.0F) * (v * v) *
             cosf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F)) *
             sinf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F))) /
           4.0F;

    /* 'SUKF:84' Rv13 = -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2; */
    Rv13 = -(q[0] * q[0] * powf(t, 3.0F) * v *
             sinf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F))) /
           2.0F;

    /* 'SUKF:85' Rv14 = 0; */
    /* 'SUKF:86' Rv22 = (q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))^2)/4; */
    mu1_im = cosf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F));

    /* 'SUKF:87' Rv23 = (q1^2*t^3*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)))/2; */
    Rv23 = q[0] * q[0] * powf(t, 3.0F) * v *
           cosf(b_Xe[2] - t * (b_Xe[3] / 2.0F - wz / 2.0F)) / 2.0F;

    /* 'SUKF:88' Rv24 = 0; */
    /* 'SUKF:89' Rv33 = q1^2*t^2; */
    /* 'SUKF:90' Rv34 = 0; */
    /* 'SUKF:91' Rv44 =  q2^2*t^2; */
    /* 'SUKF:92' Rv=single([   Rv11, Rv12, Rv13, Rv14 */
    /* 'SUKF:93'              Rv12,   Rv22,  Rv23, Rv24 */
    /* 'SUKF:94'              Rv13,   Rv23,  Rv33, Rv34          */
    /* 'SUKF:95'              Rv14,   Rv24,  Rv34, Rv44                    */
    /* 'SUKF:96' ]); */
    /*  Rv=single([   (t^3*v^2*sin(Xe(3))^2*(20*q1^2 + 3*q2^2*t^2))/60,
     * -(t^3*v^2*sin(2*Xe(3))*(20*q1^2 + 3*q2^2*t^2))/120, -(t^2*v*sin(Xe(3))*(4*q1^2 +
     * q2^2*t^2))/8,  (q2^2*t^3*v*sin(Xe(3)))/6 */
    /*   -(t^3*v^2*sin(2*Xe(3))*(20*q1^2 + 3*q2^2*t^2))/120,
     * (t^3*v^2*cos(Xe(3))^2*(20*q1^2 + 3*q2^2*t^2))/60,  (t^2*v*cos(Xe(3))*(4*q1^2 +
     * q2^2*t^2))/8, -(q2^2*t^3*v*cos(Xe(3)))/6 */
    /*            -(t^2*v*sin(Xe(3))*(4*q1^2 + q2^2*t^2))/8, (t^2*v*cos(Xe(3))*(4*q1^2 +
     * q2^2*t^2))/8,                     q1^2*t + (q2^2*t^3)/3,              -(q2^2*t^2)/2
     */
    /*                            (q2^2*t^3*v*sin(Xe(3)))/6, -(q2^2*t^3*v*cos(Xe(3)))/6,
     * -(q2^2*t^2)/2,                     q2^2*t */
    /*  ]); */
    /* 'SUKF:102' Rv = abs(Rv); */
    /* 'SUKF:103' Rn=diag(r.^2); */
    /* 'SUKF:104'
     * [X,wm,wc]=generate_points(Xe,P,nXe,Lsigma,lambada,alpha,beta,sqrt_gamma); */
    /* 'SUKF:139' wm=zeros(Lsigma*2+1,1); */
    /* 'SUKF:140' wc=zeros(Lsigma*2+1,1); */
    for (i = 0; i < 9; i++)
    {
        wm[i] = 0.0F;
        wc[i] = 0.0F;
    }

    /* 'SUKF:141' X=zeros(Lsigma,Lsigma*2+1); */
    memset(&X[0], 0, 36U * sizeof(float));

    /* 'SUKF:142' gamma_P=zeros(Lsigma,1); */
    /* 'SUKF:143' for i=0:Lsigma */
    for (i = 0; i < 5; i++)
    {
        /* 'SUKF:144' if i==0 */
        if (i == 0)
        {
            /* 'SUKF:145' X(:,i+1)=Xe; */
            for (i2 = 0; i2 < 4; i2++)
            {
                X[i2] = b_Xe[i2];
            }

            /* 'SUKF:146' wm(i+1)=lambada/(Lsigma+lambada); */
            wm[0] = -3.0F;

            /* 'SUKF:147' wc(i+1)=wm(i+1)+1-alpha^2+beta; */
            wc[0] = -0.25F;
        }
        else
        {
            /* 'SUKF:148' else */
            /* 'SUKF:149' wm(i+1)=0.5/(Lsigma+lambada); */
            wm[i] = 0.5F;

            /* 'SUKF:150' wm(i+1+Lsigma)=wm(i+1); */
            wm[i + 4] = wm[i];

            /* 'SUKF:151' wc(i+1)=0.5/(Lsigma+lambada); */
            wc[i] = 0.5F;

            /* 'SUKF:152' wc(i+1+Lsigma)=wc(i+1); */
            wc[i + 4] = wc[i];

            /* 'SUKF:154' gamma_P(1:nXe,1)=P(1:nXe,i); */
            /* 'SUKF:156' gamma_P=sqrt_gamma*gamma_P; */
            /* 'SUKF:158' X(:,i+1)=Xe(:,1)+gamma_P; */
            for (i2 = 0; i2 < 4; i2++)
            {
                X[i2 + (i << 2)] = b_Xe[i2] + b_P[i2 + ((i - 1) << 2)];
                gamma_P[i2]      = b_P[i2 + ((i - 1) << 2)];
            }

            /* 'SUKF:159' X(:,i+1+Lsigma)=Xe(:,1)-gamma_P; */
            for (i2 = 0; i2 < 4; i2++)
            {
                X[i2 + ((i + 4) << 2)] = b_Xe[i2] - gamma_P[i2];
            }
        }
    }

    /* 'SUKF:105' [Xkk1,X_k,P_xk]=timeupdate(X,Xe,v,wz,t,Rv,wm,wc,nXe,Lsigma,car_moving);
     */
    b_q[0]  = q[0] * q[0] * powf(t, 4.0F) * (v * v) * (ajj * ajj) / 4.0F;
    b_q[4]  = Rv12;
    b_q[8]  = Rv13;
    b_q[12] = 0.0F;
    b_q[1]  = Rv12;
    b_q[5]  = q[0] * q[0] * powf(t, 4.0F) * (v * v) * (mu1_im * mu1_im) / 4.0F;
    b_q[9]  = Rv23;
    b_q[13] = 0.0F;
    b_q[2]  = Rv13;
    b_q[6]  = Rv23;
    b_q[10] = q[0] * q[0] * (t * t);
    b_q[14] = 0.0F;
    b_q[3]  = 0.0F;
    b_q[7]  = 0.0F;
    b_q[11] = 0.0F;
    b_q[15] = q[1] * q[1] * (t * t);
    b_abs(b_q, fv3);
    timeupdate(X, b_Xe, v, wz, t, fv3, wc, car_moving, Xkk1, X_k, P_xk);

    /* 'SUKF:106' [Ykk1,Y_k,P_yk]=measurementupdate(Xkk1,Rn,nRn,wm,wc,Lsigma); */
    power(r, fv4);
    diag(fv4, b_wc);
    for (i2 = 0; i2 < 4; i2++)
    {
        gamma_P[i2] = b_wc[i2];
    }

    measurementupdate(Xkk1, gamma_P, wm, wc, Ykk1, Y_k, P_yk);

    /* 'SUKF:108' XI=Xkk1(:,1)-X_k; */
    /* 'SUKF:109' YI=Ykk1(:,1)-Y_k; */
    /* 'SUKF:110' Pxy=wc(1)*XI*YI'; */
    for (i2 = 0; i2 < 4; i2++)
    {
        b_wc[i2] = wc[0] * (Xkk1[i2] - X_k[i2]);
    }

    for (i2 = 0; i2 < 2; i2++)
    {
        b_Ykk1[i2] = Ykk1[i2] - Y_k[i2];
    }

    for (i2 = 0; i2 < 4; i2++)
    {
        for (k = 0; k < 2; k++)
        {
            Pxy[i2 + (k << 2)] = b_wc[i2] * b_Ykk1[k];
        }
    }

    /* 'SUKF:111' for i=1+1 : Lsigma*2+1 */
    for (i = 0; i < 8; i++)
    {
        /* 'SUKF:112' XI=Xkk1(:,i)-X_k; */
        /* 'SUKF:113' YI=Ykk1(:,i)-Y_k; */
        /* 'SUKF:114' Pxy=Pxy+wc(i)*XI*YI'; */
        for (i2 = 0; i2 < 4; i2++)
        {
            b_wc[i2] = wc[i + 1] * (Xkk1[i2 + ((i + 1) << 2)] - X_k[i2]);
        }

        for (i2 = 0; i2 < 2; i2++)
        {
            b_Ykk1[i2] = Ykk1[i2 + ((i + 1) << 1)] - Y_k[i2];
        }

        for (i2 = 0; i2 < 4; i2++)
        {
            for (k = 0; k < 2; k++)
            {
                Pxy[i2 + (k << 2)] += b_wc[i2] * b_Ykk1[k];
            }
        }
    }

    /* 'SUKF:117' if measurement_valid ==1 */
    if (measurement_valid == 1)
    {
        /* 'SUKF:118' f = diag([1 1]); */
        /* 'SUKF:119' KK=(Pxy/P_yk')/P_yk; */
        for (i2 = 0; i2 < 2; i2++)
        {
            for (k = 0; k < 2; k++)
            {
                b_wc[k + (i2 << 1)] = P_yk[i2 + (k << 1)];
            }
        }

        for (i2 = 0; i2 < 4; i2++)
        {
            gamma_P[i2] = b_wc[i2];
        }

        if (fabsf(gamma_P[1]) > fabsf(gamma_P[0]))
        {
            jmax = 1;
            i    = 0;
        }
        else
        {
            jmax = 0;
            i    = 1;
        }

        ajj  = gamma_P[i] / gamma_P[jmax];
        Rv12 = gamma_P[i + 2] - ajj * gamma_P[jmax + 2];
        for (k = 0; k < 4; k++)
        {
            b_c[k + (jmax << 2)] = Pxy[k] / gamma_P[jmax];
            b_c[k + (i << 2)] =
                (Pxy[4 + k] - b_c[k + (jmax << 2)] * gamma_P[jmax + 2]) / Rv12;
            b_c[k + (jmax << 2)] -= b_c[k + (i << 2)] * ajj;
        }

        if (fabsf(P_yk[1]) > fabsf(P_yk[0]))
        {
            jmax = 1;
            i    = 0;
        }
        else
        {
            jmax = 0;
            i    = 1;
        }
        if (fabsf(P_yk[jmax]) < 1.0E-20F)
        {
            return;
        }
        ajj  = P_yk[i] / P_yk[jmax];
        Rv12 = P_yk[2 + i] - ajj * P_yk[2 + jmax];
        for (k = 0; k < 4; k++)
        {
            KK[k + (jmax << 2)] = b_c[k] / P_yk[jmax];
            KK[k + (i << 2)] = (b_c[4 + k] - KK[k + (jmax << 2)] * P_yk[2 + jmax]) / Rv12;
            KK[k + (jmax << 2)] -= KK[k + (i << 2)] * ajj;
        }

        /* 'SUKF:120' Xe=X_k+KK*(Ym-Y_k); */
        for (i2 = 0; i2 < 2; i2++)
        {
            b_Ykk1[i2] = z[i2] - Y_k[i2];
        }

        /* 'SUKF:121' U=KK*P_yk; */
        /* 'SUKF:122' TEMP=P_xk*P_xk'-U*U'; */
        for (i2 = 0; i2 < 4; i2++)
        {
            ajj = 0.0F;
            for (k = 0; k < 2; k++)
            {
                ajj += KK[i2 + (k << 2)] * b_Ykk1[k];
                Pxy[i2 + (k << 2)] = 0.0F;
                for (jmax = 0; jmax < 2; jmax++)
                {
                    Pxy[i2 + (k << 2)] += KK[i2 + (jmax << 2)] * P_yk[jmax + (k << 1)];
                }
            }

            b_Xe[i2] = X_k[i2] + ajj;
            for (k = 0; k < 4; k++)
            {
                b_q[i2 + (k << 2)] = 0.0F;
                for (jmax = 0; jmax < 4; jmax++)
                {
                    b_q[i2 + (k << 2)] += P_xk[i2 + (jmax << 2)] * P_xk[k + (jmax << 2)];
                }
            }
        }

        for (i2 = 0; i2 < 4; i2++)
        {
            for (k = 0; k < 4; k++)
            {
                b_Pxy[i2 + (k << 2)] = 0.0F;
                for (jmax = 0; jmax < 2; jmax++)
                {
                    b_Pxy[i2 + (k << 2)] += Pxy[i2 + (jmax << 2)] * Pxy[k + (jmax << 2)];
                }
            }
        }

        for (i2 = 0; i2 < 4; i2++)
        {
            for (k = 0; k < 4; k++)
            {
                b_P[k + (i2 << 2)] = b_q[k + (i2 << 2)] - b_Pxy[k + (i2 << 2)];
            }
        }

        /* 'SUKF:123' if all(eig(TEMP) > 0) */
        p      = true;
        j      = 0;
        exitg1 = false;
        while ((!exitg1) && (j < 4))
        {
            i = 0;
            do
            {
                exitg2 = 0;
                if (i <= j)
                {
                    if (!(b_P[i + (j << 2)] == b_P[j + (i << 2)]))
                    {
                        p      = false;
                        exitg2 = 1;
                    }
                    else
                    {
                        i++;
                    }
                }
                else
                {
                    j++;
                    exitg2 = 2;
                }
            } while (exitg2 == 0);

            if (exitg2 == 1)
            {
                exitg1 = true;
            }
        }

        if (p)
        {
            memcpy(&b_q[0], &b_P[0], sizeof(float) << 4);
            xgehrd(b_q);
            eml_dlahqr(b_q);
            b_q[3] = 0.0F;
            for (i2 = 0; i2 < 16; i2++)
            {
                T[i2].re = b_q[i2];
                T[i2].im = 0.0F;
            }

            for (jmax = 2; jmax >= 0; jmax--)
            {
                if (b_q[(jmax + (jmax << 2)) + 1] != 0.0F)
                {
                    ajj  = b_q[jmax + (jmax << 2)];
                    Rv12 = b_q[jmax + ((jmax + 1) << 2)];
                    c    = b_q[(jmax + (jmax << 2)) + 1];
                    d    = b_q[(jmax + ((jmax + 1) << 2)) + 1];
                    xdlanv2(&ajj, &Rv12, &c, &d, &Rv13, &rt1i, &Rv23, &mu1_im, &mu1_re,
                            &sn);
                    mu1_re = Rv13 - b_q[(jmax + ((jmax + 1) << 2)) + 1];
                    ajj =
                        rt_hypotf(rt_hypotf(mu1_re, rt1i), b_q[(jmax + (jmax << 2)) + 1]);
                    if (rt1i == 0.0F)
                    {
                        mu1_re /= ajj;
                        mu1_im = 0.0F;
                    }
                    else if (mu1_re == 0.0F)
                    {
                        mu1_re = 0.0F;
                        mu1_im = rt1i / ajj;
                    }
                    else
                    {
                        mu1_re /= ajj;
                        mu1_im = rt1i / ajj;
                    }

                    Rv23 = b_q[(jmax + (jmax << 2)) + 1] / ajj;
                    for (j = jmax; j + 1 < 5; j++)
                    {
                        Rv12                  = T[jmax + (j << 2)].re;
                        Rv13                  = T[jmax + (j << 2)].im;
                        ajj                   = T[jmax + (j << 2)].re;
                        T[jmax + (j << 2)].re = (mu1_re * T[jmax + (j << 2)].re +
                                                 mu1_im * T[jmax + (j << 2)].im) +
                                                Rv23 * T[(jmax + (j << 2)) + 1].re;
                        T[jmax + (j << 2)].im =
                            (mu1_re * T[jmax + (j << 2)].im - mu1_im * ajj) +
                            Rv23 * T[(jmax + (j << 2)) + 1].im;
                        ajj = mu1_re * T[(jmax + (j << 2)) + 1].im +
                              mu1_im * T[(jmax + (j << 2)) + 1].re;
                        T[(jmax + (j << 2)) + 1].re =
                            (mu1_re * T[(jmax + (j << 2)) + 1].re -
                             mu1_im * T[(jmax + (j << 2)) + 1].im) -
                            Rv23 * Rv12;
                        T[(jmax + (j << 2)) + 1].im = ajj - Rv23 * Rv13;
                    }

                    for (i = 0; i + 1 <= jmax + 2; i++)
                    {
                        Rv12 = T[i + (jmax << 2)].re;
                        Rv13 = T[i + (jmax << 2)].im;
                        ajj  = mu1_re * T[i + (jmax << 2)].im +
                              mu1_im * T[i + (jmax << 2)].re;
                        T[i + (jmax << 2)].re = (mu1_re * T[i + (jmax << 2)].re -
                                                 mu1_im * T[i + (jmax << 2)].im) +
                                                Rv23 * T[i + ((jmax + 1) << 2)].re;
                        T[i + (jmax << 2)].im = ajj + Rv23 * T[i + ((jmax + 1) << 2)].im;
                        ajj                   = T[i + ((jmax + 1) << 2)].re;
                        T[i + ((jmax + 1) << 2)].re =
                            (mu1_re * T[i + ((jmax + 1) << 2)].re +
                             mu1_im * T[i + ((jmax + 1) << 2)].im) -
                            Rv23 * Rv12;
                        T[i + ((jmax + 1) << 2)].im =
                            (mu1_re * T[i + ((jmax + 1) << 2)].im - mu1_im * ajj) -
                            Rv23 * Rv13;
                    }

                    T[(jmax + (jmax << 2)) + 1].re = 0.0F;
                    T[(jmax + (jmax << 2)) + 1].im = 0.0F;
                }
            }

            for (k = 0; k < 4; k++)
            {
                alpha1[k] = T[k + (k << 2)];
            }
        }
        else
        {
            xzgeev(b_P, &info, alpha1, beta1);
            for (i2 = 0; i2 < 4; i2++)
            {
                mu1_im = alpha1[i2].re;
                if (beta1[i2].im == 0.0F)
                {
                    if (alpha1[i2].im == 0.0F)
                    {
                        alpha1[i2].re /= beta1[i2].re;
                        alpha1[i2].im = 0.0F;
                    }
                    else if (alpha1[i2].re == 0.0F)
                    {
                        alpha1[i2].re = 0.0F;
                        alpha1[i2].im /= beta1[i2].re;
                    }
                    else
                    {
                        alpha1[i2].re /= beta1[i2].re;
                        alpha1[i2].im /= beta1[i2].re;
                    }
                }
                else if (beta1[i2].re == 0.0F)
                {
                    if (alpha1[i2].re == 0.0F)
                    {
                        alpha1[i2].re = alpha1[i2].im / beta1[i2].im;
                        alpha1[i2].im = 0.0F;
                    }
                    else if (alpha1[i2].im == 0.0F)
                    {
                        alpha1[i2].re = 0.0F;
                        alpha1[i2].im = -(mu1_im / beta1[i2].im);
                    }
                    else
                    {
                        alpha1[i2].re = alpha1[i2].im / beta1[i2].im;
                        alpha1[i2].im = -(mu1_im / beta1[i2].im);
                    }
                }
                else
                {
                    Rv13 = fabsf(beta1[i2].re);
                    ajj  = fabsf(beta1[i2].im);
                    if (Rv13 > ajj)
                    {
                        Rv23          = beta1[i2].im / beta1[i2].re;
                        d             = beta1[i2].re + Rv23 * beta1[i2].im;
                        alpha1[i2].re = (alpha1[i2].re + Rv23 * alpha1[i2].im) / d;
                        alpha1[i2].im = (alpha1[i2].im - Rv23 * mu1_im) / d;
                    }
                    else if (ajj == Rv13)
                    {
                        if (beta1[i2].re > 0.0F)
                        {
                            ajj = 0.5F;
                        }
                        else
                        {
                            ajj = -0.5F;
                        }

                        if (beta1[i2].im > 0.0F)
                        {
                            Rv12 = 0.5F;
                        }
                        else
                        {
                            Rv12 = -0.5F;
                        }

                        alpha1[i2].re =
                            (alpha1[i2].re * ajj + alpha1[i2].im * Rv12) / Rv13;
                        alpha1[i2].im = (alpha1[i2].im * ajj - mu1_im * Rv12) / Rv13;
                    }
                    else
                    {
                        Rv23          = beta1[i2].re / beta1[i2].im;
                        d             = beta1[i2].im + Rv23 * beta1[i2].re;
                        alpha1[i2].re = (Rv23 * alpha1[i2].re + alpha1[i2].im) / d;
                        alpha1[i2].im = (Rv23 * alpha1[i2].im - mu1_im) / d;
                    }
                }
            }
        }

        for (i = 0; i < 4; i++)
        {
            x[i] = (alpha1[i].re > 0.0F);
        }

        p      = true;
        k      = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 4))
        {
            if (!x[k])
            {
                p      = false;
                exitg1 = true;
            }
            else
            {
                k++;
            }
        }

        if (p)
        {
            /* 'SUKF:124' P=chol(TEMP,'lower'); */
            info   = 0;
            j      = 0;
            exitg1 = false;
            while ((!exitg1) && (j + 1 < 5))
            {
                jj = j + (j << 2);
                d  = 0.0F;
                if (!(j < 1))
                {
                    ix = j;
                    iy = j;
                    for (k = 1; k <= j; k++)
                    {
                        d += b_P[ix] * b_P[iy];
                        ix += 4;
                        iy += 4;
                    }
                }

                ajj = b_P[jj] - d;
                if (ajj > 0.0F)
                {
                    ajj     = sqrtf(ajj);
                    b_P[jj] = ajj;
                    if (j + 1 < 4)
                    {
                        if (j != 0)
                        {
                            ix = j;
                            i2 = (j + ((j - 1) << 2)) + 2;
                            for (jmax = j + 2; jmax <= i2; jmax += 4)
                            {
                                c  = -b_P[ix];
                                iy = jj + 1;
                                k  = (jmax - j) + 2;
                                for (i = jmax; i <= k; i++)
                                {
                                    b_P[iy] += b_P[i - 1] * c;
                                    iy++;
                                }

                                ix += 4;
                            }
                        }

                        ajj = 1.0F / ajj;
                        i2  = (jj - j) + 4;
                        for (k = jj + 1; k + 1 <= i2; k++)
                        {
                            b_P[k] *= ajj;
                        }
                    }

                    j++;
                }
                else
                {
                    b_P[jj] = ajj;
                    info    = j + 1;
                    exitg1  = true;
                }
            }

            if (info == 0)
            {
                jmax = 4;
            }
            else
            {
                jmax = info - 1;
            }

            for (j = 1; j + 1 <= jmax; j++)
            {
                for (i = 1; i <= j; i++)
                {
                    b_P[(i + (j << 2)) - 1] = 0.0F;
                }
            }
        }
        else
        {
            /* 'SUKF:125' else */
            /* 'SUKF:126' P=P2; */
            memcpy(&b_P[0], &P2[0], sizeof(float) << 4);
        }
    }
    else
    {
        /* 'SUKF:128' else */
        /* 'SUKF:129' f = diag([0 0]); */
        /* 'SUKF:130' Xe=X_k; */
        for (i = 0; i < 4; i++)
        {
            b_Xe[i] = X_k[i];
        }
    }

    /* 'SUKF:132' P = (P + P')/2; */
    for (i2 = 0; i2 < 4; i2++)
    {
        for (k = 0; k < 4; k++)
        {
            b_q[k + (i2 << 2)] = (b_P[k + (i2 << 2)] + b_P[i2 + (k << 2)]) / 2.0F;
        }
    }

    for (i2 = 0; i2 < 4; i2++)
    {
        for (k = 0; k < 4; k++)
        {
            b_P[k + (i2 << 2)] = b_q[k + (i2 << 2)];
        }
    }

    /* 'SUKF:133' P2=P; */
    memcpy(&P2[0], &b_P[0], sizeof(float) << 4);

    /* 'SUKF:134' Pdiag = diag(P); */
    /* 'SUKF:135' y=Xe; */
    for (j = 0; j < 4; j++)
    {
        Pdiag[j] = b_P[j * 5];
        y[j]     = b_Xe[j];
    }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void b_P_not_empty_init(void) { b_P_not_empty = false; }

/*
 * Arguments    : void
 * Return Type  : void
 */
void b_Xe_not_empty_init(void) { b_Xe_not_empty = false; }

/*
 * File trailer for SUKF.c
 *
 * [EOF]
 */
