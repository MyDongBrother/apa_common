#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: EKF_LC.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "EKF_LC.h"
#include "PK_LocationImpl.h"

/* Variable Definitions */
static float P[16];
static boolean_T P_not_empty;
static float Xe[4];
static boolean_T Xe_not_empty;

/* Function Definitions */

/*
 * function [y,Pdiag] =
 * EKF_LC(z,wz,v,dS,t,q,r,measurement_valid,reset,set_origin_heading,psi0,reset_EKF_LC_origin,
 * car_moving) cfg = coder.config('lib','ecoder',true); cfg.TargetLangStandard = 'C99
 * (ISO)'; codegen  -config cfg -singleC EKF_LC.m -args {z,t,q_r,q_w,r} %-O disable:inline
 *  phi and Xe(3) must be restrained within -pi/2 to pi/2
 * Arguments    : float z[2]
 *                float wz
 *                float v
 *                float t
 *                const float q[2]
 *                const float r[2]
 *                unsigned char measurement_valid
 *                unsigned int reset
 *                unsigned char set_origin_heading
 *                float psi0
 *                unsigned char reset_EKF_LC_origin
 *                unsigned char car_moving
 *                float y[4]
 *                float Pdiag[4]
 * Return Type  : void
 */
void EKF_LC(float z[2], float wz, float v, float t, const float q[2], const float r[2],
            unsigned char measurement_valid, unsigned int reset,
            unsigned char set_origin_heading, float psi0,
            unsigned char reset_EKF_LC_origin, unsigned char car_moving, float y[4],
            float Pdiag[4])
{
    static const float fv0[16] = {0.1F, 0.0F, 0.0F,  0.0F, 0.0F, 0.1F, 0.0F, 0.0F,
                                  0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F};

    int i;
    float a21;
    float Q12;
    float Q13;
    float x;
    float Q23;
    float Q[16];
    float b_y[16];
    int i0;
    float HXe[2];
    float R[4];
    static const signed char iv0[4] = {0, 0, 0, 1};

    signed char f[4];
    static const signed char iv1[4] = {1, 0, 0, 1};

    int i1;
    float fv1[16];
    float c[4];
    float b_c[8];
    int r1;
    int r2;
    float a[8];
    static const signed char b[8] = {1, 0, 0, 0, 0, 1, 0, 0};

    static const signed char b_a[8] = {1, 0, 0, 1, 0, 0, 0, 0};

    float K[8];
    float b_z[2];
    float b_K[8];

    /* 'EKF_LC:6' coder.inline('never') */
    /*  persistent v_buffer; */
    /* 'EKF_LC:11' n = 4; */
    /* 'EKF_LC:12' if isempty(Xe) || reset */
    if ((!Xe_not_empty) || (reset != 0U))
    {
        /*      Xe=single([0 0 1.8513 0]');%!1.8513!很远!!!!!!!!!!! */
        /*      Xe=single([0 0 2.1802 0]');%东荟城外面一圈 */
        /*       Xe=single([0 0 2.1894 0]');%东荟城3圈 */
        /*   Xe=single([0 0 -0.045526 0]');%PT6园外一圈2018-3-12 */
        /*    Xe=single([0 0 -0.155526 0]');  % PT6园外1_2018-3-12 */
        /*    Xe=single([0 0 -0.265526 0]');  %PT6东荟城2圈2018-3-12 */
        /*    Xe=single([0 0 -2.365526 0]');  %PT6公司2圈2018-3-12 */
        /*  Xe=single([0 0 -1.0 0]');%PT6东荟城1圈-2018-3-14 */
        /*  Xe=single([0 0 -1.01 0]');%PT6东荟城3圈-2018-3-14 */
        /*  Xe=single([0 0 1.8 0]');%PT6远永和隧道-2018-3-14 */
        /* 'EKF_LC:24' Xe=single([z(1) z(2) psi0 0]'); */
        Xe[0]        = z[0];
        Xe[1]        = z[1];
        Xe[2]        = psi0;
        Xe[3]        = 0.0F;
        Xe_not_empty = true;

        /* PT6远永和隧道-2018-3-14 */
        /* 'EKF_LC:25' Xe(4) = single(0); */
        Xe[3] = 0.0F;

        /*      v_buffer = zeros(1,4); */
    }

    /* 'EKF_LC:29' Xe_ = Xe; */
    /* !!!!!!!!!!Xe_ delete later */
    /* 'EKF_LC:31' if isempty(P) || reset */
    if ((!P_not_empty) || (reset != 0U))
    {
        /* 'EKF_LC:32' P=single(diag([.1 .1 10 0.001]))*1 ; */
        memcpy(&P[0], &fv0[0], sizeof(float) << 4);
        P_not_empty = true;
    }

    /* 'EKF_LC:34' if set_origin_heading == 1 */
    if (set_origin_heading == 1)
    {
        /* 'EKF_LC:35' Xe=single([z(1) z(2) psi0 0]'); */
        Xe[0] = z[0];
        Xe[1] = z[1];
        Xe[2] = psi0;
        Xe[3] = 0.0F;

        /* PT6远永和隧道-2018-3-14   */
    }

    /* 'EKF_LC:37' if reset_EKF_LC_origin == 1 */
    if (reset_EKF_LC_origin == 1)
    {
        /* 'EKF_LC:38' Xe(1:2) = z; */
        for (i = 0; i < 2; i++)
        {
            Xe[i] = z[i];
        }
    }

    /*  v_buffer(1:end-1) = v_buffer(2:end); */
    /*  v_buffer(end) = v; */
    /*  if abs(sum(abs(v_buffer)) -
     * max(v_buffer))<1e-4%速度小时不相信GPS测量!!!!!!!!!!!!重要 */
    /*  % if (min(abs(v_buffer)))==0%速度小时不相信GPS测量!!!!!!!!!!!!重要 */
    /*  %     measurement_valid = uint8(0); */
    /*  %     Xe(4) = Xe(4) * 199/200+1/200*wz;%!!!!!!!为什么导致不准确的结果 */
    /*  %to do !!!! 当速度判断为0时, wz依然较大,
     * 是速度出了问题?速度来自电机,其实应该用轮脉冲 */
    /*      car_moving = uint8(0);%!!!!!!!为什么导致不准确的结果 */
    /*      z(1) = Xe(1); */
    /*      z(2) = Xe(2); */
    /*      dS = 0; */
    /*      % % %     Xe(4) = Xe(4);!!!!!!!!! */
    /*      % else */
    /*      measurement_valid = uint8(1); */
    /*  else */
    /*      car_moving = uint8(1); */
    /*  end */
    /*  car_moving = car_moving_state(dS); */
    /* 'EKF_LC:58' if car_moving == 0 */
    if (car_moving == 0)
    {
        /* 'EKF_LC:59' z(1) = Xe(1); */
        z[0] = Xe[0];

        /* 'EKF_LC:60' z(2) = Xe(2); */
        z[1] = Xe[1];

        /* 'EKF_LC:61' dS = 0; */
        /* 'EKF_LC:62' measurement_valid = uint8(1); */
        measurement_valid = 1;
    }

    /*  v = dS / t;%added 2018-4-18 */
    /*  measurements */
    /* 'EKF_LC:68' c_fading = single(1.000002); */
    /*  状态噪声方差 */
    /* 'EKF_LC:70' q1 = q(1); */
    /* 'EKF_LC:71' q2 = q(2); */
    /*  Q =single([                              (q1^2*t^4*v^2*sin(Xe(3) - t*(Xe(4)/2 -
     * wz/2))^2)/4, -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) - t*(Xe(4)/2
     * - wz/2)))/4, -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2,        0 */
    /*      -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) - t*(Xe(4)/2 -
     * wz/2)))/4,                              (q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 -
     * wz/2))^2)/4,  (q1^2*t^3*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)))/2,        0 */
    /*      -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2, (q1^2*t^3*v*cos(Xe(3) -
     * t*(Xe(4)/2 - wz/2)))/2,                                      q1^2*t^2,        0 */
    /*      0, 0,                                             0, q2^2*t^2]); */
    /*  Q = abs(Q); */
    /* 'EKF_LC:77' Q11 = (q1^2*t^4*v^2*sin(Xe(3) - t*(Xe(4)/2 - wz/2))^2)/4; */
    a21 = sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));

    /* 'EKF_LC:78' Q12 = -(q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))*sin(Xe(3) -
     * t*(Xe(4)/2 - wz/2)))/4; */
    Q12 = -(q[0] * q[0] * powf(t, 4.0F) * (v * v) *
            cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F)) *
            sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F))) /
          4.0F;

    /* 'EKF_LC:79' Q13 = -(q1^2*t^3*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2; */
    Q13 = -(q[0] * q[0] * powf(t, 3.0F) * v *
            sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F))) /
          2.0F;

    /* 'EKF_LC:80' Q14 = 0; */
    /* 'EKF_LC:81' Q22 = (q1^2*t^4*v^2*cos(Xe(3) - t*(Xe(4)/2 - wz/2))^2)/4; */
    x = cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));

    /* 'EKF_LC:82' Q23 = (q1^2*t^3*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)))/2; */
    Q23 = q[0] * q[0] * powf(t, 3.0F) * v * cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F)) /
          2.0F;

    /* 'EKF_LC:83' Q24 = 0; */
    /* 'EKF_LC:84' Q33 = q1^2*t^2; */
    /* 'EKF_LC:85' Q34 =  0; */
    /* 'EKF_LC:86' Q44 =  q2^2*t^2; */
    /* 'EKF_LC:87' Q=single([   Q11, Q12, Q13, Q14 */
    /* 'EKF_LC:88'              Q12,   Q22,  Q23, Q24 */
    /* 'EKF_LC:89'              Q13,   Q23,  Q33, Q34          */
    /* 'EKF_LC:90'              Q14,   Q24,  Q34, Q44                    */
    /* 'EKF_LC:91' ]); */
    Q[0]  = q[0] * q[0] * powf(t, 4.0F) * (v * v) * (a21 * a21) / 4.0F;
    Q[4]  = Q12;
    Q[8]  = Q13;
    Q[12] = 0.0F;
    Q[1]  = Q12;
    Q[5]  = q[0] * q[0] * powf(t, 4.0F) * (v * v) * (x * x) / 4.0F;
    Q[9]  = Q23;
    Q[13] = 0.0F;
    Q[2]  = Q13;
    Q[6]  = Q23;
    Q[10] = q[0] * q[0] * (t * t);
    Q[14] = 0.0F;
    Q[3]  = 0.0F;
    Q[7]  = 0.0F;
    Q[11] = 0.0F;
    Q[15] = q[1] * q[1] * (t * t);

    /* 'EKF_LC:93' Q = abs(Q); */
    for (i = 0; i < 16; i++)
    {
        b_y[i] = fabsf(Q[i]);
    }

    /*  Q = single([  (q1^2*t^3*v^2*sin(Xe(3))^2)/3, -(q1^2*t^3*v^2*sin(2*Xe(3)))/6,
     * -(q1^2*t^2*v*sin(Xe(3)))/2,             0 */
    /*   -(q1^2*t^3*v^2*sin(2*Xe(3)))/6,  (q1^2*t^3*v^2*cos(Xe(3))^2)/3,
     * (q1^2*t^2*v*cos(Xe(3)))/2,             0 */
    /*       -(q1^2*t^2*v*sin(Xe(3)))/2,      (q1^2*t^2*v*cos(Xe(3)))/2,      q1^2*t +
     * (q2^2*t^3)/3, -(q2^2*t^2)/2 */
    /*                                0,                              0, -(q2^2*t^2)/2,
     * q2^2*t]);                       */
    /*  A=[ 1, 0, -t*v*sin(Xe(3)),  (t^2*v*sin(Xe(3)))/2 */
    /*   0, 1,  t*v*cos(Xe(3)), -(t^2*v*cos(Xe(3)))/2 */
    /*   0, 0,               1,                    -t */
    /*   0, 0,               0,                     1]; */
    /*   测量噪声 */
    /* 'EKF_LC:104' R = single(diag(r.^2)); */
    for (i = 0; i < 2; i++)
    {
        HXe[i] = r[i] * r[i];
    }

    for (i0 = 0; i0 < 4; i0++)
    {
        R[i0] = 0.0F;
    }

    for (i = 0; i < 2; i++)
    {
        R[i + (i << 1)] = HXe[i];
    }

    /*  */
    /* 'EKF_LC:106' JA=single([ 1, 0, -t*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)),
     * (t^2*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)))/2 */
    /* 'EKF_LC:107'     0, 1,  t*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)), -(t^2*v*cos(Xe(3) -
     * t*(Xe(4)/2 - wz/2)))/2 */
    /* 'EKF_LC:108'     0, 0,                                  1, -t */
    /* 'EKF_LC:109'     0, 0,                                  0, 1]); */
    Q[0]  = 1.0F;
    Q[4]  = 0.0F;
    Q[8]  = -t * v * sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));
    Q[12] = t * t * v * sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F)) / 2.0F;
    Q[1]  = 0.0F;
    Q[5]  = 1.0F;
    Q[9]  = t * v * cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));
    Q[13] = -(t * t * v * cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F))) / 2.0F;
    Q[2]  = 0.0F;
    Q[6]  = 0.0F;
    Q[10] = 1.0F;
    Q[14] = -t;
    for (i0 = 0; i0 < 4; i0++)
    {
        Q[3 + (i0 << 2)] = iv0[i0];
    }

    /*  EKF */
    /*  Xe = RKint(t,Xe,F,zeros(7,1)); */
    /* 'EKF_LC:114' HXe = single([                Xe(1) */
    /* 'EKF_LC:115'     Xe(2)]); */
    HXe[0] = Xe[0];
    HXe[1] = Xe[1];

    /* 'EKF_LC:116' JH =  single([1,          0,  0, 0 */
    /* 'EKF_LC:117'     0,          1,  0, 0]); */
    /* 'EKF_LC:118' if measurement_valid ==1 */
    if (measurement_valid == 1)
    {
        /* 'EKF_LC:119' f = diag([1 1]); */
        /* 'EKF_LC:120' P = c_fading*JA * P * JA' + Q; */
        for (i0 = 0; i0 < 4; i0++)
        {
            f[i0] = iv1[i0];
            for (i1 = 0; i1 < 4; i1++)
            {
                fv1[i0 + (i1 << 2)] = 0.0F;
                for (i = 0; i < 4; i++)
                {
                    fv1[i0 + (i1 << 2)] +=
                        1.00000203F * Q[i0 + (i << 2)] * P[i + (i1 << 2)];
                }
            }
        }

        for (i0 = 0; i0 < 4; i0++)
        {
            for (i1 = 0; i1 < 4; i1++)
            {
                a21 = 0.0F;
                for (i = 0; i < 4; i++)
                {
                    a21 += fv1[i0 + (i << 2)] * Q[i1 + (i << 2)];
                }

                P[i0 + (i1 << 2)] = a21 + b_y[i0 + (i1 << 2)];
            }
        }
    }
    else
    {
        /* 'EKF_LC:121' else */
        /* 'EKF_LC:122' f = diag([0 0]); */
        for (i0 = 0; i0 < 4; i0++)
        {
            f[i0] = 0;
        }

        /*      z = HXe; */
        /*  P = c_fading*JA * P * JA' + Q;%!!!!!!!!!!!!!不更新P!!!!! */
    }

    /* 'EKF_LC:126' K = P*JH'/(JH*P*JH'+R); */
    for (i0 = 0; i0 < 4; i0++)
    {
        for (i1 = 0; i1 < 2; i1++)
        {
            b_c[i0 + (i1 << 2)] = 0.0F;
            for (i = 0; i < 4; i++)
            {
                b_c[i0 + (i1 << 2)] += P[i0 + (i << 2)] * (float)b[i + (i1 << 2)];
            }
        }
    }

    for (i0 = 0; i0 < 2; i0++)
    {
        for (i1 = 0; i1 < 4; i1++)
        {
            a[i0 + (i1 << 1)] = 0.0F;
            for (i = 0; i < 4; i++)
            {
                a[i0 + (i1 << 1)] += (float)b_a[i0 + (i << 1)] * P[i + (i1 << 2)];
            }
        }

        for (i1 = 0; i1 < 2; i1++)
        {
            a21 = 0.0F;
            for (i = 0; i < 4; i++)
            {
                a21 += a[i0 + (i << 1)] * (float)b[i + (i1 << 2)];
            }

            c[i0 + (i1 << 1)] = a21 + R[i0 + (i1 << 1)];
        }
    }

    if (fabsf(c[1]) > fabsf(c[0]))
    {
        r1 = 1;
        r2 = 0;
    }
    else
    {
        r1 = 0;
        r2 = 1;
    }

    a21 = c[r2] / c[r1];
    Q12 = c[2 + r2] - a21 * c[2 + r1];

    /*  if car_moving */
    /* 'EKF_LC:128' Xe = [ Xe(1) + t*v*cos(Xe(3) - t*(Xe(4)/2 - wz/2)) */
    /* 'EKF_LC:129'     Xe(2) + t*v*sin(Xe(3) - t*(Xe(4)/2 - wz/2)) */
    /* 'EKF_LC:130'     Xe(3) - t*(Xe(4) - wz)*single(car_moving) */
    /* 'EKF_LC:131'     Xe(4)/200*single(car_moving)+Xe(4) *
     * 199/200+1/200*wz*(1-single(car_moving))]; */
    Xe[0] += t * v * cosf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));
    Xe[1] += t * v * sinf(Xe[2] - t * (Xe[3] / 2.0F - wz / 2.0F));
    Xe[2] -= t * (Xe[3] - wz) * (float)car_moving;
    Xe[3] = (Xe[3] / 200.0F * (float)car_moving + Xe[3] * 199.0F / 200.0F) +
            0.005F * wz * (1.0F - (float)car_moving);

    /*  end */
    /*  f = diag([0 0]);%!!!!!!!!测试用，需要删掉 */
    /*  if norm(z-HXe)<500 && abs(v)>=0.001   %!!!!!!!! */
    /* 'EKF_LC:137' Xe = Xe +  K*f*(z-HXe); */
    for (i = 0; i < 4; i++)
    {
        K[i + (r1 << 2)] = b_c[i] / c[r1];
        K[i + (r2 << 2)] = (b_c[4 + i] - K[i + (r1 << 2)] * c[2 + r1]) / Q12;
        K[i + (r1 << 2)] -= K[i + (r2 << 2)] * a21;
        for (i0 = 0; i0 < 2; i0++)
        {
            b_K[i + (i0 << 2)] = 0.0F;
            for (i1 = 0; i1 < 2; i1++)
            {
                b_K[i + (i0 << 2)] += K[i + (i1 << 2)] * (float)f[i1 + (i0 << 1)];
            }
        }
    }

    for (i0 = 0; i0 < 2; i0++)
    {
        b_z[i0] = z[i0] - HXe[i0];
    }

    /* 'EKF_LC:138' if norm(Xe_(1:2)-Xe(1:2))>1 */
    /*  end */
    /* 'EKF_LC:142' P = P -  K*f*JH*P; */
    for (i0 = 0; i0 < 4; i0++)
    {
        a21 = 0.0F;
        for (i1 = 0; i1 < 2; i1++)
        {
            a21 += b_K[i0 + (i1 << 2)] * b_z[i1];
            a[i0 + (i1 << 2)] = 0.0F;
            for (i = 0; i < 2; i++)
            {
                a[i0 + (i1 << 2)] += K[i0 + (i << 2)] * (float)f[i + (i1 << 1)];
            }
        }

        Xe[i0] += a21;
        for (i1 = 0; i1 < 4; i1++)
        {
            Q[i0 + (i1 << 2)] = 0.0F;
            for (i = 0; i < 2; i++)
            {
                Q[i0 + (i1 << 2)] += a[i0 + (i << 2)] * (float)b_a[i + (i1 << 1)];
            }
        }

        for (i1 = 0; i1 < 4; i1++)
        {
            a21 = 0.0F;
            for (i = 0; i < 4; i++)
            {
                a21 += Q[i0 + (i << 2)] * P[i + (i1 << 2)];
            }

            fv1[i0 + (i1 << 2)] = P[i0 + (i1 << 2)] - a21;
        }
    }

    for (i0 = 0; i0 < 4; i0++)
    {
        for (i1 = 0; i1 < 4; i1++)
        {
            P[i1 + (i0 << 2)] = fv1[i1 + (i0 << 2)];
        }
    }

    /* !!!!!在if里还是外 */
    /* 'EKF_LC:143' P=0.5*(P+P'); */
    for (i0 = 0; i0 < 4; i0++)
    {
        for (i1 = 0; i1 < 4; i1++)
        {
            fv1[i1 + (i0 << 2)] = 0.5F * (P[i1 + (i0 << 2)] + P[i0 + (i1 << 2)]);
        }
    }

    for (i0 = 0; i0 < 4; i0++)
    {
        for (i1 = 0; i1 < 4; i1++)
        {
            P[i1 + (i0 << 2)] = fv1[i1 + (i0 << 2)];
        }
    }

    /*  */
    /* 'EKF_LC:146' Pdiag = diag(P); */
    /* 'EKF_LC:147' y = single(Xe); */
    for (i = 0; i < 4; i++)
    {
        Pdiag[i] = P[i * 5];
        y[i]     = Xe[i];
    }

    /*  %% Runge-Kutta4 */
    /*  function y = RK4(t,X,z) */
    /*  k1=process(X,z); */
    /*  k2=process(X+0.5*k1*t,z); */
    /*  k3=process(X+0.5*k2*t,z); */
    /*  k4=process(X+k3*t,z); */
    /*  y=X+t/6*(k1+2*k2+2*k3+k4); */
    /*  */
    /*  function y = process(Xe,z) */
    /*   y = [[1 sin(Xe(1))*tan(Xe(2)) cos(Xe(1))*tan(Xe(2)) */
    /*          0 cos(Xe(1)) -sin(Xe(1))]*[z(1)-Xe(3) z(2)-Xe(4) z(3)-Xe(5)]' */
    /*          0 */
    /*          0 */
    /*          0 */
    /*          ]; */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void P_not_empty_init(void) { P_not_empty = false; }

/*
 * Arguments    : void
 * Return Type  : void
 */
void Xe_not_empty_init(void) { Xe_not_empty = false; }

/*
 * File trailer for EKF_LC.c
 *
 * [EOF]
 */
