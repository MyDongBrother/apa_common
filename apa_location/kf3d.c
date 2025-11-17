#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kf3d.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "kf3d.h"
#include "svd.h"

/* Variable Definitions */
static float d_P[9];
static boolean_T d_P_not_empty;
static float d_Xe[3];
static boolean_T d_Xe_not_empty;
static float S;
static float pre_v_motor;
static unsigned char b_reset_numerical_error;

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void d_P_not_empty_init(void) { d_P_not_empty = false; }

/*
 * Arguments    : void
 * Return Type  : void
 */
void d_Xe_not_empty_init(void) { d_Xe_not_empty = false; }

/*
 * function y = kf3d(veh_speed_motor,t,q,r,ds,reset)
 * Arguments    : float veh_speed_motor
 *                float t
 *                float q                 0.005
 *                const float r[2]
 *                float ds
 *                unsigned int reset
 *                float y[3]
 * Return Type  : void
 */

void kf3d(float veh_speed_motor, float t, float q, const float r[2], float ds,
          unsigned int reset, float y[3])
{
    int i;
    float accel_motor;
    float v_wheel;
    int r2;
    static const signed char iv2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    float A[9];
    static const signed char iv3[3] = {0, 0, 1};

    float s[2];
    float R[4];
    float b_q[9];
    float fv5[9];
    int r1;
    float fv6[9];
    float tempT[4];
    float a[6];
    static const signed char b_a[6] = {1, 0, 0, 1, 0, 0};

    static const signed char b[6] = {1, 0, 0, 0, 1, 0};

    float c[6];
    float K[6];
    float B[3];
    float b_s[2];
    static const signed char b_b[4] = {1, 0, 0, 1};

    /*  filter motor abnormal vibration */
    /* 'kf3d:3' coder.inline('never') */
    /* 'kf3d:9' if isempty(reset_numerical_error) */
    /* 'kf3d:12' max_possible_accelx = single(50); */
    /* 'kf3d:13' if isempty(Xe)||reset==1|| reset_numerical_error */
    if ((!d_Xe_not_empty) || (reset == 1U) || (b_reset_numerical_error != 0))
    {
        /* 'kf3d:14' Xe=single(zeros(3,1)); */
        for (i = 0; i < 3; i++)
        {
            d_Xe[i] = 0.0F;
        }
        d_Xe[1]        = veh_speed_motor;
        d_Xe_not_empty = true;

        /* 'kf3d:15' S = single(0); */
        S = 0.0F;

        /* 'kf3d:16' pre_v_motor = single(0); */
        pre_v_motor = 0.0F;

        /* 'kf3d:17' reset_numerical_error = uint8(0); */
        b_reset_numerical_error = 0;
    }

    /* 'kf3d:19' y=single(Xe); */
    for (i = 0; i < 3; i++)
    {
        y[i] = d_Xe[i];
    }

    /* 'kf3d:20' if abs(t)>0 */
    if (fabsf(t) > 0.0F)
    {
        /* 'kf3d:21' accel_motor = (veh_speed_motor - pre_v_motor) / t; */
        accel_motor = (veh_speed_motor - pre_v_motor) / t;

        /* 'kf3d:22' v_wheel = ds / t; */
        v_wheel = ds / t;
    }
    else
    {
        /* 'kf3d:23' else */
        /* 'kf3d:24' accel_motor = 0; */
        accel_motor = 0.0F;

        /* 'kf3d:25' v_wheel = 0; */
        v_wheel = 0.0F;
    }

    /* 'kf3d:27' if isempty(P)||reset==1 || reset_numerical_error */
    if ((!d_P_not_empty) || (reset == 1U) || (b_reset_numerical_error != 0))
    {
        /* 'kf3d:28' P=single(eye(3)); */
        for (r2 = 0; r2 < 9; r2++)
        {
            d_P[r2] = iv2[r2];
        }

        d_P_not_empty = true;
    }

    /* 'kf3d:30' S = S + ds; */
    S += ds;

    /* 'kf3d:31' f = single(eye(2)); */
    /* 'kf3d:33' if abs(accel_motor) > max_possible_accelx */
    if (fabsf(accel_motor) > 50.0F)
    {
        /*      r(2) = r(2)*10; */
        /*      f(2,2) = single(0); */
        /*      q = q*1000; */
        /* 'kf3d:37' veh_speed_motor = v_wheel; */
        veh_speed_motor = v_wheel;
    }

    /* 'kf3d:40' alpha = single(1.000002); */
    /* 'kf3d:41' A = single([ 1, t, t^2/2 */
    /* 'kf3d:42'  0, 1,     t */
    /* 'kf3d:43'  0, 0,     1]); */
    A[0] = 1.0F;
    A[3] = t;
    A[6] = t * t / 2.0F;
    A[1] = 0.0F;
    A[4] = 1.0F;
    A[7] = t;
    for (r2 = 0; r2 < 3; r2++)
    {
        A[2 + 3 * r2] = iv3[r2];
    }

    /* 'kf3d:44' H=single([1 0 0 */
    /* 'kf3d:45'     0 1 0]); */
    /* 'kf3d:46' Q = single([ (q*t^5)/20, (q*t^4)/8, (q*t^3)/6 */
    /* 'kf3d:47'   (q*t^4)/8, (q*t^3)/3, (q*t^2)/2 */
    /* 'kf3d:48'   (q*t^3)/6, (q*t^2)/2,       q*t]); */
    /* 'kf3d:49' R = single(diag(r.^2)); */
    for (i = 0; i < 2; i++)
    {
        s[i] = r[i] * r[i];
    }

    for (r2 = 0; r2 < 4; r2++)
    {
        R[r2] = 0.0F;
    }

    for (i = 0; i < 2; i++)
    {
        R[i + (i << 1)] = s[i];
    }

    /* 'kf3d:50' P = alpha*A * P * A' + Q; */
    for (r2 = 0; r2 < 3; r2++)
    {
        for (i = 0; i < 3; i++)
        {
            fv5[r2 + 3 * i] = 0.0F;
            for (r1 = 0; r1 < 3; r1++)
            {
                fv5[r2 + 3 * i] += 1.00000203F * A[r2 + 3 * r1] * d_P[r1 + 3 * i];
            }
        }

        for (i = 0; i < 3; i++)
        {
            fv6[r2 + 3 * i] = 0.0F;
            for (r1 = 0; r1 < 3; r1++)
            {
                fv6[r2 + 3 * i] += fv5[r2 + 3 * r1] * A[i + 3 * r1];
            }
        }
    }

    b_q[0] = q * powf(t, 5.0F) / 20.0F;
    b_q[3] = q * powf(t, 4.0F) / 8.0F;
    b_q[6] = q * powf(t, 3.0F) / 6.0F;
    b_q[1] = q * powf(t, 4.0F) / 8.0F;
    b_q[4] = q * powf(t, 3.0F) / 3.0F;
    b_q[7] = q * (t * t) / 2.0F;
    b_q[2] = q * powf(t, 3.0F) / 6.0F;
    b_q[5] = q * (t * t) / 2.0F;
    b_q[8] = q * t;
    for (r2 = 0; r2 < 3; r2++)
    {
        for (i = 0; i < 3; i++)
        {
            d_P[i + 3 * r2] = fv6[i + 3 * r2] + b_q[i + 3 * r2];
        }
    }

    /* 'kf3d:52' tempT = (H*P*H'+R); */
    for (r2 = 0; r2 < 2; r2++)
    {
        for (i = 0; i < 3; i++)
        {
            a[r2 + (i << 1)] = 0.0F;
            for (r1 = 0; r1 < 3; r1++)
            {
                a[r2 + (i << 1)] += (float)b_a[r2 + (r1 << 1)] * d_P[r1 + 3 * i];
            }
        }

        for (i = 0; i < 2; i++)
        {
            accel_motor = 0.0F;
            for (r1 = 0; r1 < 3; r1++)
            {
                accel_motor += a[r2 + (r1 << 1)] * (float)b[r1 + 3 * i];
            }

            tempT[r2 + (i << 1)] = accel_motor + R[r2 + (i << 1)];
        }
    }

    /* 'kf3d:53' if (cond(tempT)<1e6) */
    svd(tempT, s);
    if (s[1] == 0.0F)
    {
        accel_motor = 3.402823466E+38F;
    }
    else
    {
        accel_motor = s[0] / s[1];
    }

    if (accel_motor < 1.0E+6F)
    {
        /* 'kf3d:54' K = P*H'/tempT; */
        for (r2 = 0; r2 < 3; r2++)
        {
            for (i = 0; i < 2; i++)
            {
                c[r2 + 3 * i] = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    c[r2 + 3 * i] += d_P[r2 + 3 * r1] * (float)b[r1 + 3 * i];
                }
            }
        }

        if (fabsf(tempT[1]) > fabsf(tempT[0]))
        {
            r1 = 1;
            r2 = 0;
        }
        else
        {
            r1 = 0;
            r2 = 1;
        }

        accel_motor = tempT[r2] / tempT[r1];
        v_wheel     = tempT[2 + r2] - accel_motor * tempT[2 + r1];

        /*  K = [   0.0010000   0.0096074 */
        /*     0.0000961   0.0764426 */
        /*    -0.0000057   0.3039083]; */
        /*  K = P*H'/(H*P*H'+R); */
        /* 'kf3d:63' Xe=A*Xe; */
        for (i = 0; i < 3; i++)
        {
            K[i + 3 * r1] = c[i] / tempT[r1];
            K[i + 3 * r2] = (c[3 + i] - K[i + 3 * r1] * tempT[2 + r1]) / v_wheel;
            K[i + 3 * r1] -= K[i + 3 * r2] * accel_motor;
            B[i] = d_Xe[i];
        }

        for (r2 = 0; r2 < 3; r2++)
        {
            d_Xe[r2] = 0.0F;
            for (i = 0; i < 3; i++)
            {
                d_Xe[r2] += A[r2 + 3 * i] * B[i];
            }
        }

        /* 'kf3d:65' if Xe(1) > 20 */
        if (d_Xe[0] > 20.0F)
        {
            /*  set to 0 every 20m */
            /* 'kf3d:66' S = S - Xe(1); */
            S -= d_Xe[0];

            /* 'kf3d:67' Xe(1) = 0; */
            d_Xe[0] = 0.0F;
        }

        /* 'kf3d:69' z = [S;veh_speed_motor]; */
        s[0] = S;
        s[1] = veh_speed_motor;

        /* 'kf3d:71' if abs(z(2))<0.001 */
        if (fabsf(veh_speed_motor) < 0.001F)
        {
            /* !!!!!!!!!! */
            /* 'kf3d:72' Xe(2) = 0; */
            d_Xe[1] = 0.0F;

            /* 'kf3d:73' Xe(3) = 0; */
            d_Xe[2] = 0.0F;
        }
        else
        {
            /* 'kf3d:74' else */
            /* 'kf3d:75' Xe = Xe +  K*f*(z-H*Xe); */
            for (r2 = 0; r2 < 3; r2++)
            {
                for (i = 0; i < 2; i++)
                {
                    a[r2 + 3 * i] = 0.0F;
                    for (r1 = 0; r1 < 2; r1++)
                    {
                        a[r2 + 3 * i] += K[r2 + 3 * r1] * (float)b_b[r1 + (i << 1)];
                    }
                }
            }

            for (r2 = 0; r2 < 2; r2++)
            {
                accel_motor = 0.0F;
                for (i = 0; i < 3; i++)
                {
                    accel_motor += (float)b_a[r2 + (i << 1)] * d_Xe[i];
                }

                b_s[r2] = s[r2] - accel_motor;
            }

            for (r2 = 0; r2 < 3; r2++)
            {
                accel_motor = 0.0F;
                for (i = 0; i < 2; i++)
                {
                    accel_motor += a[r2 + 3 * i] * b_s[i];
                }

                d_Xe[r2] += accel_motor;
            }
        }

        /* 'kf3d:77' P = P -  K*H*P; */
        for (r2 = 0; r2 < 3; r2++)
        {
            for (i = 0; i < 3; i++)
            {
                A[r2 + 3 * i] = 0.0F;
                for (r1 = 0; r1 < 2; r1++)
                {
                    A[r2 + 3 * i] += K[r2 + 3 * r1] * (float)b_a[r1 + (i << 1)];
                }
            }

            for (i = 0; i < 3; i++)
            {
                accel_motor = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    accel_motor += A[r2 + 3 * r1] * d_P[r1 + 3 * i];
                }

                fv5[r2 + 3 * i] = d_P[r2 + 3 * i] - accel_motor;
            }
        }

        for (r2 = 0; r2 < 3; r2++)
        {
            for (i = 0; i < 3; i++)
            {
                d_P[i + 3 * r2] = fv5[i + 3 * r2];
            }
        }

        /* 'kf3d:78' P=0.5*(P+P'); */
        for (r2 = 0; r2 < 3; r2++)
        {
            for (i = 0; i < 3; i++)
            {
                fv5[i + 3 * r2] = 0.5F * (d_P[i + 3 * r2] + d_P[r2 + 3 * i]);
            }
        }

        /* 'kf3d:79' pre_v_motor = veh_speed_motor; */
        pre_v_motor = veh_speed_motor;

        /* 'kf3d:81' y=single(Xe); */
        for (i = 0; i < 3; i++)
        {
            for (r2 = 0; r2 < 3; r2++)
            {
                d_P[r2 + 3 * i] = fv5[r2 + 3 * i];
            }

            y[i] = d_Xe[i];
        }
    }
    else
    {
        /* 'kf3d:55' else */
        /* 'kf3d:56' reset_numerical_error = uint8(1); */
        b_reset_numerical_error = 1;
    }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kf3d_init(void)
{
    /* 'kf3d:10' reset_numerical_error = uint8(0); */
    b_reset_numerical_error = 0;
}

/*
 * File trailer for kf3d.c
 *
 * [EOF]
 */
