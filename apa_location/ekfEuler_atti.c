#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekfEuler_atti.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "ekfEuler_atti.h"
#include "svd.h"
#include "norm.h"
#include "mean.h"

/* Variable Definitions */
static float c_P[25];
static boolean_T c_P_not_empty;
static float c_Xe[5];
static boolean_T c_Xe_not_empty;
static unsigned char reset_numerical_error;
static float accel[30];
static unsigned char idx_accel;
static unsigned short bias_counter;
static unsigned int bias_valid_counter;
static unsigned short accel_valid_counter;

/* Function Declarations */
// static void b_ekfEuler_atti(ekfEuler_atti_arg_Type *ekfEuler_atti_arg);

/* Function Definitions */

/*
 * function ekfEuler_atti_arg = ekfEuler_atti(ekfEuler_atti_arg)
 * codegen  -config:lib -c ekfEuler_atti.m -args
 * {ekfEuler_atti_arg.z,ekfEuler_atti_arg.dt,ekfEuler_atti_arg.q_r,ekfEuler_atti_arg.q_w,ekfEuler_atti_arg.r}
 * %-O disable:inline phi and theta must be restrained within -pi/2 to pi/2
 * {
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       July 2, 2018
 *  @version    V2.5.8
 *  @since      add ekfEuler_atti_arg.r=ekfEuler_atti_arg.r*(1+((norm_z - gravity)*10)^6);
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       July 25, 2018
 *  @version    V2.5.9
 *  @since      add ekfEuler_atti_arg.reset
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.10
 *  @since      add reset_numerical_error and cond()
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      change logic for f
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      add max_wb
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      add accel_valid_counter_shreshold
 * -------------------------------------------------------------------------------------
 * }
 * Arguments    : ekfEuler_atti_arg_Type *ekfEuler_atti_arg
 * Return Type  : void
 */
void b_ekfEuler_atti(ekfEuler_atti_arg_Type *ekfEuler_atti_arg)
{
    float a21;
    int i;
    static const float fv11[25] = {
        0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,  0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F,
        0.0F,  0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,  0.0F, 0.0F, 0.0F, 0.0F, 0.01F};

    static const signed char iv5[30] = {1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5,  5,  5,
                                        6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10};

    float fv12[3];
    signed char f[4];
    static const signed char iv6[4] = {1, 0, 0, 1};

    float a22;
    float x;
    float b_x;
    float c_x;
    float d_x;
    float s[2];
    float R[4];
    float e_x;
    float f_x;
    float JA[25];
    static const signed char iv7[5] = {0, 0, 1, 0, 0};

    static const signed char iv8[5] = {0, 0, 0, 1, 0};

    static const signed char iv9[5] = {0, 0, 0, 0, 1};

    float JH[10];
    float b_ekfEuler_atti_arg[25];
    int r1;
    float fv13[25];
    int r2;
    float fv14[25];
    float tempT[4];
    float b_JH[10];
    float c[10];
    float K[10];
    float c_ekfEuler_atti_arg[3];
    float fv15[6];
    float k1[5];
    float b_c[5];
    float k2[5];
    float k3[5];
    float d_ekfEuler_atti_arg[2];

    /* 'ekfEuler_atti:37' coder.inline('never') */
    /* 'ekfEuler_atti:46' accel_valid_counter_shreshold = uint16(50); */
    /* coder.opaque('int','100'); */
    /* 'ekfEuler_atti:47' n = 5; */
    /*  g =
     * single(sqrt(ekfEuler_atti_arg.z(1)^2+ekfEuler_atti_arg.z(2)^2+ekfEuler_atti_arg.z(3)^2));%应该修改为IMU测量的加速度的模
     */
    /* 'ekfEuler_atti:49' g = 1; */
    /* Normalized */
    /* 'ekfEuler_atti:50' ekfEuler_atti_arg.result = ([0 0 0 0 0]'); */
    /* 'ekfEuler_atti:51' gravity = 9.8; */
    /* 'ekfEuler_atti:52' norm_z = (norm((ekfEuler_atti_arg.z(1:3)))); */
    a21 = b_norm(*(float(*)[3]) & ekfEuler_atti_arg->z[0]);

    /* 'ekfEuler_atti:54' if isempty(reset_numerical_error) */
    /* 'ekfEuler_atti:57' if isempty(Xe)|| ekfEuler_atti_arg.reset ||
     * reset_numerical_error */
    if ((!c_Xe_not_empty) || (ekfEuler_atti_arg->reset != 0U) ||
        (reset_numerical_error != 0))
    {
        /* 'ekfEuler_atti:58' Xe=([0 0 0 0 0]'); */
        for (i = 0; i < 5; i++)
        {
            c_Xe[i] = 0.0F;
        }

        c_Xe_not_empty = true;

        /* 'ekfEuler_atti:59' accel = [1:10 */
        /* 'ekfEuler_atti:60'             1:10 */
        /* 'ekfEuler_atti:61'             1:10]; */
        for (i = 0; i < 30; i++)
        {
            accel[i] = iv5[i];
        }

        /* 'ekfEuler_atti:62' idx_accel = uint8(1); */
        idx_accel = 1;

        /* 'ekfEuler_atti:63' reset_numerical_error = uint8(0); */
        reset_numerical_error = 0;

        /* 'ekfEuler_atti:64' bias_counter = uint16(1); */
        bias_counter = 1;

        /* 'ekfEuler_atti:65' bias_valid_counter = uint32(1); */
        bias_valid_counter = 1U;

        /* 'ekfEuler_atti:66' accel_valid_counter = uint16(1); */
        accel_valid_counter = 1;
    }

    /* 'ekfEuler_atti:68' if isempty(P) || ekfEuler_atti_arg.reset ||
     * reset_numerical_error */
    if ((!c_P_not_empty) || (ekfEuler_atti_arg->reset != 0U) ||
        (reset_numerical_error != 0))
    {
        /* 'ekfEuler_atti:69' P=(eye(n,n)*0.01) ; */
        memcpy(&c_P[0], &fv11[0], 25U * sizeof(float));
        c_P_not_empty = true;
    }

    /*  Xe(5) = 0;%to do added */
    /* slope is not that steep */
    /* 'ekfEuler_atti:74' if Xe(1)>1.57 */
    if (c_Xe[0] > 1.57F)
    {
        /* 'ekfEuler_atti:75' Xe(1) = 1.57; */
        c_Xe[0] = 1.57F;
    }
    else
    {
        if (c_Xe[0] < -1.57F)
        {
            /* 'ekfEuler_atti:76' elseif Xe(1)<-1.57 */
            /* 'ekfEuler_atti:77' Xe(1) = -1.57; */
            c_Xe[0] = -1.57F;
        }
    }

    /* 'ekfEuler_atti:79' if Xe(2)>1.57 */
    if (c_Xe[1] > 1.57F)
    {
        /* 'ekfEuler_atti:80' Xe(2) = 1.57; */
        c_Xe[1] = 1.57F;
    }
    else
    {
        if (c_Xe[1] < -1.57F)
        {
            /* 'ekfEuler_atti:81' elseif Xe(2)<-1.57 */
            /* 'ekfEuler_atti:82' Xe(2) = -1.57; */
            c_Xe[1] = -1.57F;
        }
    }

    /* 'ekfEuler_atti:84' ekfEuler_atti_arg.result = single(Xe); */
    for (i = 0; i < 5; i++)
    {
        ekfEuler_atti_arg->result[i] = c_Xe[i];
    }

    /* 'ekfEuler_atti:85' if norm(ekfEuler_atti_arg.z(1:3)) < 1e-4 */
    if (!(b_norm(*(float(*)[3]) & ekfEuler_atti_arg->z[0]) < 0.0001F))
    {
        /* 'ekfEuler_atti:89' if idx_accel>= uint8(11) */
        if (idx_accel >= 11)
        {
            /* 'ekfEuler_atti:90' idx_accel = uint8(1); */
            idx_accel = 1;
        }

        /* 'ekfEuler_atti:92' if bias_counter>=200 */
        if (bias_counter >= 200)
        {
            /* 'ekfEuler_atti:93' bias_counter = uint16(200); */
            bias_counter = 200;
        }

        /* 'ekfEuler_atti:95' accel(:,idx_accel) = ekfEuler_atti_arg.z(1:3); */
        for (i = 0; i < 3; i++)
        {
            accel[i + 3 * (idx_accel - 1)] = ekfEuler_atti_arg->z[i];
        }

        /* 'ekfEuler_atti:96' if (abs(norm(mean(accel,2)) - gravity)<0.005) ||
         * (accel_valid_counter == accel_valid_counter_shreshold) */
        mean(accel, fv12);
        if ((fabsf(b_norm(fv12) - 9.8F) < 0.005F) || (accel_valid_counter == 50))
        {
            /* && abs(std_accel)<.01)% && abs(ekfEuler_atti_arg.speed) <
             * 0.05)||(abs(ekfEuler_atti_arg.speed) > 10) */
            /* 'ekfEuler_atti:97' f = diag([1 1 ])*1; */
            for (i = 0; i < 4; i++)
            {
                f[i] = iv6[i];
            }

            /* 'ekfEuler_atti:98' flag = 1; */
            /* 'ekfEuler_atti:99' accel_valid_counter = uint16(1); */
            accel_valid_counter = 1;
        }
        else
        {
            /* 'ekfEuler_atti:100' else */
            /* 'ekfEuler_atti:101' f = diag([1 1 ])*0; */
            for (i = 0; i < 4; i++)
            {
                f[i] = 0;
            }

            /* 'ekfEuler_atti:102' flag = 0; */
        }

        /*  if max(abs(Xe)) > 4/180*pi %|| abs(ekfEuler_atti_arg.speed) < 0.01 */
        /*      f = diag([1 1 ])*1; */
        /*      flag = 1; */
        /*  end */
        /*  if bias_valid_counter <= 10000 */
        /*      f = diag([1 1 ])*0;     */
        /*      flag = 0; */
        /*  end */
        /* 'ekfEuler_atti:113' if  abs(ekfEuler_atti_arg.speed) < 0.001 */
        if (fabsf(ekfEuler_atti_arg->speed) < 0.001F)
        {
            /* 'ekfEuler_atti:114' f = diag([1 1 ])*1; */
            for (i = 0; i < 4; i++)
            {
                f[i] = iv6[i];
            }

            /* 'ekfEuler_atti:115' flag = 1; */
        }

        /*  */
        /* 'ekfEuler_atti:118' if abs(ekfEuler_atti_arg.speed) < 0.001 */
        if (fabsf(ekfEuler_atti_arg->speed) < 0.001F)
        {
            /*      wb = 0.995*wb+0.005*ekfEuler_atti_arg.z(4:6); */
            /* 'ekfEuler_atti:120' if bias_counter == uint8(200) */
            if (bias_counter == 200)
            {
                /* 'ekfEuler_atti:121' bias_valid_counter = uint32(1); */
                bias_valid_counter = 1U;
            }

            /* 'ekfEuler_atti:123' bias_counter = bias_counter + 1; */
            bias_counter++;
        }
        else
        {
            /* 'ekfEuler_atti:124' else */
            /* 'ekfEuler_atti:125' bias_counter = uint16(1); */
            bias_counter = 1;
        }

        /* 'ekfEuler_atti:128' if abs(norm_z - gravity)>1 */
        if (fabsf(a21 - 9.8F) > 1.0F)
        {
            /* 'ekfEuler_atti:129' f = diag([1 1 ])*0; */
            for (i = 0; i < 4; i++)
            {
                f[i] = 0;
            }

            /* 'ekfEuler_atti:130' flag = 0; */
        }

        /*  if abs(Xe(1)) > 0.035 */
        /*      f(1,1) = 1; */
        /*      flag = 1; */
        /*  end */
        /*  if abs(Xe(2)) > 0.035 */
        /*      f(2,2) = 1; */
        /*      flag = 1; */
        /*  end */
        /* 'ekfEuler_atti:140' idx_accel = idx_accel + 1; */
        idx_accel++;

        /* 'ekfEuler_atti:141' bias_valid_counter = bias_valid_counter + 1; */
        bias_valid_counter++;

        /* 'ekfEuler_atti:142' accel_valid_counter = accel_valid_counter + 1; */
        accel_valid_counter++;

        /* 'ekfEuler_atti:143' if accel_valid_counter > accel_valid_counter_shreshold */
        if (accel_valid_counter > 50)
        {
            /* 'ekfEuler_atti:144' accel_valid_counter = uint16(1); */
            accel_valid_counter = 1;
        }

        /* 'ekfEuler_atti:146' ekfEuler_atti_arg.z(1:3) = ekfEuler_atti_arg.z(1:3)/norm_z;
         */
        for (i = 0; i < 3; i++)
        {
            ekfEuler_atti_arg->z[i] /= a21;
        }

        /*  measurements */
        /* 'ekfEuler_atti:149' c_fading = single(1.000002); */
        /*  状态噪声方差 */
        /* 'ekfEuler_atti:151' qr1 = ekfEuler_atti_arg.q_r(1); */
        /* 'ekfEuler_atti:152' qr2 = ekfEuler_atti_arg.q_r(2); */
        /* 'ekfEuler_atti:153' qr3 = ekfEuler_atti_arg.q_r(3); */
        /* 'ekfEuler_atti:154' qw1 = ekfEuler_atti_arg.q_w(1); */
        /* 'ekfEuler_atti:155' qw2 = ekfEuler_atti_arg.q_w(2); */
        /* 'ekfEuler_atti:156' qw3 = ekfEuler_atti_arg.q_w(3); */
        /* 'ekfEuler_atti:157' Q =single([ qr1^2*ekfEuler_atti_arg.dt^2 +
         * qr3^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))^2*tan(Xe(2))^2 +
         * qr2^2*ekfEuler_atti_arg.dt^2*sin(Xe(1))^2*tan(Xe(2))^2,
         * qr2^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))*sin(Xe(1))*tan(Xe(2)) -
         * qr3^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))*sin(Xe(1))*tan(Xe(2)),         0, 0, 0
         */
        /* 'ekfEuler_atti:158'
         * qr2^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))*sin(Xe(1))*tan(Xe(2)) -
         * qr3^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))*sin(Xe(1))*tan(Xe(2)),
         * qr2^2*ekfEuler_atti_arg.dt^2*cos(Xe(1))^2 +
         * qr3^2*ekfEuler_atti_arg.dt^2*sin(Xe(1))^2,         0,         0,         0 */
        /* 'ekfEuler_atti:159' 0, 0, qw1^2*ekfEuler_atti_arg.dt^2,         0,         0 */
        /* 'ekfEuler_atti:160' 0,  0,         0, qw2^2*ekfEuler_atti_arg.dt^2,         0
         */
        /* 'ekfEuler_atti:161' 0, 0,         0,         0, qw3^2*ekfEuler_atti_arg.dt^2]);
         */
        a21 = cosf(c_Xe[0]);
        a22 = tanf(c_Xe[1]);
        x   = sinf(c_Xe[0]);
        b_x = tanf(c_Xe[1]);
        c_x = cosf(c_Xe[0]);
        d_x = sinf(c_Xe[0]);

        /*   测量噪声 */
        /* 'ekfEuler_atti:163' R = single(diag(ekfEuler_atti_arg.r(1:2).^2)); */
        for (i = 0; i < 2; i++)
        {
            s[i] = ekfEuler_atti_arg->r[i] * ekfEuler_atti_arg->r[i];
        }

        for (i = 0; i < 4; i++)
        {
            R[i] = 0.0F;
        }

        for (i = 0; i < 2; i++)
        {
            R[i + (i << 1)] = s[i];
        }

        /*  */
        /* 'ekfEuler_atti:165' JA=single([ 1 -
         * ekfEuler_atti_arg.dt*(cos(Xe(1))*tan(Xe(2))*(Xe(4) - ekfEuler_atti_arg.z(5)) -
         * sin(Xe(1))*tan(Xe(2))*(Xe(5) - ekfEuler_atti_arg.z(6))),
         * -ekfEuler_atti_arg.dt*(cos(Xe(1))*(Xe(5) -
         * ekfEuler_atti_arg.z(6))*(tan(Xe(2))^2 + 1) + sin(Xe(1))*(Xe(4) -
         * ekfEuler_atti_arg.z(5))*(tan(Xe(2))^2 + 1)), -ekfEuler_atti_arg.dt,
         * -ekfEuler_atti_arg.dt*sin(Xe(1))*tan(Xe(2)),
         * -ekfEuler_atti_arg.dt*cos(Xe(1))*tan(Xe(2)) */
        /* 'ekfEuler_atti:166' ekfEuler_atti_arg.dt*(cos(Xe(1))*(Xe(5) -
         * ekfEuler_atti_arg.z(6)) + sin(Xe(1))*(Xe(4) - ekfEuler_atti_arg.z(5))), 1,  0,
         * -ekfEuler_atti_arg.dt*cos(Xe(1)),             ekfEuler_atti_arg.dt*sin(Xe(1))
         */
        /* 'ekfEuler_atti:167' 0, 0,  1,                      0,                      0 */
        /* 'ekfEuler_atti:168' 0, 0,  0,                      1,                      0 */
        /* 'ekfEuler_atti:169' 0, 0,  0,                      0,                      1]);
         */
        e_x   = tanf(c_Xe[1]);
        f_x   = tanf(c_Xe[1]);
        JA[0] = 1.0F -
                ekfEuler_atti_arg->dt *
                    (cosf(c_Xe[0]) * tanf(c_Xe[1]) * (c_Xe[3] - ekfEuler_atti_arg->z[4]) -
                     sinf(c_Xe[0]) * tanf(c_Xe[1]) * (c_Xe[4] - ekfEuler_atti_arg->z[5]));
        JA[5] =
            -ekfEuler_atti_arg->dt *
            (cosf(c_Xe[0]) * (c_Xe[4] - ekfEuler_atti_arg->z[5]) * (e_x * e_x + 1.0F) +
             sinf(c_Xe[0]) * (c_Xe[3] - ekfEuler_atti_arg->z[4]) * (f_x * f_x + 1.0F));
        JA[10] = -ekfEuler_atti_arg->dt;
        JA[15] = -ekfEuler_atti_arg->dt * sinf(c_Xe[0]) * tanf(c_Xe[1]);
        JA[20] = -ekfEuler_atti_arg->dt * cosf(c_Xe[0]) * tanf(c_Xe[1]);
        JA[1] =
            ekfEuler_atti_arg->dt * (cosf(c_Xe[0]) * (c_Xe[4] - ekfEuler_atti_arg->z[5]) +
                                     sinf(c_Xe[0]) * (c_Xe[3] - ekfEuler_atti_arg->z[4]));
        JA[6]  = 1.0F;
        JA[11] = 0.0F;
        JA[16] = -ekfEuler_atti_arg->dt * cosf(c_Xe[0]);
        JA[21] = ekfEuler_atti_arg->dt * sinf(c_Xe[0]);
        for (i = 0; i < 5; i++)
        {
            JA[2 + 5 * i] = iv7[i];
            JA[3 + 5 * i] = iv8[i];
            JA[4 + 5 * i] = iv9[i];
        }

        /*  EKF */
        /*  HXe = single([         -g*sin(Xe(2)) */
        /*   g*cos(Xe(2))*sin(Xe(1)) */
        /*   g*cos(Xe(1))*cos(Xe(2)) */
        /*   ]); */
        /*  JH =  single([                      0,          -g*cos(Xe(2)), 0, 0, 0 */
        /*    g*cos(Xe(1))*cos(Xe(2)), -g*sin(Xe(1))*sin(Xe(2)), 0, 0, 0 */
        /*   -g*cos(Xe(2))*sin(Xe(1)), -g*cos(Xe(1))*sin(Xe(2)), 0, 0, 0]); */
        /* 'ekfEuler_atti:180' JH =  single([                      0, -g*cos(Xe(2)), 0, 0,
         * 0 */
        /* 'ekfEuler_atti:181'   g*cos(Xe(1))*cos(Xe(2)), -g*sin(Xe(1))*sin(Xe(2)), 0, 0,
         * 0]); */
        JH[0] = 0.0F;
        JH[2] = -cosf(c_Xe[1]);
        JH[4] = 0.0F;
        JH[6] = 0.0F;
        JH[8] = 0.0F;
        JH[1] = cosf(c_Xe[0]) * cosf(c_Xe[1]);
        JH[3] = -sinf(c_Xe[0]) * sinf(c_Xe[1]);
        JH[5] = 0.0F;
        JH[7] = 0.0F;
        JH[9] = 0.0F;

        /* 'ekfEuler_atti:182' P = c_fading*JA * P * JA' + Q; */
        for (i = 0; i < 5; i++)
        {
            for (r1 = 0; r1 < 5; r1++)
            {
                fv13[i + 5 * r1] = 0.0F;
                for (r2 = 0; r2 < 5; r2++)
                {
                    fv13[i + 5 * r1] += 1.00000203F * JA[i + 5 * r2] * c_P[r2 + 5 * r1];
                }
            }

            for (r1 = 0; r1 < 5; r1++)
            {
                fv14[i + 5 * r1] = 0.0F;
                for (r2 = 0; r2 < 5; r2++)
                {
                    fv14[i + 5 * r1] += fv13[i + 5 * r2] * JA[r1 + 5 * r2];
                }
            }
        }

        b_ekfEuler_atti_arg[0] = (ekfEuler_atti_arg->q_r[0] * ekfEuler_atti_arg->q_r[0] *
                                      (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) +
                                  ekfEuler_atti_arg->q_r[2] * ekfEuler_atti_arg->q_r[2] *
                                      (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                      (a21 * a21) * (a22 * a22)) +
                                 ekfEuler_atti_arg->q_r[1] * ekfEuler_atti_arg->q_r[1] *
                                     (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                     (x * x) * (b_x * b_x);
        b_ekfEuler_atti_arg[5] = ekfEuler_atti_arg->q_r[1] * ekfEuler_atti_arg->q_r[1] *
                                     (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                     cosf(c_Xe[0]) * sinf(c_Xe[0]) * tanf(c_Xe[1]) -
                                 ekfEuler_atti_arg->q_r[2] * ekfEuler_atti_arg->q_r[2] *
                                     (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                     cosf(c_Xe[0]) * sinf(c_Xe[0]) * tanf(c_Xe[1]);
        b_ekfEuler_atti_arg[10] = 0.0F;
        b_ekfEuler_atti_arg[15] = 0.0F;
        b_ekfEuler_atti_arg[20] = 0.0F;
        b_ekfEuler_atti_arg[1]  = ekfEuler_atti_arg->q_r[1] * ekfEuler_atti_arg->q_r[1] *
                                     (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                     cosf(c_Xe[0]) * sinf(c_Xe[0]) * tanf(c_Xe[1]) -
                                 ekfEuler_atti_arg->q_r[2] * ekfEuler_atti_arg->q_r[2] *
                                     (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) *
                                     cosf(c_Xe[0]) * sinf(c_Xe[0]) * tanf(c_Xe[1]);
        b_ekfEuler_atti_arg[6] =
            ekfEuler_atti_arg->q_r[1] * ekfEuler_atti_arg->q_r[1] *
                (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) * (c_x * c_x) +
            ekfEuler_atti_arg->q_r[2] * ekfEuler_atti_arg->q_r[2] *
                (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt) * (d_x * d_x);
        b_ekfEuler_atti_arg[11] = 0.0F;
        b_ekfEuler_atti_arg[16] = 0.0F;
        b_ekfEuler_atti_arg[21] = 0.0F;
        b_ekfEuler_atti_arg[2]  = 0.0F;
        b_ekfEuler_atti_arg[7]  = 0.0F;
        b_ekfEuler_atti_arg[12] = ekfEuler_atti_arg->q_w[0] * ekfEuler_atti_arg->q_w[0] *
                                  (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt);
        b_ekfEuler_atti_arg[17] = 0.0F;
        b_ekfEuler_atti_arg[22] = 0.0F;
        b_ekfEuler_atti_arg[3]  = 0.0F;
        b_ekfEuler_atti_arg[8]  = 0.0F;
        b_ekfEuler_atti_arg[13] = 0.0F;
        b_ekfEuler_atti_arg[18] = ekfEuler_atti_arg->q_w[1] * ekfEuler_atti_arg->q_w[1] *
                                  (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt);
        b_ekfEuler_atti_arg[23] = 0.0F;
        b_ekfEuler_atti_arg[4]  = 0.0F;
        b_ekfEuler_atti_arg[9]  = 0.0F;
        b_ekfEuler_atti_arg[14] = 0.0F;
        b_ekfEuler_atti_arg[19] = 0.0F;
        b_ekfEuler_atti_arg[24] = ekfEuler_atti_arg->q_w[2] * ekfEuler_atti_arg->q_w[2] *
                                  (ekfEuler_atti_arg->dt * ekfEuler_atti_arg->dt);
        for (i = 0; i < 5; i++)
        {
            for (r1 = 0; r1 < 5; r1++)
            {
                c_P[r1 + 5 * i] = fv14[r1 + 5 * i] + b_ekfEuler_atti_arg[r1 + 5 * i];
            }
        }

        /* 'ekfEuler_atti:183' tempT = (JH*P*JH'+R); */
        for (i = 0; i < 2; i++)
        {
            for (r1 = 0; r1 < 5; r1++)
            {
                b_JH[i + (r1 << 1)] = 0.0F;
                for (r2 = 0; r2 < 5; r2++)
                {
                    b_JH[i + (r1 << 1)] += JH[i + (r2 << 1)] * c_P[r2 + 5 * r1];
                }
            }

            for (r1 = 0; r1 < 2; r1++)
            {
                a21 = 0.0F;
                for (r2 = 0; r2 < 5; r2++)
                {
                    a21 += b_JH[i + (r2 << 1)] * JH[r1 + (r2 << 1)];
                }

                tempT[i + (r1 << 1)] = a21 + R[i + (r1 << 1)];
            }
        }

        /* 'ekfEuler_atti:185' if (cond(tempT)<1e6) */
        svd(tempT, s);
        if (s[1] == 0.0F)
        {
            a21 = 3.402823466E+38F;
        }
        else
        {
            a21 = s[0] / s[1];
        }

        if (a21 < 1.0E+6F)
        {
            /* 'ekfEuler_atti:186' K = P*JH'/tempT; */
            for (i = 0; i < 5; i++)
            {
                for (r1 = 0; r1 < 2; r1++)
                {
                    c[i + 5 * r1] = 0.0F;
                    for (r2 = 0; r2 < 5; r2++)
                    {
                        c[i + 5 * r1] += c_P[i + 5 * r2] * JH[r1 + (r2 << 1)];
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

            a21 = tempT[r2] / tempT[r1];
            a22 = tempT[2 + r2] - a21 * tempT[2 + r1];
            for (i = 0; i < 5; i++)
            {
                K[i + 5 * r1] = c[i] / tempT[r1];
                K[i + 5 * r2] = (c[5 + i] - K[i + 5 * r1] * tempT[2 + r1]) / a22;
                K[i + 5 * r1] -= K[i + 5 * r2] * a21;
            }

            /*  [msgstr, msgid] = lastwarn;%catch warning */
            /*  if ~isempty(msgid) */
            /*      a=0; */
            /*  end */
            /*  lastwarn('')% to ekfEuler_atti_arg.reset the warning message! */
            /* 'ekfEuler_atti:196' Xe =
             * RK4(ekfEuler_atti_arg.dt,Xe,ekfEuler_atti_arg.z(4:6)); */
            /*  Runge-Kutta4 */
            /* 'ekfEuler_atti:243' k1=process(X,z); */
            /* 'ekfEuler_atti:250' y = [[1 sin(Xe(1))*tan(Xe(2)) cos(Xe(1))*tan(Xe(2)) */
            /* 'ekfEuler_atti:251'         0 cos(Xe(1)) -sin(Xe(1))]*[z(1)-Xe(3)
             * z(2)-Xe(4) z(3)-Xe(5)]' */
            /* 'ekfEuler_atti:252'         0 */
            /* 'ekfEuler_atti:253'         0 */
            /* 'ekfEuler_atti:254'         0 */
            /* 'ekfEuler_atti:255'         ]; */
            c_ekfEuler_atti_arg[0] = ekfEuler_atti_arg->z[3] - c_Xe[2];
            c_ekfEuler_atti_arg[1] = ekfEuler_atti_arg->z[4] - c_Xe[3];
            c_ekfEuler_atti_arg[2] = ekfEuler_atti_arg->z[5] - c_Xe[4];
            fv15[0]                = 1.0F;
            fv15[2]                = sinf(c_Xe[0]) * tanf(c_Xe[1]);
            fv15[4]                = cosf(c_Xe[0]) * tanf(c_Xe[1]);
            fv15[1]                = 0.0F;
            fv15[3]                = cosf(c_Xe[0]);
            fv15[5]                = -sinf(c_Xe[0]);
            for (i = 0; i < 2; i++)
            {
                s[i] = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    s[i] += fv15[i + (r1 << 1)] * c_ekfEuler_atti_arg[r1];
                }

                k1[i] = s[i];
            }

            k1[2] = 0.0F;
            k1[3] = 0.0F;
            k1[4] = 0.0F;

            /* 'ekfEuler_atti:244' k2=process(X+0.5*k1*dt,z); */
            for (i = 0; i < 5; i++)
            {
                b_c[i] = c_Xe[i] + 0.5F * k1[i] * ekfEuler_atti_arg->dt;
            }

            /* 'ekfEuler_atti:250' y = [[1 sin(Xe(1))*tan(Xe(2)) cos(Xe(1))*tan(Xe(2)) */
            /* 'ekfEuler_atti:251'         0 cos(Xe(1)) -sin(Xe(1))]*[z(1)-Xe(3)
             * z(2)-Xe(4) z(3)-Xe(5)]' */
            /* 'ekfEuler_atti:252'         0 */
            /* 'ekfEuler_atti:253'         0 */
            /* 'ekfEuler_atti:254'         0 */
            /* 'ekfEuler_atti:255'         ]; */
            c_ekfEuler_atti_arg[0] = ekfEuler_atti_arg->z[3] - b_c[2];
            c_ekfEuler_atti_arg[1] = ekfEuler_atti_arg->z[4] - b_c[3];
            c_ekfEuler_atti_arg[2] = ekfEuler_atti_arg->z[5] - b_c[4];
            fv15[0]                = 1.0F;
            fv15[2]                = sinf(b_c[0]) * tanf(b_c[1]);
            fv15[4]                = cosf(b_c[0]) * tanf(b_c[1]);
            fv15[1]                = 0.0F;
            fv15[3]                = cosf(b_c[0]);
            fv15[5]                = -sinf(b_c[0]);
            for (i = 0; i < 2; i++)
            {
                s[i] = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    s[i] += fv15[i + (r1 << 1)] * c_ekfEuler_atti_arg[r1];
                }

                k2[i] = s[i];
            }

            k2[2] = 0.0F;
            k2[3] = 0.0F;
            k2[4] = 0.0F;

            /* 'ekfEuler_atti:245' k3=process(X+0.5*k2*dt,z); */
            for (i = 0; i < 5; i++)
            {
                b_c[i] = c_Xe[i] + 0.5F * k2[i] * ekfEuler_atti_arg->dt;
            }

            /* 'ekfEuler_atti:250' y = [[1 sin(Xe(1))*tan(Xe(2)) cos(Xe(1))*tan(Xe(2)) */
            /* 'ekfEuler_atti:251'         0 cos(Xe(1)) -sin(Xe(1))]*[z(1)-Xe(3)
             * z(2)-Xe(4) z(3)-Xe(5)]' */
            /* 'ekfEuler_atti:252'         0 */
            /* 'ekfEuler_atti:253'         0 */
            /* 'ekfEuler_atti:254'         0 */
            /* 'ekfEuler_atti:255'         ]; */
            c_ekfEuler_atti_arg[0] = ekfEuler_atti_arg->z[3] - b_c[2];
            c_ekfEuler_atti_arg[1] = ekfEuler_atti_arg->z[4] - b_c[3];
            c_ekfEuler_atti_arg[2] = ekfEuler_atti_arg->z[5] - b_c[4];
            fv15[0]                = 1.0F;
            fv15[2]                = sinf(b_c[0]) * tanf(b_c[1]);
            fv15[4]                = cosf(b_c[0]) * tanf(b_c[1]);
            fv15[1]                = 0.0F;
            fv15[3]                = cosf(b_c[0]);
            fv15[5]                = -sinf(b_c[0]);
            for (i = 0; i < 2; i++)
            {
                s[i] = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    s[i] += fv15[i + (r1 << 1)] * c_ekfEuler_atti_arg[r1];
                }

                k3[i] = s[i];
            }

            k3[2] = 0.0F;
            k3[3] = 0.0F;
            k3[4] = 0.0F;

            /* 'ekfEuler_atti:246' k4=process(X+k3*dt,z); */
            for (i = 0; i < 5; i++)
            {
                b_c[i] = c_Xe[i] + k3[i] * ekfEuler_atti_arg->dt;
            }

            /* 'ekfEuler_atti:250' y = [[1 sin(Xe(1))*tan(Xe(2)) cos(Xe(1))*tan(Xe(2)) */
            /* 'ekfEuler_atti:251'         0 cos(Xe(1)) -sin(Xe(1))]*[z(1)-Xe(3)
             * z(2)-Xe(4) z(3)-Xe(5)]' */
            /* 'ekfEuler_atti:252'         0 */
            /* 'ekfEuler_atti:253'         0 */
            /* 'ekfEuler_atti:254'         0 */
            /* 'ekfEuler_atti:255'         ]; */
            /* 'ekfEuler_atti:247' y=X+dt/6*(k1+2*k2+2*k3+k4); */
            a21                    = ekfEuler_atti_arg->dt / 6.0F;
            c_ekfEuler_atti_arg[0] = ekfEuler_atti_arg->z[3] - b_c[2];
            c_ekfEuler_atti_arg[1] = ekfEuler_atti_arg->z[4] - b_c[3];
            c_ekfEuler_atti_arg[2] = ekfEuler_atti_arg->z[5] - b_c[4];
            fv15[0]                = 1.0F;
            fv15[2]                = sinf(b_c[0]) * tanf(b_c[1]);
            fv15[4]                = cosf(b_c[0]) * tanf(b_c[1]);
            fv15[1]                = 0.0F;
            fv15[3]                = cosf(b_c[0]);
            fv15[5]                = -sinf(b_c[0]);
            for (i = 0; i < 2; i++)
            {
                s[i] = 0.0F;
                for (r1 = 0; r1 < 3; r1++)
                {
                    s[i] += fv15[i + (r1 << 1)] * c_ekfEuler_atti_arg[r1];
                }

                b_c[i] = s[i];
            }

            b_c[2] = 0.0F;
            b_c[3] = 0.0F;
            b_c[4] = 0.0F;
            for (i = 0; i < 5; i++)
            {
                c_Xe[i] += a21 * (((k1[i] + 2.0F * k2[i]) + 2.0F * k3[i]) + b_c[i]);
            }

            /* 'ekfEuler_atti:197' HXe = single([         -g*sin(Xe(2)) */
            /* 'ekfEuler_atti:198'  g*cos(Xe(2))*sin(Xe(1)) */
            /* 'ekfEuler_atti:199'  ]); */
            /* 'ekfEuler_atti:200' residual = (ekfEuler_atti_arg.z(1:2)-HXe); */
            /*  residual =
             * sign(residual).*([0.01;0.01;0.001;0.001;0.001].*(abs(residual)>[0.01;0.01;0.001;0.001;0.001]));
             */
            /* 'ekfEuler_atti:203' dXe = K*f*residual; */
            /*  dXe_max = [0.001;0.001;1;1;1];%to do  tuning */
            /*  dXe = (dXe.*(abs(dXe)<=dXe_max)+sign(dXe).*dXe_max.*(abs(dXe)>dXe_max));
             */
            /* 'ekfEuler_atti:206' Xe = Xe +  dXe; */
            s[0] = -sinf(c_Xe[1]);
            s[1] = cosf(c_Xe[1]) * sinf(c_Xe[0]);
            for (i = 0; i < 5; i++)
            {
                for (r1 = 0; r1 < 2; r1++)
                {
                    b_JH[i + 5 * r1] = 0.0F;
                    for (r2 = 0; r2 < 2; r2++)
                    {
                        b_JH[i + 5 * r1] += K[i + 5 * r2] * (float)f[r2 + (r1 << 1)];
                    }
                }
            }

            for (i = 0; i < 2; i++)
            {
                d_ekfEuler_atti_arg[i] = ekfEuler_atti_arg->z[i] - s[i];
            }

            /* 'ekfEuler_atti:207' P = P -  K*JH*P; */
            for (i = 0; i < 5; i++)
            {
                a21 = 0.0F;
                for (r1 = 0; r1 < 2; r1++)
                {
                    a21 += b_JH[i + 5 * r1] * d_ekfEuler_atti_arg[r1];
                }

                c_Xe[i] += a21;
                for (r1 = 0; r1 < 5; r1++)
                {
                    JA[i + 5 * r1] = 0.0F;
                    for (r2 = 0; r2 < 2; r2++)
                    {
                        JA[i + 5 * r1] += K[i + 5 * r2] * JH[r2 + (r1 << 1)];
                    }
                }

                for (r1 = 0; r1 < 5; r1++)
                {
                    a21 = 0.0F;
                    for (r2 = 0; r2 < 5; r2++)
                    {
                        a21 += JA[i + 5 * r2] * c_P[r2 + 5 * r1];
                    }

                    fv13[i + 5 * r1] = c_P[i + 5 * r1] - a21;
                }
            }

            for (i = 0; i < 5; i++)
            {
                for (r1 = 0; r1 < 5; r1++)
                {
                    c_P[r1 + 5 * i] = fv13[r1 + 5 * i];
                }
            }

            /* 'ekfEuler_atti:208' P=0.5*(P+P'); */
            for (i = 0; i < 5; i++)
            {
                for (r1 = 0; r1 < 5; r1++)
                {
                    fv13[r1 + 5 * i] = 0.5F * (c_P[r1 + 5 * i] + c_P[i + 5 * r1]);
                }
            }

            for (i = 0; i < 5; i++)
            {
                for (r1 = 0; r1 < 5; r1++)
                {
                    c_P[r1 + 5 * i] = fv13[r1 + 5 * i];
                }
            }

            /*  */
            /* 'ekfEuler_atti:211' if Xe(1)>1.57 */
            if (c_Xe[0] > 1.57F)
            {
                /* 'ekfEuler_atti:212' Xe(1) = 1.57; */
                c_Xe[0] = 1.57F;
            }
            else
            {
                if (c_Xe[0] < -1.57F)
                {
                    /* 'ekfEuler_atti:213' elseif Xe(1)<-1.57 */
                    /* 'ekfEuler_atti:214' Xe(1) = -1.57; */
                    c_Xe[0] = -1.57F;
                }
            }

            /* 'ekfEuler_atti:216' if Xe(2)>1.57 */
            if (c_Xe[1] > 1.57F)
            {
                /* 'ekfEuler_atti:217' Xe(2) = 1.57; */
                c_Xe[1] = 1.57F;
            }
            else
            {
                if (c_Xe[1] < -1.57F)
                {
                    /* 'ekfEuler_atti:218' elseif Xe(2)<-1.57 */
                    /* 'ekfEuler_atti:219' Xe(2) = -1.57; */
                    c_Xe[1] = -1.57F;
                }
            }

            /* 'ekfEuler_atti:221' max_wb = 0.001; */
            /* 'ekfEuler_atti:222' if Xe(3)>max_wb */
            if (c_Xe[2] > 0.001F)
            {
                /* 'ekfEuler_atti:223' Xe(3) = max_wb; */
                c_Xe[2] = 0.001F;
            }
            else
            {
                if (c_Xe[2] < -0.001F)
                {
                    /* 'ekfEuler_atti:224' elseif Xe(3)<-max_wb */
                    /* 'ekfEuler_atti:225' Xe(3) = -max_wb; */
                    c_Xe[2] = -0.001F;
                }
            }

            /* 'ekfEuler_atti:227' if Xe(4)>max_wb */
            if (c_Xe[3] > 0.001F)
            {
                /* 'ekfEuler_atti:228' Xe(4) = max_wb; */
                c_Xe[3] = 0.001F;
            }
            else
            {
                if (c_Xe[3] < -0.001F)
                {
                    /* 'ekfEuler_atti:229' elseif Xe(4)<-max_wb */
                    /* 'ekfEuler_atti:230' Xe(4) = -max_wb; */
                    c_Xe[3] = -0.001F;
                }
            }

            /* 'ekfEuler_atti:232' if Xe(5)>max_wb */
            if (c_Xe[4] > 0.001F)
            {
                /* 'ekfEuler_atti:233' Xe(5) = max_wb; */
                c_Xe[4] = 0.001F;
            }
            else
            {
                if (c_Xe[4] < -0.001F)
                {
                    /* 'ekfEuler_atti:234' elseif Xe(5)<-max_wb */
                    /* 'ekfEuler_atti:235' Xe(5) = -max_wb; */
                    c_Xe[4] = -0.001F;
                }
            }

            /* 'ekfEuler_atti:238' ekfEuler_atti_arg.result = single(Xe); */
            for (i = 0; i < 5; i++)
            {
                ekfEuler_atti_arg->result[i] = c_Xe[i];
            }
        }
        else
        {
            /* 'ekfEuler_atti:187' else */
            /* 'ekfEuler_atti:188' reset_numerical_error = uint8(1); */
            reset_numerical_error = 1;
        }
    }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void c_P_not_empty_init(void) { c_P_not_empty = false; }

/*
 * Arguments    : void
 * Return Type  : void
 */
void c_Xe_not_empty_init(void) { c_Xe_not_empty = false; }

/*
 * function ekfEuler_atti_arg = ekfEuler_atti(ekfEuler_atti_arg)
 * codegen  -config:lib -c ekfEuler_atti.m -args
 * {ekfEuler_atti_arg.z,ekfEuler_atti_arg.dt,ekfEuler_atti_arg.q_r,ekfEuler_atti_arg.q_w,ekfEuler_atti_arg.r}
 * %-O disable:inline phi and theta must be restrained within -pi/2 to pi/2
 * {
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       July 2, 2018
 *  @version    V2.5.8
 *  @since      add ekfEuler_atti_arg.r=ekfEuler_atti_arg.r*(1+((norm_z - gravity)*10)^6);
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       July 25, 2018
 *  @version    V2.5.9
 *  @since      add ekfEuler_atti_arg.reset
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.10
 *  @since      add reset_numerical_error and cond()
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      change logic for f
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      add max_wb
 * -------------------------------------------------------------------------------------
 *  @author     Zhongyuan Liu
 *  @date       Sep 30, 2018
 *  @version    V2.5.11
 *  @since      add accel_valid_counter_shreshold
 * -------------------------------------------------------------------------------------
 * }
 * Arguments    : const ekfEuler_atti_arg_Type *ekfEuler_atti_arg
 *                ekfEuler_atti_arg_Type *b_ekfEuler_atti_arg
 * Return Type  : void
 */
void ekfEuler_atti(const ekfEuler_atti_arg_Type *ekfEuler_atti_arg,
                   ekfEuler_atti_arg_Type *b_ekfEuler_atti_arg)
{
    *b_ekfEuler_atti_arg = *ekfEuler_atti_arg;
    b_ekfEuler_atti(b_ekfEuler_atti_arg);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekfEuler_atti_init(void)
{
    /* 'ekfEuler_atti:55' reset_numerical_error = uint8(0); */
    reset_numerical_error = 0;
}

/*
 * File trailer for ekfEuler_atti.c
 *
 * [EOF]
 */
