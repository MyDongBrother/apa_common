/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mass_estimate.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 12-Apr-2018 19:46:10
 */

/* Include Files */
#include "mass_estimate.h"

/* Variable Definitions */
static float mass;
static float accel_array[50];
static boolean_T accel_array_not_empty;
static float torq_array[50];
static unsigned int counter;
static unsigned int idx;
static unsigned int N_samples;

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void accel_array_not_empty_init(void) { accel_array_not_empty = false; }

/*
 * function x=mass_estimate(R,r,accel, pitch,torq, reset)
 * Arguments    : float R
 *                float r
 *                float accel
 *                float pitch
 *                float torq
 *                float reset
 *                float y[2]
 * Return Type  : void
 */
void mass_estimate(float R, float r, float accel, float pitch, float torq, float reset,
                   float y[2])
{
    int r1;
    float c;
    float A[100];
    int r2;
    float b_c[4];
    int k;
    float B[100];
    float a21;
    float a22;
    float x[2];
    float c_c[50];
    (void)pitch;

    /*  /$*********************************************************************************
     */
    /*  @brief Multiple Sensor Fusion And Multiple Tracking */
    /*  @note realize fusion and tracking of object detected by sensor and camera */
    /*  ***********************************************************************************
     */
    /*  $/ */
    /*  typedef struct _ModelParam_ */
    /*  { */
    /*    float J; // every 1Nm motor torque generates R/J vehicle acceleration in m/s^2
     */
    /*    float R; // gear ratio */
    /*    float r; // tire radius in m */
    /*    float rho; // mass density of air */
    /*    float A; // the frontal area of the vehicle */
    /*    float C_x; // the aerodynamic drag coefficient */
    /*    float m; // vehicle mass in kg */
    /*    float g; // gravity unit in m/s^2 */
    /*    float f_R0; // rolling resistance coefficient 0 */
    /*    float f_R1; // rolling resistance coefficient 1 */
    /*    float f_R2; // rolling resistance coefficient 2 */
    /*  }PK_SpeedCtl_ModelParamType; // a few vehicle parameters used in speed control */
    /*  static PK_SpeedCtl_ModelParamType m_modelParam ={640, 7.9f, 0.32f, 1.29f, 2.84f,
     * 0.33f, 1550, 9.8f, 0.014f, 0.02f, 0}; */
    /*  R = single(7.9); */
    /*  r = single(0.32); */
    /*  g = single(9.81); */
    /*  N = 20; */
    /*  accel = single(rand(1,1)); */
    /*  torq = single(80*accel+randn(1,1)*10); */
    /*  pitch = single(randn(1,1)*0.01); */
    /*  codegen  -config:lib -c mass_estimate.m -args {R,r,accel, pitch,torq} */
    /* 'mass_estimate:34' N = 50; */
    /* 'mass_estimate:35' if isempty(accel_array) || reset == uint8(1) */
    if ((!accel_array_not_empty) || (reset == 1.0F))
    {
        /* 'mass_estimate:36' accel_array = single(zeros(N,1)); */
        accel_array_not_empty = true;

        /* 'mass_estimate:37' pitch_array = single(zeros(N,1)); */
        /* 'mass_estimate:38' torq_array = single(zeros(N,1)); */
        memset(&accel_array[0], 0, 50U * sizeof(float));
        memset(&torq_array[0], 0, 50U * sizeof(float));

        /* 'mass_estimate:39' counter = uint32(1); */
        counter = 1U;

        /* 'mass_estimate:40' idx = uint32(1); */
        idx = 1U;
    }

    /* 'mass_estimate:43' if isempty(mass) */
    /*  accel_array(1:end-1) = accel_array(2:end); */
    /*  pitch_array(1:end-1) = pitch_array(2:end); */
    /*  torq_array(1:end-1) = torq_array(2:end); */
    /*  accel_array(end) = accel; */
    /*  % pitch_array(end) = pitch; */
    /*  torq_array(end) = torq; */
    /* 'mass_estimate:54' accel_array(idx) = accel; */
    accel_array[(int)idx - 1] = accel;

    /* 'mass_estimate:55' pitch_array(idx) = pitch; */
    /* 'mass_estimate:56' torq_array(idx) = torq; */
    torq_array[(int)idx - 1] = torq;

    /* 'mass_estimate:57' if idx == uint32(N) */
    if (idx == 50U)
    {
        /* 'mass_estimate:58' idx  = uint32(0); */
        idx = 0U;
    }

    /* 'mass_estimate:60' x = single([1600;10]); */
    /* 'mass_estimate:61' y = single([1600;10]); */
    for (r1 = 0; r1 < 2; r1++)
    {
        y[r1] = 1600.0F + -1590.0F * (float)r1;
    }

    /* 'mass_estimate:62' torq_array10 = torq_array(1:3:end); */
    /* 'mass_estimate:63' dtorq_array10 = torq_array10(2:end)-torq_array10(1:end-1); */
    /* 'mass_estimate:64' IsAscend = 0; */
    /* 'mass_estimate:66' if std(torq_array)<1 */
    /* 'mass_estimate:69' IsAscend = 1; */
    /* 'mass_estimate:70' if counter >= N */
    if (counter >= 50U)
    {
        /* 'mass_estimate:71' if IsAscend == 1 */
        /* 'mass_estimate:72' g = single(9.81); */
        /* 'mass_estimate:73' n_rows = single(length(accel_array)); */
        /* 'mass_estimate:74' A = single([accel_array repmat(R/r,n_rows,1)]); */
        c = R / r;
        for (r1 = 0; r1 < 50; r1++)
        {
            A[r1]      = accel_array[r1];
            A[50 + r1] = c;
        }

        /* 'mass_estimate:75' b = R/r*(torq_array-5); */
        c = R / r;

        /* 'mass_estimate:76' x = (A'*A)\A'*b; */
        for (r1 = 0; r1 < 2; r1++)
        {
            for (r2 = 0; r2 < 2; r2++)
            {
                b_c[r1 + (r2 << 1)] = 0.0F;
                for (k = 0; k < 50; k++)
                {
                    b_c[r1 + (r2 << 1)] += A[k + 50 * r1] * A[k + 50 * r2];
                }
            }
        }

        for (r1 = 0; r1 < 50; r1++)
        {
            for (r2 = 0; r2 < 2; r2++)
            {
                B[r2 + (r1 << 1)] = A[r1 + 50 * r2];
            }
        }

        if (fabsf(b_c[1]) > fabsf(b_c[0]))
        {
            r1 = 1;
            r2 = 0;
        }
        else
        {
            r1 = 0;
            r2 = 1;
        }

        a21 = b_c[r2] / b_c[r1];
        a22 = b_c[2 + r2] - a21 * b_c[2 + r1];
        for (k = 0; k < 50; k++)
        {
            A[1 + (k << 1)] = (B[r2 + (k << 1)] - B[r1 + (k << 1)] * a21) / a22;
            A[k << 1] = (B[r1 + (k << 1)] - A[1 + (k << 1)] * b_c[2 + r1]) / b_c[r1];
            c_c[k]    = c * (torq_array[k] - 5.0F);
        }

        for (r1 = 0; r1 < 2; r1++)
        {
            x[r1] = 0.0F;
            for (r2 = 0; r2 < 50; r2++)
            {
                x[r1] += A[r1 + (r2 << 1)] * c_c[r2];
            }
        }

        /* 'mass_estimate:77' if x(2)>=5 && x(2)<=20 */
        if ((x[1] >= 5.0F) && (x[1] <= 20.0F))
        {
            /* 'mass_estimate:78' N_samples = N_samples + 1; */
            N_samples++;

            /* 'mass_estimate:79' mass = x(1); */
            mass = x[0];

            /* 'mass_estimate:80' y(2) = x(2); */
            y[1] = x[1];

            /* 'mass_estimate:81' mass = (single(N_samples - 1) / single(N_samples)) *
             * mass + 1 / single(N_samples) * x(1) */
            mass = (float)(N_samples - 1U) / (float)N_samples * mass +
                   1.0F / (float)N_samples * x[0];

            /*              plot(accel_array*50); */
            /*              hold on */
            /*              plot(torq_array) */
            /*              title([num2str(mass) ' ' num2str(x(2)) ]) */
            /*              hold off */
        }
    }
    else
    {
        /* 'mass_estimate:91' else */
        /* 'mass_estimate:92' counter = counter + 1; */
        counter++;
    }

    /* 'mass_estimate:94' y(1) = mass; */
    y[0] = mass;

    /* 'mass_estimate:95' idx = idx + 1; */
    idx++;

    /* 'mass_estimate:96' if N_samples >= uint32(4294967295) */
    if (N_samples >= MAX_uint32_T)
    {
        /* 'mass_estimate:97' N_samples = uint32(0); */
        N_samples = 0U;
    }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void mass_estimate_init(void)
{
    /* 'mass_estimate:44' mass = single(1600); */
    mass = 1600.0F;

    /* 'mass_estimate:45' N_samples = uint32(0); */
    N_samples = 0U;
}

/*
 * File trailer for mass_estimate.c
 *
 * [EOF]
 */
