#pragma section all "CPU1.Private"
#include "kalman_filter_group.h"
/********************Added by Zhongyuan Liu 20180105**************/ ////////////////////
                                                                    /////
/* Variable Definitions for kf1d_s*/                                //
static float kf1d_s_P  = 1.0F;                                      //
static float kf1d_s_Xe = 1600.0F;                                   // mass //
/* Variable Definitions for kf1d_roll*/                             //
static float kf1d_roll_P  = 1.0F;                                   //
static float kf1d_roll_Xe = 0.0F;                                   //
/* Variable Definitions for kf1d_pitch*/                            //
static float kf1d_pitch_P  = 1.0F;                                  //
static float kf1d_pitch_Xe = 0.0F;                                  //
                                                                    //
///////////////////////////////////////////////////////////////////////////////////////

/* Function Definitions */

/*
 * Arguments    : float z
 *                float t
 *                float q
 *                float r
 * Return Type  : float
 */
float kf1d_s(float z, float t, float q, float r)
{
    float K;
    r *= r;
    kf1d_s_P = 1.00000203F * kf1d_s_P + q * q * t;
    K        = kf1d_s_P / (kf1d_s_P + r);
    kf1d_s_Xe += K * (z - kf1d_s_Xe);
    kf1d_s_P -= K * kf1d_s_P;
    kf1d_s_P = 0.5F * (kf1d_s_P + kf1d_s_P);
    return kf1d_s_Xe;
}

/* Function Definitions */

/*
 * Arguments    : float z
 *                float t
 *                float q
 *                float r
 * Return Type  : float
 */
float kf1d_roll(float z, float t, float q, float r)
{
    float K;
    r *= r;
    kf1d_roll_P = 1.00000203F * kf1d_roll_P + q * q * t;
    K           = kf1d_roll_P / (kf1d_roll_P + r);
    kf1d_roll_Xe += K * (z - kf1d_roll_Xe);
    kf1d_roll_P -= K * kf1d_roll_P;
    kf1d_roll_P = 0.5F * (kf1d_roll_P + kf1d_roll_P);
    return kf1d_roll_Xe;
}
/* Function Definitions */

/*
 * Arguments    : float z
 *                float t
 *                float q
 *                float r
 * Return Type  : float
 */
float kf1d_pitch(float z, float t, float q, float r)
{
    float K;
    r *= r;
    kf1d_pitch_P = 1.00000203F * kf1d_pitch_P + q * q * t;
    K            = kf1d_pitch_P / (kf1d_pitch_P + r);
    kf1d_pitch_Xe += K * (z - kf1d_pitch_Xe);
    kf1d_pitch_P -= K * kf1d_pitch_P;
    kf1d_pitch_P = 0.5F * (kf1d_pitch_P + kf1d_pitch_P);
    return kf1d_pitch_Xe;
}
