/**
 * @file LC_calib.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#pragma section all "CPU1.Private"
#include "PK_LocationImpl.h"
#include "PK_Calibration.h"

/********************************************************************
NAME: calEPSOffset
FUNC: cal EPS  zero offset
PARA_IN:
    carspd: vehicle speed
    carAngle: vehicle trace angle
    EPSAngle: EPS angle

PARA_OUT:
    pCal: single calib result
RETURN:
    flag whether cal coef is valid
********************************************************************/
static unsigned char calEPSOffset(float carspd, float carAngle, float EPSAngle,
                                  float *pCal)
{
    static unsigned short addCounter = 0;
    static float sumRes              = 0;
    static float maxCarAngle         = -1000.0f;
    static float minCarAngle         = 1000.0f;
    static float maxEPSAngle         = -1000.0f;
    static float minEPSAngle         = 1000.0f;
    const unsigned short winNum      = 100;
    uint8 calFlg                     = 0;
    if (0 == addCounter)
    {
        maxCarAngle = carAngle;
        minCarAngle = carAngle;
        maxEPSAngle = EPSAngle;
        minEPSAngle = EPSAngle;
        sumRes      = 0;
    }
    else
    {
        maxCarAngle = maxCarAngle < carAngle ? carAngle : maxCarAngle;
        minCarAngle = minCarAngle > carAngle ? carAngle : minCarAngle;
        maxEPSAngle = maxEPSAngle < EPSAngle ? EPSAngle : maxEPSAngle;
        minEPSAngle = minCarAngle > EPSAngle ? EPSAngle : minEPSAngle;
    }
    sumRes += EPSAngle;
    addCounter++;
    calFlg = 0;
    if (addCounter >= winNum)
    {
        addCounter = 0;
        if ((carspd > 10) && ((maxCarAngle - minCarAngle)) < 0.2f &&
            ((maxEPSAngle - minEPSAngle) < 1.0f))
        {
            *pCal  = sumRes / winNum;
            calFlg = 1;
        }
    }
    return calFlg;
}

/********************************************************************
NAME: calibEPSOffset
FUNC: cal EPS  zero offset
PARA_IN:
    ESC_VehSpd: vehicle speed from ESC
    carAngle: vehicle trace angle
    EPSAngle: EPS angle
PARA_OUT:
    pCal: many times calib result and cal mean val
RETURN:
    flag whether cal coef is valid
********************************************************************/
static ValidType calibEPSOffset(float carSpd, float carAngle, float EPSAngle,
                                float *pEPSOffset)
{
    float eps_offset_cal           = 0.0f;
    uint8 fac_cal_flg              = NotValid;
    ValidType calib_eps_offset_flg = NotValid;
    const float eps_offset_cal_Max = 6.0f;
    const float eps_offset_cal_Min = -6.0f;
    // static uint32 counter = 10000;
    static float EPSZeroOffset = 0;

    carAngle    = carAngle * RAD2DEG;
    fac_cal_flg = calEPSOffset(carSpd, carAngle, EPSAngle, &eps_offset_cal);
    if (eps_offset_cal > eps_offset_cal_Max || eps_offset_cal < eps_offset_cal_Min)
    {
        fac_cal_flg = NotValid;
    }
    if (Valid == fac_cal_flg)
    {
        EPSZeroOffset        = *pEPSOffset;
        EPSZeroOffset        = 0.001 * eps_offset_cal + 0.999 * EPSZeroOffset;
        *pEPSOffset          = EPSZeroOffset;
        calib_eps_offset_flg = Valid;
    }

    return calib_eps_offset_flg;
}

/********************************************************************
NAME: calWheelR
FUNC: cal wheel R coef and ESC speed coef,real time
PARA_IN:
    carspd:
    dCounter_RL:
    dCounter_RR:
    GPSvel:
PARA_OUT:
    pDSCal : wheel R coef
    pESCCoef : ESC Speed coef
RETURN:
    flag whether cal coef is valid
********************************************************************/
static unsigned char calWheelR(float carSpd, int Motorspd, float ESC_VehSpd,
                               int dCounter_RL, int dCounter_RR, float GPSvel,
                               unsigned char GPSQual, float *pDSCal, float *pESCCoef,
                               float *pMotorCoef)
{
    static unsigned short addCounter = 0;
    static int sumRes                = 0;
    static int sumDConterMotor       = 0;
    static float maxSpd              = 0.0f;
    static float minSpd              = 1000.0f;
    int curdCounter                  = 0;
    const unsigned short winNum      = 50;
    uint8 calFlg                     = 0;

    curdCounter = (dCounter_RR + dCounter_RL);

    if (0 == addCounter)
    {
        maxSpd          = carSpd;
        minSpd          = carSpd;
        sumRes          = 0;
        sumDConterMotor = 0;
    }
    else
    {
        maxSpd = maxSpd < carSpd ? carSpd : maxSpd;
        minSpd = minSpd > carSpd ? carSpd : minSpd;
    }

    sumRes += curdCounter;
    sumDConterMotor += Motorspd;
    addCounter++;
    calFlg = 0;

    if (addCounter >= winNum)
    {
        addCounter = 0;
        if ((1 == GPSQual) && (carSpd > 10) && ((maxSpd - minSpd) < 0.3f) &&
            (0 != sumRes) && (0 != sumDConterMotor))
        {
            *pDSCal     = GPSvel * winNum * LocatingPeriod_dt / (float)sumRes;
            *pESCCoef   = GPSvel / ESC_VehSpd;
            *pMotorCoef = GPSvel * winNum / (float)sumDConterMotor;
            if ((*pDSCal < Rear_wheel_fac_ds_max) && (*pDSCal > Rear_wheel_fac_ds_min))
            {
                calFlg = 1;
            }
        }
    }
    return calFlg;
}

/********************************************************************
NAME: calibWheelR
FUNC: after calWheelR N times ,this function average results
PARA_IN:
    ESC_VehSpd:
    dCounter_RL:
    dCounter_RR:
    GPSvel:
PARA_OUT:
    pFacWheelDs : wheel R coef
    pFacESCspd : ESC Speed coef
RETURN:
    flag whether cal calib coef is valid
********************************************************************/
static ValidType calibWheelR(float carSpd, int Motorspd, float ESC_VehSpd,
                             int dCounter_RL, int dCounter_RR, float GPSvel,
                             unsigned char GPSQual, float *pFacWheelDs, float *pFacESCspd,
                             float *pMotorSpd)
{
    float Rear_wheel_fac_ds_cal            = 0.0f;
    float ESC_VehSpd_coef                  = 0.0f;
    float Motor_VehSpd_coef                = 0.0f;
    static int num_cal_rear_wheel_fac      = 0;
    static float rear_wheel_fac_ds_calsum  = 0.0f;
    static float Rear_wheel_fac_ds_calmean = 0.0f;
    static float ESC_VehSpd_coef_calsum    = 0.0f;
    static float ESC_VehSpd_coef_calmean   = 0.0f;
    static float Motor_VehSpd_coef_calsum  = 0.0f;
    static float Motor_VehSpd_coef_calmean = 0.0f;
    uint8 fac_cal_flg                      = NotValid;
    ValidType calib_wheelR_flg             = NotValid;
    const unsigned short sumNum            = 20;
    const float ESC_VehSpd_coef_cal_Max    = 1.03428f;
    const float ESC_VehSpd_coef_cal_Min    = 0.99372f;
    const float Motor_VehSpd_coef_cal_Max  = 0.0042469284f;
    const float Motor_VehSpd_coef_cal_Min  = 0.0040869284f;
    const float Rear_wheel_fac_ds_max      = 0.01106951f;
    const float Rear_wheel_fac_ds_min      = 0.01063541f;

    static Signal_diagnose_info IsGPS_fail_state = {ST_OK, 0U, ST_OK, ST_OK, 500U};
    IsGPS_fail_state.cur_state = (fabsf(GPSvel - (float)(int)GPSvel) < 1.0E-4F);
    count_error_signal(&IsGPS_fail_state);

    fac_cal_flg =
        calWheelR(carSpd, Motorspd, ESC_VehSpd, dCounter_RL, dCounter_RR, GPSvel, GPSQual,
                  &Rear_wheel_fac_ds_cal, &ESC_VehSpd_coef, &Motor_VehSpd_coef);
    if (fac_cal_flg && num_cal_rear_wheel_fac < sumNum)
    {
        num_cal_rear_wheel_fac++;
        rear_wheel_fac_ds_calsum += Rear_wheel_fac_ds_cal;
        ESC_VehSpd_coef_calsum += ESC_VehSpd_coef;
        Motor_VehSpd_coef_calsum += Motor_VehSpd_coef;
    }

    if (num_cal_rear_wheel_fac >= sumNum)
    {
        num_cal_rear_wheel_fac    = 0;
        Rear_wheel_fac_ds_calmean = rear_wheel_fac_ds_calsum / sumNum;
        rear_wheel_fac_ds_calsum  = 0.0f;
        ESC_VehSpd_coef_calmean   = ESC_VehSpd_coef_calsum / sumNum;
        ESC_VehSpd_coef_calsum    = 0.0f;
        Motor_VehSpd_coef_calmean = Motor_VehSpd_coef_calsum / sumNum;
        Motor_VehSpd_coef_calsum  = 0.0f;
        calib_wheelR_flg          = Valid;

        if (Rear_wheel_fac_ds_calmean > Rear_wheel_fac_ds_max ||
            Rear_wheel_fac_ds_calmean < Rear_wheel_fac_ds_min ||
            IsGPS_fail_state.state == ST_ERR)
        {
            calib_wheelR_flg = NotValid;
        }
        if (ESC_VehSpd_coef_calmean > ESC_VehSpd_coef_cal_Max ||
            ESC_VehSpd_coef_calmean < ESC_VehSpd_coef_cal_Min ||
            IsGPS_fail_state.state == ST_ERR)
        {
            calib_wheelR_flg = NotValid;
        }
        if (Motor_VehSpd_coef_calmean > Motor_VehSpd_coef_cal_Max ||
            Motor_VehSpd_coef_calmean < Motor_VehSpd_coef_cal_Min ||
            IsGPS_fail_state.state == ST_ERR)
        {
            calib_wheelR_flg = NotValid;
        }
        if (Valid == calib_wheelR_flg)
        {
            *pFacWheelDs = Rear_wheel_fac_ds_calmean;
            *pFacESCspd  = ESC_VehSpd_coef_calmean;
            *pMotorSpd   = Motor_VehSpd_coef_calmean;
        }
    }

    return calib_wheelR_flg;
}

/**
 * @brief 自动标定函数
 *
 * 根据传感器数据对EPS零点偏移、车轮脉冲因子和ESC速度偏差率进行标定
 *
 * @param func_en 函数使能
 * @param pAllData 位置传感器数据和计算数据指针
 * @return RtnType 返回函数状态，通常为 RTN_OK
 */
RtnType AutoCalibration(IsEnabled func_en, AllDataType *pAllData)
{
    float ESC_VehSpd = 0.0f;
    float carSpd     = 0.0f;
    int Motorspd     = 0;
    float carAngle = 0.0f, EPSAngle = 0.0f, GPSvel = 0.0f;
    int dCounter_RL = 0, dCounter_RR = 0;
    unsigned char GPSQual = 0;
    float EPSZeroOffset = 0, WheelDSfactor = 0, ESCspdFcator = 0, MotorspdFcator = 0;
    ValidType vlidFlag = NotValid;

    CHECK_FUNC_EN(func_en);

    // 1,get sensors data
    ESC_VehSpd  = pAllData->SensorMeasurements.ESC_Data.ESC_VehSpd;
    ESC_VehSpd  = ESC_VehSpd * KMH2MS;
    carSpd      = pAllData->LinearMotionData.SpeedForward;
    carAngle    = pAllData->LocationData.heading;
    EPSAngle    = pAllData->SensorMeasurements.EPS_Angle;
    GPSvel      = pAllData->SensorMeasurements.GPS_Data.GPS_Speed;
    GPSQual     = pAllData->SensorMeasurements.GPS_Data.GPS_Accuracy;
    dCounter_RL = pAllData->SensorMeasurements.ESC_Data.dCounter_RL;
    dCounter_RR = pAllData->SensorMeasurements.ESC_Data.dCounter_RR;
    Motorspd    = pAllData->SensorMeasurements.Motorspd;

    // 2,calib EPS zero offset
    EPSZeroOffset = pAllData->EPPROM_Data.EPS_Offset;
    vlidFlag      = calibEPSOffset(carSpd, carAngle, EPSAngle, &EPSZeroOffset);
    if (Valid == vlidFlag)
    {
        pAllData->EPPROM_Data.EPS_Offset = EPSZeroOffset;
    }
    // calib wheel pulse value and ESC Speed deviate rate
    vlidFlag = calibWheelR(carSpd, Motorspd, ESC_VehSpd, dCounter_RL, dCounter_RR, GPSvel,
                           GPSQual, &WheelDSfactor, &ESCspdFcator, &MotorspdFcator);
    if (Valid == vlidFlag)
    {
        pAllData->EPPROM_Data.Rear_wheel_fac_ds = WheelDSfactor;
        pAllData->EPPROM_Data.ESC_VehSpd_factor = ESCspdFcator;
    }
    return RTN_OK;
}
