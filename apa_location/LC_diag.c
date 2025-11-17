#include "PK_LocationImpl.h"
#include "PK_Calibration.h"
#include "Nav_math.h"
/*
    sort array descending
    param_in:
    ds_array: array to be sorted
    length:length of array
    return:
    index
*/
static unsigned char *sort_array(float ds_array[], unsigned char length)
{
    // sort array, from big to small
    unsigned char i, j, count, temp;
    float tempf;
    static unsigned char index[5]; // use length instead of 5 for a general case

    for (i = 0; i < length; i++)
    {
        index[i] = i;
    }

    count = 0;

    for (i = 0; i < length - 1; i++)
    {
        count++;

        for (j = 0; j < length - count; j++)
        {
            if (ds_array[j + 1] > ds_array[j])
            {
                temp  = index[j];
                tempf = ds_array[j];

                ds_array[j]     = ds_array[j + 1];
                ds_array[j + 1] = tempf;

                index[j]     = index[j + 1];
                index[j + 1] = temp;
            }
        }
    }

    return index;
}

/********************************************************************
NAME: IMU_ESC_IMU_diagnostic
FUNC: Note IMU takes more than 1 sec to startup after power is on，
      two IMU measure results check each other
PARA_IN:
    Accel_X，Accel_Y，Accel_Z
    AngRate_X，AngRate_Y，AngRate_Z
PARA_OUT:
    void
RETURN:
    return a err if check is not ok
********************************************************************/
static uint8 IMU_ESC_IMU_diagnostic(const float Accel_X, const float Accel_Y,
                                    const float Accel_Z, const float AngRate_X,
                                    const float AngRate_Y, const float AngRate_Z)
{
    float ESC_ActVehLongAccel = 0.0f;
    uint8 errcode             = 0;

    static Signal_diagnose_info error_state_IMU_Accel_X = {ST_OK, 0U, ST_OK, ST_OK, 10U};
    static Signal_diagnose_info error_state_IMU_Accel_Y = {ST_OK, 0U, ST_OK, ST_OK, 10U};
    static Signal_diagnose_info error_state_IMU_Accel_Z = {ST_OK, 0U, ST_OK, ST_OK, 10U};
    static Signal_diagnose_info error_state_IMU_AngRate_X = {ST_OK, 0U, ST_OK, ST_OK,
                                                             10U};
    static Signal_diagnose_info error_state_IMU_AngRate_Y = {ST_OK, 0U, ST_OK, ST_OK,
                                                             10U};
    static Signal_diagnose_info error_state_IMU_AngRate_Z = {ST_OK, 0U, ST_OK, ST_OK,
                                                             10U};
    static Signal_diagnose_info error_state_IMU_ESC_IMU = {ST_OK, 0U, ST_OK, ST_OK, 10U};
    static Signal_diagnose_info error_state_IMU_IMUERR  = {ST_OK, 0U, ST_OK, ST_OK, 10U};

    /****Cancel IMU diagnostics****
    error_state_IMU_Accel_X.cur_state = (fabsf(Accel_X) < EPLSION);
    count_error_signal(&error_state_IMU_Accel_X);
    errcode = SET_ERR_CODE(error_state_IMU_Accel_X.state,BIT(0), errcode);

    error_state_IMU_Accel_Y.cur_state = (fabsf(Accel_Y) < EPLSION);
    count_error_signal(&error_state_IMU_Accel_Y);
    errcode = SET_ERR_CODE(error_state_IMU_Accel_Y.state,BIT(1), errcode);

    error_state_IMU_Accel_Z.cur_state = (fabsf(Accel_Z) < EPLSION);
    count_error_signal(&error_state_IMU_Accel_Z);
    errcode = SET_ERR_CODE(error_state_IMU_Accel_Z.state,BIT(2), errcode);

    error_state_IMU_AngRate_X.cur_state = (fabsf(AngRate_X) < EPLSION);
    count_error_signal(&error_state_IMU_AngRate_X);
    errcode = SET_ERR_CODE(error_state_IMU_AngRate_X.state,BIT(3), errcode);

    error_state_IMU_AngRate_Y.cur_state = (fabsf(AngRate_Y) < EPLSION);
    count_error_signal(&error_state_IMU_AngRate_Y);
    errcode = SET_ERR_CODE(error_state_IMU_AngRate_Y.state,BIT(4), errcode);

    error_state_IMU_AngRate_Z.cur_state = (fabsf(AngRate_Z) < EPLSION);
    count_error_signal(&error_state_IMU_AngRate_Z);
    errcode = SET_ERR_CODE(error_state_IMU_AngRate_Z.state,BIT(5), errcode);
    */

    return errcode;
}

/********************************************************************
NAME: TMPS_Pressure_diagnostic
FUNC: check if TMPS_Pressure err
PARA_IN:
    PressureRR:real right  wheel pressure
    PressureRL:real left  wheel pressure
    PressureFR:front right wheel pressure
    PressureFL:front left wheel pressure
PARA_OUT:
    void
RETURN:
    uint8 err code
********************************************************************/
static uint8 TMPS_Pressure_diagnostic(float PressureFL, float PressureFR,
                                      float PressureRL, float PressureRR)
{
    const float low_pressure = 1.5f; // low  pressure threshhold
    uint8 errcode            = 0;
    uint8 errIDBegin         = 0;
    static Signal_diagnose_info tire_pressure_error_state_RR = {ST_OK, 0U, ST_OK, ST_OK,
                                                                5U};
    static Signal_diagnose_info tire_pressure_error_state_RL = {ST_OK, 0U, ST_OK, ST_OK,
                                                                5U};
    static Signal_diagnose_info tire_pressure_error_state_FR = {ST_OK, 0U, ST_OK, ST_OK,
                                                                5U};
    static Signal_diagnose_info tire_pressure_error_state_FL = {ST_OK, 0U, ST_OK, ST_OK,
                                                                5U};

    tire_pressure_error_state_RR.cur_state = (PressureRR < low_pressure);
    count_error_signal(&tire_pressure_error_state_RR);

    tire_pressure_error_state_RL.cur_state = (PressureRL < low_pressure);
    count_error_signal(&tire_pressure_error_state_RL);

    tire_pressure_error_state_FR.cur_state = (PressureFR < low_pressure);
    count_error_signal(&tire_pressure_error_state_FR);

    tire_pressure_error_state_FL.cur_state = (PressureFL < low_pressure);
    count_error_signal(&tire_pressure_error_state_FL);

    errIDBegin = WHEEL_PRESSURE_FL_ID; // No1 of 4
    errcode    = SET_ERR_CODE(tire_pressure_error_state_FL.state,
                              WHEEL_PRESSURE_FL_ID - errIDBegin, errcode);
    errcode    = SET_ERR_CODE(tire_pressure_error_state_FR.state,
                              WHEEL_PRESSURE_FR_ID - errIDBegin, errcode);
    errcode    = SET_ERR_CODE(tire_pressure_error_state_RL.state,
                              WHEEL_PRESSURE_RL_ID - errIDBegin, errcode);
    errcode    = SET_ERR_CODE(tire_pressure_error_state_RR.state,
                              WHEEL_PRESSURE_RR_ID - errIDBegin, errcode);

    return errcode;
}

/********************************************************************
NAME: MotorSpeed_diagnostic
FUNC: check if motorSpeed err
PARA_IN:
    MotorSpd
PARA_OUT:
    void
RETURN:
    uint8 err code
********************************************************************/
static uint8 MotorSpeed_diagnostic(float MotorSpd)
{
    uint8 errcode = 0;

    static Signal_diagnose_info motorspd_error_state = {ST_OK, 0U, ST_OK, ST_OK, 5U};
    static const int MAX_MOTO_SPEED                  = 12000; // motorSpd threshhold

    motorspd_error_state.cur_state = (fabsf(MotorSpd) > MAX_MOTO_SPEED);
    count_error_signal(&motorspd_error_state);

    errcode = SET_ERR_CODE(motorspd_error_state.state, BIT(0), errcode);

    return errcode;
}

/********************************************************************
NAME: EPSAngle_diagnostic
FUNC: check if EPSAngle err
PARA_IN:
    EPSAngle
PARA_OUT:
    void
RETURN:
    uint8 err code
********************************************************************/
static uint8 EPSAngle_diagnostic(float EPSAngle)
{
    uint8 errcode = 0;

    static Signal_diagnose_info EPS_angle_error_state = {ST_OK, 0U, ST_OK, ST_OK, 5U};
    static const int MAX_EPS_ANGEL                    = 550; // EPS threshHold

    EPS_angle_error_state.cur_state = (fabsf(EPSAngle) > MAX_EPS_ANGEL);
    count_error_signal(&EPS_angle_error_state);

    errcode = SET_ERR_CODE(EPS_angle_error_state.state, BIT(0), errcode);

    return errcode;
}

/********************************************************************
NAME: DCounter_diagnostic
FUNC: check if DCounter err
PARA_IN:
    DCounter
PARA_OUT:
    void
RETURN:
    uint8 err code
********************************************************************/
static uint8 DCounter_diagnostic(float carSpeed, float dCounter_FL, float dCounter_FR,
                                 int dCounter_RL, int dCounter_RR, float counterFactor,
                                 float deltaT)
{
    float carSpeed_abs              = 0;
    uint8 errcode                   = 0;
    uint8 errIDBegin                = 0;
    const float min_speed           = 1.5F;
    static const float expanFactorR = 3.5f; // 1.5f;
    static const float expanFactorF = 3.6f; // 1.6f;
    StateType SensorErrStt          = ST_OK;
    carSpeed_abs                    = fabsf(carSpeed);
    errIDBegin                      = WHEEL_FL_ID; // No1 of 4
    if (carSpeed_abs > min_speed)
    {
        SensorErrStt =
            (dCounter_FL * counterFactor > deltaT * carSpeed_abs * expanFactorF) ? ST_ERR
                                                                                 : ST_OK;
        errcode = SET_ERR_CODE(SensorErrStt, WHEEL_FL_ID - errIDBegin, errcode);
        SensorErrStt =
            (dCounter_FR * counterFactor > deltaT * carSpeed_abs * expanFactorF) ? ST_ERR
                                                                                 : ST_OK;
        errcode = SET_ERR_CODE(SensorErrStt, WHEEL_FR_ID - errIDBegin, errcode);
        SensorErrStt =
            (dCounter_RL * counterFactor > deltaT * carSpeed_abs * expanFactorR) ? ST_ERR
                                                                                 : ST_OK;
        errcode = SET_ERR_CODE(SensorErrStt, WHEEL_RL_ID - errIDBegin, errcode);
        SensorErrStt =
            (dCounter_RR * counterFactor > deltaT * carSpeed_abs * expanFactorR) ? ST_ERR
                                                                                 : ST_OK;
        errcode = SET_ERR_CODE(SensorErrStt, WHEEL_RR_ID - errIDBegin, errcode);
    }

    return errcode;
}

/********************************************************************
NAME: wheel_motor_diagnostics
FUNC: wheel motor diagnostics. contidtion: EPS_angle is small
PARA_IN:
   ds_motor：
   dCounter_FR:
   dCounter_FL：
   dCounter_RR：
   dCounter_RL：
PARA_OUT:
   void
RETURN:
   wheel_motor_diagnostics status
********************************************************************/
static uint8 wheel_motor_diagnostics(float ds_motor, float dCounter_FL, float dCounter_FR,
                                     int dCounter_RL, int dCounter_RR)
{
    // to do: get Front_wheel_fac_ds
    static float s_FL = 0.0f, s_FR = 0.0f, s_RL = 0.0f, s_RR = 0.0f, s_motor = 0.0f;
    static float ds_array[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // distance from mean
    static float s_array[5]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // distance from mean
    float mean_s = 0.0f, mean_s_ = 0.0f;
    const float s_max        = 10.0f;
    float Front_wheel_fac_ds = 1.0f;
    const float shreshold    = 0.4f;
    unsigned char *index;
    uint8 errcode               = 0;
    static uint32 counter       = 0;
    static uint32 error_counter = 0;
    static uint32 max_count;
    max_count = (int)(1.0f / LocatingPeriod_dt); // 2 second

    Front_wheel_fac_ds = Rear_wheel_fac_ds; // to do use AllData.Rear_wheel_fac_ds

    if (counter < max_count)
    {
        s_FL    = s_FL + Front_wheel_fac_ds * (float)dCounter_FL * 2.0f;
        s_FR    = s_FR + Front_wheel_fac_ds * (float)dCounter_FR * 2.0f;
        s_RL    = s_RL + Rear_wheel_fac_ds * (float)dCounter_RL * 2.0f;
        s_RR    = s_RR + Rear_wheel_fac_ds * (float)dCounter_RR * 2.0f;
        s_motor = s_motor + fabsf(ds_motor);
        counter++;
    }
    else
    {
        counter = 0;
    }

    s_array[0] = s_FL + 10.0f;
    s_array[1] = s_FR + 10.0f;
    s_array[2] = s_RL + 10.0f;
    s_array[3] = s_RR + 10.0f;
    s_array[4] = s_motor + 10.0f;

    // if((s_FL>=s_max)||(s_FR>=s_max)||(s_RL>=s_max)||(s_RR>=s_max)||(s_motor>=s_max))//triger
    // diagnosing
    if (counter == 0)
    {
        mean_s = (s_FL + s_FR + s_RL + s_RR + s_motor) / 5.0f + 10.0f;

        ds_array[0] = fabsf(s_FL - mean_s);
        ds_array[1] = fabsf(s_FR - mean_s);
        ds_array[2] = fabsf(s_RL - mean_s);
        ds_array[3] = fabsf(s_RR - mean_s);
        ds_array[4] = fabsf(s_motor - mean_s);
        index       = sort_array(ds_array, 5); //

        mean_s_ = (mean_s * 5.0f - s_array[index[0]]) / 4.0f; // mean after remove the max

        if (((fabsf(mean_s_ - s_array[index[0]])) / fabsf(mean_s_)) >
            shreshold) // error% > shreshold
        {
            errcode = 1 << index[0]; // each bit indicates the corresponding sensor, 0 is
                                     // normal, 1 is error.
            // to do: fix temporary error
            error_counter++;
            if (error_counter > 100)
            {
                error_counter = 0;
            }
        }
        s_FL    = 0.0f;
        s_FR    = 0.0f;
        s_RL    = 0.0f;
        s_RR    = 0.0f;
        s_motor = 0.0f;
    }
    return errcode;
}

/**
 * @brief ???????????????????
 *
 * ???????????????????????????????????????????????
 * ???????
 * - IMU?????? & ????
 * - TPMS????4 ????
 * - ????
 * - EPS ??
 * - ????????????????????
 *
 * ????? `SET_ERR_CODE` ?????? `uint32` ??????????????????
 *
 * @param[in]  func_en    ???????ENABLED / NOT_ENABLED?
 * @param[in]  pAllData   ?????????????????????
 * @param[out] pErrCode   ????????????? OR ?????
 * @retval     RTN_OK     ??????
 */
RtnType DirectSensorDiagnostics(IsEnabled func_en, AllDataType *pAllData,
                                uint32 *pErrCode)
{
    IMU_DataType imu_data;
    TPMS_PresType TPMS_data;
    StateType SensorErrStt = ST_OK;
    uint8 diagnosticCode   = 0;
    uint32 errcode         = 0;

    CHECK_FUNC_EN(func_en);

    // 1,diagnostic imu data
    memcpy(&imu_data, &(pAllData->SensorMeasurements.IMU_Data), sizeof(imu_data));
    diagnosticCode = IMU_ESC_IMU_diagnostic(
        imu_data.Accel_X, imu_data.Accel_Y, imu_data.Accel_Z, imu_data.AngRate_X_deg,
        imu_data.AngRate_Y_deg, imu_data.AngRate_Z_deg);

    SensorErrStt = (diagnosticCode == 0) ? ST_OK : ST_ERR;
    errcode      = SET_ERR_CODE(SensorErrStt, IMU_ID, errcode);

    // 2,diagnostic TPMS_Pressure
    memcpy(&TPMS_data, &(pAllData->SensorMeasurements.TPMS_Pressure),
           sizeof(TPMS_PresType));

    diagnosticCode = TMPS_Pressure_diagnostic(TPMS_data.PressureRR, TPMS_data.PressureRL,
                                              TPMS_data.PressureFR, TPMS_data.PressureFL);
    diagnosticCode = 0; // to do remove
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(0));
    errcode        = SET_ERR_CODE(SensorErrStt, WHEEL_PRESSURE_FL_ID, errcode);
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(1));
    errcode        = SET_ERR_CODE(SensorErrStt, WHEEL_PRESSURE_FR_ID, errcode);
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(2));
    errcode        = SET_ERR_CODE(SensorErrStt, WHEEL_PRESSURE_RL_ID, errcode);
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(3));
    errcode        = SET_ERR_CODE(SensorErrStt, WHEEL_PRESSURE_RR_ID, errcode);

    // 3,diagnostic motorspeed
    diagnosticCode = MotorSpeed_diagnostic(pAllData->SensorMeasurements.Motorspd);
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(0));
    errcode        = SET_ERR_CODE(SensorErrStt, MOTOR_SPEED_ID, errcode);

    // 4,diagnostic EPS angle
    diagnosticCode = MotorSpeed_diagnostic(pAllData->SensorMeasurements.EPS_Angle);
    SensorErrStt   = (StateType)GET_BIT(diagnosticCode, BIT(0));
    errcode        = SET_ERR_CODE(SensorErrStt, EPS_ANGLE_ID, errcode);

    // 5,diagnostic wheel counter
    // there is no diagnostic,diagnostic wheel counter behind.
    *pErrCode = errcode;
    return RTN_OK;
}

/**
 * @brief ?????????
 *
 * ???????????????????????????????
 *
 * @param[in] func_en ????
 * @param[in] pAllData ??????????
 * @param[out] pErrCode ???????
 *
 * @return RTN_OK ??????
 */
RtnType IndirectDirectSensorDiagnostics(IsEnabled func_en, AllDataType *pAllData,
                                        uint32 *pErrCode)
{
    static const float SPEED_BOUND = 36.0F;
    StateType SensorErrStt         = ST_OK;
    uint8 diagnosticCode           = 0;
    uint32 errcode                 = 0;
    float couterFactor             = 0.0f; // couter * couterFactor *2 = ds
    float deltaT                   = 0.01f;

    ESC_DataType *pEscData      = &(pAllData->SensorMeasurements.ESC_Data);
    LinearMotionType *pLineData = &(pAllData->LinearMotionData);
    CHECK_FUNC_EN(func_en);

    deltaT       = pAllData->location_dt;
    couterFactor = pAllData->EPPROM_Data.Rear_wheel_fac_ds;
    // 1,dCounter Diagnostics
    diagnosticCode = DCounter_diagnostic(
        pLineData->SpeedForward,
        pEscData->dCounter_FL * pAllData->LinearMotionData.R_F2R_Rate,
        pEscData->dCounter_FR * pAllData->LinearMotionData.R_F2R_Rate,
        pEscData->dCounter_RL, pEscData->dCounter_RR, couterFactor, deltaT);
    // diagnosticCode = 0;//to do remove
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(0));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_FL_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(1));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_FR_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(2));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_RL_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(3));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_RR_ID, errcode);

    // 2,motor speed,wheel speed and ESC speed Diagnostics
    if (pEscData->CounterState == ST_OK && pEscData->ESC_VehSpd < SPEED_BOUND)
    {
        diagnosticCode = wheel_motor_diagnostics(
            pAllData->LinearMotionData.ds_motor,
            pEscData->dCounter_FL * pAllData->LinearMotionData.R_F2R_Rate,
            pEscData->dCounter_FR * pAllData->LinearMotionData.R_F2R_Rate,
            pEscData->dCounter_RL, pEscData->dCounter_RR);
    }

    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(0));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_FL_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(1));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_FR_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(2));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_RL_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(3));
    errcode      = SET_ERR_CODE(SensorErrStt, WHEEL_RR_ID, errcode);
    SensorErrStt = (StateType)GET_BIT(diagnosticCode, BIT(4));
    errcode      = SET_ERR_CODE(SensorErrStt, MOTOR_SPEED_ID, errcode);

    *pErrCode = errcode;

    return RTN_OK;
}

/**
 * @brief ??????
 *
 * ??IMU??????????????????????????????????????????
 * ??????????????????????????????
 *
 * @param[in]  func_en  ??????
 * @param[in,out] pAllData ?????????????????
 *
 * @return RTN_OK ????????
 */
RtnType Collision_detect(IsEnabled func_en, AllDataType *pAllData)
{
    float Accel_X_Var_max, Accel_Y_Var_max, Accel_Z_Var_max, Accel_X_E_max, Accel_Y_E_max,
        Accel_Z_E_max;
    const float radar_min_shreshold = 260.0F;
    const int delay_T               = 10;
    float IMU_data[3]               = {0.0F, 0.0F, 0.0F};
    static float Accel_X_Var = 0.0f, Accel_Y_Var = 0.0f, Accel_Z_Var = 0.0f,
                 Accel_X_E = 0.0f, Accel_Y_E = 0.0f, Accel_Z_E = 0.0f;
    static float AccX_Var_delay = 0.0f, AccY_Var_delay = 0.0f, AccZ_Var_delay = 0.0f,
                 AccX_E_delay = 0.0f, AccY_E_delay = 0.0f, AccZ_E_delay = 0.0f;
    static uint16 Collision_count = 0, Collision_sum = 0;
    static boolean Collision_flag = FALSE;

    static float delay_acc_e[10][3] = {0.0f}, delay_acc_var[10][3] = {0.0f};
    int j;
    static int still_count = 0, door_count = 0, door_state = 0, door_state_last = 0;
    static unsigned char door_change_flag = 0, reset_first = 1;
    uint8 Is_PK_Enable;

    IMU_DataType pIMU_Data     = pAllData->SensorMeasurements.IMU_Data;
    DoorStateType DoorState_in = pAllData->SensorMeasurements.DoorState;
    U_RadarType radar_data     = pAllData->SensorMeasurements.radar_data;
    Vel_MoveMentStType MoveSt  = pAllData->LinearMotionData.Vel_MoveMentSt;
    Is_PK_Enable               = pAllData->SensorMeasurements.Is_PK_Enable;

    IMU_data[0] = pIMU_Data.Accel_X;
    IMU_data[1] = pIMU_Data.Accel_Y;
    IMU_data[2] = pIMU_Data.Accel_Z;

    if (MoveSt == LOCAT_STILL && reset_first == 1)
    {
        reset_first = 0;
        Accel_X_E   = 0.0f;
        Accel_Y_E   = 0.0f;
        Accel_Z_E   = 0.0f;
        Accel_X_Var = 0.0f;
        Accel_Y_Var = 0.0f;
        Accel_Z_Var = 0.0f;
    }
    if (MoveSt != LOCAT_STILL)
    {
        reset_first = 1;
    }

    Math_UpdateEVar(IMU_data[0], &Accel_X_E, &Accel_X_Var, 10);
    Math_UpdateEVar(IMU_data[1], &Accel_Y_E, &Accel_Y_Var, 10);
    Math_UpdateEVar(IMU_data[2], &Accel_Z_E, &Accel_Z_Var, 10);

    for (j = delay_T - 1; j > 0; j--)
    {
        delay_acc_var[j][0] = delay_acc_var[j - 1][0];
        delay_acc_var[j][1] = delay_acc_var[j - 1][1];
        delay_acc_var[j][2] = delay_acc_var[j - 1][2];

        delay_acc_e[j][0] = delay_acc_e[j - 1][0];
        delay_acc_e[j][1] = delay_acc_e[j - 1][1];
        delay_acc_e[j][2] = delay_acc_e[j - 1][2];
    }
    delay_acc_e[0][0]   = Accel_X_E;
    delay_acc_e[0][1]   = Accel_Y_E;
    delay_acc_e[0][2]   = Accel_Z_E;
    delay_acc_var[0][0] = Accel_X_Var;
    delay_acc_var[0][1] = Accel_Y_Var;
    delay_acc_var[0][2] = Accel_Z_Var;

    door_state = DoorState_in.BCM_DriverDoorAjarSt + DoorState_in.BCM_RRDoorAjarSt +
                 DoorState_in.BCM_RLDoorAjarSt + DoorState_in.BCM_PsngrDoorAjarSt;
    if (door_state != door_state_last)
    {
        door_count       = 0;
        door_change_flag = 1;
    }
    else
    {
        if (door_count < 100)
            door_count++;
        if (door_count >= 100)
            door_change_flag = 0;
    }
    door_state_last = door_state;

    if ((door_change_flag == 0) &&
        ((pAllData->SensorMeasurements.GearSt == GEAR_P) ||
         radar_data.FOL <= radar_min_shreshold || radar_data.FCL <= radar_min_shreshold ||
         radar_data.FCR <= radar_min_shreshold || radar_data.FOR <= radar_min_shreshold ||
         radar_data.ROL <= radar_min_shreshold || radar_data.RCL <= radar_min_shreshold ||
         radar_data.RCR <= radar_min_shreshold || radar_data.ROR <= radar_min_shreshold ||
         radar_data.FSR <= radar_min_shreshold || radar_data.FSL <= radar_min_shreshold ||
         radar_data.RSR <= radar_min_shreshold || radar_data.RSL <= radar_min_shreshold))
    {
        if (Is_PK_Enable == 1)
        {
            Accel_X_Var_max = 2.0f;
            Accel_Y_Var_max = 1.5f;
            Accel_Z_Var_max = 1.0f;
            Accel_X_E_max   = 3.0f;
            Accel_Y_E_max   = 1.0f;
            Accel_Z_E_max   = 0.4f;
        }
        else
        {
            Accel_X_Var_max = 6.0f;
            Accel_Y_Var_max = 4.0f;
            Accel_Z_Var_max = 2.0f;
            Accel_X_E_max   = 5.0f;
            Accel_Y_E_max   = 3.0f;
            Accel_Z_E_max   = 0.4f;
            if (MoveSt == LOCAT_STILL && pAllData->SensorMeasurements.GearSt == GEAR_P)
            {
                if (still_count < 200)
                    still_count++;
                if (still_count >= 200)
                {
                    Accel_X_Var_max = 0.3f;
                    Accel_Y_Var_max = 0.1f;
                    Accel_Z_Var_max = 0.3f;
                    Accel_X_E_max   = 0.3f;
                    Accel_Y_E_max   = 0.3f;
                    Accel_Z_E_max   = 0.2f;
                }
            }
            else
            {
                still_count = 0;
            }
        }

        if (door_change_flag == 0)
        {
            AccX_E_delay   = delay_acc_e[delay_T - 1][0];
            AccY_E_delay   = delay_acc_e[delay_T - 1][1];
            AccZ_E_delay   = delay_acc_e[delay_T - 1][2];
            AccX_Var_delay = delay_acc_var[delay_T - 1][0];
            AccY_Var_delay = delay_acc_var[delay_T - 1][1];
            AccZ_Var_delay = delay_acc_var[delay_T - 1][2];
        }
        else
        {
            AccX_E_delay   = delay_acc_e[0][0];
            AccY_E_delay   = delay_acc_e[0][1];
            AccZ_E_delay   = delay_acc_e[0][2];
            AccX_Var_delay = delay_acc_var[0][0];
            AccY_Var_delay = delay_acc_var[0][1];
            AccZ_Var_delay = delay_acc_var[0][2];
        }

        if ((AccX_Var_delay > Accel_X_Var_max || AccY_Var_delay > Accel_Y_Var_max) &&
            AccZ_Var_delay < Accel_Z_Var_max &&
            (fabsf(AccX_E_delay) > Accel_X_E_max ||
             fabsf(AccY_E_delay) > Accel_Y_E_max) &&
            fabsf(AccZ_E_delay - 9.80f * cosf(pAllData->RotationalMotionData.pitch_rad)) <
                Accel_Z_E_max)
        {
            Collision_count    = 0;
            Collision_flag_ccp = 1;
            Collision_flag     = TRUE;
        }
        else
        {
            if (Collision_count < 10)
                Collision_count++;
            if (Collision_count >= 10)
            {
                Collision_flag_ccp = 0;
                Collision_flag     = FALSE;
            }
        }

        IMU_acc_X_E_ccp   = Accel_X_E;
        IMU_acc_Y_E_ccp   = Accel_Y_E;
        IMU_acc_Z_E_ccp   = Accel_Z_E;
        IMU_acc_X_Var_ccp = Accel_X_Var;
        IMU_acc_Y_Var_ccp = Accel_Y_Var;
        IMU_acc_Z_Var_ccp = Accel_Z_Var;
    }
    if (Collision_flag_ccp == 1)
    {
        Collision_sum++;
        if (Collision_sum > 10)
        {
            Collision_sum      = 0;
            Collision_flag     = FALSE;
            Collision_flag_ccp = 0;
        }
    }
    pAllData->LinearMotionData.Collision_flag = Collision_flag;
    return RTN_OK;
}
