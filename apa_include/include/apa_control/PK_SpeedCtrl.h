
#ifndef PK_SPEEDCTRL_H
#define PK_SPEEDCTRL_H

#include "Std_Types.h"
typedef enum VehicleStateMachine
{
    VSM_PARKED             = 1,
    VSM_STARTUP_FORWARD    = 2,
    VSM_STARTUP_BACKWARD   = 3,
    VSM_SPEEDCTRL_FORWARD  = 4,
    VSM_SPEEDCTRL_BACKWARD = 5,
    VSM_PARKING            = 6,
    VSM_MANUAL             = 7,
    VSM_SEMI_MANUAL        = 8,
} PK_SpeedCtl_VehicleStateMachine;

typedef enum ParkedStateMachine
{
    wait = 0, // PK_EPB_SM_WAIT = 0
    pull = 1,
    hold = 2,
    push = 3,
} PK_SpeedCtl_ParkedStateMachineType;

typedef struct _ModelParam_
{
    float J;
    float R;
    float r;
    float tau;
    float rho;
    float A;
    float C_x;
    float m;
    float g;
    float f_R0;
    float f_R1;
    float f_R2;
} PK_SpeedCtl_MODELPARAM;
typedef struct _controlParam
{
    float brake;       // MPa
    float driveTorque; //[-3000,3000]Nm
} PK_SpeedCtl_ControlParamType;

typedef struct _SpdControl_
{
    double v_num_set[5], v_den_set[5];
    int torque_sign; // 扭矩的符号
    boolean initial;
    int direction;
    double m_uDesire;  //期望速度,单位m/s
                       //	IVMCONTROL m_controlParam;//控制量
    float v_est;       // estimated velocity
    float d_est;       // estimated disturbance
    float est_gain[2]; // estimated gain
    float tilde_Treq;  // 非线性补偿后的扭矩
    float pre_torque;
    float pre_v;
} PK_SpeedCtl_SpdControlType;

typedef struct _StartUp_
{
    float time_limit;
    float direction;
    float last_output;
    int current_time;
} PK_SpeedCtl_StartUpType;

typedef struct _Stopping_
{
    float current_time;
    float last_pressure;
    float stopping_time;
    float pressure_on_time;
    boolean stopped;
    float pre_Tor;
} PK_SpeedCtl_StoppingType;

typedef struct _SemiManual_
{
    float pre_Tor;
} PK_SpeedCtl_SemiManualType;

#include "MathFunc.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void PK_SpeedCtrl(void);

#ifdef __cplusplus
}
#endif

#endif
