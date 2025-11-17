#ifndef LOCALIZATIONIMPL_H
#define LOCALIZATIONIMPL_H

#include "MathFunc.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Rte.h"
#include "PK_Location_Debug.h"
#include "PK_Calibration.h"
#include "Std_Types.h"
#include "Rte_ComIF.h"
#include "PK_Location.h"

#define DBG_STATIC static

#define DEG2RAD 0.01745329F
#define RAD2DEG 57.29578F
#define MS2KMH  3.6F
#define KMH2MS  0.2777778F
#define GRAVITY 9.8F
#define EPLSION 1.0E-20F

#define BIT(n) (n)

#define SET_BIT(x, n)                      ((x) | (1U << (n)))
#define GET_BIT(x, n)                      (((x) & (1U << (n))) >> (n))
#define CLEAR_BIT(x, n)                    ((x) & ~(1U << (n)))
#define GETBITS(x, n, m)                   ((x) & ((~((~0U) << (m - n))) << (n))) >> (n)
#define BYTE_LD(val)                       ((uint8_t)((uint16_t)(val) & 0xFF))
#define BYTE_HI(val)                       ((uint8_t)((uint16_t)(val) >> 8))
#define SHORT_LD(val)                      ((uint16_t)((uint32_t)(val) & 0xFFFF))
#define SHORT_HI(val)                      ((uint16_t)((uint32_t)(val) >> 16))
#define BTYE2SHORT(high, low)              ((((uint8_t)high & 0xFF) << 8) | ((uint8_t)low & 0xFF))
#define SHORT2WORD(high, low)              ((((uint16_t)high & 0xFF) << 16) | ((uint16_t)low & 0xFFFF))
#define DROUND_COUNTER(curv, lastv, range) (((curv) + (range) - (lastv)) % (range))

#define SET_ERR_CODE(errstt, errid, errcode) (((errstt) << (errid)) | (errcode))

typedef enum
{
    RTN_ERR = 1,
    RTN_OK  = 0
} RtnType;

typedef enum
{
    RESET     = 1,
    NOT_RESET = 0
} ResetType;

typedef enum
{
    ENABLED     = 1,
    NOT_ENABLED = 0
} IsEnabled;
typedef enum
{
    Valid    = 1,
    NotValid = 0
} ValidType;

typedef enum
{
    ST_ERR = 1,
    ST_OK  = 0
} StateType;

typedef enum
{
    BSTATE_NORMAL            = 1,  // state normal
    BSTATE_CALC_ERR          = 0,  // calculation invalid
    BSTATE_EPS_DIFFER_ERR    = -1, // EPS and differ match error
    BSTATE_MOTO_SPD_ERR      = -2, // moto speed error
    BSTATE_WHEEL_SPD_ERR     = -3, // wheel speed error
    BSTATE_IMU_ERR           = -4,
    BSTATE_WHEEL_COUNTER_ERR = -5,
    BSTATE_EKF_LC_reset      = 2,
    BSTATE_ESC_SPD_ERR       = -6
} BStateType;

#define CHECK_FUNC_EN(func_en)  \
    {                           \
        if (func_en != ENABLED) \
        {                       \
            return RTN_OK;      \
        }                       \
    }

/* Struct Definitions */
typedef enum
{
    LC_NORMAL,
    LC_ABNORM,
    LC_OFF,
    LC_DATA_RESET,
    LC_SENSOR_ERR
} Localization_state;

typedef enum
{
    WHEEL_FL_ID          = 1,
    WHEEL_FR_ID          = 2,
    WHEEL_RL_ID          = 3,
    WHEEL_RR_ID          = 4,
    EPS_ANGLE_ID         = 5,
    ESC_SPEED_ID         = 6,
    MOTOR_SPEED_ID       = 7,
    GPS_ID               = 8,
    WHEEL_PRESSURE_FL_ID = 9,
    WHEEL_PRESSURE_FR_ID = 10,
    WHEEL_PRESSURE_RL_ID = 11,
    WHEEL_PRESSURE_RR_ID = 12,
    IMU_ID               = 13
} ErrorID_type;

typedef struct
{
    StateType IMU_ST;
    StateType WHEEL_FL_ST;
    StateType WHEEL_FR_ST;
    StateType WHEEL_RL_ST;
    StateType WHEEL_RR_ST;
    StateType EPS_ANGLE_ST;
    StateType ESC_SPEED_ST;
    StateType MOTOR_SPEED_ST;
    StateType GPS_ST;
    StateType WHEEL_PRESSURE_FL_ST;
    StateType WHEEL_PRESSURE_FR_ST;
    StateType WHEEL_PRESSURE_RL_ST;
    StateType WHEEL_PRESSURE_RR_ST;
} ErrorStatetype;

typedef struct
{
    uint8_t cur_state;       // current state
    uint16_t state_counter;  //
    uint8_t state;           // no error
    uint8_t error_state_pre; // no error
    uint16_t window_size;
} Signal_diagnose_info;

typedef struct
{
    uint8_t
        BCM_DriverDoorAjarSt; // CCAN_Msg_375.BCM_DriverDoorAjarSt//BCAN_Msg_318.BCM_DriverDoorAjarSt
    uint8_t BCM_RRDoorAjarSt;    // BCAN_Msg_318.BCM_RRDoorAjarSt
    uint8_t BCM_RLDoorAjarSt;    // BCAN_Msg_318.BCM_RLDoorAjarSt
    uint8_t BCM_PsngrDoorAjarSt; // BCAN_Msg_318.BCM_PsngrDoorAjarSt
} DoorStateType;

typedef struct
{
    IsEnabled LocationGetEPPROM_EnableState;
    IsEnabled LocationInitialization_EnableState;
    IsEnabled LocationRTEGetSensor_EnableState;
    IsEnabled ParameterTuning_EnableState;
    IsEnabled DirectSensorDiagnostics_EnableState;
    IsEnabled CalIndirectSensorData_EnableState;
    IsEnabled TranslationalMotion_EnableState;
    IsEnabled IndirectDirectSensorDiagnostics_EnableState;
    IsEnabled RotationalMotion_EnableState;
    IsEnabled AutoCalibration_EnableState;
    IsEnabled Localization_EnableState;
    IsEnabled LocationRTESetGlobalVar_EnableState;
    IsEnabled LocationRTESetData_EnableState;
    IsEnabled LocationSetEPPROM_EnableState;
    IsEnabled CollisionDetect_EnableState;
} FunctionEnableStateType;

typedef struct
{
    ResetType LocationGetEPPROM_ResetState;
    ResetType LocationInitialization_ResetState;
    ResetType LocationRTEGetSensor_ResetState;
    ResetType ParameterTuning_ResetState;
    ResetType DirectSensorDiagnostics_ResetState;
    ResetType CalIndirectSensorData_ResetState;
    ResetType TranslationalMotion_ResetState;
    ResetType IndirectDirectSensorDiagnostics_ResetState;
    ResetType RotationalMotion_ResetState;
    ResetType AutoCalibration_ResetState;
    ResetType Localization_ResetState;
    ResetType LocationRTESetGlobalVar_ResetState;
    ResetType LocationRTESetData_ResetState;
    ResetType LocationSetEPPROM_ResetState;
    ResetType CollisionDetect_ResetState;
} FunctionResetStateType;

typedef struct
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float AngRate_X_deg;
    float AngRate_Y_deg;
    float AngRate_Z_deg;
    float AngRate_X_rad;
    float AngRate_Y_rad;
    float AngRate_Z_rad;
    float AngRate_Z_offset_rad;
} IMU_DataType;

typedef struct
{
    float ESC_VehSpd;
    float ESC_VehSpd_corrected;
    float ESC_ActVehLongAccel;
    float ESC_ActVehLateralAccel;
    float ESC_YAW_rate;
    float ESC_YAW_rate_offset;
    uint8_t ESC_ActVehLateralAccelVD;
    uint8_t ESC_VehSpdVD;
    uint8_t ESC_ActVehLongAccelVD;
    uint8_t ESC_YAWVD;

    int Counter_FL;
    int Counter_FR;
    int Counter_RL;
    int Counter_RR;

    int dCounter_FL;
    int dCounter_FR;
    int dCounter_RL;
    int dCounter_RR;
    StateType CounterState;

} ESC_DataType;

typedef struct
{
    int GPS_Longitude;
    int GPS_Latitude;
    float GPS_Altitude;
    float GPS_Bearing;
    float GPS_Speed;
    uint32_t GPS_Accuracy;
} GPS_DataType;

typedef struct
{
    float PressureFL;
    float PressureFR;
    float PressureRL;
    float PressureRR;
} TPMS_PresType;

typedef struct
{
    uint32_t System_time;
    IMU_DataType IMU_Data;
    ESC_DataType ESC_Data;
    GPS_DataType GPS_Data;
    DoorStateType DoorState;
    TPMS_PresType TPMS_Pressure;
    U_RadarType radar_data;
    float EPS_Angle;
    float EPS_AngleSpd;
    float Motorspd;
    float MotorTorq;
    float Veh_mass_level;
    float Veh_mass;
    // float               DsMotor;
    float curvatureByEPS;
    int gear;
    uint8_t GearSt;
    uint8_t ReadyState; // Power ready state
    uint8_t slotnumber;
    uint8_t Is_PK_Enable;
} SensorMeasurementsType;

typedef struct
{
    float Rear_wheel_fac_ds;
    float ESC_VehSpd_factor;
    float EPS_Offset;
    float Motorspd_fac_vspd;
} EPPROM_DataType;

typedef struct
{
    float SpeedForward;
    float vel_speed_filtered_RD;
    float AccelForwardbyIMU;
    float AccelForwardByWheel;
    float v_motor;
    float ds_wheel;
    float ds_motor;
    Vel_MoveMentStType Vel_MoveMentSt;
    uint32_t car_moving;
    float R_F2R_Rate;
    ValidType Valid;
    bool Collision_flag;
} LinearMotionType;

typedef struct
{
    ValidType Valid;
    float x;
    float y;
    float z;
    float s;
    float heading;
} LocationType;

typedef struct
{
    ResetType reset;
    float speed;
    float dt;
    float z[6];
    float q_r[3];
    float q_w[3];
    float r[3];
    float result[5];
} ekfEuler_atti_arg_Type;

typedef struct
{
    float roll_deg;
    float roll_rad;
    float pitch_deg;
    float pitch_rad;
    float yaw_deg;
    float yaw_rad;
    float curvatureByIMU;
    float SideAcc;
    ekfEuler_atti_arg_Type ekfEuler_atti_arg;
    ValidType Valid;
} RotationalMotionType;

typedef struct
{
    int GPS_Longitude;
    int GPS_Latitude;
    float GPS_Altitude;
    float AngRate_Z_rad;
    float SpeedForward;
    float v_motor;
    float dS_wheel;
    uint32_t GPS_Accuracy;
    float roll;
    float pitch;
    float EPS_angle;
    float dt;
    ResetType reset_KF;
    ResetType reset_all;
    uint8_t slotnumber;
    uint8_t KF_selection;
    uint8_t car_moving;
    uint8_t state;
    float X[4];
    float Pdiag[4];
    float PK_locat[4];
    float q[2];
    float r[2];
} EKF_LC_PROCESS_arg_Type;

typedef struct
{
    ResetType reset_all;
    EPPROM_DataType EPPROM_Data;
    SensorMeasurementsType SensorMeasurements;
    LinearMotionType LinearMotionData;
    LocationType LocationData;
    RotationalMotionType RotationalMotionData;
    EKF_LC_PROCESS_arg_Type EKF_LC_PROCESS_arg;
    uint32_t location_err_code;
    float location_dt;
    Localization_state state_code;
    BStateType B_state;
} AllDataType;

int DeltaWheelCounter(int count_cur, int count_last);
int car_moving_stateByMotor(int Motorspd);
unsigned char car_moving_stateByDcounter(int dCounter_RL, int dCounter_RR);
float cal_curvature(const float AngRate_Z, const float speed, const uint8_t stateFlg);
uint8_t Is_gear_done(uint8_t gear_state);
void count_error_signal(Signal_diagnose_info *error_state_in);
uint8_t hold_EKF_LC_reset(uint8_t error_state);
RtnType AutoCalibration(IsEnabled func_en, AllDataType *pAllData);
RtnType IndirectDirectSensorDiagnostics(IsEnabled func_en, AllDataType *pAllData,
                                        uint32_t *pDiagnosticsState);
RtnType DirectSensorDiagnostics(IsEnabled func_en, AllDataType *pAllData,
                                uint32_t *pDiagnosticsState);
uint8_t mass_level_function(float vel_speed_motor, float Accel_X, float Vehicle_Pitch,
                            float MotorTorq, float mass_estimate_res[2]);
RtnType Collision_detect(IsEnabled func_en, AllDataType *pAllData);

#endif
