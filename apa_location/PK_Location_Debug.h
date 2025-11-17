#ifndef _LOCATION_DEBUG_H
#define _LOCATION_DEBUG_H

#include "Std_Types.h"
#include "SystemPara.h"

extern float imu_position_ccp[4];
extern uint32_t dCounter_RL_ccp; //
extern uint32_t dCounter_RR_ccp; //
extern uint32_t dCounter_FL_ccp;
extern uint32_t dCounter_FR_ccp; //
extern uint8_t Vel_MoveMentSt_ccp;
extern float EPS_angle_ccp;            //
extern float EPS_angle_spd_ccp;        //
extern float Motorspd_ccp;             //
extern float Vel_TrajCurve_ccp;        //
extern float Vel_Yawrate_ccp;          //
extern float CurPos_ccp[4];            //
extern float CurSpd_filtered_ccp;      // filtered speed
extern float CurSpd_ccp;               //
extern float CurSpd_Motor_ccp;         //
extern float CurSpd_Wheel_ccp;         //
extern float IMU_AngRate_Z_Offset_ccp; //
extern float Accel_X1_CCP;
extern float AccelForwardIMU_CCP;
extern float Accel_Y1_CCP;
extern float Accel_Z1_CCP;
extern float AngRate_X_CCP;
extern float AngRate_Y_CCP;
extern float AngRate_Z_CCP;
extern float Vehicle_Roll_ccp;
extern float Vehicle_Pitch_ccp;
extern int GPS_Longitude_ccp; // BCAN_Msg_33E.CDU_SCU_GPS_Longitude
extern int
    GPS_Latitude_ccp; // 车辆GPS坐标经度;纬度 10^-6deg//BCAN_Msg_33E.CDU_SCU_GPS_Latitude
extern float GPS_Altitude_ccp; // 车辆GPS坐标海拔 m//BCAN_Msg_33C.CDU_SCU_GPS_Altitude
extern float
    GPS_Bearing_ccp; // 车辆GPS坐标轴朝?? 10^-2deg//BCAN_Msg_33C.CDU_SCU_GPS_Bearing
extern uint8_t GPS_Accuracy_ccp; // 车辆GPS坐标精度 m//BCAN_Msg_33C.CDU_SCU_GPS_Accuracy
extern uint16_t Location_error_code_ccp;
extern uint32_t Counter_RL_ccp;
extern uint32_t Counter_RR_ccp;
extern uint32_t Counter_FL_ccp;
extern uint32_t Counter_FR_ccp;
extern float accelx_filtered_ccp;  // filtered speed
extern float Veh_mass_ccp;         // vehicle mass estimated
extern uint8_t Veh_mass_level_ccp; // vehicle mass estimated
extern float curvature_ccp;
extern float ESC_VehSpd_ccp;
extern float GPS_speed_ccp;
extern float SideAcc_ccp;
extern float keep_static_time_ccp;
extern float dfinx_ccp;
extern int tra_stat_ccp;
extern float dfradio_ccp;

extern uint8_t Collision_flag_ccp;
extern float IMU_gro_X_Offset_ccp;
extern float IMU_gro_Y_Offset_ccp;
extern float IMU_gro_Z_Offset_ccp;
extern float IMU_gro_X_E_ccp;
extern float IMU_gro_Y_E_ccp;
extern float IMU_gro_Z_E_ccp;
extern float IMU_gro_X_Var_ccp;
extern float IMU_gro_Y_Var_ccp;
extern float IMU_gro_Z_Var_ccp;
extern float IMU_acc_X_E_ccp;
extern float IMU_acc_Y_E_ccp;
extern float IMU_acc_Z_E_ccp;
extern float IMU_acc_X_Var_ccp;
extern float IMU_acc_Y_Var_ccp;
extern float IMU_acc_Z_Var_ccp;

#endif
