#include "PK_Location_Debug.h"

uint8 Radar_Err_ccp[12];
float imu_position_ccp[4];

//  2. 模块输入
//  Measure Para
uint32 dCounter_RL_ccp; //
uint32 dCounter_RR_ccp; //
uint32 dCounter_FL_ccp; //
uint32 dCounter_FR_ccp; //
uint8 Vel_MoveMentSt_ccp;
float EPS_angle_ccp;            //
float EPS_angle_spd_ccp;        //
float Motorspd_ccp;             //
float Vel_TrajCurve_ccp;        //
float Vel_Yawrate_ccp;          //
float CurPos_ccp[4];            //
float CurSpd_ccp;               //
float CurSpd_filtered_ccp;      // filtered speed
float CurSpd_Motor_ccp;         //
float CurSpd_Wheel_ccp;         //
float IMU_AngRate_Z_Offset_ccp; //
float Accel_X1_CCP;
float AccelForwardIMU_CCP;
float Accel_Y1_CCP;
float Accel_Z1_CCP;
float AngRate_X_CCP;
float AngRate_Y_CCP;
float AngRate_Z_CCP;
float Vehicle_Roll_ccp;
float Vehicle_Pitch_ccp;
int GPS_Longitude_ccp;  // BCAN_Msg_33E.CDU_SCU_GPS_Longitude
int GPS_Latitude_ccp;   // 车辆GPS坐标经度;纬度
                        // 10^-6deg//BCAN_Msg_33E.CDU_SCU_GPS_Latitude
float GPS_Altitude_ccp; // 车辆GPS坐标海拔 m//BCAN_Msg_33C.CDU_SCU_GPS_Altitude
float GPS_Bearing_ccp;  // 车辆GPS坐标轴朝�?
                        // 10^-2deg//BCAN_Msg_33C.CDU_SCU_GPS_Bearing
uint8 GPS_Accuracy_ccp; // 车辆GPS坐标精度 m//BCAN_Msg_33C.CDU_SCU_GPS_Accuracy
uint16 Location_error_code_ccp;
uint32 Counter_RL_ccp;
uint32 Counter_RR_ccp;
uint32 Counter_FL_ccp;
uint32 Counter_FR_ccp;
float accelx_filtered_ccp; // filtered speed
float Veh_mass_ccp;        // vehicle mass estimated
uint8 Veh_mass_level_ccp;  // vehicle mass estimated
float curvature_ccp;
float ESC_VehSpd_ccp;
float GPS_speed_ccp;
float SideAcc_ccp;

// MODULE PK_SensorFusion
//   ***************** PK_SensorFusion ******************************//
uint8 Collision_flag_ccp;
float IMU_gro_X_Offset_ccp;
float IMU_gro_Y_Offset_ccp;
float IMU_gro_Z_Offset_ccp;
float IMU_gro_X_E_ccp;
float IMU_gro_Y_E_ccp;
float IMU_gro_Z_E_ccp;
float IMU_gro_X_Var_ccp;
float IMU_gro_Y_Var_ccp;
float IMU_gro_Z_Var_ccp;
float IMU_acc_X_E_ccp;
float IMU_acc_Y_E_ccp;
float IMU_acc_Z_E_ccp;
float IMU_acc_X_Var_ccp;
float IMU_acc_Y_Var_ccp;
float IMU_acc_Z_Var_ccp;