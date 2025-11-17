#ifndef _CALIBRATION_CONFIG_H
#define _CALIBRATION_CONFIG_H

#include "Std_Types.h"
#include "SystemPara.h"

#define RootSig
#define MidSig

// Calibration Para
extern const volatile float Rear_wheel_fac_ds_max;
extern const volatile float Motorspd_fac_vspd_max;
extern const volatile float Rear_wheel_fac_ds_min;
extern const volatile float Motorspd_fac_vspd_min;
extern const volatile float
    MIN_S; // 3.0f;     //2.7f                  //  前后车辆沿车前进方向的最小长
extern const volatile float
    MAX_VEHICLE_LEN; // 6.0f;                   //  构成车位的车辆最大的可能长度:m
extern const volatile float
    MAX_S; // 6.0f;     //  前后车辆沿车前进方向的最大长度，排除路沿、连续墙面的情况
extern const volatile float
    MAX_PARK_LEN_PARALLEL; // 10.0f;                    //12.0f//   平行车位的最大车位长
extern const volatile float MIN_PARK_LEN_PARALLEL; // 5.1f;//(VEHICLE_LEN+1.0f); //
                                                   // 平行车位的最小车位长
extern const volatile float
    MAX_PARK_LEN_VERTICAL; //  5.1f;//MIN_PARK_LEN_PARALLEL;//VEHICLE_LEN //
extern const volatile float MIN_PARK_LEN_VERTICAL; // 2.46f;//(VEHICLE_WID+0.6f); //
                                                   // 垂直车位的最小车位长
extern const volatile float MID_S; // 5.3f; // MIN_PARK_LEN_PARALLEL; //
                                   // 前后两车中间沿车前进方向的最小长
extern const volatile uint16
    MAX_VALID_DIST_ULTRASONIC; //  2100;                   //
                               //  侧向超声波雷达检测的有效距离最大mm
extern const volatile uint16
    MABEY_JUMP_ULTRA_DIST; //  500;                //  车位检测疑似跳变的最小距?mm
extern const volatile uint16
    REAL_JUMP_ULTRA_DIST; // 1500;             //  确认发生跳变的最小距离：mm
extern const volatile float
    JUMP_DETCT_DELTA_PASS_PATH; // 0.4f;                    //
                                // 进行跳变沿检测时，排除连续跳变干扰的检测距离：m
extern const volatile float
    MIN_PASS_PATH_START_DETECT; // 5.0f;    //
                                // 前面发生过泊车中断或泊入完成的情况，寻车位触发需要车位移动的最小距
extern const volatile float
    FRONT_SAFE_RATIO; // 0.5f;      //  目标停车位姿车头距离前方占全部车位空间的比例
extern const volatile float BIG_VERT_SLOT_LEN_DIFF;
extern const volatile float BIG_VERT_SEC_VEH_DIST;
extern const volatile float BIG_PARA_SLOT_LEN_DIFF;
extern const volatile float BIG_PARA_SEC_VEH_DIST;
extern const volatile float SlotMidObjLen_min;
extern const volatile float AVM_TarPos_CD_sidedist;
extern const volatile float SlotTransDistMax_ccp;
extern const volatile float Para_SafeDist_Curb_ccp;
extern const volatile float AVM_Slot_Trans_SafeDist_ccp;
extern const volatile float Vert_AVM_Targpos_bias_max;
extern const volatile float Para_AVM_Targpos_bias_max;
extern const volatile float AVM_Para_SlotUpdate_Safe_dist_ccp;
extern const volatile float Update_Rotate_Start_Dist_ccp;
extern const volatile float
    MAX_AVM_PARK_LEN_PARALLEL; // 2018/08/08 Å·ÌØÃ÷¶Ô½ÓÆ½ÐÐ³µÎ»Ïß×î´óÎª7.0Ã×
extern const volatile float
    MIN_AVM_PARK_LEN_PARALLEL; // 2018/08/08 Å·ÌØÃ÷¶Ô½ÓÆ½ÐÐ³µÎ»ÏØ×îÐ¡Îª4.7Ã×
extern const volatile float
    MAX_AVM_PARK_LEN_VERTICAL; // 2018/08/08 Å·ÌØÃ÷¶Ô½Ó´¹Ö±³µÎ»Ïß×î´óÎª3.6Ã×
extern volatile float
    MIN_AVM_PARK_LEN_VERTICAL; // 2018/08/08 Å·ÌØÃ÷¶Ô½Ó´¹Ö±³µÎ»Ïß×îÐ¡Îª2.2Ã×
extern const volatile float CurbParaSlotDepth_min;
extern const volatile float Vertical_Target_Max_mov_ccp; //
extern const volatile float Parallel_Target_Max_mov_ccp; //
extern const volatile float Para_VehEdge2CD_Min_Dist;
extern const volatile float Para_CD_Search_Region_Offset;
extern const volatile float
    WIDE_OFFSET; // 0.6f;           //  泊入车位后车辆距离路沿的距离
extern const volatile float
    LEN_OFFSET; // 0.2f;            //  泊入车位后车辆距离路沿的距离

//  Calibration Para
extern volatile float AVM_Para_TargPos_off_dy;
extern volatile float AVM_Para_TargPos_off_ldx;
extern volatile float AVM_Para_TargPos_off_rdx;
extern volatile float AVM_Vert_TargPos_off_ldy;
extern volatile float AVM_Vert_TargPos_off_rdy;
extern volatile float AVM_Vert_TargPos_off_ldx;
extern volatile float AVM_Vert_TargPos_off_rdx;
extern const volatile float PEPS_CALL_RUN_VEL; // 0.25f;          //  召唤车速：m/s
extern volatile float Rear_wheel_fac_ds;       // 0.01066925f; //
extern volatile float ESC_VehSpd_factor;
extern volatile float EPS_Offset;
extern volatile float Motorspd_fac_vspd;       // 0.0042929766f;  //
extern const volatile float LocatingPeriod_dt; // 0.02f;       //
extern const volatile float EPS_SPD_CCP;       // 350.0f;   //D2
extern const volatile float V_FAC_PARKING_CCP; // 0.65; //
extern const volatile float K1_MDB_CCP;        // 300.0f; //model based control para1,
extern const volatile float K2_MDB_CCP;        // 0.03f;  //model based control para2,
extern const volatile float
    MAXEPS_MDB_CCP; //=300.0f; // the max EPS_angle PI module can tune.
extern const volatile float DE_GAP_CCP;

extern const volatile float g_local_swell; // 0.3;
extern MidSig float Rrmin;                 // 4.0;

#endif
