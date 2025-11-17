#ifndef _CALIBRATION_DEBUG_H
#define _CALIBRATION_DEBUG_H

#include "Std_Types.h"
#include "SystemPara.h"
extern uint8_t State_SensorT2S_CL_CCP;

extern uint8_t CDU_SCU_SuperParkActive_CCP;
extern float PK_SENSORFUSION_A_ARR_CCP[4];
extern float PK_SENSORFUSION_OUT_ARR_CCP[4];
extern uint32_t PK_SENSORFUSION_OUT_ARR_NUM_CCP;
extern int PK_SENSORFUSION_A_ARR_NUM_CCP;
extern int PK_SENSORFUSION_A_ARR_DIR_CCP;
extern uint8_t PK_SENSORFUSION_OUT_ARR_DIR_CCP;
extern float PK_SENSORFUSION_B_ARR_CCP[4];
extern int PK_SENSORFUSION_B_ARR_NUM_CCP;
extern int PK_SENSORFUSION_B_ARR_DIR_CCP;

//  ***************** PK_SensorFusion ******************************//
extern float Avm_Min_Delay_Ms;
extern float AVM_MaxProjRx_SlotCal;
extern uint8 Avm_CheckStop;
extern float expand_slot_width;
extern float eps_delaydt_ccp;
extern float eps_vel_factor;
extern int stop_vision_slot;            //是否关闭视觉车位 0 - 关闭 1 - 打开
extern float Radar_Vert_TargPos_off_dx; //
extern float Parkout_off_dx;            //垂直泊出直线行驶的最短距离
extern float min_object_length;         //障碍物有效的最短距离
extern char local_ip_address[16];       //本地ip
extern float avm_vert_slot_min_dx;      //垂直车位最佳取值范围
extern float avm_vert_slot_max_dx;      //垂直车位最佳取值范围
extern float big_eps_angle_gap;         // deg: the gap bewteen ceps and cureps
extern float avm_slotinfo_b_factor;     // B过程新旧视觉车位权重（0.0 ~ 1.0）
extern float
    slot_ang_reverse_off_dy; //逆鱼骨斜车位限位杆防撞参数（车辆目标点向车位外移动）
extern float
    searching_vert_slot_off_ldy; //搜索车位时左垂直车位偏移量（车辆目标点向车位外移动）
extern float
    searching_vert_slot_off_rdy; //搜索车位时右垂直车位偏移量（车辆目标点向车位外移动）
extern float vehicle_travel_dist_min;  //最小车辆行驶距离
extern float offset_curb;              //向车位外偏移量（方便副驾驶下客）
extern float offset_suspension;        //后悬挂的修正值
extern float max_esp_spd;              //方向盘转动最大预期速度 元 300 deg/s 汉 900 deg/s
extern int avm_slot_stopbar;           //视觉车位限位杆 0 - 不应用 1 - 应用
extern int avm_slot_update_on_parking; //泊车过程视觉车位更新（B过程） 0 - 不更新 1 - 更新
extern int obj_of_para_slot_bottom;    //平行车位底部障碍物 0 - 不应用 1 - 应用
extern int is_save_log;                //是否保存日志
extern int min_frame_num;              //车位释放最小有效数量
extern int a_frame_num;                // A过程（搜索车位时）车位更新最小有效数量
extern int b_frame_num;                // B过程（泊车时）车位更新最小有效数量
extern int ultrasonic_parking_space;   //超声波车位有效性 1 - 启用 0 - 不启用
extern float searching_vert_slot_off_ltheta;
extern float searching_vert_slot_off_rtheta;
extern char cfg_log_path[64]; //日志存储目录

#endif
