#include "PK_Calibration_Debug.h"

// Module State
uint8_t State_SensorT2S_CL_CCP;
uint8_t CDU_SCU_SuperParkActive_CCP; // weizq0717

// MODULE PK_SensorFusion
//   ***************** PK_SensorFusion ******************************//
float PK_SENSORFUSION_A_ARR_CCP[4];
float PK_SENSORFUSION_OUT_ARR_CCP[4];
uint32_t PK_SENSORFUSION_OUT_ARR_NUM_CCP;
int PK_SENSORFUSION_A_ARR_NUM_CCP;
int PK_SENSORFUSION_A_ARR_DIR_CCP;
uint8_t PK_SENSORFUSION_OUT_ARR_DIR_CCP;
float PK_SENSORFUSION_B_ARR_CCP[4];
int PK_SENSORFUSION_B_ARR_NUM_CCP;
int PK_SENSORFUSION_B_ARR_DIR_CCP;

float Avm_Min_Delay_Ms          = 0.0;
float AVM_MaxProjRx_SlotCal     = 10.0f;
uint8_t Avm_CheckStop           = 0;
float expand_slot_width         = 2.8;
float eps_delaydt_ccp           = 0.40f; // 1.1 1.5
float eps_vel_factor            = 1.10f; // 1.8 1,5 1.8
int stop_vision_slot            = 0;
float Radar_Vert_TargPos_off_dx = 0.0f;
float Parkout_off_dx            = 0.0f; //超声波未扫描到障碍物时自动泊出直线泊出距离
float min_object_length         = 0.35; //超声波障碍物最短有效长度（未应用）
char local_ip_address[16]       = "192.168.1.10"; //本地ip
float avm_vert_slot_min_dx =
    -2.5f; //视觉车位BE中点相对于摄像头的x坐标，大于此数值即为可信任的
float avm_vert_slot_max_dx =
    2.5f; //视觉车位BE中点相对于摄像头的x坐标，小于此数值即为可信任的
float big_eps_angle_gap     = 150.0f; //方向盘距离目标角度小于此数值时车辆开始提速
float avm_slotinfo_b_factor = 1.0; // B过程更新车位与旧车位数据加权平均的权重（0.0 ~ 1.0）
float slot_ang_reverse_off_dy =
    0.0; //逆鱼骨斜车位（车头泊入）偏移量（车辆目标点向车位外移动）
float searching_vert_slot_off_ldy =
    0.0; //搜索车位时左垂直车位偏移量（车辆目标点向车位外移动）
float searching_vert_slot_off_rdy =
    0.0; //搜索车位时右垂直车位偏移量（车辆目标点向车位外移动）
float vehicle_travel_dist_min  = 0.10; //搜索车位时释放车位需满足的车辆最小行驶距离
float offset_curb              = 0.00; //向车位外偏移量（方便副驾驶下客）
float offset_suspension        = 0.15; //后悬挂的修正值
float max_esp_spd              = 900;  //方向盘转动最大预期速度 元 300 deg/s 汉 900 deg/s
int avm_slot_stopbar           = 1;    //视觉车位限位杆 0 - 不应用 1 - 应用
int avm_slot_update_on_parking = 1;    //泊车过程视觉车位更新（B过程） 0 - 不更新 1 - 更新
int obj_of_para_slot_bottom    = 1;    //平行车位底部障碍物 0 - 不应用 1 - 应用
int is_save_log                = 1;    //是否保存日志
int min_frame_num              = 3;    //车位释放最小有效数量
int a_frame_num                = 12;   // A过程（搜索车位时）车位更新最小有效数量
int b_frame_num                = 8;    // B过程（泊车时）车位更新最小有效数量
int ultrasonic_parking_space   = 1;    //超声波车位有效性 1 - 启用 0 - 不启用
float searching_vert_slot_off_ltheta = 0.0;
float searching_vert_slot_off_rtheta = 0.0;
char cfg_log_path[64]                = "/tmp/apalog/"; //日志存储目录