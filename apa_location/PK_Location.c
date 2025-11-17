/**
 * @file PK_Location.c
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
#include "PK_LocationImpl.h"
#include "MathFunc.h"
#include "rtwtypes.h"
#include "codegenall_types.h"
#include "norm.h"
#include "ekfEuler_atti.h"
#include "EKF_LC_PROCESS.h"
#include "kalman_filter_group.h"
#include "codegenall_initialize.h"
#include "kf3d.h"
#include "Nav_math.h"
#include "mass_estimate.h"
#include "CanIf.h"
#include "Rte_Func.h"
#include "Ram.h"
#include "Rte_ComIF.h"
#include "Record_Buff.h"
#include "Record_Log.h"
#include <sys/time.h>
#include "json/json.h"
#include <fstream>

// 参数解析
uint8_t running_mark                              = false;
int8_t para_use_imu                               = -1;
float para_veh_yaw_resolution                     = 002132603;
float para_veh_yaw_still_offset                   = 0;
float para_veh_yaw_moving_clockwise_offset        = 0;
float para_veh_yaw_moving_counterClockwise_offset = 0;
uint8_t para_save_history_num                     = 60;

uint8_t use_imu            = 1;
int para_loc_debug_enabled = 1;

#define READJSONDOUBLE(data)                   \
    {                                          \
        if (jsonRoot.isMember(#data))          \
            data = jsonRoot[#data].asDouble(); \
        printf(#data);                         \
        printf(":%f\n", data);                 \
    }
#define READJSONINT(data)                   \
    {                                       \
        if (jsonRoot.isMember(#data))       \
            data = jsonRoot[#data].asInt(); \
        printf(#data);                      \
        printf(":%d\n", data);              \
    }

// #include "bicycleMotionModel/use_wheel_optimal.h"
#include "bicycleMotionModel/OrientationMeasurementModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
// #include "bicycleMotionModel/SystemModel.hpp"

// 输出打印
#define LOGBUF_SIZE 1024
static int loc_buffLen = 0;
char location_PrintBuff[LOGBUF_SIZE];
#define DEBUG_PRINT_BUF(fmt, ...)                                                        \
    do                                                                                   \
    {                                                                                    \
        if (para_loc_debug_enabled)                                                      \
        {                                                                                \
            loc_buffLen =                                                                \
                (loc_buffLen) + snprintf(&location_PrintBuff[loc_buffLen],               \
                                         LOGBUF_SIZE - loc_buffLen, fmt, ##__VA_ARGS__); \
        }                                                                                \
    } while (0)

static void *locationLog = NULL;
static void *InitLocationLog()
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);
    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_loc_.csv",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return NULL;
    }

    static uint8_t logbuff[40 * LOGBUF_SIZE] = {0};
    void *logId = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (logId == NULL)
    {
        fclose(logFid);
    }

    return logId;
}

void UnitLocationLog(void *locationLog) { Buff_UnInit(locationLog); }

void RD_LocationCtrl_Log(const char *info, const int infLen)
{
    if (locationLog != NULL)
    {
        Buff_Put(locationLog, info, infLen);
    }
}

/* Variable Definitions */
// Maximum coordinate X and Y
static const float MAX_CAL_XY = 50000.0f;
// Maximum calculation speed
static const float MAX_CAL_SPD = 100.0f;
// gear state
static const int GEAR_FWD        = 1;
static const int GEAR_STILL      = 0;
static const int GEAR_BKWD       = -1;
static uint32 LocationUpdataTime = 0;
// EEROM
RTE_FAR float EEROM_Rear_wheel_fac_ds;
RTE_FAR float EEROM_ESC_VehSpd_factor;
RTE_FAR float EEROM_Motorspd_fac_vspd;
RTE_FAR float EEROM_EPS_Offset;
PK_ModuleStateType hold_ModuleState_Location(
    PK_ModuleStateType error_state) // hold error for 5 times
{
    static uint8_t counter                    = 0;
    static PK_ModuleStateType state           = MSTAT_NORM; // 0 no error, 1 error
    static PK_ModuleStateType error_state_pre = MSTAT_NORM;

    if (error_state > 0)
    {
        error_state_pre = error_state;
        counter         = 1;
    }

    if (counter >= 1)
    {
        state = error_state_pre;
        counter++;
    }
    else
    {
        state = error_state;
    }

    if (counter > 5)
    {
        counter = 0;
    }

    return state;
}

#define Wheel_OPTIMAL_RECORD
#ifdef Wheel_OPTIMAL_RECORD
class Wheel_optimal
{
    const float count_coeff = 0.01137f;
    const float _W          = 1.64f;

  public:
    Wheel_optimal(const float &last_angle = 0, const bool &direction = 0)
        : m_last_angle(last_angle), m_rotate_direction(direction) {};
    void update_last_angle(const float &last_angle, const float &last_angle_vehicle)
    {
        m_last_angle         = last_angle;
        m_last_angle_vehicle = last_angle_vehicle;
    };
    void update_cur_angle(const float &cur_angle, const float &cur_angle_vehicle)
    {
        if (cur_angle > PI)
        {
            m_cur_angle = cur_angle - 2 * PI;
        }
        else if (cur_angle < -PI)
        {
            m_cur_angle = cur_angle + 2 * PI;
        }
        else
        {
            m_cur_angle = cur_angle;
        }
        m_cur_angle_vehicle = cur_angle_vehicle;
    }
    void reset() { mark_init = true; }
    int Locating_WheelCounter(int count_last, int count_cur)
    {
        int dcount;
        if (count_cur - count_last >= 0)
            dcount = count_cur - count_last;
        else
            dcount = count_cur + 1022 - count_last;

        if (abs(dcount) > 100 || (count_cur == 0 && count_last < 610))
            dcount = 0;
        return dcount;
    }
    void update_cnt(const int &straight_direciton, const float &cur_RR_cnt,
                    const float &cur_RL_cnt)
    {
        m_acc_RR_cnt = m_acc_RR_cnt + straight_direciton * Locating_WheelCounter(
                                                               m_last_RR_cnt, cur_RR_cnt);
        m_acc_RL_cnt = m_acc_RL_cnt + straight_direciton * Locating_WheelCounter(
                                                               m_last_RL_cnt, cur_RL_cnt);
        m_last_RR_cnt = cur_RR_cnt;
        m_last_RL_cnt = cur_RL_cnt;
    }
    void is_valid(const int &rotate_direction, const float &cur_RR_cnt,
                  const float &cur_RL_cnt)
    {
        if (rotate_direction != m_rotate_direction || mark_init)
        {
            m_rotate_direction = rotate_direction;
            valid              = false;
            time_cnt           = 0;
            update_last_angle(m_cur_angle, m_cur_angle_vehicle);
            m_acc_RR_cnt  = 0;
            m_acc_RL_cnt  = 0;
            m_last_RR_cnt = cur_RR_cnt;
            m_last_RL_cnt = cur_RL_cnt;
            mark_init     = false;
        }
        else
        {
            time_cnt++;
            if (time_cnt >= 10)
            {
                valid = true;
            };
        }
    }
    bool omptial(const int &rotate_direction, const int &straight_direciton,
                 const float &cur_angle, const float &cur_RR_cnt, const float &cur_RL_cnt,
                 const float &vehicle_angle, const float &vehicle_angle_rate,
                 float &optimal_angle)
    {

        DEBUG_PRINT_BUF("%d,%d,%f,%f,%f,%f,%f,", rotate_direction, straight_direciton,
                        cur_angle, cur_RR_cnt, cur_RL_cnt, vehicle_angle,
                        vehicle_angle_rate);

        update_cur_angle(cur_angle, vehicle_angle);
        is_valid(rotate_direction, cur_RR_cnt, cur_RL_cnt);
        update_cnt(straight_direciton, cur_RR_cnt, cur_RL_cnt);
        if (valid)
        {
            diff_angle         = m_cur_angle - m_last_angle;
            diff_angle_vehicle = m_cur_angle_vehicle - m_last_angle_vehicle;
            angle_by_wheel =
                static_cast<float>(m_acc_RR_cnt - m_acc_RL_cnt) * count_coeff * 2 / _W;
            if (fabs(angle_by_wheel) > PI / 3) // 调用一次滤波
            {
                optimal_angle = angle_by_wheel;
                mark_init     = true;
                return true;
            }
        }
        return false;
    }
    void record()
    {
        if (valid)
            DEBUG_PRINT_BUF("%f,%f,%f,%f\n", diff_angle, angle_by_wheel,
                            diff_angle - angle_by_wheel, diff_angle_vehicle - diff_angle);
        else
            DEBUG_PRINT_BUF("%f,%f,%f,%f\n", 0.0, 0.0, 0.0, 0.0);
    }

  public:
    // imu来计算
    float m_last_angle{0};
    float m_cur_angle{0};
    float diff_angle{0};

    // 车载来计算
    float m_last_angle_vehicle{0};
    float m_cur_angle_vehicle{0};
    float diff_angle_vehicle{0};

    int m_rotate_direction{-1};  // -1为逆时针，1为顺时针
    int m_straight_direciton{1}; // 1为前进，-1为后退
    bool valid{false};           // 满足优化条件
    bool mark_init{true};
    uint32_t time_cnt{0}; // 每个周期约20ms

    int16_t m_last_RR_cnt;
    int16_t m_last_RL_cnt;
    int16_t m_acc_RR_cnt;
    int16_t m_acc_RL_cnt;
    float angle_by_wheel;
};

#endif

static int Locating_WheelCounter(int count_last, int count_cur) // B_init=1,������ʷ��¼
{
    int dcount;
    if (count_cur - count_last >= 0)
        dcount = count_cur - count_last;
    else
        dcount = count_cur + 1022 - count_last;

    if (abs(dcount) > 100 || (count_cur == 0 && count_last < 610))
        dcount = 0;
    return dcount;
}

static void Location_vehImu(IsEnabled func_en, ResetType reset_all, AllDataType *pAllData)
{
    auto InitPos = [](float point[4]) {
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = 0.0;
        point[3] = 0.0;
    };

    /* =============================变量定义=============================*/
    // ====== imu定位数据，记录imu数据用于比较
    static float g_lastimuAngle = 0.0f;
    static int count            = 0;
    static float stpoint[4], basepoint[4];
    float finpoint[4];
    static float imu_result[4];
    static float imu_angle_init = 0;

// #define USE_WHEEL_DIFF // 使用轮速计优化
#ifdef USE_WHEEL_DIFF
    typedef float T;
    typedef bicycle_motion::State<T> State;
    typedef bicycle_motion::Control<T> Control;
    typedef bicycle_motion::SystemModel<T> SystemModel;

    typedef bicycle_motion::OrientationMeasurement<T> OrientationMeasurement;
    typedef bicycle_motion::OrientationMeasurementModel<T> OrientationModel;

    SystemModel sys;
    OrientationModel om;
    static Kalman::ExtendedKalmanFilter<State> ekf;
    Kalman::KalmanFilterBase<bicycle_motion::State<float>>::State x_ekf;
#endif
    // ====== 输入
    static int g_last_wheelcount_RR = 0;
    static int g_last_wheelcount_RL = 0;
    static uint32_t last_time       = 0;
    uint32_t cur_time               = RTE_BSW_Get_CurTime();
    uint32_t time_cycle;
    float motor_velocity = RTE_BSW_Get_Motorspd();

    // ====== veh定位数据
    static float vehicle_init_angle          = 0;
    static float vehicle_angle_last          = 0;
    static float vehicle_x                   = 0;
    static float vehicle_y                   = 0;
    static float vehicle_dsrw                = 0;
    static uint64_t still_count              = 0;
    static float still_AngRate_Z_offset_rad  = 0; // 静止状态下测得零漂
    static float moving_AngRate_Z_offset_rad = 0; // 动态漂移
    float vehicle_angle;
    static int first = 0;

    static Wheel_optimal wheel_optimal;

    /* =============================重置=============================*/
    if (func_en == NOT_ENABLED || reset_all == RESET)
    {
        InitPos(imu_position_ccp);
        first = 0;
        wheel_optimal.reset();
        return;
    }

    /* =============================数据更新=============================*/
    // ====== imu 数据
    float angles[3] = {0.0, 0.0, 0.0};
    RTE_BSW_Get_ImuAngle(angles);
    // ====== veh 数据
    float vehicle_angle_rate =
        pAllData->SensorMeasurements.ESC_Data.ESC_YAW_rate; // 当前角速度
    float vehicle_angle_rate_offset =
        RTE_BSW_Get_ESC_YAW_RATE_Offset(); // 车载陀螺仪提供零漂
    // ====== 轮速计
    int Counter_RL = RTE_BSW_Get_WheelCounter_RL();
    int Counter_RR = RTE_BSW_Get_WheelCounter_RR();

    /* =============================初始化=============================*/
    if (first == 0)
    {
        // 存储最后计算结果
        InitPos(imu_position_ccp);
        // imu数据初始化
        InitPos(basepoint);
        InitPos(stpoint);
        InitPos(imu_result);
        imu_angle_init = (0.0f - angles[2] * 0.0174532925199f);
        g_lastimuAngle = imu_angle_init;

#ifdef USE_WHEEL_DIFF
        State x;
        x.setZero();
        ekf.init(x);
#endif

        g_last_wheelcount_RL = Counter_RL;
        g_last_wheelcount_RR = Counter_RR;

        vehicle_init_angle = 0;
        vehicle_angle_last = 0;
        last_time          = cur_time;
        vehicle_x          = 0;
        vehicle_y          = 0;
        vehicle_dsrw       = 0;
        still_count        = 0;

        still_AngRate_Z_offset_rad  = para_veh_yaw_still_offset; // 初始化零漂
        moving_AngRate_Z_offset_rad = 0;                         // 动态偏置初始化为零

        if (locationLog != NULL)
        {
            UnitLocationLog(locationLog);
        }
        locationLog = InitLocationLog();

        loc_buffLen = 0;
        DEBUG_PRINT_BUF(
            "cur_time,time_cycle,curAngle_by_imu,curAngle_by_imu-init_angle,dsrw,motor_"
            "velocity,");
        DEBUG_PRINT_BUF(
            "imu_position_ccp[0],imu_position_ccp[1],imu_position_ccp[2],imu_position_"
            "ccp[3],");
        DEBUG_PRINT_BUF(
            "vehicle_x,vehicle_y,vehicle_angle,vehicle_angle_rate,vehicle_angle_rate_"
            "offset,still_AngRate_Z_offset_rad,moving_AngRate_Z_offset_rad,dCounter_RL,"
            "dCounter_RR,");
        DEBUG_PRINT_BUF(
            "imu_result[0],imu_result[1],imu_result[2],imu_result[3],direct_offset,final_"
            "offset,");
        DEBUG_PRINT_BUF(
            "rotate_direction,straight_direciton,cur_angle,cur_RR_cnt,cur_RL_cnt,vehicle_"
            "angle,vehicle_angle_rate,");
        DEBUG_PRINT_BUF("diff_angle,angle_by_wheel,dist_angle,dist_angle_vehicle\n");
        RD_LocationCtrl_Log(location_PrintBuff, loc_buffLen);

        first = 1;
        return;
    }

    /* ==================================车速计算=============================*/
    int gear = 0.0;
    if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_D)
    {
        if (motor_velocity > 0)
        {
            gear = 1;
        }
        else if (motor_velocity < 0)
        {
            gear = -1;
        }
        else
        {
            gear = 1;
        }
    }
    else if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_R)
    {
        if (motor_velocity > 0)
        {
            gear = 1;
        }
        else if (motor_velocity < 0)
        {
            gear = -1;
        }
        else
        {
            gear = -1;
        }
    }
    int dCounter_RL = Locating_WheelCounter(g_last_wheelcount_RL, Counter_RL);
    int dCounter_RR = Locating_WheelCounter(g_last_wheelcount_RR, Counter_RR);
    float dsrw      = gear * Rear_wheel_fac_ds * (dCounter_RL + dCounter_RR);

    /* ==================================imu的处理计算=============================*/
    float curAngle = (0.0f - angles[2] * 0.0174532925199f);
    count          = (dCounter_RL + dCounter_RR == 0) ? count + 1 : 0;
    if (count >= 10 && fabs(g_lastimuAngle - curAngle) < 0.0015)
    {
        curAngle = g_lastimuAngle;
    }

    if (fabs(g_lastimuAngle - curAngle) > PI)
    {
        finpoint[0] = stpoint[0] + cosf(g_lastimuAngle) * dsrw;
        finpoint[1] = stpoint[1] + sinf(g_lastimuAngle) * dsrw;
        finpoint[2] = curAngle;
    }
    else
    {
        finpoint[0] = stpoint[0] + cosf(0.2 * curAngle + 0.8f * g_lastimuAngle) * dsrw;
        finpoint[1] = stpoint[1] + sinf(0.2 * curAngle + 0.8f * g_lastimuAngle) * dsrw;
        finpoint[2] = 0.85f * curAngle + 0.15f * g_lastimuAngle;
    }
    finpoint[3] = stpoint[3] + fabs(dsrw);

    stpoint[0] = finpoint[0];
    stpoint[1] = finpoint[1];
    stpoint[2] = finpoint[2];
    stpoint[3] = finpoint[3];

    g_last_wheelcount_RL = Counter_RL;
    g_last_wheelcount_RR = Counter_RR;
    g_lastimuAngle       = curAngle;

    if (first == 1)
    {
        basepoint[0] = finpoint[0];
        basepoint[1] = finpoint[1];
        basepoint[2] = finpoint[2];
        basepoint[3] = finpoint[3];
        first        = 2;
    }

    /* ==================================veh的处理计算=============================*/
    // ====== 判断是否静止
    if (fabs(dsrw) < 0.000001)
    {
        still_count++;
    }
    else
    {
        still_count = 0;
    };
    // ====== 计算持续时间
    time_cycle = cur_time - last_time;
    last_time  = cur_time;
    // ======
    // 设置静漂：如果没有人为设置para_veh_yaw_still_offset，就使用车载offset作为静漂
    if (fabs(para_veh_yaw_still_offset) < 0.000001)
    {
        still_AngRate_Z_offset_rad = vehicle_angle_rate_offset;
    }
    else
    {
        still_AngRate_Z_offset_rad = para_veh_yaw_still_offset;
    }
    // ====== 设置动漂，分为顺时针、逆时针
    moving_AngRate_Z_offset_rad  = 0;
    float vehicle_angle_rate_cur = vehicle_angle_rate - still_AngRate_Z_offset_rad;
    if (still_count >= 5)
    {
        moving_AngRate_Z_offset_rad = 0;
    }
    else if (vehicle_angle_rate_cur > para_veh_yaw_resolution)
    {

        moving_AngRate_Z_offset_rad = para_veh_yaw_moving_counterClockwise_offset;
    }
    else if (vehicle_angle_rate_cur < -para_veh_yaw_resolution)
    {

        moving_AngRate_Z_offset_rad = para_veh_yaw_moving_clockwise_offset;
    }
    // ====== 更新当前角度
    if ((still_count >= 5) || (fabs(vehicle_angle_rate) > PI) ||
        fabs(vehicle_angle_rate_cur) < para_veh_yaw_resolution)
    {
        vehicle_angle = vehicle_angle_last;
    }
    else
    {
        vehicle_angle =
            vehicle_angle_last +
            (vehicle_angle_rate_cur - moving_AngRate_Z_offset_rad) * time_cycle * 0.001;
#ifdef USE_WHEEL_DIFF
        Control u;
        u.dtheta() =
            (vehicle_angle_rate_cur - moving_AngRate_Z_offset_rad) * time_cycle * 0.001;
        u.v() = dsrw;
        x_ekf = ekf.predict(sys, u);
#endif
    }

    // ====== 更新当前位置
    vehicle_x          = vehicle_x + dsrw * cosf(vehicle_angle);
    vehicle_y          = vehicle_y + dsrw * sinf(vehicle_angle);
    vehicle_angle_last = vehicle_angle;
    vehicle_dsrw       = vehicle_dsrw + fabs(dsrw);

    // float cur_point[4];
    // cur_point[0] = vehicle_x;
    // cur_point[1] = vehicle_y;
    // cur_point[2] = vehicle_angle;
    // cur_point[3] = vehicle_dsrw;

    /* ==================================更新位置全局变量=============================*/
    if (func_en == ENABLED && first == 2)
    {
        // imu
        CoordinadteTransfer(basepoint, finpoint, imu_result);
        imu_result[3] = finpoint[3] - basepoint[3];

        // CoordinadteTransfer(basepoint, cur_point, imu_position_ccp);
        // imu_position_ccp[3] = cur_point[3] - basepoint[3];

        // veh
        imu_position_ccp[0] = vehicle_x;
        imu_position_ccp[1] = vehicle_y;
        imu_position_ccp[2] = vehicle_angle_last;
        imu_position_ccp[3] = vehicle_dsrw;
    }

    /* ======================================日志输出=============================*/

    loc_buffLen = 0;
    DEBUG_PRINT_BUF("%u,%u,%f,%f,%f,%f,", cur_time, time_cycle, curAngle,
                    curAngle - imu_angle_init, dsrw, motor_velocity);
    DEBUG_PRINT_BUF("%f,%f,%f,%f,", imu_position_ccp[0], imu_position_ccp[1],
                    imu_position_ccp[2], imu_position_ccp[3]);
    DEBUG_PRINT_BUF("%f,%f,%f,%f,%f,%f,%f,%d,%d,", vehicle_x, vehicle_y, vehicle_angle,
                    vehicle_angle_rate, vehicle_angle_rate_offset,
                    still_AngRate_Z_offset_rad, moving_AngRate_Z_offset_rad, dCounter_RL,
                    dCounter_RR);
    DEBUG_PRINT_BUF("%f,%f,%f,%f,", imu_result[0], imu_result[1], imu_result[2],
                    imu_result[3]);
    DEBUG_PRINT_BUF("%f,%f,", vehicle_angle - curAngle,
                    imu_position_ccp[2] - imu_result[2]);
    float optimal_angle;
    bool succ = wheel_optimal.omptial(sign(vehicle_angle_rate_cur), gear, curAngle,
                                      Counter_RR, Counter_RL, vehicle_angle,
                                      vehicle_angle_rate, optimal_angle);
#ifdef USE_WHEEL_DIFF
    if (succ)
    {
        OrientationMeasurement orientation;
        orientation.theta() = optimal_angle;
        x_ekf               = ekf.update(om, orientation);
    }
#endif
    wheel_optimal.record();
    RD_LocationCtrl_Log(location_PrintBuff, loc_buffLen);
}

static void Location_imu(IsEnabled func_en, ResetType reset_all)
{
    static float g_lastimuAngle     = 0.0f;
    static int g_last_wheelcount_RR = 0;
    static int g_last_wheelcount_RL = 0;
    static int first                = 0;
    static int count                = 0;
    static float stpoint[4], basepoint[4];
    float finpoint[4];

    uint32_t cur_time = RTE_BSW_Get_CurTime();

    auto InitPos = [](float point[4]) {
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = 0.0;
        point[3] = 0.0;
    };

    if (func_en == NOT_ENABLED || reset_all == RESET)
    {
        InitPos(imu_position_ccp);
        first = 0;
        return;
    }

    int Counter_RL = RTE_BSW_Get_WheelCounter_RL();
    int Counter_RR = RTE_BSW_Get_WheelCounter_RR();

    float angles[3] = {0.0, 0.0, 0.0};
    RTE_BSW_Get_ImuAngle(angles);
    float curAngle = 0.0f - angles[2] * 0.0174532925199f;

    float groudAcc = RTE_BSW_Get_Accel_Z();

    // printf("LocationRTESetData current pos %f %f %f func_en %d first %d groudAcc %f\n",
    //     imu_position_ccp[0], imu_position_ccp[1], imu_position_ccp[2],
    //     func_en, first,  groudAcc);

    if (fabs(groudAcc) < 6.0)
    {
        return;
    }

    if (first == 0)
    {
        InitPos(imu_position_ccp);
        InitPos(basepoint);
        InitPos(stpoint);

        g_last_wheelcount_RL = Counter_RL;
        g_last_wheelcount_RR = Counter_RR;
        g_lastimuAngle       = curAngle;
        first                = 1;

        if (locationLog != NULL)
        {
            UnitLocationLog(locationLog);
        }
        locationLog = InitLocationLog();
        loc_buffLen = 0;
        DEBUG_PRINT_BUF(
            "cur_time,curAngle,lastimuAngle,gear,Counter_RL,Counter_RR,dCounter_RL,"
            "dCounter_RR,groudAcc,Rear_wheel_fac_ds,");
        DEBUG_PRINT_BUF(
            "imu_position_ccp[0],imu_position_ccp[1],imu_position_ccp[2],imu_position_"
            "ccp[3]");
        DEBUG_PRINT_BUF("basepoint[0],basepoint[1],basepoint[2],basepoint[3],");
        DEBUG_PRINT_BUF("finpoint[0],finpoint[1],finpoint[2],finpoint[3],");
        DEBUG_PRINT_BUF("imu_x,imu_y,imu_theta\n");
        RD_LocationCtrl_Log(location_PrintBuff, loc_buffLen);
        return;
    }

    int gear       = 0.0;
    float motorSpd = RTE_BSW_Get_Motorspd();
    if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_D)
    {
        if (motorSpd > 0)
        {
            gear = 1;
        }
        else if (motorSpd < 0)
        {
            gear = -1;
        }
        else
        {
            gear = 1;
        }
    }
    else if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_R)
    {
        if (motorSpd > 0)
        {
            gear = 1;
        }
        else if (motorSpd < 0)
        {
            gear = -1;
        }
        else
        {
            gear = -1;
        }
    }

    int dCounter_RL = Locating_WheelCounter(g_last_wheelcount_RL, Counter_RL);
    int dCounter_RR = Locating_WheelCounter(g_last_wheelcount_RR, Counter_RR);

    count = (dCounter_RL + dCounter_RR == 0) ? count + 1 : 0;
    if (count >= 10 && fabs(g_lastimuAngle - curAngle) < 0.0015)
    {
        curAngle = g_lastimuAngle;
    }

    float dsrw = gear * Rear_wheel_fac_ds * (dCounter_RL + dCounter_RR);
    if (fabs(g_lastimuAngle - curAngle) > PI)
    {
        finpoint[0] = stpoint[0] + cosf(g_lastimuAngle) * dsrw;
        finpoint[1] = stpoint[1] + sinf(g_lastimuAngle) * dsrw;
        finpoint[2] = curAngle;
    }
    else
    {
        finpoint[0] = stpoint[0] + cosf(0.2 * curAngle + 0.8f * g_lastimuAngle) * dsrw;
        finpoint[1] = stpoint[1] + sinf(0.2 * curAngle + 0.8f * g_lastimuAngle) * dsrw;
        finpoint[2] = 0.85f * curAngle + 0.15f * g_lastimuAngle;
    }
    finpoint[3] = stpoint[3] + fabs(dsrw);

    stpoint[0] = finpoint[0];
    stpoint[1] = finpoint[1];
    stpoint[2] = finpoint[2];
    stpoint[3] = finpoint[3];

    g_last_wheelcount_RL = Counter_RL;
    g_last_wheelcount_RR = Counter_RR;
    g_lastimuAngle       = curAngle;

    if (first == 1)
    {
        basepoint[0] = finpoint[0];
        basepoint[1] = finpoint[1];
        basepoint[2] = finpoint[2];
        basepoint[3] = finpoint[3];
        first        = 2;
    }

    if (func_en == ENABLED && first == 2)
    {
        CoordinadteTransfer(basepoint, finpoint, imu_position_ccp);
        imu_position_ccp[3] = finpoint[3] - basepoint[3];
    }

    loc_buffLen = 0;
    DEBUG_PRINT_BUF("%u,%f,%f,%d,%d,%d,%d,%d,%f,%f,", cur_time, curAngle, g_lastimuAngle,
                    gear, Counter_RL, Counter_RR, dCounter_RL, dCounter_RR, groudAcc,
                    Rear_wheel_fac_ds);
    DEBUG_PRINT_BUF("%f,%f,%f,%f,", imu_position_ccp[0], imu_position_ccp[1],
                    imu_position_ccp[2], imu_position_ccp[3]);
    DEBUG_PRINT_BUF("%f,%f,%f,%f,", basepoint[0], basepoint[1], basepoint[2],
                    basepoint[3]);
    DEBUG_PRINT_BUF("%f,%f,%f,%f\n", finpoint[0], finpoint[1], finpoint[2], finpoint[3]);
    RD_LocationCtrl_Log(location_PrintBuff, loc_buffLen);
}

/**
 * @brief 位置计算主函数
 *
 * 根据使能状态和复位标志，选择使用IMU或车辆IMU进行位置计算
 *
 * @param func_en 函数使能
 * @param reset_all 复位标志
 * @param pAllData 位置传感器数据和计算数据指针
 */
static void Location(IsEnabled func_en, ResetType reset_all, AllDataType *pAllData)
{
    if (func_en == NOT_ENABLED || reset_all == RESET)
    {
        if (para_use_imu == -1)
        {
            use_imu = RTE_BSW_Get_IMU_SensorSt();
        }
        else
        {
            use_imu = para_use_imu;
        }
    }
    if (use_imu)
    {
        Location_imu(func_en, reset_all);
    }
    else
    {
        Location_vehImu(func_en, reset_all, pAllData);
    }
}

static void Location_NoImu(IsEnabled func_en, ResetType reset_all)
{
    static int first = 0;
    static float stpoint[4], basepoint[4];

    auto InitPos = [](float point[4]) {
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = 0.0;
        point[3] = 0.0;
    };

    if (func_en == NOT_ENABLED)
    {
        InitPos(imu_position_ccp);
        first = 0;
        return;
    }

    if (first == 0)
    {
        InitPos(imu_position_ccp);
        InitPos(basepoint);
        InitPos(stpoint);
        first = 1;
        return;
    }

    if (reset_all == RESET || first == 1)
    {
        basepoint[0] = CurPos_ccp[0];
        basepoint[1] = CurPos_ccp[1];
        basepoint[2] = CurPos_ccp[2];
        basepoint[3] = CurPos_ccp[3];
        first        = 2;
    }

    if (func_en == NOT_ENABLED)
    {
        imu_position_ccp[0] = 0.00;
        imu_position_ccp[1] = 0.00;
        imu_position_ccp[2] = 0.00;
        imu_position_ccp[3] = 0.00;
    }
    else if (first == 2)
    {
        CoordinadteTransfer(basepoint, CurPos_ccp, imu_position_ccp);
        imu_position_ccp[3] = CurPos_ccp[3] - basepoint[3];
        uint8_t candata[8];
        uint8_t offset = 0;
        offset += Uint16ToBuf((uint16_t)(imu_position_ccp[0] * 100), &candata[offset]);
        offset += Uint16ToBuf((uint16_t)(imu_position_ccp[1] * 100), &candata[offset]);
        offset += Uint16ToBuf((uint16_t)(imu_position_ccp[2] * 100), &candata[offset]);
        offset += Uint16ToBuf((uint16_t)(imu_position_ccp[3] * 100), &candata[offset]);
        Can_PushData(0x6301, candata);
    }
}

RtnType LocationGetEPPROM(IsEnabled func_en, EPPROM_DataType *pRomData)
{
    const float ESC_VehSpd_factor_min = 0.8F;
    const float ESC_VehSpd_factor_max = 1.2F;

    CHECK_FUNC_EN(func_en);

    if (EEROM_Motorspd_fac_vspd > Motorspd_fac_vspd_max ||
        EEROM_Motorspd_fac_vspd < Motorspd_fac_vspd_min ||
        EEROM_ESC_VehSpd_factor > ESC_VehSpd_factor_max ||
        EEROM_ESC_VehSpd_factor < ESC_VehSpd_factor_min)
    {
        // RTE_BSW_DIDCF00_getVehicleCfg(&VehicleCfg);
        uint8_t VehicleCfg = 0;
        if (((VehicleCfg & 0x0F) == 0x1) || ((VehicleCfg & 0x0F) == 0x2) ||
            ((VehicleCfg & 0x0F) == 0x4))
        {
            EEROM_Motorspd_fac_vspd = 0.0041669284f;
            EEROM_ESC_VehSpd_factor = 1.005377f;
            EEROM_EPS_Offset        = 0.0F;
        }
        else
        {

            EEROM_Motorspd_fac_vspd = 0.004166928f;
            EEROM_ESC_VehSpd_factor = 1.014f;
            EEROM_EPS_Offset        = 0.0F;
        }
    }

    EEROM_Rear_wheel_fac_ds = PI * WHELL_RADIUS / 96;

    pRomData->EPS_Offset        = EEROM_EPS_Offset;
    pRomData->ESC_VehSpd_factor = EEROM_ESC_VehSpd_factor;
    pRomData->Motorspd_fac_vspd = EEROM_Motorspd_fac_vspd;
    pRomData->Rear_wheel_fac_ds = EEROM_Rear_wheel_fac_ds;
    Rear_wheel_fac_ds           = pRomData->Rear_wheel_fac_ds;
    ESC_VehSpd_factor           = pRomData->ESC_VehSpd_factor;
    Motorspd_fac_vspd           = pRomData->Motorspd_fac_vspd;

    return RTN_OK;
}

float PK_InterPolationOneDim(const float *x, const float *y, const float xx, int n)
{
    int i, sign_x, out_intp = 0;
    float yy = 0;
    int l, h;
    if (x[1] - x[0] >= 0)
    {
        sign_x = 1;
        if (xx >= x[n - 1])
        {
            out_intp = 1;
            yy       = y[n - 1];
        }
        else if (xx <= x[0])
        {
            out_intp = 1;
            yy       = y[0];
        }
    }
    else
    {
        sign_x = -1;
        if (xx >= x[0])
        {
            out_intp = 1;
            yy       = y[0];
        }
        else if (xx <= x[n - 1])
        {
            out_intp = 1;
            yy       = y[n - 1];
        }
    }

    if (out_intp == 0)
    {
        l = 0;
        h = n - 1;
        while (l < h)
        {
            i = (l + h) / 2;
            if (sign_x * xx >= sign_x * x[i] && sign_x * xx <= sign_x * x[i + 1])
            {
                yy = y[i] + (xx - x[i]) * (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
                break;
            }
            else if (sign_x * xx > sign_x * x[i + 1])
            {
                l = i + 1;
            }
            else
            {
                h = i;
            }
        }
    }
    return yy;
}

static float calYaw_rate1(void)
{
    float rad = 0;
    if (RTE_BSW_Get_CurrentGear() == 1 &&
        (RTE_BSW_Get_EPS_Angle() > 5 || RTE_BSW_Get_EPS_Angle() < -5)) // D
    {
        rad = PK_EPS_to_Rou(RTE_BSW_Get_EPS_Angle(), 1);
    }
    else if (RTE_BSW_Get_CurrentGear() == 3 &&
             (RTE_BSW_Get_EPS_Angle() > 5 || RTE_BSW_Get_EPS_Angle() < -5)) // R
    {
        rad = PK_EPS_to_Rou(RTE_BSW_Get_EPS_Angle(), -1);
    }

    return CurSpd_ccp * rad;
}

static float calYaw_rate2(void)
{
    float yawRate       = RTE_BSW_Get_ESC_YAW_RATE();
    float yawRateOffset = RTE_BSW_Get_ESC_YAW_RATE_Offset();

    float rlSpd = RTE_BSW_Get_ESC_WheelSpdRL();
    float rrSpd = RTE_BSW_Get_ESC_WheelSpdRR();
    float flSpd = RTE_BSW_Get_ESC_WheelSpdFL();
    float frSpd = RTE_BSW_Get_ESC_WheelSpdFR();

    if (fabs(rlSpd) < ZERO_FLOAT || fabs(rrSpd) < ZERO_FLOAT ||
        fabs(flSpd) < ZERO_FLOAT || fabs(frSpd) < ZERO_FLOAT)
    {
        return 0.0;
    }

    return (fabs(yawRate - yawRateOffset) < 0.002132603) ? 0.0 : yawRate;
}

/**
 * @brief 从各传感器采集原始数据并封装至结构体
 *
 * 本函数负责从 RTE 层读取所有定位相关的传感器原始数据，并将其组织填充到
 * `SensorMeasurementsType` 数据结构中。 包括但不限于：
 * - IMU 加速度与角速度（角速度单位转换 deg → rad）
 * - EPS 转角及转角速度
 * - 电机转速、挡位状态
 * - 车轮计数器（前后左右）
 * - 胎压（TPMS）
 * - 雷达、GPS 信息
 * - ESC 数据：车速、纵向/横向加速度、偏航率等
 * - 车门状态与泊车功能状态
 * - 车位识别数量（多车位）
 *
 * @param[in]  func_en     功能是否使能（ENABLED / NOT_ENABLED）
 * @param[out] pSensorMea  指向传感器测量数据结构的指针，函数将数据写入此结构体
 * @retval     RTN_OK      数据读取成功
 */
RtnType LocationRTEGetSensor(IsEnabled func_en, SensorMeasurementsType *pSensorMea)
{
    SensorMeasurementsType SensorData;
    SensorData.System_time = RTE_BSW_Get_CurTime();
    LocationUpdataTime     = SensorData.System_time; //

    CHECK_FUNC_EN(func_en);
    memset((uint8_t *)&SensorData, 0, sizeof(SensorMeasurementsType));
    SensorData.ReadyState = RTE_BSW_Get_EvReadySt();

    /*********************************************************/
    SensorData.IMU_Data.Accel_X       = RTE_BSW_Get_Accel_X();
    SensorData.IMU_Data.Accel_Y       = RTE_BSW_Get_Accel_Y();
    SensorData.IMU_Data.Accel_Z       = RTE_BSW_Get_Accel_Z();
    SensorData.IMU_Data.AngRate_X_deg = RTE_BSW_Get_AngRate_X();
    SensorData.IMU_Data.AngRate_Y_deg = RTE_BSW_Get_AngRate_Y();
    SensorData.IMU_Data.AngRate_Z_deg = RTE_BSW_Get_AngRate_Z();
    /*********************************************************/
    SensorData.IMU_Data.AngRate_X_rad = SensorData.IMU_Data.AngRate_X_deg * DEG2RAD;
    SensorData.IMU_Data.AngRate_Y_rad = SensorData.IMU_Data.AngRate_Y_deg * DEG2RAD;
    SensorData.IMU_Data.AngRate_Z_rad = SensorData.IMU_Data.AngRate_Z_deg * DEG2RAD;
    SensorData.EPS_Angle              = RTE_BSW_Get_EPS_Angle();
    SensorData.EPS_AngleSpd           = RTE_BSW_Get_EPS_AngleSpd();
    SensorData.Motorspd               = RTE_BSW_Get_Motorspd();
    SensorData.GearSt                 = RTE_BSW_Get_CurrentGear();
    SensorData.ESC_Data.Counter_FL    = RTE_BSW_Get_WheelCounter_FL();
    SensorData.ESC_Data.Counter_FR    = RTE_BSW_Get_WheelCounter_FR();
    SensorData.ESC_Data.Counter_RL    = RTE_BSW_Get_WheelCounter_RL();
    SensorData.ESC_Data.Counter_RR    = RTE_BSW_Get_WheelCounter_RR();

    SensorData.TPMS_Pressure.PressureFL = RTE_BSW_Get_TPMS_PressureFL();
    SensorData.TPMS_Pressure.PressureFR = RTE_BSW_Get_TPMS_PressureFR();
    SensorData.TPMS_Pressure.PressureRL = RTE_BSW_Get_TPMS_PressureRL();
    SensorData.TPMS_Pressure.PressureRR = RTE_BSW_Get_TPMS_PressureRR();

    RTE_PD_Get_U_Radar(&SensorData.radar_data);

    SensorData.GPS_Data.GPS_Longitude = RTE_BSW_Get_GPS_Longitude();
    SensorData.GPS_Data.GPS_Latitude  = RTE_BSW_Get_GPS_Latitude();
    SensorData.GPS_Data.GPS_Altitude  = RTE_BSW_Get_GPS_Altitude();
    SensorData.GPS_Data.GPS_Bearing   = RTE_BSW_Get_GPS_Bearing();
    SensorData.GPS_Data.GPS_Accuracy  = RTE_BSW_Get_GPS_Accuracy();
    SensorData.GPS_Data.GPS_Speed     = 0.0;

    SensorData.ESC_Data.ESC_VehSpd = sign(SensorData.Motorspd) * RTE_BSW_Get_ESC_VehSpd();
    SensorData.ESC_Data.ESC_VehSpdVD             = RTE_BSW_Get_ESC_VehSpdVD();
    SensorData.ESC_Data.ESC_ActVehLongAccel      = 1.0 * 9.8f;
    SensorData.ESC_Data.ESC_ActVehLongAccelVD    = 1;
    SensorData.ESC_Data.ESC_ActVehLateralAccel   = 0.0;
    SensorData.ESC_Data.ESC_ActVehLateralAccelVD = 1;
    SensorData.ESC_Data.ESC_YAW_rate             = RTE_BSW_Get_ESC_YAW_RATE();
    // SensorData.ESC_Data.ESC_YAW_rate_offset = RTE_BSW_Get_ESC_YAW_RATE_Offset();
    SensorData.ESC_Data.ESC_YAWVD = RTE_BSW_Get_ESC_YAW_RATE_VD();

    SensorData.MotorTorq = RTE_BSW_Get_IPU_MotorTorq();

    SensorData.DoorState.BCM_DriverDoorAjarSt = 0;
    SensorData.DoorState.BCM_PsngrDoorAjarSt  = 0;
    SensorData.DoorState.BCM_RLDoorAjarSt     = 0;
    SensorData.DoorState.BCM_RRDoorAjarSt     = 0;

    Multi_Slot_Array_T multiSlots;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&multiSlots);
    SensorData.slotnumber   = (uint8_t)multiSlots.multiNum;
    SensorData.Is_PK_Enable = 0;
    memcpy((uint8_t *)pSensorMea, (uint8_t *)&SensorData, sizeof(SensorMeasurementsType));

    return RTN_OK;
}
/**
 * @brief 计算间接传感器数据
 *
 * 根据轮速计计数，IMU，电机速度，EPS角度等信息，计算车辆运动状态、
 * 曲率、车速及质量等级等关键量。
 *
 * @param[in] func_en 功能使能标志
 * @param[in,out] pAllData 车辆所有传感器及计算数据结构指针
 *
 * @return RTN_OK 始终返回成功
 */
RtnType CalIndirectSensorData(IsEnabled func_en, AllDataType *pAllData)
{
    const float SMALL_EPSANGLE      = 20.0F;
    static int g_last_wheelcount_RR = 0, g_last_wheelcount_RL = 0,
               g_last_wheelcount_FR = 0, g_last_wheelcount_FL = 0;
    static int directed_car_moving      = 1;
    static const int gearMotorThresh    = 5; // check vehicle move state thresh
    SensorMeasurementsType *pSensorData = &pAllData->SensorMeasurements;
    ESC_DataType *pESCData              = &pAllData->SensorMeasurements.ESC_Data;
    IMU_DataType *pImuData              = &pAllData->SensorMeasurements.IMU_Data;
    float *pR_F2R_Rate                  = &pAllData->LinearMotionData.R_F2R_Rate; //
    float R_F2R_Rate                    = 0;
    static float mass_estimate_res[2]   = {1600.0F, 10.0F};
    static uint8_t Veh_mass_level       = 1;

    CHECK_FUNC_EN(func_en);

    // 1,cal dCounter
    pESCData->dCounter_FL = DeltaWheelCounter(pESCData->Counter_FL, g_last_wheelcount_FL);
    pESCData->dCounter_FR = DeltaWheelCounter(pESCData->Counter_FR, g_last_wheelcount_FR);
    pESCData->dCounter_RL = DeltaWheelCounter(pESCData->Counter_RL, g_last_wheelcount_RL);
    pESCData->dCounter_RR = DeltaWheelCounter(pESCData->Counter_RR, g_last_wheelcount_RR);
    g_last_wheelcount_FL  = pESCData->Counter_FL;
    g_last_wheelcount_FR  = pESCData->Counter_FR;
    g_last_wheelcount_RL  = pESCData->Counter_RL;
    g_last_wheelcount_RR  = pESCData->Counter_RR;
    pAllData->SensorMeasurements.ESC_Data.ESC_VehSpd_corrected =
        pAllData->SensorMeasurements.ESC_Data.ESC_VehSpd *
        pAllData->EPPROM_Data.ESC_VehSpd_factor;
    // 2,cal vehicle state,forward/backward/still
    pAllData->LinearMotionData.car_moving =
        car_moving_stateByDcounter(pESCData->dCounter_RL, pESCData->dCounter_RR);
    if (pSensorData->Motorspd > gearMotorThresh)
    {
        pSensorData->gear = GEAR_FWD;
    }
    else if (pSensorData->Motorspd < -gearMotorThresh)
    {
        pSensorData->gear = GEAR_BKWD;
    }
    else
    {
        pSensorData->gear = GEAR_STILL;
    }

    // wait for gear change done
    if (Is_gear_done(pSensorData->GearSt))
    {
        directed_car_moving =
            car_moving_stateByMotor(pAllData->SensorMeasurements.Motorspd);

        if (directed_car_moving == 1)
        {
            pAllData->LinearMotionData.Vel_MoveMentSt = LOCAT_CONSTSPD_FOR;
        }
        else if (directed_car_moving == -1)
        {
            pAllData->LinearMotionData.Vel_MoveMentSt = LOCAT_CONSTSPD_BACK;
        }
        else
        {
            pAllData->LinearMotionData.Vel_MoveMentSt = LOCAT_STILL;
        }
    }

    /*Identify if car is moving or not 2018-7-3*/
    if (fabsf(pAllData->SensorMeasurements.Motorspd) >= 5 ||
        fabsf(pAllData->SensorMeasurements.EPS_AngleSpd) >= 2.0f)
    {
        pAllData->LinearMotionData.car_moving = 1;
    }

    pAllData->SensorMeasurements.Motorspd =
        pAllData->SensorMeasurements.Motorspd *
        (float)(pAllData->LinearMotionData.car_moving); // added 2018-5-4

    // 3,cal IMU zero offset

    // 4,cal TrajCurve
    pSensorData->curvatureByEPS =
        PK_EPS_to_Rou(pSensorData->EPS_Angle, pSensorData->gear);
    R_F2R_Rate = 1.0F + pSensorData->curvatureByEPS * pSensorData->curvatureByEPS *
                            WHEEL_BASE * WHEEL_BASE;
    R_F2R_Rate   = 1.0F / sqrtf(R_F2R_Rate); // front wheel to real wheel rate
    *pR_F2R_Rate = R_F2R_Rate;               //
    // 5,cal dsMotor
    pAllData->LinearMotionData.v_motor =
        Motorspd_fac_vspd * pSensorData->Motorspd * R_F2R_Rate;
    pAllData->LinearMotionData.ds_motor =
        pAllData->LinearMotionData.v_motor * pAllData->location_dt;

    if (fabsf(pSensorData->EPS_Angle) < SMALL_EPSANGLE)
    {
        Veh_mass_level = mass_level_function(pAllData->LinearMotionData.SpeedForward,
                                             pImuData->Accel_X, 0.0F,
                                             pSensorData->MotorTorq, mass_estimate_res);
    }
    pSensorData->Veh_mass_level = Veh_mass_level;
    pSensorData->Veh_mass       = mass_estimate_res[0];

    return RTN_OK;
}

/**
 * @brief 更新定位相关的全局变量，并根据状态选择性写入日志文件
 *
 * 该函数从传入的 pAllData 中读取IMU、ESC、GPS、EPS等传感器数据和车辆状态，
 * 更新到对应的全局变量中。同时根据定位模块状态和记录标志，打开或关闭日志文件，
 * 并将关键数据写入日志，便于后续分析与调试。
 *
 * @param FcnEnableState 功能使能状态结构体，控制各子模块是否使能
 * @param pAllData 指向所有传感器数据和状态数据的结构体指针
 * @return RtnType 返回函数执行状态，通常为 RTN_OK
 */
RtnType LocationSetGlobalVar(FunctionEnableStateType FcnEnableState,
                             AllDataType *pAllData)
{
    static FILE *flog        = NULL;
    const int recordflag     = 0;
    PK_ModuleComType locMode = RTE_PK_StateManage_Get_ModuleCom_Location();
    if (locMode == MCOM_ON && flog == NULL && recordflag == 1)
    {
        char fileName[128] = {0};
        struct tm absStamp;
        RTE_BSW_Get_AbsStamp(&absStamp);
        snprintf(fileName, sizeof(fileName) - 1,
                 "log/%d_%02d_%02d_%02d_%02d_%02d_loc.txt", absStamp.tm_year,
                 absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour, absStamp.tm_min,
                 absStamp.tm_sec);

        flog = fopen(fileName, "w");
    }
    else if (locMode != MCOM_ON && flog != NULL)
    {
        fclose(flog);
        flog = NULL;
    }

    CHECK_FUNC_EN(FcnEnableState.LocationRTESetGlobalVar_EnableState);

    Accel_X1_CCP      = pAllData->SensorMeasurements.IMU_Data.Accel_X;
    Accel_Y1_CCP      = pAllData->SensorMeasurements.IMU_Data.Accel_Y;
    Accel_Z1_CCP      = pAllData->SensorMeasurements.IMU_Data.Accel_Z;
    AngRate_X_CCP     = pAllData->SensorMeasurements.IMU_Data.AngRate_X_deg;
    AngRate_Y_CCP     = pAllData->SensorMeasurements.IMU_Data.AngRate_Y_deg;
    AngRate_Z_CCP     = pAllData->SensorMeasurements.IMU_Data.AngRate_Z_deg;
    Counter_RL_ccp    = pAllData->SensorMeasurements.ESC_Data.Counter_RL;
    Counter_RR_ccp    = pAllData->SensorMeasurements.ESC_Data.Counter_RR;
    Counter_FL_ccp    = pAllData->SensorMeasurements.ESC_Data.Counter_FL;
    Counter_FR_ccp    = pAllData->SensorMeasurements.ESC_Data.Counter_FR;
    dCounter_RL_ccp   = pAllData->SensorMeasurements.ESC_Data.dCounter_RL;
    dCounter_RR_ccp   = pAllData->SensorMeasurements.ESC_Data.dCounter_RR;
    dCounter_FL_ccp   = pAllData->SensorMeasurements.ESC_Data.dCounter_FL;
    dCounter_FR_ccp   = pAllData->SensorMeasurements.ESC_Data.dCounter_FR;
    GPS_Longitude_ccp = pAllData->SensorMeasurements.GPS_Data.GPS_Longitude;
    GPS_Latitude_ccp  = pAllData->SensorMeasurements.GPS_Data.GPS_Latitude;
    GPS_Altitude_ccp  = pAllData->SensorMeasurements.GPS_Data.GPS_Altitude;
    GPS_Bearing_ccp   = pAllData->SensorMeasurements.GPS_Data.GPS_Bearing;
    GPS_Accuracy_ccp  = pAllData->SensorMeasurements.GPS_Data.GPS_Accuracy;
    GPS_speed_ccp     = pAllData->SensorMeasurements.GPS_Data.GPS_Speed;
    EPS_angle_ccp     = pAllData->SensorMeasurements.EPS_Angle;
    EPS_angle_spd_ccp = pAllData->SensorMeasurements.EPS_AngleSpd;
    Motorspd_ccp      = pAllData->SensorMeasurements.Motorspd;
    ESC_VehSpd_ccp    = pAllData->SensorMeasurements.ESC_Data.ESC_VehSpd;

    SideAcc_ccp             = pAllData->RotationalMotionData.SideAcc;
    CurPos_ccp[0]           = pAllData->LocationData.x; // replaced by EPPROM
    CurPos_ccp[1]           = pAllData->LocationData.y;
    CurPos_ccp[2]           = pAllData->LocationData.heading;
    CurPos_ccp[3]           = pAllData->LocationData.s;
    CurSpd_ccp              = pAllData->LinearMotionData.v_motor;
    accelx_filtered_ccp     = pAllData->LinearMotionData.AccelForwardByWheel;
    Vehicle_Roll_ccp        = pAllData->RotationalMotionData.roll_deg;
    Vehicle_Pitch_ccp       = pAllData->RotationalMotionData.pitch_deg;
    AccelForwardIMU_CCP     = pAllData->LinearMotionData.AccelForwardbyIMU;
    Location_error_code_ccp = pAllData->location_err_code;

    Vel_Yawrate_ccp = pAllData->SensorMeasurements.IMU_Data.AngRate_Z_rad -
                      pAllData->SensorMeasurements.IMU_Data.AngRate_Z_offset_rad;
    CurSpd_filtered_ccp = pAllData->LinearMotionData.SpeedForward;
    CurSpd_Motor_ccp    = pAllData->LinearMotionData.v_motor;

    Veh_mass_ccp = pAllData->SensorMeasurements.Veh_mass; // vehicle mass estimated
    Veh_mass_level_ccp =
        pAllData->SensorMeasurements.Veh_mass_level; // vehicle mass estimated
    Vel_MoveMentSt_ccp = pAllData->LinearMotionData.Vel_MoveMentSt;
    Vel_TrajCurve_ccp  = pAllData->SensorMeasurements.curvatureByEPS;
    curvature_ccp      = pAllData->RotationalMotionData.curvatureByIMU;

    EPS_Offset        = pAllData->EPPROM_Data.EPS_Offset;
    ESC_VehSpd_factor = pAllData->EPPROM_Data.ESC_VehSpd_factor;
    Motorspd_fac_vspd = pAllData->EPPROM_Data.Motorspd_fac_vspd;
    Rear_wheel_fac_ds = pAllData->EPPROM_Data.Rear_wheel_fac_ds;

    float flspd      = RTE_BSW_Get_ESC_WheelSpdFL() / 3.6;
    float frspd      = RTE_BSW_Get_ESC_WheelSpdFR() / 3.6;
    float rlspd      = RTE_BSW_Get_ESC_WheelSpdRL() / 3.6;
    float rrspd      = RTE_BSW_Get_ESC_WheelSpdRR() / 3.6;
    float motorSpd   = RTE_BSW_Get_Motorspd();
    float yaw_esc    = RTE_BSW_Get_ESC_YAW_RATE();
    float yaw_offset = RTE_BSW_Get_ESC_YAW_RATE_Offset();
    float yaw_imu    = RTE_BSW_Get_AngRate_Z();

    uint16_t state = 0;
    state          = state | (FcnEnableState.LocationGetEPPROM_EnableState << 0);
    state          = state | (FcnEnableState.LocationInitialization_EnableState << 1);
    state          = state | (FcnEnableState.LocationRTEGetSensor_EnableState << 2);
    state          = state | (FcnEnableState.ParameterTuning_EnableState << 3);
    state          = state | (FcnEnableState.DirectSensorDiagnostics_EnableState << 4);
    state          = state | (FcnEnableState.CalIndirectSensorData_EnableState << 5);
    state          = state | (FcnEnableState.TranslationalMotion_EnableState << 6);
    state = state | (FcnEnableState.IndirectDirectSensorDiagnostics_EnableState << 7);
    state = state | (FcnEnableState.RotationalMotion_EnableState << 8);
    state = state | (FcnEnableState.AutoCalibration_EnableState << 9);
    state = state | (FcnEnableState.Localization_EnableState << 10);
    state = state | (FcnEnableState.LocationRTESetGlobalVar_EnableState << 11);
    state = state | (FcnEnableState.LocationRTESetData_EnableState << 12);
    state = state | (FcnEnableState.LocationSetEPPROM_EnableState << 13);
    state = state | (FcnEnableState.CollisionDetect_EnableState << 14);

    if (flog != NULL)
    {
        struct timeval stamp;
        gettimeofday(&stamp, NULL);
        fprintf(flog,
                "%06ld:%06ld state:0x%04x pos: %06f %06f %06f %06f %06f %06f "
                "counter:%02d %02d %02d %02d counter:%06f %06f %06f %06f vel: %06f %06f, "
                "angle: %06f %06f %06f %06f %06f\n",
                stamp.tv_sec, stamp.tv_usec, state, CurPos_ccp[0], CurPos_ccp[1],
                CurPos_ccp[2], imu_position_ccp[0], imu_position_ccp[1],
                imu_position_ccp[2], dCounter_FL_ccp, dCounter_FR_ccp, dCounter_RL_ccp,
                dCounter_RR_ccp, flspd, frspd, rlspd, rrspd, motorSpd, CurSpd_Motor_ccp,
                Vel_Yawrate_ccp, EPS_angle_ccp, yaw_esc, yaw_offset, yaw_imu);
    }

    return RTN_OK;
}

/**
 * @brief 设置定位模块相关的RTE数据
 *
 * 该函数根据使能状态，将车辆当前位置信息及速度、轨迹曲率、偏航率等
 * 运动学数据写入到RTE接口，供其他模块调用。同时更新定位模块状态和碰撞标志。
 *
 * @param func_en 功能使能标志，控制是否执行数据写入
 * @param pAllData 指向包含所有传感器与状态数据的结构体指针
 * @return RtnType 返回函数执行状态，通常为 RTN_OK
 */
RtnType LocationRTESetData(IsEnabled func_en, AllDataType *pAllData)
{
    Localization_state state_code = LC_NORMAL;
    // set data to PK
    RD_PosStamp pos;
    pos.t = LocationUpdataTime;
    memcpy(pos.pos, imu_position_ccp, sizeof(pos.pos));
    if (func_en != ENABLED)
    {
        pos.pos[3] = -1.0f;
    }
    RTE_PK_Location_Set_CurPos_With_Timestamp(&pos);
    RTE_PK_Location_Set_CurPos(pos.pos);

    CHECK_FUNC_EN(func_en);
    state_code = pAllData->state_code;
    // RTE_PK_Location_Set_Vel_Spd(CurSpd_ccp);
    // RTE_PK_Location_Set_Vel_TrajCurve(Vel_TrajCurve_ccp);
    // RTE_PK_Location_Set_Vel_Yawrate(Vel_Yawrate_ccp);
    RTE_PK_Location_Set_ModuleState_Location((PK_ModuleStateType)state_code);
    RTE_PK_Location_Set_Vel_MoveMentSt(pAllData->LinearMotionData.Vel_MoveMentSt);
    RTE_PK_Location_Set_Vel_Spd(CurSpd_Motor_ccp);
    RTE_PK_Location_Set_Vel_TrajCurve(Vel_TrajCurve_ccp);
    RTE_PK_Location_Set_Vel_Yawrate(Vel_Yawrate_ccp);
    RTE_PK_Location_Set_Vel_SpdFiltered(CurSpd_filtered_ccp);
    RTE_PK_Location_Set_Vel_Pitch(Vehicle_Pitch_ccp);

    return RTN_OK;
}

/**
 * @brief 旋转运动计算
 *
 * @param func_en 函数使能
 * @param reset_all 是否重置卡尔曼滤波器
 * @param pAllData 位置传感器数据和计算数据指针
 * @return RtnType 返回函数状态，通常为 RTN_OK
 */
static RtnType RotationalMotion(IsEnabled func_en, ResetType reset_all,
                                AllDataType *pAllData)
{
    ekfEuler_atti_arg_Type *pekfEuler_atti_arg =
        &(pAllData->RotationalMotionData.ekfEuler_atti_arg); // static ???
    IMU_DataType *pIMU_Data = &(pAllData->SensorMeasurements.IMU_Data);
    RotationalMotionType *pRotationalMotionData = &(pAllData->RotationalMotionData);

    float a_centr_y;
    float a_centr_z;

    CHECK_FUNC_EN(func_en);

    a_centr_y = pAllData->LinearMotionData.SpeedForward *
                (pIMU_Data->AngRate_Z_rad - pIMU_Data->AngRate_Z_offset_rad); // to radian
    a_centr_z =
        pAllData->LinearMotionData.SpeedForward * pIMU_Data->AngRate_Y_rad; // to radian

    pekfEuler_atti_arg->z[0] =
        pIMU_Data->Accel_X -
        pAllData->LinearMotionData.AccelForwardByWheel;         // ax correction
    pekfEuler_atti_arg->z[1]  = pIMU_Data->Accel_Y - a_centr_y; // ay correction
    pekfEuler_atti_arg->z[2]  = pIMU_Data->Accel_Z - a_centr_z; // az correction
    pekfEuler_atti_arg->z[3]  = pIMU_Data->AngRate_X_rad;
    pekfEuler_atti_arg->z[4]  = pIMU_Data->AngRate_Y_rad;
    pekfEuler_atti_arg->z[5]  = pIMU_Data->AngRate_Z_rad;
    pekfEuler_atti_arg->speed = pAllData->LinearMotionData.SpeedForward;
    pekfEuler_atti_arg->dt    = pAllData->location_dt;

    if (reset_all == RESET)
    {
        pekfEuler_atti_arg->reset = RESET;
    }

    b_ekfEuler_atti(pekfEuler_atti_arg); // Attitude estimate
    pekfEuler_atti_arg->reset = NOT_RESET;

    if (fabsf(pRotationalMotionData->roll_rad) > 40.0F * DEG2RAD) // restraint 40
    {
        pRotationalMotionData->roll_rad =
            sign(pRotationalMotionData->roll_rad) * 40.0f * DEG2RAD;
        pekfEuler_atti_arg->reset = RESET;
    }

    if (fabsf(pRotationalMotionData->pitch_rad) > 40.0F * DEG2RAD) // restraint 40
    {
        pRotationalMotionData->pitch_rad =
            sign(pRotationalMotionData->pitch_rad) * 40.0f * DEG2RAD;
        pekfEuler_atti_arg->reset = RESET;
    }

    pRotationalMotionData->roll_rad  = pekfEuler_atti_arg->result[0];
    pRotationalMotionData->roll_deg  = pRotationalMotionData->roll_rad * RAD2DEG;
    pRotationalMotionData->pitch_rad = pekfEuler_atti_arg->result[1];
    pRotationalMotionData->pitch_deg = pRotationalMotionData->pitch_rad * RAD2DEG;
    pAllData->LinearMotionData.AccelForwardbyIMU =
        pIMU_Data->Accel_X + GRAVITY * sinf(pRotationalMotionData->pitch_rad);

    // calculate curvature
    pAllData->RotationalMotionData.curvatureByIMU =
        cal_curvature(pIMU_Data->AngRate_Z_rad - pIMU_Data->AngRate_Z_offset_rad,
                      pAllData->LinearMotionData.SpeedForward, 0U);
    pAllData->RotationalMotionData.SideAcc =
        pAllData->LinearMotionData.SpeedForward *
        (pIMU_Data->AngRate_Z_rad - pIMU_Data->AngRate_Z_offset_rad);
    return RTN_OK;
}

/**
 * @brief 计算车辆平移运动状态
 *
 * 根据轮速计、车辆速度、纠正速度及加速度，结合卡尔曼滤波计算
 * 车辆前向速度、加速度及轮速相关参数，识别前轮打滑状态。
 *
 * @param[in] func_en 功能使能标志
 * @param[in,out] pAllData 车辆传感器及运动数据结构指针
 *
 * @return RTN_OK 始终返回成功
 */
RtnType TranslationalMotion(IsEnabled func_en, AllDataType *pAllData)
{
    static uint8_t reset_kf3d            = 0;
    static float kf3d_z[2]               = {0.0f, 0.0f};
    float kf3d_q                         = 0.005f;
    float kf3d_r[2]                      = {0.01f, 0.01f};
    static float kf3d_res[3]             = {0.0f, 0.0f, 0.0f};
    static const float UPPER_LIMIT       = 36.0F;
    static const float LOWER_LIMIT       = 26.0F;
    static const float SPEED_LOWER_LIMIT = 1.5F;
    static const float SPEED_BOUND       = 36.0F;
    static const float SLOPE             = 0.1F;
    static float ds_rear_wheel           = 0.0F;
    int dcounterByMotor                  = 0;
    static uint8_t slip_wheel_front      = 0;

    ESC_DataType *pESC_Data             = &(pAllData->SensorMeasurements.ESC_Data);
    LinearMotionType *pLinearMotionData = &pAllData->LinearMotionData;

    CHECK_FUNC_EN(func_en);

    ds_rear_wheel = pAllData->SensorMeasurements.gear *
                    pAllData->EPPROM_Data.Rear_wheel_fac_ds *
                    (pESC_Data->dCounter_RL + pESC_Data->dCounter_RR);
    if (pAllData->EPPROM_Data.Rear_wheel_fac_ds > 0.0F) // avoid divided by zero
    {
        dcounterByMotor = (int)(fabsf(pLinearMotionData->ds_motor) /
                                pAllData->EPPROM_Data.Rear_wheel_fac_ds * 0.5F);
    }
    else
    {
        dcounterByMotor = 0;
    }

    if ((fabsf(pLinearMotionData->SpeedForward) >= SPEED_LOWER_LIMIT) &&
        (((pESC_Data->dCounter_FL == 0) && (pESC_Data->dCounter_FR == 0) &&
          (pESC_Data->dCounter_RL == 0) && (pESC_Data->dCounter_RR == 0)) ||
         (((pESC_Data->dCounter_FL * pLinearMotionData->R_F2R_Rate - dcounterByMotor) >
           dcounterByMotor - 2) &&
          ((pESC_Data->dCounter_FR * pLinearMotionData->R_F2R_Rate - dcounterByMotor) >
           dcounterByMotor - 2) &&
          ((pESC_Data->dCounter_RL - dcounterByMotor) > dcounterByMotor - 2) &&
          ((pESC_Data->dCounter_RR - dcounterByMotor) >
           dcounterByMotor - 2)))) // error or dcounter too small
    {
        pESC_Data->CounterState = ST_ERR;
    }
    else
    {
        pESC_Data->CounterState = ST_OK;
    }

    // pESC_Data->CounterState = ST_OK;//to do remove
    if (pESC_Data->CounterState == ST_ERR)
    {
        ds_rear_wheel = pAllData->LinearMotionData.ds_motor;
    }

    kf3d_z[1] = pAllData->LinearMotionData.v_motor;

    if (GETBITS(pAllData->location_err_code, BIT(1), BIT(4)) > 0) // to do???????
    {
        reset_kf3d = 1;
    }

    // avoid accel jump when switching to ESC_VehSpd_corrected
    if (fabsf(pESC_Data->ESC_VehSpd_corrected) <= UPPER_LIMIT &&
        fabsf(pESC_Data->ESC_VehSpd_corrected) >= LOWER_LIMIT)
    {
        float temp0 = fabsf(pESC_Data->ESC_VehSpd_corrected) - LOWER_LIMIT;
        kf3d_z[1]   = (1.0f - SLOPE * temp0) * pAllData->LinearMotionData.v_motor +
                    0.1f * temp0 * pESC_Data->ESC_VehSpd_corrected * KMH2MS;
        ds_rear_wheel = (1.0f - SLOPE * temp0) * ds_rear_wheel +
                        SLOPE * temp0 * pESC_Data->ESC_VehSpd_corrected * KMH2MS *
                            pAllData->location_dt;
    }

    if (((pESC_Data->ESC_VehSpd_corrected > UPPER_LIMIT) &&
         (pESC_Data->ESC_VehSpdVD == 1)) ||
        slip_wheel_front == 1)
    {
        ds_rear_wheel = pESC_Data->ESC_VehSpd_corrected * KMH2MS * pAllData->location_dt;
        kf3d_z[1]     = pESC_Data->ESC_VehSpd_corrected * KMH2MS;
    }

    pAllData->LinearMotionData.ds_wheel = ds_rear_wheel;

    kf3d(kf3d_z[1], pAllData->location_dt, kf3d_q, kf3d_r, ds_rear_wheel, reset_kf3d,
         kf3d_res); // to do if speed is high, motorspd is not accurate at ACC
    reset_kf3d = 0;

    if (fabsf(kf3d_res[1]) >= 50.0f) // speed more than 50m/s
    {
        kf3d_res[1] = sign(kf3d_res[1]) * 50.0f;
        reset_kf3d  = 1;
    }

    if (fabsf(kf3d_res[2]) >= 10.0f) // accel more than 10 m/s^2
    {
        kf3d_res[2] = sign(kf3d_res[2]) * 10.0f;
        reset_kf3d  = 1;
    }

    pAllData->LinearMotionData.SpeedForward          = kf3d_res[1];
    pAllData->LinearMotionData.AccelForwardByWheel   = kf3d_res[2];
    pAllData->LinearMotionData.vel_speed_filtered_RD = kf3d_res[1];

    if (fabsf(pAllData->LinearMotionData.AccelForwardByWheel -
              pAllData->LinearMotionData.AccelForwardbyIMU) >
        2.0f) // if accel differs more than 2.0, to do determin shreshold
    {
        slip_wheel_front = 1;
    }
    else
    {
        slip_wheel_front = 0;
    }

    if (((pESC_Data->ESC_VehSpd_corrected > SPEED_BOUND) &&
         (pESC_Data->ESC_VehSpd_corrected == 1)) ||
        slip_wheel_front == 1) // slip_wheel_front is global
    {
        pAllData->LinearMotionData.vel_speed_filtered_RD = pESC_Data->ESC_VehSpd * KMH2MS;
        pAllData->LinearMotionData.SpeedForward =
            pESC_Data->ESC_VehSpd_corrected * KMH2MS;
    }

    return RTN_OK;
}

/**
 * @brief 调整 Kalman 滤波器相关参数
 *
 * 本函数在定位模块中用于配置或动态调整 Kalman 滤波器的协方差参数，主要包括：
 * - 姿态估计算法 `ekfEuler_atti_arg` 中的过程噪声（q_r, q_w）和观测噪声（r）；
 * - 位置估计算法 `EKF_LC_PROCESS_arg` 中的过程噪声（q）和观测噪声（r）。
 *
 * 设置参数为固定数值，一般在模块初始化后执行一次。如需动态调参，可将参数挂载到调试接口或配置文件中。
 *
 * @param[in]  func_en   功能是否启用（ENABLED / NOT_ENABLED）
 * @param[out] pAllData  定位模块主数据结构指针，内部结构体将被修改
 * @retval     RTN_OK    参数设置成功
 */
RtnType ParameterTuning(IsEnabled func_en, AllDataType *pAllData)
{
    CHECK_FUNC_EN(func_en);
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_r[0] = 0.001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_r[1] = 0.001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_r[2] = 0.001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_w[0] = 0.0001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_w[1] = 0.0001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.q_w[2] = 0.0001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.r[0]   = 0.001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.r[1]   = 0.001F;
    pAllData->RotationalMotionData.ekfEuler_atti_arg.r[2]   = 0.001F;
    pAllData->EKF_LC_PROCESS_arg.q[0]                       = 0.0001F;
    pAllData->EKF_LC_PROCESS_arg.q[1]                       = 0.0001F;
    pAllData->EKF_LC_PROCESS_arg.r[0]                       = 4.0F;
    pAllData->EKF_LC_PROCESS_arg.r[1]                       = 4.0F;

    return RTN_OK;
}
/**
 * @brief 本地化估计函数，使用扩展卡尔曼滤波进行位置和姿态估计
 *
 * 该函数读取传感器数据，更新卡尔曼滤波参数，执行滤波过程，
 * 并根据滤波结果更新全局位置、姿态等状态信息。
 *
 * @param func_en 函数使能标志
 * @param reset_all 复位标志，用于重置滤波器状态
 * @param pAllData 指向包含传感器数据及状态信息的结构体指针
 *
 * @return 返回操作结果，RTN_OK表示成功
 */
RtnType Localization(IsEnabled func_en, ResetType reset_all, AllDataType *pAllData)
{
    EKF_LC_PROCESS_arg_Type *pEKF_LC_PROCESS_arg = &pAllData->EKF_LC_PROCESS_arg;
    unsigned char EKF_LC_state                   = 0;
    CHECK_FUNC_EN(func_en);

    pEKF_LC_PROCESS_arg->GPS_Longitude =
        pAllData->SensorMeasurements.GPS_Data.GPS_Longitude;
    pEKF_LC_PROCESS_arg->GPS_Latitude =
        pAllData->SensorMeasurements.GPS_Data.GPS_Latitude;
    pEKF_LC_PROCESS_arg->GPS_Altitude =
        pAllData->SensorMeasurements.GPS_Data.GPS_Altitude;
    pEKF_LC_PROCESS_arg->GPS_Accuracy =
        pAllData->SensorMeasurements.GPS_Data.GPS_Accuracy;
    pEKF_LC_PROCESS_arg->GPS_Accuracy = 40; // block GPS
    pEKF_LC_PROCESS_arg->AngRate_Z_rad =
        pAllData->SensorMeasurements.IMU_Data.AngRate_Z_rad;
    pEKF_LC_PROCESS_arg->SpeedForward = pAllData->LinearMotionData.SpeedForward;
    pEKF_LC_PROCESS_arg->v_motor      = pAllData->LinearMotionData.v_motor;
    pEKF_LC_PROCESS_arg->dS_wheel     = pAllData->LinearMotionData.ds_wheel;
    pEKF_LC_PROCESS_arg->EPS_angle    = pAllData->SensorMeasurements.EPS_Angle;
    pEKF_LC_PROCESS_arg->AngRate_Z_rad =
        pAllData->SensorMeasurements.IMU_Data.AngRate_Z_rad;
    pEKF_LC_PROCESS_arg->car_moving   = pAllData->LinearMotionData.car_moving;
    pEKF_LC_PROCESS_arg->roll         = pAllData->RotationalMotionData.roll_rad;
    pEKF_LC_PROCESS_arg->pitch        = pAllData->RotationalMotionData.pitch_rad;
    pEKF_LC_PROCESS_arg->EPS_angle    = pAllData->SensorMeasurements.EPS_Angle;
    pEKF_LC_PROCESS_arg->dt           = pAllData->location_dt;
    pEKF_LC_PROCESS_arg->KF_selection = 2;
    pEKF_LC_PROCESS_arg->reset_KF     = (ResetType)0; // todo
    pEKF_LC_PROCESS_arg->reset_all    = reset_all;
    pEKF_LC_PROCESS_arg->slotnumber   = pAllData->SensorMeasurements.slotnumber;
    b_EKF_LC_PROCESS(pEKF_LC_PROCESS_arg);
    pEKF_LC_PROCESS_arg->reset_all = NOT_RESET; // recover from reset

    EKF_LC_state = hold_EKF_LC_reset(pEKF_LC_PROCESS_arg->state == 1 ||
                                     pEKF_LC_PROCESS_arg->state == 3 ||
                                     pEKF_LC_PROCESS_arg->state == 5);
    if (EKF_LC_state == 1)
    {
        pAllData->state_code = LC_DATA_RESET;
    }

    if (pEKF_LC_PROCESS_arg->Pdiag[0] + pEKF_LC_PROCESS_arg->Pdiag[1] +
            pEKF_LC_PROCESS_arg->Pdiag[2] + pEKF_LC_PROCESS_arg->Pdiag[3] >
        1.0e10)
    {
        pEKF_LC_PROCESS_arg->reset_all = RESET;
        pAllData->state_code           = LC_ABNORM;
    }

    pAllData->state_code = (Localization_state)hold_ModuleState_Location(
        (PK_ModuleStateType)pAllData->state_code);
    pAllData->LocationData.Valid   = Valid;
    pAllData->LocationData.x       = pEKF_LC_PROCESS_arg->X[0];
    pAllData->LocationData.y       = pEKF_LC_PROCESS_arg->X[1];
    pAllData->LocationData.heading = pEKF_LC_PROCESS_arg->X[2];
    pAllData->LocationData.s       = pEKF_LC_PROCESS_arg->PK_locat[3];
    pAllData->SensorMeasurements.IMU_Data.AngRate_Z_offset_rad =
        pEKF_LC_PROCESS_arg->X[3];

    return RTN_OK;
}

/**
 * @brief 初始化定位模块数据
 *
 * 本函数用于在模块首次启用时，对定位模块相关的数据结构进行清零和初始化操作。
 * 包括：
 * - 清空 `AllDataType` 数据结构；
 * - 从 EEPROM 读取持久化参数；
 * - 设置定位周期时间；
 * - 调用 codegenall 初始化函数。
 *
 * @param[in]  func_en   模块是否启用（ENABLED / NOT_ENABLED）
 * @param[out] pAllData  存储传感器与定位计算的主数据结构指针
 * @retval     RTN_OK    初始化成功
 */
RtnType LocationInitialization(IsEnabled func_en, AllDataType *pAllData)
{
    CHECK_FUNC_EN(func_en);
    memset((uint8_t *)pAllData, 0, sizeof(AllDataType));
    // get data from EPPROM
    LocationGetEPPROM(func_en, &(pAllData->EPPROM_Data));
    pAllData->location_dt = LocatingPeriod_dt;
    codegenall_initialize();

    return RTN_OK;
}

/****************************************************************************
FUNC NAME  : LocationCOM
DESCRIPTION: set location module ON or OFF
PARA IN    : void
PARA OUT   : void
RETURN     : void
****************************************************************************/
PK_ModuleComType LocationCOM(void) { return RTE_PK_StateManage_Get_ModuleCom_Location(); }

/**
 * @brief 管理定位模块的功能使能与重置状态
 *
 * 本函数用于根据通信状态 `LocationCOM()`
 * 的结果，统一管理各个定位子模块的使能状态（ENABLED/NOT_ENABLED）和重置状态（RESET/NOT_RESET）。
 * - 如果通信状态为 MCOM_ON 且已经完成初始化，则启用全部功能模块，并清除所有重置状态；
 * - 如果通信状态非 MCOM_ON，或初始化未完成，则禁用所有功能模块，并设置所有模块为 RESET；
 * - 若是第一次进入 MCOM_ON 状态，会仅启用部分初始化相关模块，并将 init_location 标记为
 * true。
 *
 * @param[in,out] pFcnEnableState 各功能模块的使能状态控制结构体指针
 * @param[in,out] pFcnResetState  各功能模块的重置状态控制结构体指针
 * @return void
 */
void LocationManagement(FunctionEnableStateType *pFcnEnableState,
                        FunctionResetStateType *pFcnResetState)
{
    PK_ModuleComType ModuleCom_Location;
    static boolean init_location = false; // flag to initialize all location ccp
    ModuleCom_Location           = LocationCOM();
    if (ModuleCom_Location == MCOM_ON && init_location == true)
    {
        memset(pFcnResetState, 0, sizeof(FunctionResetStateType));
        pFcnEnableState->LocationGetEPPROM_EnableState               = ENABLED;
        pFcnEnableState->LocationInitialization_EnableState          = NOT_ENABLED;
        pFcnEnableState->LocationRTEGetSensor_EnableState            = ENABLED;
        pFcnEnableState->ParameterTuning_EnableState                 = ENABLED;
        pFcnEnableState->DirectSensorDiagnostics_EnableState         = ENABLED;
        pFcnEnableState->CalIndirectSensorData_EnableState           = ENABLED;
        pFcnEnableState->TranslationalMotion_EnableState             = ENABLED;
        pFcnEnableState->IndirectDirectSensorDiagnostics_EnableState = ENABLED;
        pFcnEnableState->RotationalMotion_EnableState                = ENABLED;
        pFcnEnableState->AutoCalibration_EnableState                 = ENABLED;
        pFcnEnableState->Localization_EnableState                    = ENABLED;
        pFcnEnableState->LocationRTESetGlobalVar_EnableState         = ENABLED;
        pFcnEnableState->LocationRTESetData_EnableState              = ENABLED;
        pFcnEnableState->LocationSetEPPROM_EnableState               = ENABLED;
        pFcnEnableState->CollisionDetect_EnableState                 = ENABLED;
    }
    else
    {
        memset(pFcnEnableState, 0, sizeof(FunctionEnableStateType));
        pFcnResetState->LocationGetEPPROM_ResetState               = RESET;
        pFcnResetState->LocationInitialization_ResetState          = RESET;
        pFcnResetState->LocationRTEGetSensor_ResetState            = RESET;
        pFcnResetState->ParameterTuning_ResetState                 = RESET;
        pFcnResetState->DirectSensorDiagnostics_ResetState         = RESET;
        pFcnResetState->CalIndirectSensorData_ResetState           = RESET;
        pFcnResetState->TranslationalMotion_ResetState             = RESET;
        pFcnResetState->IndirectDirectSensorDiagnostics_ResetState = RESET;
        pFcnResetState->RotationalMotion_ResetState                = RESET;
        pFcnResetState->AutoCalibration_ResetState                 = RESET;
        pFcnResetState->Localization_ResetState                    = RESET;
        pFcnResetState->LocationRTESetGlobalVar_ResetState         = RESET;
        pFcnResetState->LocationRTESetData_ResetState              = RESET;
        pFcnResetState->LocationSetEPPROM_ResetState               = RESET;
        pFcnResetState->CollisionDetect_ResetState                 = RESET;
        init_location                                              = false;
    }

    if ((init_location == false) && (ModuleCom_Location == MCOM_ON))
    {
        pFcnEnableState->LocationInitialization_EnableState  = ENABLED;
        pFcnEnableState->LocationRTEGetSensor_EnableState    = ENABLED;
        pFcnEnableState->LocationRTESetGlobalVar_EnableState = ENABLED;
        init_location                                        = true;
    }
}

/**
 * @brief 根据错误码管理定位功能的启用与复位状态
 *
 * 本函数用于根据传感器诊断结果（即
 * ErrorCode），统一控制各定位相关子模块的启用状态（EnableState）与复位标志（ResetState）。
 * 若发现关键传感器存在错误（如 IMU、EPS、轮速、角度等），将：
 * - 禁用相关功能模块的执行；
 * - 设置相应模块的复位标志位；
 * - 更新系统状态码为 `LC_SENSOR_ERR`。
 *
 * 若所有相关传感器状态均正常，则系统状态设为 `LC_NORMAL`。
 *
 * @param[in]  ErrorCode         综合错误码（每个位表示某传感器的诊断状态）
 * @param[out] pFcnEnableState   各功能模块是否启用的结构体指针
 * @param[out] pFcnResetState    各功能模块是否复位的结构体指针
 * @param[out] pAllData          定位主数据结构体指针，用于记录当前状态码
 */
void LocationManagementByErrorCode(uint32 ErrorCode,
                                   FunctionEnableStateType *pFcnEnableState,
                                   FunctionResetStateType *pFcnResetState,
                                   AllDataType *pAllData)
{
    ErrorStatetype ErrorStateID;

    ErrorStateID.IMU_ST         = (StateType)GET_BIT(ErrorCode, IMU_ID);
    ErrorStateID.WHEEL_FL_ST    = (StateType)GET_BIT(ErrorCode, WHEEL_FL_ID);
    ErrorStateID.WHEEL_FR_ST    = (StateType)GET_BIT(ErrorCode, WHEEL_FR_ID);
    ErrorStateID.WHEEL_RL_ST    = (StateType)GET_BIT(ErrorCode, WHEEL_RL_ID);
    ErrorStateID.WHEEL_RR_ST    = (StateType)GET_BIT(ErrorCode, WHEEL_RR_ID);
    ErrorStateID.EPS_ANGLE_ST   = (StateType)GET_BIT(ErrorCode, EPS_ANGLE_ID);
    ErrorStateID.ESC_SPEED_ST   = (StateType)GET_BIT(ErrorCode, ESC_SPEED_ID);
    ErrorStateID.MOTOR_SPEED_ST = (StateType)GET_BIT(ErrorCode, MOTOR_SPEED_ID);
    ErrorStateID.GPS_ST         = (StateType)GET_BIT(ErrorCode, GPS_ID);
    ErrorStateID.WHEEL_PRESSURE_FL_ST =
        (StateType)GET_BIT(ErrorCode, WHEEL_PRESSURE_FL_ID);
    ErrorStateID.WHEEL_PRESSURE_FR_ST =
        (StateType)GET_BIT(ErrorCode, WHEEL_PRESSURE_FR_ID);
    ErrorStateID.WHEEL_PRESSURE_RL_ST =
        (StateType)GET_BIT(ErrorCode, WHEEL_PRESSURE_RL_ID);
    ErrorStateID.WHEEL_PRESSURE_RR_ST =
        (StateType)GET_BIT(ErrorCode, WHEEL_PRESSURE_RR_ID);

    if (ErrorStateID.IMU_ST == ST_ERR || ErrorStateID.WHEEL_RL_ST == ST_ERR ||
        ErrorStateID.WHEEL_RR_ST == ST_ERR || ErrorStateID.EPS_ANGLE_ST == ST_ERR ||
        ErrorStateID.ESC_SPEED_ST == ST_ERR || ErrorStateID.MOTOR_SPEED_ST == ST_ERR)
    {
        // pFcnEnableState->LocationGetEPPROM_EnableState = NOT_ENABLED;
        // pFcnEnableState->LocationInitialization_EnableState = NOT_ENABLED;
        // pFcnEnableState->LocationRTEGetSensor_EnableState = NOT_ENABLED;
        // pFcnEnableState->ParameterTuning_EnableState = NOT_ENABLED;
        // pFcnEnableState->DirectSensorDiagnostics_EnableState = NOT_ENABLED;
        pFcnEnableState->CalIndirectSensorData_EnableState           = NOT_ENABLED;
        pFcnEnableState->TranslationalMotion_EnableState             = NOT_ENABLED;
        pFcnEnableState->RotationalMotion_EnableState                = NOT_ENABLED;
        pFcnEnableState->IndirectDirectSensorDiagnostics_EnableState = NOT_ENABLED;
        pFcnEnableState->AutoCalibration_EnableState                 = NOT_ENABLED;
        pFcnEnableState->Localization_EnableState                    = NOT_ENABLED;
        // pFcnEnableState->LocationRTESetGlobalVar_EnableState = NOT_ENABLED;
        // pFcnEnableState->LocationRTESetData_EnableState = NOT_ENABLED;
        pFcnEnableState->LocationSetEPPROM_EnableState = NOT_ENABLED;

        pFcnResetState->LocationGetEPPROM_ResetState               = RESET;
        pFcnResetState->LocationInitialization_ResetState          = RESET;
        pFcnResetState->LocationRTEGetSensor_ResetState            = RESET;
        pFcnResetState->ParameterTuning_ResetState                 = RESET;
        pFcnResetState->DirectSensorDiagnostics_ResetState         = RESET;
        pFcnResetState->CalIndirectSensorData_ResetState           = RESET;
        pFcnResetState->TranslationalMotion_ResetState             = RESET;
        pFcnResetState->IndirectDirectSensorDiagnostics_ResetState = RESET;
        pFcnResetState->RotationalMotion_ResetState                = RESET;
        pFcnResetState->AutoCalibration_ResetState                 = RESET;
        pFcnResetState->Localization_ResetState                    = RESET;
        // pFcnResetState->LocationRTESetGlobalVar_ResetState                =   RESET;
        // pFcnResetState->LocationRTESetData_ResetState                 =   RESET;
        pFcnResetState->LocationSetEPPROM_ResetState = RESET;

        // memset(pFcnResetState,1,sizeof(FunctionResetStateType));//!!!!
        pAllData->state_code = LC_SENSOR_ERR;
    }
    else
    {

        pAllData->state_code = LC_NORMAL;
    }
}

/**
 * @brief 从EEPROM数据结构中读取参数并更新全局变量
 *
 * 根据使能状态，将EEPROM中存储的转向角偏移、车辆速度因子、马达速度因子、
 * 以及后轮里程因子读取并赋值到对应的全局变量，供定位模块使用。
 *
 * @param func_en 功能使能标志，控制是否执行数据更新
 * @param pAllData 指向包含EEPROM数据的结构体指针
 * @return RtnType 返回函数执行状态，通常为 RTN_OK
 */
RtnType LocationSetEPPROM(IsEnabled func_en, AllDataType *pAllData)
{
    CHECK_FUNC_EN(func_en);

    EEROM_EPS_Offset        = pAllData->EPPROM_Data.EPS_Offset;
    EEROM_ESC_VehSpd_factor = pAllData->EPPROM_Data.ESC_VehSpd_factor;
    EEROM_Motorspd_fac_vspd = pAllData->EPPROM_Data.Motorspd_fac_vspd;
    EEROM_Rear_wheel_fac_ds = pAllData->EPPROM_Data.Rear_wheel_fac_ds;

    return RTN_OK;
}

const uint32_t item_count                                      = 75; // 20ms * 30 = 400ms
static uint8_t location_buff[sizeof(RD_PosStamp) * item_count] = {0};
static RAM_INF locationRam = {location_buff, sizeof(RD_PosStamp),
                              sizeof(RD_PosStamp) * item_count, 0, 0};

/**
 * @brief 保存当前位置数据到内存缓存
 *
 * 根据功能使能和重置标志，决定是否清空缓存或保存当前位置及时间戳。
 * 使用临界区保护缓存操作，防止并发冲突。
 *
 * @param func_en 功能使能标志，若未使能则清空缓存
 * @param reset_all 重置标志，若重置则清空缓存
 * @return RtnType 返回函数执行状态，通常为 RTN_OK
 */
RtnType LocationSaveLocation(IsEnabled func_en, ResetType reset_all)
{
    RD_PosStamp posStamp;
    RTE_PK_Location_Get_CurPos_With_Timestamp(&posStamp);

    Rte_EnterCriticalSection();
    if (func_en == NOT_ENABLED || reset_all == RESET)
    {
        Ram_Clear(&locationRam);
    }
    else
    {
        // if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_R) {  Ram_Clear(&locationRam);  }
        Ram_Put(&locationRam, -1, (uint8_t *)&posStamp, 1);
    }
    Rte_LeaveCriticalSection();

    return RTN_OK;
}

/**
 * @brief 根据时间戳预估定位信息（线性插值）
 *
 * 从缓存中读取历史定位数据，根据传入的时间戳 preMs，
 * 找到对应时间段的数据并进行线性插值，输出估算的定位数据。
 * 如果时间戳超出缓存范围或数据异常，则返回错误。
 *
 * @param preMs 目标时间戳（毫秒）
 * @param posStamp 输出的估算定位数据指针
 * @return int 成功返回 RTN_OK，失败返回 RTN_ERR
 */
int PK_PreLocation(const uint32_t preMs, RD_PosStamp *posStamp)
{
    RD_PosStamp posStamps[item_count];
    Rte_EnterCriticalSection();
    uint32_t readLen = Ram_Get(&locationRam, 0, (uint8_t *)&posStamps, item_count);
    Rte_LeaveCriticalSection();

    // use current locaiton values
    if (readLen < 2 || posStamps[0].t > posStamps[readLen - 1].t)
    {
        return RTN_ERR;
    }

    if (preMs >= posStamps[readLen - 1].t)
    {
        return RTN_ERR;
    }

    // �ӳ�һ��ֻ�м�ʮ���룬ֱ�Ӳ��Ҹ���
    for (int idx = readLen - 1; idx > 0; idx--)
    {
        if (posStamps[idx].t >= preMs && posStamps[idx - 1].t < preMs)
        {
            float factor  = float(preMs - posStamps[idx - 1].t) * 1.0;
            factor        = factor / (float)(posStamps[idx].t - posStamps[idx - 1].t);
            posStamp[0].t = posStamps[idx - 1].t;
            posStamp[0].pos[0] =
                factor * (posStamps[idx].pos[0] - posStamps[idx - 1].pos[0]) +
                posStamps[idx - 1].pos[0];
            posStamp[0].pos[1] =
                factor * (posStamps[idx].pos[1] - posStamps[idx - 1].pos[1]) +
                posStamps[idx - 1].pos[1];
            posStamp[0].pos[2] =
                factor * (posStamps[idx].pos[2] - posStamps[idx - 1].pos[2]) +
                posStamps[idx - 1].pos[2];
            posStamp[0].pos[3] =
                factor * (posStamps[idx].pos[3] - posStamps[idx - 1].pos[3]) +
                posStamps[idx - 1].pos[3];

            // printf("factor %f curstamp %d ,idx %d t: %d pos: %f %f %f %f idx - 1: %d t:
            // %d pos: %f %f %f %f ret: %f %f %f %f\n", factor, preMs,
            //  idx, posStamps[idx].t, posStamps[idx].pos[0], posStamps[idx].pos[1],
            //  posStamps[idx].pos[2], posStamps[idx].pos[3], idx - 1, posStamps[idx -
            //  1].t, posStamps[idx - 1].pos[0], posStamps[idx - 1].pos[1], posStamps[idx
            //  - 1].pos[2], posStamps[idx - 1].pos[3], posStamp[0].pos[0],
            //  posStamp[0].pos[1], posStamp[0].pos[2], posStamp[0].pos[3]);

            // memcpy((void *)posStamp, (void *)&posStamps[idx - 1], sizeof(RD_PosStamp));
            return RTN_OK;
        }
    }

    memcpy((void *)posStamp, (void *)&posStamps[0], sizeof(RD_PosStamp));

    return RTN_ERR;
}

/**
 * @brief 车辆定位主流程函数
 * @note 该函数周期调用，实现实时定位功能。
 */
void PK_Location(void)
{
    if (!running_mark) // 第一次进入
    {
        // 读取配置文件
        Json::Reader jsonReader;
        Json::Value jsonRoot;
        std::ifstream ifs;
        jsonRoot.clear();
        ifs.open("loc.json");
        if (!ifs)
        {
            printf("open loc.json failed!\n");
        }
        else
        {
            printf("open loc.json sucess !\n");
        }
        if (!jsonReader.parse(ifs, jsonRoot))
        {
            printf("read json file failed\n");
        }
        READJSONINT(para_use_imu);
        READJSONDOUBLE(para_veh_yaw_resolution);
        READJSONDOUBLE(para_veh_yaw_still_offset);
        READJSONDOUBLE(para_veh_yaw_moving_clockwise_offset);
        READJSONDOUBLE(para_veh_yaw_moving_counterClockwise_offset);
        READJSONINT(para_save_history_num);
        READJSONINT(para_loc_debug_enabled);
        ifs.close();
        running_mark = true;
    }
    // location run err code
    uint32 LC_SenosrErrorIndicator = 0x01;
    uint32 DiagnosticsState        = 0;
    static AllDataType AllData;
    static FunctionResetStateType FcnResetState = {
        NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET,
        NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET,
        NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET, NOT_RESET};
    static FunctionEnableStateType FcnEnableState = {
        NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED,
        NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED,
        NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED, NOT_ENABLED};

    // 0, Manage all functions
    LocationManagement(&FcnEnableState, &FcnResetState);

    // 2,initialize location module
    LocationInitialization(FcnEnableState.LocationInitialization_EnableState,
                           &AllData); // include wheel counter

    // 3,get data from sensors
    LocationRTEGetSensor(FcnEnableState.LocationRTEGetSensor_EnableState,
                         &AllData.SensorMeasurements); // direct sensor data

    // 4, tune Kalman filter parameters
    ParameterTuning(FcnEnableState.ParameterTuning_EnableState, &AllData);

    // 5,Diagnostics direct from Sensor data,check if errors
    DirectSensorDiagnostics(FcnEnableState.DirectSensorDiagnostics_EnableState, &AllData,
                            &DiagnosticsState);

    LC_SenosrErrorIndicator |= DiagnosticsState;

    LocationManagementByErrorCode(LC_SenosrErrorIndicator, &FcnEnableState,
                                  &FcnResetState, &AllData);

    Collision_detect(FcnEnableState.CollisionDetect_EnableState, &AllData);
    // 6,calculation from direct sensor data
    CalIndirectSensorData(FcnEnableState.CalIndirectSensorData_EnableState, &AllData);

    // 7,cal speed and acc
    TranslationalMotion(FcnEnableState.TranslationalMotion_EnableState, &AllData);

    // 8,Diagnostics direct from Sensor data,check if errors
    IndirectDirectSensorDiagnostics(
        FcnEnableState.IndirectDirectSensorDiagnostics_EnableState, &AllData,
        &DiagnosticsState); // from calculation
    LC_SenosrErrorIndicator |= DiagnosticsState;
    AllData.location_err_code = LC_SenosrErrorIndicator;
    LocationManagementByErrorCode(LC_SenosrErrorIndicator, &FcnEnableState,
                                  &FcnResetState, &AllData);
    // 9,cal att
    RotationalMotion(FcnEnableState.RotationalMotion_EnableState,
                     FcnResetState.RotationalMotion_ResetState, &AllData);

    // 10,Auto Calibration EPS zero offset ��wheel DS factor and ESC speed factor
    AutoCalibration(FcnEnableState.AutoCalibration_EnableState, &AllData);

    Location(FcnEnableState.Localization_EnableState,
             FcnResetState.Localization_ResetState, &AllData);

    // 11,cal pos
    Localization(FcnEnableState.Localization_EnableState,
                 FcnResetState.Localization_ResetState, &AllData);

    // 12,output data for RD/ACC/RECORD
    LocationSetGlobalVar(FcnEnableState, &AllData);
    // Location_NoImu(FcnEnableState.Localization_EnableState,
    // FcnResetState.Localization_ResetState);

    // 13, set all data including RTE and ccp
    LocationRTESetData(FcnEnableState.LocationRTESetData_EnableState, &AllData);

    // 14,set EPROM
    LocationSetEPPROM(FcnEnableState.LocationSetEPPROM_EnableState, &AllData);

    LocationSaveLocation(FcnEnableState.Localization_EnableState,
                         FcnResetState.Localization_ResetState);

    // printf("[ODOdata] ODO-X: %f, ODO-Y: %f, ODO-S: %f, ODO-Ang: %f\n",
    //        imu_position_ccp[0] * 1000, imu_position_ccp[1] * 1000,
    //        imu_position_ccp[3] * 1000, imu_position_ccp[2] * 1000);
}
