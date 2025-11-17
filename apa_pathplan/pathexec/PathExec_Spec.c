#include "MathFunc.h"
#include "PK_Utility.h"
#include "SystemPara.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Debug.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "Rte.h"

const int bar_hold_time    = 60;
const float bar_run_threld = 0.20;
const int rev_fun_time     = 10;

/**
 * @brief 判断当前车辆是否在目标位置附近
 *
 * @param curSt 当前泊车模块参数结构体
 * @return bool
 */
static bool PathExec_NearTarget(const PK_Cur_AutodrPara curSt[1])
{
    Rect_T rectVeh;
    VehPos_T curVehPos = {curSt[0].curpos[0], curSt[0].curpos[1], curSt[0].curpos[2]};
    Get_Veh_CornerPt(curVehPos, &rectVeh, 0);

    float avmPoint[4][2];

    RevConvert(curSt[0].finpos, curSt[0].curpos, avmPoint[0]);

    if (fabs(avmPoint[0][0]) > 0.3 * VEHICLE_LEN)
    {
        return false;
    }

    memcpy(avmPoint, &rectVeh, sizeof(rectVeh));
    for (int i = 0; i < 4; i++)
    {
        float ry = Project_PosTo1st_ry(curSt[0].finpos, avmPoint[i]);
        if (fabs(ry) < VEHICLE_WID * 0.4)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 检查是否在减速带上并更新状态
 *
 * 该函数用于检测车辆是否经过减速带，并根据结果更新当前泊车模块的状态参数。
 *
 * @param curSt 当前泊车模块参数结构体，更新相关状态
 * @return void
 */
void PathExec_SpeedBump(PK_Cur_AutodrPara curSt[1])
{
    static int checkYawCount   = 0;   // 检查横摆计数
    const float bar_yaw_threld = 220; // 横摆阈值

    // 如果接近目标，重置计数并清除危险标志位
    if (PathExec_NearTarget(curSt))
    {
        checkYawCount            = 0;
        curSt[0].B_Danger_filter = curSt[0].B_Danger_filter & 0xFB; // 清除危险标志位
        return;
    }

    float yawr   = RTE_BSW_Get_IPU_MotorRTorq(); // 获取后轮扭矩
    float yaw    = RTE_BSW_Get_IPU_MotorTorq();  // 获取前轮扭矩
    float velYaw = rte_max(yawr, yaw);           // 取前后扭矩中的最大值

    float curSpd = RTE_PK_Location_Get_Vel_Spd();  // 获取当前速度
    float velSpd = RTE_BSW_Get_ESC_VehSpd() / 3.6; // 换算后的车速

    // 如果横摆速度超过阈值且当前速度或车速低于0.10
    if (velYaw > 1.2 * bar_yaw_threld && (fabs(curSpd) < 0.10 || fabs(velSpd) < 0.10))
    {
        checkYawCount++; // 增加检查计数
    }
    else
    {
        checkYawCount--; // 减少检查计数
    }

    checkYawCount = rte_max(checkYawCount, 0); // 确保检查计数不小于0

    // 如果检查计数超过50，设置危险标志位，否则清除危险标志位
    if (checkYawCount > 50)
    {
        curSt[0].B_Danger_filter = curSt[0].B_Danger_filter | 0x04; // 设置危险标志位
    }
    else
    {
        curSt[0].B_Danger_filter = curSt[0].B_Danger_filter & 0xFB; // 清除危险标志位
    }
}

/**
 * @brief 执行限位杆控制逻辑
 *
 * 该函数根据车辆的当前状态和各种参数（如扭矩、速度和档位）实现限位杆控制逻辑。
 *
 * @param curSt 当前泊车模块参数结构体
 * @return void
 */
void PathExec_ParkingLever(PK_Cur_AutodrPara curSt[1])
{
    const float parklevel_yaw_threld = 350; // 限位杆的横摆阈值
    const float motor_speed          = 0.1; // 电机的最低速度

    float torqr = RTE_BSW_Get_IPU_MotorRTorq(); //后扭矩
    float torqf = RTE_BSW_Get_IPU_MotorTorq();  //前扭矩

    float curSpd    = RTE_PK_Location_Get_Vel_Spd();  // 当前速度
    float velSpd    = RTE_BSW_Get_ESC_VehSpd() / 3.6; // 换算后的ESC车速
    uint8_t curGear = RTE_BSW_Get_CurrentGear();      // 当前档位

    float curpos[4]; // 当前位置信息
    RTE_PK_Location_Get_CurPos(curpos);

    static int watch_count   = 0;              // 观察计数
    static int watch_gear    = GEAR_REAL_NONE; // 观察档位
    static float watch_speed = 0;              // 观察速度
    static int lasth_StopBar = 0;              // 上一次限位杆状态

    // 判断是否需要清除限位杆状态

    uint32_t state = 0;
    auto nearStop  = fabs(curSt[0].rx_cur_to_fin) < 0.2 * VEHICLE_LEN &&
                    PathExec_IsParkingLever(curSt);
    if ((fabs(curSt[0].rx_cur_to_fin) > 0.3 * VEHICLE_LEN ||
         fabs(curSt[0].ry_cur_to_fin) > VEHICLE_WID) ||            // 位置判断
        (watch_gear != GEAR_REAL_NONE && watch_gear != curGear) || // 挡位判断
        (watch_gear != GEAR_REAL_NONE && curGear == GEAR_REAL_D &&
         nearStop)) // 位置加挡位
    {
        state         = state | 0x100;
        watch_gear    = GEAR_REAL_NONE;
        watch_count   = 0;
        lasth_StopBar = 0;
        PathExec_ClearParkingLever(curSt); // 清除限位杆状态
    }

    float velTorq    = rte_max(torqr, torqf); // 最大扭矩
    float torqthread = parklevel_yaw_threld * (fabs(curSt[0].cur_eps) / 3600.0 + 1.0);

    // 大扭矩低速度场景
    if (velSpd < 0.2 && velTorq > torqthread &&
        (curGear == GEAR_REAL_D || curGear == GEAR_REAL_R) && watch_count <= 0)
    {
        watch_gear  = curGear;
        watch_speed = rte_max(fabs(curSpd), 0.01);
        if (GEAR_REAL_R == watch_gear)
        {
            watch_speed = -1.0 * watch_speed;
        }
        watch_count = 1;
        state       = state | 0x01;
    }

    // 小扭矩高速场景
    if (velSpd > 0.1 && fabs(curSpd) > motor_speed &&
        (curGear == GEAR_REAL_D || curGear == GEAR_REAL_R) && watch_count <= 0)
    {
        state = state | 0x02;
        if ((curGear == GEAR_REAL_D && curSpd < 0.0 - motor_speed) ||
            (curGear == GEAR_REAL_R && curSpd > motor_speed))
        {
            state       = state | 0x04;
            watch_gear  = curGear;
            watch_count = 1;
            watch_speed = rte_max(fabs(curSpd), 0.01);
            if (GEAR_REAL_R == watch_gear)
            {
                watch_speed = -1.0 * watch_speed;
            }
        }
    }

    // 根据当前状态调整观察计数
    if (watch_count > 0)
    {
        state = state | 0x08;
        if (velTorq > torqthread * 0.8)
        {
            state       = state | 0x200;
            watch_count = watch_count + rte_max(velTorq * 3 / torqthread, 2);
        }

        if (sign(curSpd) * sign(watch_speed) < ZERO_FLOAT && fabs(curSpd) >= 0.1)
        {
            state       = state | 0x10;
            watch_count = watch_count + rte_max(fabs(curSpd) * 30, 3);
            if (fabs(curSpd) > fabs(watch_speed) + 0.05)
            {
                state = state | 0x20;
                watch_count =
                    watch_count + rte_max((fabs(curSpd) - fabs(watch_speed)) * 6, 3);
                watch_speed = fabs(curSpd) * sign(watch_speed);
            }
        }
        else if (sign(curSpd) * sign(watch_speed) > ZERO_FLOAT && fabs(curSpd) >= 0.1)
        {
            state       = state | 0x40;
            watch_count = watch_count - rte_max(fabs(curSpd) * 30, 1);
            if (velTorq < torqthread * 0.3)
            {
                state       = state | 0x80;
                watch_count = watch_count - rte_max((torqthread * 0.3 - velTorq) / 15, 3);
            }
        }
    }

    // 保证观察计数在合理范围内
    watch_count = rte_max(watch_count, 0);
    watch_count = rte_min(watch_count, 3 * bar_hold_time);

    // 判断是否需要设置限位杆状态
    int isStopBar = (10 * watch_count > 15 * bar_hold_time) ? 1 : 0;
    if (lasth_StopBar == 0 && isStopBar > 0)
    {
        PathExec_SetParkingLever(curSt); // 设置限位杆状态
        curSt[0].stillCount = 0;
        lasth_StopBar       = isStopBar;
    }

    // printf("------------------------%d %d 0x%x %f %f %f torq:%f,%f, rx_cur_to_fin:%f
    // state: %d -----------------------\n",
    //     watch_count, lasth_StopBar, state, curpos[3], watch_speed, curSpd, velTorq,
    //     torqthread, curSt[0].rx_cur_to_fin, PathExec_IsParkingLever(curSt));
}

int PathExec_ParkingLeverUpdate(PK_Cur_AutodrPara curSt[1], float enviObj[5][4])
{
    float dist = Cal_Dis_Pt2Pt(curSt[0].finpos, curSt[0].curpos);
    if (curSt[0].cvel < 0 && curSt[0].rx_cur_to_fin > 0.01 && dist < 1.0)
    {
        float offset[2] = {curSt[0].rx_cur_to_fin, 0};
        float newTarget[3];
        Convert(curSt[0].finpos, offset, newTarget);
        newTarget[2] = curSt[0].finpos[2];
        RTE_PK_SlotDetect_Set_TargPos(newTarget);

        SlotObj_T slotObj;
        float points[6][2];
        RTE_PK_SlotDetect_Get_SlotObj(&slotObj);
        memcpy(points, &slotObj, sizeof(points));
        for (int i = 0; i < 6; i++)
        {
            float temp[3];
            RevConvert(curSt[0].finpos, points[i], temp);
            Convert(newTarget, temp, points[i]);
        }
        memcpy(&slotObj, points, sizeof(points));
        RTE_PK_SlotDetect_Set_SlotObj(&slotObj);

        SlotObj_Convert_float(slotObj, enviObj);

        float avmPoints[8];
        PathExec_GetAvmPoint(avmPoints);
        PRINT_SWITCH_INFO("ParkingLeverUpdate");
        PathExec_PrintSlotInfo(enviObj, newTarget, avmPoints);

        PK_CopyPos(curSt[0].finpos, newTarget);
        curSt[0].hasStop = 0x01;
        return 1;
    }

    return 0;
}
