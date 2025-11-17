#include "PK_Calibration.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PathExec_Debug.h"
#include "PK_Utility.h"

/**
 * @brief 预测车辆位置
 *
 * 根据当前车辆位置、转向角和速度，预测一段时间 `dt` 之后车辆的位置和姿态。
 *
 * @param curpoint 当前车辆位置和姿态，包含四个元素：x 坐标、y 坐标、航向角 theta
 * 和路径长度 s
 * @param finpoint 预测得到的未来车辆位置和姿态，包含四个元素：x 坐标、y 坐标、航向角
 * theta 和路径长度 s
 * @param EPS_angle 当前转向角（电动助力转向角）
 * @param cur_vel 当前速度
 * @param dt 时间间隔
 */
void PK_Pre_locat(const float curpoint[4], float finpoint[4], const float EPS_angle,
                  const float cur_vel, const float dt)
{
    float theta0 = curpoint[2]; // 当前航向角（弧度）
    float s0     = curpoint[3]; // 当前路径长度（米）

    // 计算曲率半径 rou，基于转向角和速度
    float rou = PK_EPS_to_Rou(EPS_angle, cur_vel);

    // 计算在时间 dt 内车辆行驶的距离 ds
    float ds = dt * cur_vel;

    // 计算车辆行驶路径上的角度增量 g
    float g = ds * rou;

    // 计算预测位置的 x 坐标
    finpoint[0] = cosf(g / 2 + theta0) * ds + curpoint[0];

    // 计算预测位置的 y 坐标
    finpoint[1] = sinf(g / 2 + theta0) * ds + curpoint[1];

    // 计算预测的航向角 theta
    finpoint[2] = theta0 + g;

    // 计算预测的路径长度 s
    finpoint[3] = s0 + fabsf(ds);
}

/**
 * @brief 获取目标视觉车位的4个角点坐标
 *
 * @param points 角点坐标数组
 * @return
 */
void PathExec_GetAvmPoint(float points[8])
{
    Avm_Pot_T avmPot;
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmPot);
    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 1 || RTE_PK_DataConvt_Get_IsAVM_Slot() == 3)
    {
        PathExec_AvmSlotToPoint(avmPot, points);
    }
    else
    {
        memset(&avmPot, 0, sizeof(avmPot));
    }
}

/**
 * @brief 修正视觉车位角点
 *
 * 将视觉车位修整为一个规范的矩形
 *
 * @param avmPot 视觉车位结构体
 * @return
 */
static void NormalAvmPoint(Avm_Pot_T &avmPot)
{
    Vec2_T slotVec;
    slotVec.vx = {avmPot.near_front.x + avmPot.near_rear.x - avmPot.far_front.x -
                  avmPot.far_rear.x};
    slotVec.vy = {avmPot.near_front.y + avmPot.near_rear.y - avmPot.far_front.y -
                  avmPot.far_rear.y};
    slotVec.vx = 0.5 * slotVec.vx;
    slotVec.vy = 0.5 * slotVec.vy;
    float dist = Get_Norm_Vec2(slotVec);
    Normalize_Vec2(&slotVec);
    float rearDist = dist * REAR_SUSPENSION / VEHICLE_LEN;

    float basex = (avmPot.far_front.x + avmPot.far_rear.x) * 0.5;
    float basey = (avmPot.far_front.y + avmPot.far_rear.y) * 0.5;

    float target[3];
    target[0] = basex + rearDist * slotVec.vx;
    target[1] = basey + rearDist * slotVec.vy;
    target[2] = atan2(slotVec.vy, slotVec.vx);

    float rspoint[3][2];
    RevConvert(target, (float *)&avmPot.near_rear, rspoint[0]);
    RevConvert(target, (float *)&avmPot.far_rear, rspoint[1]);

    rspoint[2][0] = (VEHICLE_LEN - REAR_SUSPENSION);
    rspoint[2][1] = (rspoint[0][1] + rspoint[1][1]) * 0.5;
    Convert(target, rspoint[2], (float *)&avmPot.near_rear);

    rspoint[2][0] = -REAR_SUSPENSION;
    Convert(target, rspoint[2], (float *)&avmPot.far_rear);

    RevConvert(target, (float *)&avmPot.near_front, rspoint[0]);
    RevConvert(target, (float *)&avmPot.far_front, rspoint[1]);

    rspoint[2][0] = (VEHICLE_LEN - REAR_SUSPENSION);
    rspoint[2][1] = (rspoint[0][1] + rspoint[1][1]) * 0.5;
    Convert(target, rspoint[2], (float *)&avmPot.near_front);

    rspoint[2][0] = -REAR_SUSPENSION;
    Convert(target, rspoint[2], (float *)&avmPot.far_front);
}

/**
 * @brief 更新视觉车位点信息
 *
 * 该函数更新泊车模块中当前使用的视觉车位信息。如果满足特定条件，它会从传感器获取新的车位信息，
 * 并根据需要融合当前和旧的车位数据来生成新的目标车位点信息。此函数还处理车位的第一次规划和
 * 当车辆接近目标点时的特定行为。
 *
 * @param curSt 当前泊车模块参数结构体，包含车位形状、规划标志等信息
 * @param avmSlot 目标视觉车位，将被更新为新的车位信息
 */
void PathExec_UpdateAvmPoint(PK_Cur_AutodrPara curSt[1], Avm_Pot_T &avmSlot)
{
    // 如果车位不是垂直车位，返回
    if (curSt[0].slotshape != PK_SLOT_RIGHT_VERT &&
        curSt[0].slotshape != PK_SLOT_LEFT_VERT)
    {
        return;
    }

    // 如果当前车位信息来源是超声波或指定车位，返回
    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 0 || RTE_PK_DataConvt_Get_IsAVM_Slot() == 2)
    {
        return;
    }

    // 获取目标视觉车位信息
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);

    // 处理车位的非第一次规划情况
    if (curSt[0].B_FirstPlanFlag == 0) // 不是第一次规划
    {
        Avm_Pot_T newavmSlot, oldAvmSlot, targetAvmSlot;

        // 获取新车位信息
        RTE_PD_Get_AVM_SlotInfo_B(&newavmSlot);
        if (newavmSlot.slot_index == 0)
        {
            return;
        }

        // 获取旧车位信息
        RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&oldAvmSlot);
        if (newavmSlot.slot_index == oldAvmSlot.slot_index)
        {
            return;
        }

        // 如果当前车位到目标点的距离大于一定值
        if (curSt[0].rx_cur_to_fin > 1.5 * REAR_SUSPENSION)
        {
            // 标准化新旧车位点
            NormalAvmPoint(newavmSlot);
            NormalAvmPoint(oldAvmSlot);

            // 复制旧车位点到目标车位点
            memcpy((float *)&targetAvmSlot, (float *)&oldAvmSlot, sizeof(oldAvmSlot));

            // 融合新旧车位点信息，采用加权平均
            float factor = avm_slotinfo_b_factor;
            targetAvmSlot.near_front.x =
                factor * newavmSlot.near_front.x + (1 - factor) * oldAvmSlot.near_front.x;
            targetAvmSlot.near_front.y =
                factor * newavmSlot.near_front.y + (1 - factor) * oldAvmSlot.near_front.y;
            targetAvmSlot.near_rear.x =
                factor * newavmSlot.near_rear.x + (1 - factor) * oldAvmSlot.near_rear.x;
            targetAvmSlot.near_rear.y =
                factor * newavmSlot.near_rear.y + (1 - factor) * oldAvmSlot.near_rear.y;
            targetAvmSlot.far_front.x =
                factor * newavmSlot.far_front.x + (1 - factor) * oldAvmSlot.far_front.x;
            targetAvmSlot.far_front.y =
                factor * newavmSlot.far_front.y + (1 - factor) * oldAvmSlot.far_front.y;
            targetAvmSlot.far_rear.x =
                factor * newavmSlot.far_rear.x + (1 - factor) * oldAvmSlot.far_rear.x;
            targetAvmSlot.far_rear.y =
                factor * newavmSlot.far_rear.y + (1 - factor) * oldAvmSlot.far_rear.y;

            // 标准化目标车位点
            NormalAvmPoint(targetAvmSlot);

            // 打印调试信息
            printf("oldAvmSlot: %f %f %f %f %f %f %f %f\n", oldAvmSlot.near_rear.x,
                   oldAvmSlot.near_rear.y, oldAvmSlot.far_rear.x, oldAvmSlot.far_rear.y,
                   oldAvmSlot.far_front.x, oldAvmSlot.far_front.y,
                   oldAvmSlot.near_front.x, oldAvmSlot.near_front.y);
            printf("newavmSlot: %f %f %f %f %f %f %f %f\n", newavmSlot.near_rear.x,
                   newavmSlot.near_rear.y, newavmSlot.far_rear.x, newavmSlot.far_rear.y,
                   newavmSlot.far_front.x, newavmSlot.far_front.y,
                   newavmSlot.near_front.x, newavmSlot.near_front.y);
            printf("targetAvmSlot: %f %f %f %f %f %f %f %f\n", targetAvmSlot.near_rear.x,
                   targetAvmSlot.near_rear.y, targetAvmSlot.far_rear.x,
                   targetAvmSlot.far_rear.y, targetAvmSlot.far_front.x,
                   targetAvmSlot.far_front.y, targetAvmSlot.near_front.x,
                   targetAvmSlot.near_front.y);

            // 设置目标车位点索引，并更新目标车位信息
            targetAvmSlot.slot_index = newavmSlot.slot_index;
            RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&targetAvmSlot);
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);
        }
    }

    // 如果车辆接近目标点并且不是第一次规划
    if (curSt[0].rx_cur_to_fin < 2.5 * REAR_SUSPENSION && curSt[0].B_FirstPlanFlag == 0)
    {
        curSt[0].B_FirstPlanFlag = 1; // 设置为第一次规划
        float obj[4]             = {0.0, 0.0, 0.0, 0.0};
        PathExec_AddObsObjNearSlot(obj, curSt[0].finpos); // 添加靠近车位的障碍物对象
    }
}

bool PathExec_IsVehStill()
{
    // 获取当前车辆运动状态
    auto velMoveSt = RTE_PK_Location_Get_Vel_MoveMentSt();

    // 获取当前电机速度
    auto motVel = RTE_PK_Location_Get_Vel_Spd();

    // 获取当前ESC（电子稳定控制系统）速度
    auto velSpd = RTE_BSW_Get_ESC_VehSpd();

    // 获取ESC允许的最小速度
    auto velMinSpd = RTE_BSW_Get_ESC_MinVehSpd();

    // 获取当前EPS（电动助力转向）角速度
    auto epsSpd = RTE_BSW_Get_EPS_AngleSpd();

    // 判断车辆是否静止的条件：
    // 1. 运动状态为静止
    // 2. 电机速度接近于0（小于0.03）
    // 3. 当前速度小于最小允许速度
    // 4. EPS角速度小于50

    static int motorSpdCount = 0;
    float motorSpd           = RTE_BSW_Get_Motorspd();
    motorSpdCount++;
    if (fabs(motorSpd) > 0.01f)
    {
        motorSpdCount = 0;
    }
    if (motorSpdCount > 65535)
    {
        motorSpdCount = 65535;
    }

    bool condition1 = fabsf(motVel) < 0.03f && velSpd < velMinSpd;
    bool condition2 =
        fabsf(motVel) < 0.03f && velSpd < 3 * velMinSpd && motorSpdCount > 10;

    // return (velMoveSt == LOCAT_STILL && fabsf(motVel) < 0.03f && velSpd < velMinSpd &&
    // epsSpd < 50);
    return (velMoveSt == LOCAT_STILL && (condition1 || condition2) && epsSpd < 50);
}

bool PathExec_IsEndLastTraj(const PK_Cur_AutodrPara *curSt)
{
    const float min_ds_gone_to_chagPath = 0.05f;
    const float vert_near_brake_dist    = 0.3f;
    const float para_near_brake_yoffset = 0.03f;
    const float ThetaGap_Allow          = 0.03f; // rad   [-3,3]deg

    if (curSt[0].slotshape == PK_SLOT_RIGHT_VERT ||
        curSt[0].slotshape == PK_SLOT_LEFT_VERT)
    {
        return (curSt[0].cur_tra_nth == curSt[0].tra_num - 1 &&
                curSt[0].ds_gone_dir > 6 * min_ds_gone_to_chagPath &&
                fabsf(curSt[0].rx_brake) < vert_near_brake_dist &&
                fabsf(curSt[0].rx_cur_to_fin) < vert_near_brake_dist &&
                fabsf(curSt[0].rtheta_cur_to_fin) < 1.5 * ThetaGap_Allow);
    }
    else
    {
        return (curSt[0].cur_tra_nth == curSt[0].tra_num - 1 &&
                curSt[0].ds_gone_dir > 6 * min_ds_gone_to_chagPath &&
                fabsf(curSt[0].rx_cur_to_fin) < REAR_SUSPENSION &&
                fabsf(curSt[0].ry_cur_to_fin) < para_near_brake_yoffset &&
                fabsf(curSt[0].rtheta_cur_to_fin) < ThetaGap_Allow);
    }
}

void PathExec_AvmSlotToPoint(Avm_Pot_T &avmPot, float points[8])
{
    points[0] = avmPot.near_rear.x;
    points[1] = avmPot.near_rear.y;
    points[2] = avmPot.far_rear.x;
    points[3] = avmPot.far_rear.y;
    points[4] = avmPot.far_front.x;
    points[5] = avmPot.far_front.y;
    points[6] = avmPot.near_front.x;
    points[7] = avmPot.near_front.y;
}

int PathExec_RevertCount(const float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                         const int lineNum)
{
    if (lineNum <= 2)
    {
        return 0;
    }

    int revertCount = 0;
    for (int i = 1; i < lineNum; i++)
    {
        if (trajs[i][3] * trajs[i - 1][3] < 0)
        {
            revertCount = revertCount + 1;
        }
    }
    return revertCount;
}

void PathExec_StartNewPlanPath(PK_Cur_AutodrPara *curSt, int trajNum, float leftDist)
{
    curSt[0].cur_tra_nth    = 0;
    curSt[0].step_gone      = 0;
    curSt[0].tra_num        = trajNum;
    curSt[0].leftDist       = leftDist;
    curSt[0].stillCount     = 0;
    curSt[0].B_BrakeState   = 0;
    curSt[0].B_ReplanResult = PATHEXECUTE_SUCCESS;
    curSt[0].sum_de         = 0;
    curSt[0].de             = 0;
    curSt[0].ds_start_dir   = curSt[0].curpos[3];
    curSt[0].cvel_last      = 0;
}

void PathExec_ContinueInvTraj(PK_Cur_AutodrPara *curSt)
{
    curSt[0].cur_tra_nth++;
    curSt[0].cvel           = 0;
    curSt[0].sum_de         = 0.0f; // change direction, clear compensate
    curSt[0].B_BrakeState   = 0;
    curSt[0].step_gone      = 0;
    curSt[0].ds_start_dir   = curSt[0].curpos[3];
    curSt[0].B_ReplanResult = 0;
}

void PathExec_SkipToLastTraj(PK_Cur_AutodrPara *curSt)
{
    PathExec_ContinueInvTraj(curSt);
    curSt[0].rx_brake    = 0.0;
    curSt[0].cvel_last   = 0.0;
    curSt[0].cur_tra_nth = curSt[0].tra_num;
}

void PathExec_UpdateX(const float basePoint[3], const float src[2], float dst[2])
{
    float rsSrc[2], rsDst[2];
    RevConvert(basePoint, src, rsSrc);
    RevConvert(basePoint, dst, rsDst);
    rsDst[0] = rsSrc[0];
    Convert(basePoint, rsDst, dst);
}

void PathExec_UpdateY(const float basePoint[3], const float src[2], float dst[2])
{
    float rsSrc[2], rsDst[2];
    RevConvert(basePoint, src, rsSrc);
    RevConvert(basePoint, dst, rsDst);
    rsDst[1] = rsSrc[1];
    Convert(basePoint, rsDst, dst);
}
