#ifndef _PK_PATHEXECUTEFUNC_H_
#define _PK_PATHEXECUTEFUNC_H_

#include "PK_PathExecuteP.h"
#include "PK_Calibration.h"
#include "PathExec_Config.h"

/**
 * @brief 判断车辆是否静止
 *
 * 该函数通过检查多个传感器和系统状态来确定车辆是否处于静止状态。
 *
 * @return bool 如果车辆静止则返回true，否则返回false
 */
bool PathExec_IsVehStill();

inline void PathExec_SetParkUpdate(PK_Cur_AutodrPara *curSt) //停止更新标志
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x20);
}

inline bool PathExec_IsParkUpdate(PK_Cur_AutodrPara *curSt) //停止更新标志
{
    return (curSt[0].bUpdate & 0x20) == 0x20;
}

inline void PathExec_ClearParkUpdate(PK_Cur_AutodrPara *curSt) //停止更新标志
{
    curSt[0].bUpdate = (curSt[0].bUpdate & 0xdf);
}

inline void PathExec_SetRearUpdateTarg(PK_Cur_AutodrPara *curSt)
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x08);
}

inline void PathExec_SetStopUpdateTarg(PK_Cur_AutodrPara *curSt) //停止更新标志
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x10);
}

inline bool PathExec_IsRearUpdateTarg(PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].bUpdate & 0x08) == 0;
}

inline bool PathExec_IsStopUpdateTarg(PK_Cur_AutodrPara *curSt) //停止更新标志
{
    return (curSt[0].bUpdate & 0x10) == 0;
}

inline bool PathExec_IsSideUpdateTarg(const PK_Cur_AutodrPara *curSt)
{
    return ((curSt[0].bUpdate & 0x06) != 0);
}

inline void PathExec_ClrSideUpdateTarg(PK_Cur_AutodrPara *curSt)
{
    curSt[0].bUpdate = (curSt[0].bUpdate & 0xF9);
}

inline void PathExec_SetSideUpdateAvmTarg(PK_Cur_AutodrPara *curSt)
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x2);
}

inline void PathExec_SetSideUpdateSpaceTarg(PK_Cur_AutodrPara *curSt)
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x4);
}

inline bool PathExec_IsLastAdjust(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].bUpdate & 0x1) == 0x1;
}

inline void PathExec_SetLastAdjust(PK_Cur_AutodrPara *curSt) //最后调整标志
{
    curSt[0].bUpdate = (curSt[0].bUpdate | 0x1);
}

inline bool PathExec_ReplanFlag(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].B_DynaPlanCounter != 0);
}

inline bool PathExec_ReplanSlotShape(const PK_Cur_AutodrPara *curSt)
{
    bool disable =
        (curSt[0].B_DynaPlanCounter >= 10 || curSt[0].slotshape == PK_SLOT_LEFT_PARA ||
         curSt[0].slotshape == PK_SLOT_RIGHT_PARA || curSt[0].slotshape == PK_SLOT_NO ||
         curSt[0].B_ReplanResult != PATHEXECUTE_SUCCESS);
    return !disable;
}

inline bool PathExec_IsForward(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].cvel > ZERO_FLOAT);
}

inline bool PathExec_IsBackward(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].cvel < ZERO_FLOAT);
}

inline bool PathExec_IsFrontDanger(const PK_Cur_AutodrPara *curSt) //前方障碍物
{
    return ((curSt[0].B_Danger_filter & 0x01) != 0);
}

inline bool PathExec_IsRearDanger(const PK_Cur_AutodrPara *curSt) //后方障碍物
{
    return ((curSt[0].B_Danger_filter & 0x02) != 0);
}

inline bool PathExec_IsObjDanger(const PK_Cur_AutodrPara *curSt) //障碍物
{
    return ((curSt[0].B_Danger_filter & 0x03) != 0);
}

inline void PathExec_SetObjDanger(PK_Cur_AutodrPara *curSt, const uint8_t type)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter | (type & 0X03);
}

inline void PathExec_ClearObjDanger(PK_Cur_AutodrPara *curSt)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter & 0xFC;
}

inline bool PathExec_IsSpeedBump(
    const PK_Cur_AutodrPara *curSt) //限位杆，第一次撞限位杆后停下后的标志
{
    return ((curSt[0].B_Danger_filter & 0x04) != 0);
}

inline bool PathExec_IsParkingLever(const PK_Cur_AutodrPara *curSt) //限位杆
{
    return ((curSt[0].B_Danger_filter & 0x08) != 0);
}

inline bool PathExec_NearToTarget(const PK_Cur_AutodrPara *curSt)
{
    return (fabsf(curSt[0].ry_cur_to_fin) < 0.06 && fabsf(curSt[0].rthetaf) < 0.03);
}

inline void PathExec_SetParkingLever(PK_Cur_AutodrPara *curSt)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter | 0x08;
}

inline void PathExec_ClearParkingLever(PK_Cur_AutodrPara *curSt)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter & 0xF7;
}

inline bool PathExec_IsSuspendByObjs(const PK_Cur_AutodrPara *curSt) //被障碍物迫停
{
    return ((curSt[0].B_Danger_filter & 0x10) != 0);
}

inline void PathExec_SetSuspendByObjs(PK_Cur_AutodrPara *curSt)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter | 0x10;
}

inline void PathExec_ClearSuspendByObjs(PK_Cur_AutodrPara *curSt)
{
    curSt[0].B_Danger_filter = curSt[0].B_Danger_filter & 0xEF;
}

enum
{
    replan_idle,
    replan_success,
    replan_start
};
// dereplan  0 - 1 bit
inline bool PathExec_IsDeReplan(const PK_Cur_AutodrPara *curSt) //误差重规划
{
    uint16_t replan = curSt[0].B_replan & 0x3;
    return (replan == replan_start);
}

inline void PathExec_SetDeReplan(PK_Cur_AutodrPara *curSt, uint8_t flag)
{
    uint16_t replan   = flag;
    curSt[0].B_replan = (curSt[0].B_replan & 0xffc) | replan;
}

// updatereplan  2 - 3 bit
inline bool PathExec_IsUpdateReplan(const PK_Cur_AutodrPara *curSt) //路径更新重规划
{
    uint16_t replan = (curSt[0].B_replan >> 2) & 0x3;
    return (replan == replan_start);
}

inline void PathExec_SetUpdateReplan(PK_Cur_AutodrPara *curSt, uint8_t flag)
{
    uint16_t replan   = flag;
    replan            = (replan << 2) | (curSt[0].B_replan & 0x3); // 0 - 3 bit
    curSt[0].B_replan = (curSt[0].B_replan & 0xfff0) | replan;
}

// dereplan  4 - 5 bit
inline bool PathExec_IsCrossReplan(const PK_Cur_AutodrPara *curSt) //路径更新重规划
{
    uint16_t replan = (curSt[0].B_replan >> 4) & 0x3;
    return (replan == replan_start);
}

inline void PathExec_SetCrossReplan(PK_Cur_AutodrPara *curSt, uint8_t flag)
{
    uint16_t replan   = flag;
    replan            = (replan << 4) | (curSt[0].B_replan & 0xf); // 0 - 6 bit
    curSt[0].B_replan = (curSt[0].B_replan & 0xffc0) | replan;
}

// dereplan  6 - 7 bit
inline bool PathExec_IsDangerReplan(const PK_Cur_AutodrPara *curSt) //路径更新重规划
{
    uint16_t replan = (curSt[0].B_replan >> 6) & 0x03;
    return (replan == replan_start);
}

inline void PathExec_SetDangerReplan(PK_Cur_AutodrPara *curSt, uint8_t flag)
{
    uint16_t replan   = flag;
    replan            = (replan << 6) | (curSt[0].B_replan & 0x3f); // 0 - 8 bit
    curSt[0].B_replan = (curSt[0].B_replan & 0xff00) | replan;
}

// lastreplan  8 - 9 bit
inline bool PathExec_IsLastReplan(const PK_Cur_AutodrPara *curSt) //路径更新重规划
{
    uint16_t replan = (curSt[0].B_replan >> 8) & 0x03;
    return (replan == replan_start);
}

inline void PathExec_SetLastReplan(PK_Cur_AutodrPara *curSt, uint8_t flag)
{
    uint16_t replan   = flag;
    replan            = (replan << 8) | (curSt[0].B_replan & 0xff); // 0 - 10 bit
    curSt[0].B_replan = (curSt[0].B_replan & 0xfc00) | replan;
}

inline bool PathExec_IsUpdateStopBar()
{
    return ((avm_slot_update_on_parking & 0x01) != 0);
}

inline bool PathExec_IsUpdateSideVision()
{
    return ((avm_slot_update_on_parking & 0x02) != 0);
}

inline bool PathExec_IsUpdateRearVision()
{
    return ((avm_slot_update_on_parking & 0x04) != 0);
}

inline void PathExec_ContinueNextTraj(PK_Cur_AutodrPara *curSt)
{
    curSt[0].cur_tra_nth++;
}

inline void PathExec_UpdateLastCVel(PK_Cur_AutodrPara *curSt,
                                    const float Planned_Path[TRAJITEM_LEN])
{
    curSt[0].cvel_last = 0.5 * sign(Planned_Path[3]);
}

inline bool PathExec_ReplanSuccess(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].B_ReplanResult == PATHEXECUTE_SUCCESS);
}

inline bool PathExec_IsFrontPath(const float Planned_Path[TRAJITEM_LEN])
{
    return (Planned_Path[3] > 0);
}

inline bool PathExec_SwitchDelayTimeOut(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].stillCount > 15);
}

inline bool PathExec_IsPreTrajNoMove(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].ds_gone_dir < 0.05 && fabs(curSt[0].leftDist) > 0.1);
}

inline bool PathExec_ObjDelayTimeOut(const PK_Cur_AutodrPara *curSt)
{
    return (curSt[0].stillCount > 4);
}

inline bool PathExec_BYIdentify(const PK_Cur_AutodrPara *curSt)
{
    return ((curSt[0].targState & 0x40) != 0);
}

inline bool PathExec_EYIdentify(const PK_Cur_AutodrPara *curSt)
{
    return ((curSt[0].targState & 0x04) != 0);
}

inline void PathExec_ClrPostBrakeState(PK_Cur_AutodrPara *curSt) //前方障碍物
{
    curSt[0].postState = (curSt[0].postState & 0xFE);
}

inline void PathExec_SetPostBrakeState(PK_Cur_AutodrPara *curSt) //前方障碍物
{
    curSt[0].postState = (curSt[0].postState | 0x01);
}

inline bool PathExec_IsPostBrakeState(const PK_Cur_AutodrPara *curSt) //前方障碍物
{
    return ((curSt[0].postState & 0x01) != 0);
}

void PathExec_UpdateX(const float basePoint[3], const float src[2], float dst[2]);

void PathExec_UpdateY(const float basePoint[3], const float src[2], float dst[2]);

/**
 * @brief 初始化进度条
 * @param curSt 当前泊车模块参数结构体
 * @return
 */
inline void PathExec_ClearPercent(PK_Cur_AutodrPara *curSt)
{
    curSt[0].percent     = -1.0;
    curSt[0].lastpercent = 0.0;
}

int PathExec_RevertCount(const float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                         const int lineNum);

void PathExec_AvmSlotToPoint(Avm_Pot_T &avmPot, float points[8]);

bool PathExec_IsEndLastTraj(const PK_Cur_AutodrPara *curSt);

void PathExec_ContinueInvTraj(PK_Cur_AutodrPara *curSt);

void PathExec_SkipToLastTraj(PK_Cur_AutodrPara *curSt);

/**
 * @brief 根据新路径更新并初始化B过程数据
 *
 * @param curSt 当前泊车模块参数结构体
 * @param trajNum 新路径段数
 * @param leftDist 新路径中第一段路径长度
 * @return
 */
void PathExec_StartNewPlanPath(PK_Cur_AutodrPara *curSt, int trajNum, float leftDist);

#endif
