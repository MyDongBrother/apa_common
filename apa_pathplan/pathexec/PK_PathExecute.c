#include "PK_Calibration.h"
#include "PK_ObjAvoid.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PK_StateManage.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PathExec_Debug.h"
#include "PathExec_Replan.h"
#include "PathExec_Config.h"
#include "PK_PathPlan.h"
#include "PK_Utility.h"
#include "PK_B.h"
#include "Rte_ComIF.h"

/* Check if source file and COMMON header file are of the same software version */
int pre_nout_ccp = 0;

const int path_execute_period  = 20;
const int path_dangerquit_time = 60; // 3S
const int path_apaquit_time    = path_dangerquit_time * 4;
static PK_Cur_AutodrPara curent_state[1];
static uint8_t cfg_updatepath_stat = 1;
static float cfg_sideupdate_dist   = 0.0f;

static int PathExec_MainControl(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                float enviObj[5][4], PK_Cur_AutodrPara curSt[1]);
void PathExec_MovePath_dyna(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            float newTarget[3], PK_Cur_AutodrPara CurSt[1]);
bool PathExec_UpdateTarget(PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                           float newTarget[3], bool update, int flag = 0,
                           const float swell = 0.02);

inline bool IsBrakeVeh()
{
    return ((RTE_BSW_Get_FailureBrakeModeVal() == 0 && RTE_BSW_Get_BrakeSt() &&
             RTE_BSW_Get_ESC_VehSpd() <= RTE_BSW_Get_ESC_MinVehSpd()) ||
            (RTE_BSW_Get_CurrentGear() == GEAR_REAL_P));
}

void PK_PathExecute_RunRate(float paras[8])
{
    static float filter[4] = {0};

    float percent = curent_state[0].percent;
    if (curent_state[0].percent >= 0)
    {
        percent = curent_state[0].percent;
        if (curent_state[0].percent > 0.99)
        {
            percent = 1.0;
        }
    }
    else
    {
        percent = 0.0;
    }

    // 确认先到100%，然后才下降
    if (curent_state[0].lastpercent > 0.1 && curent_state[0].lastpercent < 1.0 &&
        curent_state[0].percent < 0.005)
    {
        percent = 1.0;
    }
    else
    {
        curent_state[0].lastpercent = percent;
    }
    paras[0]              = 100 * percent;
    float steerAngle      = RTE_BSW_Get_EPS_Angle();
    RTE_BSW_GearType gear = RTE_BSW_Get_CurrentGear();

    float radius = 0;
    if (fabs(steerAngle) > 10)
    {
        if (gear == GEAR_REAL_R)
        {
            radius = PK_EPS_to_Rou(steerAngle, -1.0);
        }
        else if (gear == GEAR_REAL_D)
        {
            radius = PK_EPS_to_Rou(steerAngle, 1.0);
        }
        radius = 1 / radius;
    }

    for (int i = 0; i < 3; i++)
    {
        filter[i] = filter[i + 1];
    }
    filter[3] = radius;

    radius = 0;
    for (int i = 0; i < 4; i++)
    {
        radius = radius + filter[i];
    }

    curent_state[0].radius = radius / 4.0;
    paras[1]               = curent_state[0].radius;
    paras[2]               = sign(curent_state[0].cvel);
}

static void PathExec_StartNextInvTraj(
    float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], PK_Cur_AutodrPara curSt[1])
{
    float dir = sign(Planned_Path[curSt[0].cur_tra_nth][3]);
    while (curSt[0].cur_tra_nth < curSt[0].tra_num)
    {
        if ((PathExec_IsFrontDanger(curSt) &&
             PathExec_IsFrontPath(Planned_Path[curSt[0].cur_tra_nth])) ||
            (PathExec_IsRearDanger(curSt) &&
             !PathExec_IsFrontPath(Planned_Path[curSt[0].cur_tra_nth])) ||
            (PathExec_IsParkingLever(curSt) &&
             dir * Planned_Path[curSt[0].cur_tra_nth][3] > ZERO_FLOAT))
        {
            PathExec_UpdateLastCVel(curSt, Planned_Path[curSt[0].cur_tra_nth]);
            PathExec_ContinueInvTraj(curSt);
        }
        else
        {
            PRINT_SWITCH_INFO("PathExec_StartNextInvTraj!");
            break;
        }
    }
}

static void MoveLeftTrajsByStart(const float trajs[][TRAJITEM_LEN], const int trajNum,
                                 const float start[3], float end[3])
{
    float temp[TRAJITEM_LEN];
    PK_CopyPos(temp, start);
    for (int i = 0; i < trajNum; i++)
    {
        temp[3] = trajs[i][3];
        temp[4] = trajs[i][4];
        PK_Get_Path_EndPos(temp, end);
        PK_CopyPos(temp, end);
    }
}

static int CheckCollisionWithObj(const float trajs[][TRAJITEM_LEN], int trajNum,
                                 const float obj[][4])
{
    for (int i = 0; i < trajNum; i++)
    {
        if (PK_PathPlan_PathVerify(trajs[i], 1, obj, 0.3) > 0)
        {
            return i;
        }
    }

    return -1;
}

/**
 * @brief 更新路径执行完成率
 *
 * 根据当前状态和路径信息，计算并更新路径执行的完成率百分比
 *
 * @param planned_Path 规划的路径，包含多个路径点
 * @param curSt 当前自动驾驶参数，包括当前位置、当前路径段数、完成率等
 */
void PathExec_CompleteRate(float planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                           PK_Cur_AutodrPara curSt[1])
{
    static float basePercent = 0.0; // 基础完成率
    static int startPathIdx  = -1;  // 起始路径段索引

    int pathIdx = 0;
    for (pathIdx = curSt[0].cur_tra_nth; pathIdx < curSt[0].tra_num; pathIdx++)
    {
        if (pathIdx == curSt[0].tra_num - 1) // 如果是最后一段路径
        {
            break;
        }
        else if (planned_Path[pathIdx][3] * planned_Path[pathIdx + 1][3] <
                 0) // 如果前后路径的速度方向不同
        {
            break;
        }
    }

    float endpos[3];                                   // 终点位置
    PK_Get_Path_EndPos(planned_Path[pathIdx], endpos); // 获取当前路径段的终点位置
    float dist =
        Cal_Dis_Pt2Pt(endpos, curSt[0].curpos); // 计算终点位置和当前车辆位置之间的距离

    RTE_BSW_GearType gear = RTE_BSW_Get_CurrentGear(); // 获取当前档位
    if ((curSt[0].percent < 0) ||
        ((gear == GEAR_REAL_R || gear == GEAR_REAL_D) &&
         (curSt[0].gear == GEAR_REAL_R || curSt[0].gear == GEAR_REAL_D) &&
         gear != curSt[0].gear))
    {
        curSt[0].pathDist = dist;                 // 更新路径距离
        startPathIdx      = curSt[0].cur_tra_nth; // 更新起始路径段索引
        basePercent       = 0;                    // 重置基础完成率
        curSt[0].percent  = 0.0;                  // 重置完成率
    }
    else if ((gear == GEAR_REAL_R || gear == GEAR_REAL_D) && gear == curSt[0].gear)
    {
        // 正常更新完成率
        if (curSt[0].pathDist > 0 && curSt[0].cur_tra_nth >= startPathIdx)
        {
            startPathIdx = curSt[0].cur_tra_nth; // 更新起始路径段索引
            float percent =
                basePercent +
                (1 - basePercent) * (1 - dist / curSt[0].pathDist); // 计算新的完成率
            if (curSt[0].percent < percent)
            {
                curSt[0].percent = percent;
            } // 更新完成率
        }
        else if (curSt[0].pathDist > 0 &&
                 curSt[0].cur_tra_nth < startPathIdx) // 重新规划路径的情况
        {
            startPathIdx      = curSt[0].cur_tra_nth; // 更新起始路径段索引
            curSt[0].pathDist = dist;                 // 更新路径距离
            basePercent       = curSt[0].percent;     // 更新基础完成率
        }
    }

    if (curSt[0].percent >= 0)
    {
        curSt[0].percent = rte_min(curSt[0].percent, 1.0); // 确保完成率不超过100%
    }

    if (gear == GEAR_REAL_R || gear == GEAR_REAL_D)
    {
        curSt[0].gear = gear; // 更新当前档位
    }
}

/**
 * @brief 检查车辆当前位置是否在路径起点附近
 * @param path 路径
 * @param curpos 车辆当前位置
 * @return int 1-在附近 0-不在附近
 */
static int Check_If_CurPosNearTrajStPos(const float path[TRAJITEM_LEN],
                                        const float curpos[3])
{
    const float d_ry_max     = 0.6f;
    const float d_rtheta_max = 0.26f; //  15 deg
    float ry                 = Project_PosTo1st_ry(curpos, path);
    float rtheta             = Round_PI(curpos[2] - path[2]);
    if (fabsf(ry) > d_ry_max || fabsf(rtheta) > d_rtheta_max)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 计算当前位置与目标点的距离的平方
 * @param curpos 当前位置
 * @param targPos 目标点
 * @return float 距离的平方
 */
static float Cal_PowDist_CurPos_TargPos(const float curpos[4], const float targPos[3])
{
    return (pow2(curpos[0] - targPos[0]) + pow2(curpos[1] - targPos[1]));
}

static void PathExec_SaveToCCP(PK_Cur_AutodrPara &curState)
{
    // set ccp values
    if (curState.step_gone > 0 && fabs(curState.cur_vel) > 0.2)
    {
        curState.cvel_last = curState.cvel;
    }
}

void PK_PathExecuteRun(void)
{
    const float tuning_rx_brake = 0.51f; //  for parallel parking in slot set a little bit
                                         //  long dist to tune the vehicle's angle

    static int B_state = 0; // running state
    static float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    static PK_ModuleStateType ModuleState_PathExecute =
        MSTAT_OFF; //  20181105: MSTAT_NORM -> MSTAT_OFF
    static PK_ModuleComType lastModuleCom = MCOM_INIT;

    float enviObj[5][4];
    PlanInfoType targetPlanInfo; // 路径
    memset(&targetPlanInfo, 0, sizeof(targetPlanInfo));

    SlotObj_T slotObj;
    int IsCurPosNearTrajStPos = 1;

    PK_ModuleComType ModuleCom_PathExecute =
        RTE_PK_StateManage_Get_ModuleCom_PathExecute();
    if (ModuleCom_PathExecute == MCOM_ON_AVMPARKING) // Do the initialize  AVP
    {
        B_state = 1;
        RTE_PK_Location_Get_CurPos(curent_state[0].curpos);
        if (ModuleState_PathExecute == MSTAT_OFF) // 如果模块处于关闭状态
        {
            printf("------------ parking start --------------------\r\n");
            PRINT_SWITCH_INFO("Avp Start");
            PathExec_PrintPathStart();
            ModuleState_PathExecute = MSTAT_NORM; // 跳转为正常工作状态
            memset(Planned_Path, 0, sizeof(Planned_Path));
            curent_state[0].tra_num = 0;
        }

        PathExec_ClearPercent(curent_state); // 初始化百分比
        PathExec_UpdateAvpAutodrPara(curent_state, Planned_Path);
    }
    else if (ModuleCom_PathExecute == MCOM_ON_PARKING &&
             lastModuleCom == MCOM_ON_AVMPARKING) // AVP -> APA
    {
        ModuleState_PathExecute = MSTAT_OFF;
        PRINT_SWITCH_INFO("PK_PathExecuteRun avp to apa!");
        PathExec_ClearPercent(curent_state);
        if (!PathExec_IsVehStill())
        {
            PathExec_EndAvpAutodrPara(curent_state, Planned_Path);
        }
        PathExec_PrintPathEnd();
    }

    if (ModuleCom_PathExecute == MCOM_ON_PARKING &&
        ModuleState_PathExecute == MSTAT_OFF) // Do the initialize
    {
        printf("------------ parking start --------------------\r\n");
        memset(curent_state, 0, sizeof(curent_state));

        // 1.Get Original Path
        targetPlanInfo.Path_num = 0;
        RTE_PK_DataConvt_Get_Target_PlanInfo(&targetPlanInfo);
        memcpy(Planned_Path, targetPlanInfo.Act_traj, sizeof(Planned_Path));
        curent_state[0].slotshape = RTE_PK_SlotDetect_Get_SlotShape();

        RTE_PK_SlotDetect_Get_TargPos(curent_state[0].finpos);

        // 2.Get start_ds,step_gone
        RTE_PK_Location_Get_CurPos(curent_state[0].curpos);
        PathExec_StartNewPlanPath(curent_state, targetPlanInfo.Path_num,
                                  Planned_Path[0][3]); // 新路径

        // 3.Set state
        IsCurPosNearTrajStPos =
            Check_If_CurPosNearTrajStPos(Planned_Path[0], curent_state[0].curpos);
        if (curent_state[0].tra_num <= MAX_SINGLE_TRAJ_NUM &&
            curent_state[0].tra_num > 0 && IsCurPosNearTrajStPos == 1 &&
            Cal_PowDist_CurPos_TargPos(curent_state[0].curpos, curent_state[0].finpos) <
                625.0f) // 25m && Check_If_ShortInvalidPath(curent_state[0].tra_num,
                        // Planned_Path) == 0
        {
            ModuleState_PathExecute = MSTAT_NORM; // 正常工作状态
            B_state                 = 1;
            pre_nout_ccp            = 1;
            PRINT_SWITCH_INFO("Parking Start");
            PathExec_PrintPathStart();

            float avmpoint[8];
            PathExec_GetAvmPoint(avmpoint);

            RTE_PK_DataConvt_Get_Target_PlanInfo(&targetPlanInfo);   // 获取路径
            SlotObj_Convert_float(targetPlanInfo.slotObjs, enviObj); // B 过程会更新谁
            PRINT_SWITCH_INFO("PK_PathExecuteRun: Slot, Obj, Path");
            PathExec_PrintSlotInfo(enviObj, curent_state[0].finpos, avmpoint); // 日志

            curent_state[0].Slot_index = targetPlanInfo.Slot_index;
            SlotObj_Convert_struct(&slotObj, enviObj);
            RTE_PK_SlotDetect_Set_SlotObj(&slotObj);

            PathExec_UpdateObsObj_AroundTargSlot();
            PathExec_PrintObjInfo();                                       // 日志
            PathExec_PrintPathInfo(Planned_Path, curent_state[0].tra_num); // 日志

            Avm_Pot_T avmSlot;
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);
            curent_state[0].hasStop = avmSlot.AlignIndex[0];

            // 清除车辆位置
            memset(&avmSlot, 0, sizeof(avmSlot));
            RTE_PD_Set_AVM_SlotInfo_B(&avmSlot); // 车位置0

            // 清除限位杆位置
            Avm_Obj_T stopBar;
            memset(&avmSlot, 0, sizeof(avmSlot));
            stopBar.slot_index = -1;
            RTE_PD_Set_AVM_SlotObs_B(&stopBar); // 限位杆置0

            float obj[4] = {0.0, 0.0, 0.0, 0.0};
            PathExec_AddObsObjNearSlot(obj, curent_state[0].finpos);

            float rx =
                Project_PosTo1st_rx(curent_state[0].curpos, curent_state[0].finpos);
            auto sensorType =
                RTE_PK_DataConvt_Get_IsAVM_Slot(); // 是否是视觉车位 0 - 超声波； 1 -
                                                   // 视觉； 2 - 指定； 3 -
                                                   // 视觉垂直车位且目标点在总路径起点（按下开始泊入键）前方4m外
            if (sensorType == 3)
            {
                PathExec_SetSideUpdateAvmTarg(curent_state);
            } // 如果是视觉垂直车位且目标点在总路径起点前方4m外
            if (rx > WHEEL_BASE)
            {
                PathExec_SetSideUpdateSpaceTarg(curent_state);
            } // 目标点在当前位置前方轴距距离外

            PK_SlotShapeType slotshape = RTE_PK_SlotDetect_Get_SlotShape();
            if (sensorType == 0 || sensorType == 2 || slotshape < PK_SLOT_LEFT_VERT)
            {
                PathExec_SetStopUpdateTarg(curent_state);
            } // 超声波或指定车位，不更新

            float beConfer[2][4];
            beConfer[0][0] = slotObj.ptB.x;
            beConfer[0][1] = slotObj.ptB.y;
            beConfer[0][2] = 0.5;
            beConfer[0][3] = 0.5;

            beConfer[1][0] = slotObj.ptE.x;
            beConfer[1][1] = slotObj.ptE.y;
            beConfer[1][2] = 0.5;
            beConfer[1][3] = 0.5;
            PathExec_SetBeConference(beConfer); // BE

            PathExec_ClearPercent(curent_state); // 初始化进度条
        }
        else
        {
            RTE_PK_PathExecute_Set_ModuleState_PathExecute(MSTAT_ABNORM); // 模块异常
            ModuleState_PathExecute = MSTAT_ABNORM;
            B_state                 = 0;
            if (curent_state[0].tra_num > MAX_SINGLE_TRAJ_NUM ||
                curent_state[0].tra_num <= 0)
            {
                pre_nout_ccp = 41;
            }
            else if (IsCurPosNearTrajStPos == 0) // 车辆当前位置不在路径起点附近
            {
                pre_nout_ccp = 4;
            }
            else
            {
                pre_nout_ccp = 5; // the targpos is too far away
            }
        }

        //  recheck slot shap
        float leftFac;
        curent_state[0].slotshape =
            Cal_SlotShap(curent_state[0].finpos, enviObj, &leftFac); // 车位类型
    }
    // response to the commond
    else if (ModuleCom_PathExecute == MCOM_OFF || ModuleCom_PathExecute == MCOM_INIT ||
             ModuleCom_PathExecute == MCOM_DATCLC) // DO NOT run Locating Condition;
    {
        ModuleState_PathExecute = MSTAT_OFF;
        RTE_PK_PathExecute_Set_ModuleState_PathExecute(MSTAT_OFF);
        targetPlanInfo.Path_num = 0;
        RTE_PK_DataConvt_Set_Target_PlanInfo(&targetPlanInfo);

        if (ModuleCom_PathExecute == MCOM_DATCLC)
        {
            SlotInfo_T parkSlot;
            memset(&parkSlot, 0, sizeof(parkSlot));
            RTE_PK_DataConv_Set_SlotInfo(&parkSlot);
            memset(curent_state, 0, sizeof(curent_state));

            PathExec_ClearParkObj();

            memset(&targetPlanInfo, 0, sizeof(targetPlanInfo));
            targetPlanInfo.IsDirConect = PARK_NOUPDATE;
            curent_state[0].replanSt   = PARK_NOUPDATE;
            RTE_PK_PathExecute_Set_CurTraj(&targetPlanInfo);
        }

        if (B_state > 0)
        {
            pre_nout_ccp = 111;
        } // OutSide interrupt

        B_state = 0;
        PRINT_SWITCH_INFO("Parking end");
        PathExec_PrintPathEnd(); // 结束
    }

    if (ModuleState_PathExecute == MSTAT_NORM) // 模块正常
    {
        // the main process
        RTE_PK_Location_Get_CurPos(curent_state[0].curpos);

        RTE_PK_SlotDetect_Get_SlotObj(&slotObj);
        SlotObj_Convert_float(slotObj, enviObj);
        memcpy(curent_state[0].objs, &slotObj, sizeof(slotObj));
        RTE_PK_SlotDetect_Get_TargPos(curent_state[0].finpos);

        B_state = PathExec_MainControl(
            Planned_Path, enviObj,
            curent_state); // 路径跟踪控制函数**************************************

        SlotObj_Convert_struct(&slotObj, enviObj);
        RTE_PK_SlotDetect_Set_SlotObj(&slotObj);
        RTE_PK_SlotDetect_Set_TargPos(curent_state[0].finpos);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // accumulate gone step
        // float velSpd = RTE_BSW_Get_ESC_VehSpd();
        if (fabsf(curent_state[0].cur_vel) > 0.05f &&
            sign(curent_state[0].cur_vel) * sign(curent_state[0].cvel) >
                0) // && fabs(velSpd) > 0.01)
        {
            curent_state[0].step_gone++;
            if (fabs(curent_state[0].leftDist) > 0.20f)
            {
                curent_state[0].stillCount = 0;
            }
        }
        else if (!IsBrakeVeh())
        {
            curent_state[0].stillCount =
                (PathExec_IsVehStill() &&
                 CarSta_GuidanceSuspend != RTE_SM_Get_ApaWorkState() &&
                 (RTE_BSW_Get_CurrentGear() != GEAR_REAL_P ||
                  curent_state[0].ds_gone_dir > 0.10f))
                    ? (curent_state[0].stillCount + 1)
                    : 0;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // output state
        if (B_state > 0) // 正常运行
        {
            ModuleState_PathExecute = MSTAT_NORM;
        }
        else if (B_state == 0 && RTE_PK_SM_GetApaMode() == PARKOUT_MODE) // 泊出
        {
            ModuleState_PathExecute = MSTAT_COMPLETE;
        }
        else if (B_state == 0 && fabs(curent_state[0].rx_cur_to_fin) < 0.60f &&
                 fabs(curent_state[0].ry_cur_to_fin) < 0.30f) // 完成
        {
            ModuleState_PathExecute = MSTAT_COMPLETE;
        }
        else // 模块异常
        {
            ModuleState_PathExecute = MSTAT_ABNORM;
        }
    }

    /*速度控制**********************************************************************************************************************/
    float brakeDist = tuning_rx_brake; // 初始刹车距离

    // 如果当前路径的段数小于总段数减一，或者停车位形状不是平行车位
    if (curent_state[0].cur_tra_nth < curent_state[0].tra_num - 1 ||
        curent_state[0].slotshape > PK_SLOT_RIGHT_PARA)
    {
        float filterSpd = RTE_PK_Location_Get_Vel_SpdFiltered(); // 获取滤波后的速度

        // 如果当前路径的段数对应的路径点的速度和刹车信号乘积大于0，或者
        // 当前路径段数为0且需要重新规划路径，并且当前速度与滤波后的速度乘积小于-0.01且滤波后的速度小于-0.16
        if (Planned_Path[curent_state[0].cur_tra_nth][3] * curent_state[0].rx_brake >
                0.0f ||
            (curent_state[0].cur_tra_nth == 0 && PathExec_ReplanFlag(curent_state) &&
             curent_state[0].cvel * filterSpd < -0.01f &&
             filterSpd < -0.16f)) // curent_state[0].cur_vel has switched to next path but
                                  // curpos is behind the path's start pos
        {
            // 20181213: 当前速度即使在约0.16m/s左右也不稳定
            // printf("PK_PathExecuteRun1 %d %f %f cfgspd %f filterspd %f\r\n",
            // curent_state[0].cur_tra_nth, Planned_Path[curent_state[0].cur_tra_nth][3],
            // curent_state[0].rx_brake, curent_state[0].cvel, filterSpd);
            brakeDist = 0.0f; // 立即停止
        }
        else
        {
            brakeDist = curent_state[0].rx_brake; // 使用当前路径的刹车距离
        }
    }
    else // 仅针对平行停车的最后路径或前后微调路径
    {
        // 20181101: 在停车位内慢慢移动
        if (fabsf(curent_state[0].cvel) > 0.4f)
        {
            float temp = rte_max(fabs(curent_state[0].cvel) * 0.96, 0.35); // 计算新的速度
            curent_state[0].cvel = sign(curent_state[0].cvel) * temp;      // 调整速度
        }
        brakeDist = curent_state[0].rx_brake; // 使用当前路径的刹车距离
    }

    if (ModuleState_PathExecute == MSTAT_NORM)
    {
        RTE_PK_PathExecute_Set_TargVelspd(curent_state[0].cvel);

        if (curent_state[0].cur_tra_nth + 1 == curent_state[0].tra_num &&
            fabs(brakeDist) < 0.15)
        {
            RTE_PK_PathExecute_Set_TargEPS_Angle(0);
        }
        else
        {
            RTE_PK_PathExecute_Set_TargEPS_Angle(curent_state[0].ceps);
        }

        RTE_PK_PathExecute_Set_rx_brake(
            brakeDist); //  20181101: 0.5 -> 0.45 suit for pk_objavoid to send ccd_st
        RTE_PK_PathExecute_Set_BrkSt(curent_state[0].brakeSt);
        PathExec_PrintCurState(curent_state[0]);
        RTE_PK_ObjAvoid_Set_ModuleState_ObjAvoid(MSTAT_NORM);
    }
    else if (ModuleState_PathExecute == MSTAT_COMPLETE ||
             ModuleState_PathExecute == MSTAT_ABNORM)
    {
        RTE_PK_ObjAvoid_Set_ModuleState_ObjAvoid(MSTAT_COMPLETE); // 泊车结束标志
        if (ModuleState_PathExecute == MSTAT_ABNORM)
        {
            RTE_PK_PathExecute_Set_ModuleState_ExecuteFail(1); // 泊车失败标志
        }
        else
        {
            RTE_PK_PathExecute_Set_ModuleState_ExecuteFail(0);
        }
        curent_state[0].percent = 1.0;
        PathExec_ClearParkObj();
    }

    lastModuleCom = ModuleCom_PathExecute;
    PathExec_SaveToCCP(curent_state[0]);
}

/**
 * @brief 计算路径执行的后续步骤
 *
 * 该函数计算在当前状态下，路径执行的后续步骤。
 *
 * @param curSt 当前自动驾驶参数状态数组
 * @param r_cur 当前曲率半径
 * @param r_next 下一个曲率半径
 * @return float 返回路径执行的后续步骤距离
 */
static float PathExec_ContinueR(const PK_Cur_AutodrPara curSt[1], const float r_cur,
                                const float r_next)
{
    const float cvel_base  = 0.3f; // 基础速度
    const float high_ratio = 0.6f; // 置信度较高的比例
    const float low_ratio  = 0.3f; // 较低部分的比例

    float drou, drx;                        // 路径的差异量
    float TargSpd_After = curSt[0].cvel;    // 目标速度，初始化为当前速度
    float vel_spd       = curSt[0].cur_vel; // 当前速度
    float cur_eps       = curSt[0].cur_eps; // 当前EPS（电动助力转向）值

    // 检查车辆是否按照计划路径移动，如果不是，则不更改路径
    if (TargSpd_After * vel_spd < 0 || fabsf(TargSpd_After) < 0.001f)
    {
        return 0.0f;
    }

    // 计算rou_spd_EPS值
    float rou = 0.0;
    rou += fabs(PK_EPS_to_Rou(PK_EPS_ANGLE_MAX, 1.0));
    rou += fabs(PK_EPS_to_Rou(PK_EPS_ANGLE_MAX, -1.0));
    rou += fabs(PK_EPS_to_Rou(-1.0 * PK_EPS_ANGLE_MAX, 1.0));
    rou += fabs(PK_EPS_to_Rou(-1.0 * PK_EPS_ANGLE_MAX, -1.0));

    float rou_spd_EPS =
        0.25 * rou / PK_EPS_ANGLE_MAX; // (Cmax-Cmin)/(EPS_angle_max-EPS_angle_min)
    float rou_spd = rou_spd_EPS * EPS_SPD_CCP;

    // 计算当前和下一步的曲率半径
    float rou_cur  = PK_EPS_to_Rou(cur_eps, vel_spd);
    float rou_next = (fabsf(r_next) < 1) ? 0 : (1 / r_next);

    // 计算drou值
    if (rou_cur * rou_next > 0)
    {
        drou = rte_max(fabsf(rou_next), fabsf(rou_cur)) / 2; // 如果同号，则取较大值的一半
    }
    else
    {
        drou = fabsf(rou_cur - rou_next) / 2; // 如果异号，则取差值的一半
    }

    // 根据速度和目标速度计算drx值
    if (TargSpd_After * vel_spd >= -0.001f && fabsf(vel_spd) < fabsf(TargSpd_After))
    {
        if (fabsf(vel_spd) < low_ratio * fabsf(TargSpd_After))
        {
            // 使用高比例和低比例混合计算drx
            drx = fabsf(high_ratio * vel_spd + (1.0f - high_ratio) * TargSpd_After) *
                  (drou / rou_spd * eps_vel_factor + eps_delaydt_ccp);
        }
        else
        {
            drx = fabsf((1.0f - high_ratio) * vel_spd + high_ratio * TargSpd_After) *
                  (drou / rou_spd * eps_vel_factor + eps_delaydt_ccp);
        }
    }
    else
    {
        // 使用基础速度和速度最大值计算drx
        drx = rte_max(fabsf(vel_spd), cvel_base) *
              (drou / rou_spd * eps_vel_factor + eps_delaydt_ccp); // 计算距离
    }

    return drx; // 返回路径执行的后续步骤距离
}

static int SmoothNewTraj(const PlanInfoType &slotPlane, const float curpos[3],
                         float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN])
{
    // 延长路径
    const float step = 0.05f;
    float traject[TRAJITEM_LEN];
    PK_CopyTra(traject, slotPlane.Act_traj[0]);
    PK_ReverseTraject(traject);
    traject[3] = (fabs(traject[3]) + 0.6) * sign(traject[3]);
    PK_ReverseTraject(traject);

    float trajEnd[3];
    int minIdx    = 0;
    float trajlen = fabs(traject[3]);
    auto minDist  = Cal_Dis_Pt2Pt(curpos, traject);
    for (int i = 1; i < 20 && i * step < trajlen && minDist > 0.5 * step; i++)
    {
        traject[3] = i * step * sign(traject[3]);
        PK_Get_Path_EndPos(traject, trajEnd);
        float dist = Cal_Dis_Pt2Pt(curpos, trajEnd);
        if (minDist > dist)
        {
            minDist = dist;
            minIdx  = i;
        }
    }

    traject[3] = minIdx * step * sign(traject[3]);
    PK_Get_Path_EndPos(traject, trajEnd);
    PK_CopyPos(traject, trajEnd);
    traject[3] = (trajlen - minIdx * step) * sign(traject[3]);
    if (fabs(traject[3]) < 0.1)
    {
        memcpy(Planned_Path, slotPlane.Act_traj[1],
               (slotPlane.Path_num - 1) * sizeof(float) * TRAJITEM_LEN);
        return slotPlane.Path_num - 1;
    }
    else
    {
        PK_CopyTra(Planned_Path[0], traject);
        memcpy(Planned_Path[1], slotPlane.Act_traj[1],
               (slotPlane.Path_num - 1) * sizeof(float) * TRAJITEM_LEN);
        return slotPlane.Path_num;
    }
}

static bool IsNarrowSlot(const float enviObj[5][4], const float target[3], float wide)
{
    float distBC = PK_PointToLineDist(enviObj[1], target);
    float distDE = PK_PointToLineDist(enviObj[3], target);
    return (0.5f * (distBC + distDE) < VEHICLE_WID + wide);
}

inline bool PathExec_StopReplan(const PK_Cur_AutodrPara *curSt, const float enviObj[5][4],
                                const float target[3])
{
    if (curSt[0].rx_cur_to_fin < REAR_SUSPENSION &&
        fabs(curSt[0].rtheta_cur_to_fin) < 10.0f * PI_RAD &&
        IsNarrowSlot(enviObj, target, 0.75f))
    {
        return true;
    }

    if (curSt[0].rx_cur_to_fin < (VEHICLE_LEN - 0.50f) &&
        fabs(curSt[0].rtheta_cur_to_fin) < 10.0f * PI_RAD &&
        IsNarrowSlot(enviObj, target, 0.65f))
    {
        return true;
    }

    if (PathExec_IsRearDanger(curSt) && fabs(curSt[0].rtheta_cur_to_fin) < 10.0f * PI_RAD)
    {
        U_RadarType uradar; // 超声波实时反馈数据
        RTE_PD_Get_U_Radar(&uradar);
        float uRadarDist = (uradar.RSR + uradar.RSL) / 1000.0f;
        if ((curSt[0].rx_cur_to_fin < REAR_SUSPENSION && uRadarDist < 0.75f) ||
            (curSt[0].rx_cur_to_fin < (VEHICLE_LEN - 0.50f) && uRadarDist < 0.65f))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 重新规划路径前的预处理函数
 *
 * 该函数根据当前状态和环境信息，确定是否需要重新规划路径，并执行相应的路径规划算法。
 *
 * @param plannedPath 计划路径数组
 * @param enviObj 环境对象数组
 * @param target 目标位置数组
 * @param dir 方向标志
 * @param curSt 当前泊车状态参数结构体
 * @param swell 扩展参数引用
 * @return int 返回重新规划路径的数量
 */
static int ReplanPathPre(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                         const float enviObj[5][4], const float target[3], const int dir,
                         PK_Cur_AutodrPara curSt[1], float &swell)
{
    // 检查当前状态前方和后方是否存在危险
    if (PathExec_IsFrontDanger(curSt) && PathExec_IsRearDanger(curSt))
    {
        // 如果前后都有危险，增加动态规划计数器，并设置重新规划结果为失败
        curSt[0].B_DynaPlanCounter = curSt[0].B_DynaPlanCounter + 2;
        curSt[0].B_ReplanResult    = PATHEXECUTE_FAIL;
        return 0;
    }

    int trajNum = 0; // 路径数量初始化为0

    // 检查当前停车位形状是否为垂直停车
    if (curSt[0].slotshape == PK_SLOT_LEFT_VERT ||
        curSt[0].slotshape == PK_SLOT_RIGHT_VERT)
    {
        // 如果后方没有危险，进行后方移动路径规划
        if (!PathExec_IsRearDanger(curSt))
        {
            PRINT_SWITCH_INFO("dynamic replan PathExec_B_MoveOutRear");
            trajNum =
                PathExec_B_MoveOutRear(plannedPath, target, enviObj, curSt[0].curpos);
        }

        // 如果方向标志为2，直接返回路径数量
        if (dir == 2)
        {
            return trajNum;
        }

        // 如果路径数量不大于0且前方没有危险，尝试在停车位内重新规划路径
        if (trajNum <= 0 && !PathExec_IsFrontDanger(curSt))
        {
            // 检查当前角度与目标角度的差异
            if (fabs(curSt[0].rtheta_cur_to_fin) < 30 * PI_RAD)
            {
                PRINT_SWITCH_INFO("dynamic replan PathExec_SpaceExplorer_RePlanInSlot");
                float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                trajNum = PathExec_SpaceExplorer_RePlanInSlot(updatePath, target, enviObj,
                                                              curSt[0].curpos);

                // 如果重新规划出路径且后方有危险，进一步检查路径的安全性
                if (trajNum > 0 && PathExec_IsRearDanger(curSt))
                {
                    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();
                    if (updatePath[0][4] > 1.0 && (dangerSt & 0x20) != 0)
                    {
                        trajNum = 0;
                    }
                    else if (updatePath[0][4] < 1.0 && (dangerSt & 0x10) != 0)
                    {
                        trajNum = 0;
                    }
                }

                // 将更新后的路径复制到计划路径
                memcpy(plannedPath, updatePath, trajNum * TRAJITEM_LEN * sizeof(float));
            }

            // 如果路径数量仍然小于等于0，进行前方移动路径规划
            if (trajNum <= 0)
            {
                PRINT_SWITCH_INFO("dynamic replan PathExec_B_MoveOutFront");
                trajNum = PathExec_B_MoveOutFront(plannedPath, target, enviObj,
                                                  curSt[0].curpos, curSt[0].slotshape);
            }
        }
    }
    else
    {
        // 如果停车位形状不是垂直停车，进行平行车位的路径重规划
        trajNum = PathExec_B_MoveOut_ParaRePlan(plannedPath, target, enviObj,
                                                curSt[0].curpos, curSt[0].slotshape);
    }

    return trajNum; // 返回路径数量
}

/**
 * @brief 路径重规划后的处理函数
 *
 * 在路径重规划成功后，更新车辆状态参数，并初始化和准备执行重新规划的路径。
 *
 * @param plannedPath 规划的路径数组，存储了轨迹点的信息
 * @param trajNum 规划路径的轨迹点数量
 * @param enviObj 环境障碍物数组，存储了障碍物的信息
 * @param target 目标位置数组
 * @param curSt 当前车辆状态结构体，包含了车辆的各种状态信息
 */
static void ReplanPathPost(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                           int trajNum, const float enviObj[5][4], const float target[3],
                           PK_Cur_AutodrPara curSt[1])
{
    // 设置重规划结果为成功
    curSt[0].B_ReplanResult = PATHEXECUTE_SUCCESS;

    // 更新动态规划计数器
    if (PathExec_IsCrossReplan(curSt) || PathExec_IsDangerReplan(curSt) ||
        PathExec_IsLastReplan(curSt))
    {
        curSt[0].B_DynaPlanCounter = curSt[0].B_DynaPlanCounter + 2;
    }
    else
    {
        curSt[0].B_DynaPlanCounter = curSt[0].B_DynaPlanCounter + 1;
    }

    // 更新危险重规划标志
    if (PathExec_IsDangerReplan(curSt))
    {
        PathExec_SetDangerReplan(curSt, replan_success);
    }
    else
    {
        PathExec_SetDangerReplan(curSt, replan_idle);
    }

    // 更新去除重规划标志
    if (PathExec_IsDeReplan(curSt))
    {
        PathExec_SetDeReplan(curSt, replan_success);
    }
    else
    {
        PathExec_SetDeReplan(curSt, replan_idle);
    }

    // 更新交叉重规划标志
    if (PathExec_IsCrossReplan(curSt))
    {
        PathExec_SetCrossReplan(curSt, replan_success);
    }
    else
    {
        PathExec_SetCrossReplan(curSt, replan_idle);
    }

    // 复制新的目标位置到当前状态
    PK_CopyPos(curSt[0].finpos, target);

    // 转换环境障碍物结构体并设置槽对象
    SlotObj_T slotObj;
    SlotObj_Convert_struct(&slotObj, enviObj);
    RTE_PK_SlotDetect_Set_SlotObj(&slotObj);
    RTE_PK_SlotDetect_Set_TargPos(curSt[0].finpos);

    PRINT_SWITCH_INFO("Display line number");
    PathExec_PrintPathInfo(plannedPath, trajNum);
    if (curSt[0].slotshape >= PK_SLOT_LEFT_VERT &&
        PathExec_StopReplan(curSt, enviObj, curSt[0].finpos) && plannedPath[0][3] > 0)
    {
        return;
    }

    // 初始化并准备执行重新规划的路径
    PathExec_StartNewPlanPath(curSt, trajNum, plannedPath[0][3]);
}

/**
 * @brief 更新目标位置并重新规划路径
 *
 * 根据当前车辆状态和环境信息，判断是否需要更新目标位置并重新规划路径。
 *
 * @param plannedPath 规划的路径数组
 * @param enviObj 环境障碍物数组
 * @param curSt 当前车辆状态结构体
 * @param swell 路径膨胀系数的引用
 * @return bool 返回是否成功更新目标位置和重新规划路径
 */
static bool UpdateTargReplan(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                             float enviObj[5][4], PK_Cur_AutodrPara curSt[1],
                             float &swell)
{
    float newTarget[3], newEnviObj[5][4];
    memcpy(newTarget, curSt[0].finpos, sizeof(newTarget)); // 复制当前目标位置到 newTarget
    memcpy(newEnviObj, enviObj,
           sizeof(newEnviObj)); // 复制当前环境障碍物信息到 newEnviObj

    static float dxodm            = 0.0; // 用于存储上一次的路径长度
    const float check_target_dist = 0.2; // 检查目标距离阈值
    float dir =
        (curSt[0].slotshape == PK_SLOT_LEFT_VERT) ? 1.0 : -1.0; // 根据泊车槽形状确定方向

    // 判断是否需要更新目标位置
    bool rearCond =
        (curSt[0].rx_cur_to_fin > 0.5 && curSt[0].rx_cur_to_fin < 3.5 * REAR_SUSPENSION &&
         fabs(curSt[0].rtheta_cur_to_fin) < 13 * PI_RAD && curSt[0].cvel < 0);
    bool frontCond =
        (curSt[0].rx_cur_to_fin < -0.5 &&
         curSt[0].rx_cur_to_fin > -3.5 * REAR_SUSPENSION &&
         fabs(curSt[0].rtheta_cur_to_fin) < 13 * PI_RAD && curSt[0].cvel > 0);
    bool condition1 = (PathExec_IsRearUpdateTarg(curSt) && (rearCond || frontCond));
    bool condition2 = (PathExec_IsSideUpdateTarg(curSt) &&
                       dir * curSt[0].ry_cur_to_fin > cfg_sideupdate_dist * VEHICLE_WID &&
                       fabs(curSt[0].rx_brake) > check_target_dist);

    if ((!condition1 || !PathExec_IsUpdateRearVision()) &&
        (!condition2 || !PathExec_IsUpdateSideVision())) // 如果不满足更新目标位置的条件
    {
        dxodm = curSt[0].curpos[3]; // 更新路径长度
        return false;
    }

    if (fabs(curSt[0].curpos[3] - dxodm) < check_target_dist)
    {
        return false;
    } // 如果路径长度变化小于阈值，返回 false
    dxodm = curSt[0].curpos[3]; // 更新路径长度

    if (curSt[0].slotshape == PK_SLOT_LEFT_VERT ||
        curSt[0].slotshape == PK_SLOT_RIGHT_VERT)
    {
        if (IsNarrowSlot(newEnviObj, newTarget, 0.75f))
        {
            return false;
        }
    }

    // 更新目标位置
    bool isTargUpdate =
        PathExec_UpdateTarget(curSt, newEnviObj, newTarget, false, __LINE__);
    int collion2 =
        CheckCollisionWithObj(&plannedPath[curSt[0].cur_tra_nth],
                              curSt[0].tra_num - curSt[0].cur_tra_nth, &newEnviObj[3]);

    if (!isTargUpdate && collion2 < 0)
    {
        return false;
    } // 如果目标位置没有更新且没有碰撞，返回 false

    float ryf = Project_PosTo1st_ry(
        newTarget, curSt[0].finpos); // 计算新的目标位置与当前终点的 y 方向偏差
    if ((isTargUpdate && fabs(ryf) > 0.04f &&
         curSt[0].B_ReplanResult == PATHEXECUTE_SUCCESS) ||
        collion2 >= 0)
    {
        char info[128];
        snprintf(info, sizeof(info), "PathExec_ReplanCal:%d,%d,%d,%d", condition1,
                 condition2, rearCond, frontCond);
        PRINT_SWITCH_INFO(info);

        if (condition2)
        {
            PathExec_MovePath_dyna(plannedPath, newTarget, curSt);
            PathExec_ClrSideUpdateTarg(curSt); // 清除侧向目标更新标志
            memcpy(enviObj, newEnviObj,
                   sizeof(newEnviObj)); // 复制更新后的环境信息到 enviObj
            return true;                // 返回 true 表示成功更新目标位置和重新规划路径
        }

        float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
        memcpy(updatePath, plannedPath, sizeof(updatePath)); // 复制当前路径到 updatePath
        int trajNum = ReplanPathPre(updatePath, newEnviObj, newTarget, 2, curSt,
                                    swell); // 重新规划路径

        if (trajNum > 0 && sign(updatePath[0][3]) ==
                               sign(plannedPath[curSt[0].cur_tra_nth][3])) // 方向一致
        {
            PRINT_SWITCH_INFO("PathExec_ReplanPath1");
            ReplanPathPost(updatePath, trajNum, newEnviObj, newTarget,
                           curSt); // 更新路径后处理
            memcpy(plannedPath, updatePath,
                   sizeof(updatePath)); // 复制更新后的路径到 plannedPath
            memcpy(enviObj, newEnviObj,
                   sizeof(newEnviObj)); // 复制更新后的环境信息到 enviObj
            if (condition1)
            {
                PathExec_SetRearUpdateTarg(curSt);
            } // 设置后向目标更新标志
            return true; // 返回 true 表示成功更新目标位置和重新规划路径
        }
        else if (trajNum > 0)
        {
            PRINT_SWITCH_INFO("PathExec_ReplanPath2");
            if (condition1 && fabs(curSt[0].rx_cur_to_fin) > 2.0 * REAR_SUSPENSION)
            {
                return false;
            }

            PathExec_SetUpdateReplan(curSt, replan_start); // 设置更新重规划标志
            if (condition1)
            {
                PathExec_SetRearUpdateTarg(curSt);
            } // 设置后向目标更新标志
        }
#if 0
        else
        {
            PRINT_SWITCH_INFO("PK_PathPlan_RangeDataProcess");
            trajNum = PK_PathPlan_RangeDataProcess(updatePath, newTarget, newEnviObj, curSt[0].curpos, curSt[0].slotshape, RTE_PK_DataConvt_Get_IsAVM_Slot());
            printf("PK_PathPlan_RangeDataProcess trajnum: %d\n", trajNum);
            if (trajNum > 0 && sign(updatePath[0][3]) == sign(plannedPath[curSt[0].cur_tra_nth][3])) //方向一致
            {
                PRINT_SWITCH_INFO("PK_PathPlan_RangeDataProcess1");
                ReplanPathPost(updatePath, trajNum, newEnviObj, newTarget, curSt); // 更新路径后处理
                memcpy(plannedPath, updatePath, sizeof(updatePath)); // 复制更新后的路径到 plannedPath
                memcpy(enviObj, newEnviObj, sizeof(newEnviObj)); // 复制更新后的环境信息到 enviObj
                if (condition2) { PathExec_ClrSideUpdateTarg(curSt); } // 清除侧向目标更新标志
                if (condition1) { PathExec_SetRearUpdateTarg(curSt); } // 设置后向目标更新标志
                curSt[0].updatePathNum = 0; // 重置更新路径次数
                return true; // 返回 true 表示成功更新目标位置和重新规划路径
            }
            else if (trajNum > 0)
            {
                PRINT_SWITCH_INFO("PK_PathPlan_RangeDataProcess2");
                if (condition1 && fabs(curSt[0].rx_cur_to_fin) > 2.0 * REAR_SUSPENSION) {  return false;  }
                PathExec_SetUpdateReplan(curSt); // 设置更新重规划标志
                if (condition1) { PathExec_SetRearUpdateTarg(curSt); } // 设置后向目标更新标志
            }
        }
#endif
    }

    return false; // 返回 false 表示没有更新目标位置和重新规划路径
}

/**
 * @brief PathExec_ReplanCal 重新计算规划路径的函数
 *
 * 根据周围环境和车辆的当前状态，重新规划车辆的路径。
 *
 * @param plannedPath 车辆的计划轨迹
 * @param enviObj 影响车辆路径的环境对象
 * @param curSt 车辆的当前状态参数
 * @param swell 用于路径规划的膨胀值
 * @return void
 */
static void PathExec_ReplanCal(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                               float enviObj[5][4], PK_Cur_AutodrPara curSt[1],
                               float &swell)
{
    // 重规划条件的阈值常量
    const float rx_dynaplan_min =
        0.5f;                        // 动态规划最小值，临时修改为0.5，支持B过程后改回1.2
    const float rx_coss_ds   = 5.5f; // 从3.5修改为5.0，用于优化B过程
    const float rx_danger_ds = 7.0f; // 临时修改为7.0，支持B过程后改回4.2
    const float rx_replan_deep_min =
        0.1f; // 如果之前发生了重规划，则当前位置应更深入车位以打开重规划
    const float ry_replan_deep_min  = 0.03f; // 同上，y方向
    const float big_follow_error    = 0.45f;
    const float rtheta_dange_replan = 1.0; // 0.785f
    const float danger_side_dist =
        0.04f; // 在停车时，如果侧距小于此值，将考虑发生碰撞危险
    const float crossReplan_side_dist = 0.1f;
    const float swell_min             = 0.35f;
    const float swell_max             = 0.45f; // swell范围

    float rx_de_over_ds_max = 3.2f; // 20181005: 从2.7f修改为4.1f，更早进行重规划
    float sum_de_rep_min    = 3.2f; //  20181018: 从3.0f修改为3.2f，以减少过多的重规划
    float Envi_obj_side[2][4];
    float temp_pos[4];

    // 截取车位两侧数据以适应环境
    PK_PathPlan_Envi_obj_cut(enviObj, 0.92f, Envi_obj_side);
    // 计算当前周围最小距离并赋值给curSt
    curSt[0].min_arround_dist =
        PK_PosObjCrossMinDist(curSt[0].curpos, 2, Envi_obj_side); // swell [0.1   0.45]

    // 根据当前侧距计算重规划的深度阈值
    float replan_de =
        rte_min(rte_max(0.11f, (curSt[0].min_arround_dist - 0.25f) * 0.5f + 0.11f), 0.2f);

    // 根据当前侧距计算swell
    swell = rte_min(rte_max(swell_min, 0.5f * curSt[0].min_arround_dist), swell_max);
    // 获取1秒后的车辆位置
    PK_Pre_locat(curSt[0].curpos, temp_pos, curSt[0].cur_eps, curSt[0].cur_vel, 1.0f);

    float endPos[3];
    // 获取规划路径的终点位置
    PK_Get_Path_EndPos(plannedPath[curSt[0].cur_tra_nth], endPos);
    // 计算当前位置到临时位置的距离
    float distToTemp = Cal_Dis_Pt2Pt(curSt[0].curpos, temp_pos);
    // 计算当前位置到终点位置的距离
    float distToEnd = Cal_Dis_Pt2Pt(curSt[0].curpos, endPos);
    if (distToEnd > distToTemp + crossReplan_side_dist)
    {
        curSt[0].min_arround_dist_dt = PK_PosObjCrossMinDist(temp_pos, 2, Envi_obj_side);
    }
    else
    {
        curSt[0].min_arround_dist_dt = 0;
    }

    float newTarget[3], newEnviObj[5][4];
    memcpy(newTarget, curSt[0].finpos, sizeof(newTarget));
    memcpy(newEnviObj, enviObj, sizeof(newEnviObj));

    // 如果当前速度小于-0.01
    if (curSt[0].cvel < -0.01f)
    {
        // 如果最小侧距小于危险侧距，则增加停止计数器
        if (curSt[0].min_arround_dist < danger_side_dist)
        {
            // curSt[0].B_Stop_Counter++;
        }
        else
        {
            curSt[0].B_Stop_Counter = 0;
        }

        // 判断三种重规划类型：de-replan，cross-replan，danger-replan
        float rspoint[3];
        RevConvert(curSt[0].finpos, plannedPath[0], rspoint);
        curSt[0].rx_RePathSt = rspoint[0];
        curSt[0].ry_RePathSt = rspoint[1];

        // 如果之前没有发生过重规划，或者重规划的深度足够，则允许重规划
        if (!PathExec_ReplanFlag(curSt) ||
            (PathExec_ReplanFlag(curSt) &&
             (fabs(curSt[0].rx_RePathSt - curSt[0].rx_cur_to_fin) > rx_replan_deep_min ||
              fabs(curSt[0].ry_RePathSt - curSt[0].ry_cur_to_fin) >
                  ry_replan_deep_min)) // 终点到起点的X像距离 - 终点到当前点的距离
                                       // 大于门限，即车辆需要行驶一段距离后才可以重新规划
            // rx_RePathSt_to_fin - curSt[0].rx_cur_to_fin > rx_replan_deep_min)  //
            // 终点到起点的X像距离 - 终点到当前点的距离
            // 大于门限，即车辆需要行驶一段距离后才可以重新规划
        )
        {
            // 如果检测到危险并且距离小于阈值，则进行危险重规划
            if (PathExec_IsObjDanger(curSt) && curSt[0].rx_cur_to_fin < rx_danger_ds &&
                fabsf(curSt[0].rtheta_cur_to_fin) < rtheta_dange_replan)
            {
                PathExec_SetDangerReplan(curSt, replan_start);
            }
            // 如果速度小于-0.01且满足cross重规划条件，则进行cross重规划
            else if (curSt[0].cvel < -0.01f && curSt[0].rx_cur_to_fin < rx_coss_ds &&
                     ((curSt[0].min_arround_dist_dt < crossReplan_side_dist &&
                       curSt[0].min_arround_dist < crossReplan_side_dist) ||
                      (curSt[0].min_arround_dist_dt <
                           (curSt[0].min_arround_dist - 0.01f) &&
                       curSt[0].min_arround_dist_dt < crossReplan_side_dist &&
                       curSt[0].min_arround_dist < 2 * crossReplan_side_dist)))
            {
                PathExec_SetCrossReplan(curSt, replan_start);
            }
            // 如果跟随误差较大
            else if (fabsf(curSt[0].de) > replan_de &&
                     fabsf(curSt[0].sum_de) > sum_de_rep_min &&
                     curSt[0].rx_cur_to_fin < rx_de_over_ds_max)
            {
                PathExec_SetDeReplan(curSt, replan_start);
            }
            // 如果跟随误差超出阈值
            else if (fabsf(curSt[0].de) > big_follow_error &&
                     fabsf(curSt[0].sum_de) > sum_de_rep_min)
            {
                PathExec_SetDeReplan(curSt, replan_start);
            }
        }
    }
    else
    {
        curSt[0].B_Stop_Counter = 0;
    }

    // 告警触发的重规划有可能是短暂的告警,
    if (!PathExec_IsObjDanger(curSt) && PathExec_IsDangerReplan(curSt))
    {
        PathExec_SetDangerReplan(curSt, replan_idle);
    }

    // 如果没有进行任何重规划或调整，则更新目标重规划
    if (!PathExec_IsDeReplan(curSt) && !PathExec_IsCrossReplan(curSt) &&
        !PathExec_IsDangerReplan(curSt) && !PathExec_IsLastAdjust(curSt))
    {
        if (UpdateTargReplan(plannedPath, enviObj, curSt, swell))
        {
            return;
        }
    }

    // 如果当前到终点的距离大于动态规划最小值
    if (curSt[0].rx_cur_to_fin > rx_dynaplan_min)
    {
        // 如果需要进行重规划且速度小于-0.01或更新重规划，则进行重规划操作
        if (((PathExec_IsDeReplan(curSt) || PathExec_IsCrossReplan(curSt) ||
              PathExec_IsDangerReplan(curSt)) &&
             curSt[0].cvel < -0.01f) ||
            PathExec_IsUpdateReplan(curSt))
        {
            // 如果车辆静止，则进行目标更新和重规划
            if (PathExec_IsVehStill() && !PathExec_IsParkUpdate(curSt))
            {
                if (PathExec_IsSuspendByObjs(curSt))
                {
                    return;
                }
                PRINT_SWITCH_INFO("PathExec_ReplanCal");
                bool updateTarget = false;
                if (!PathExec_IsDeReplan(curSt) && !PathExec_IsCrossReplan(curSt) &&
                    !PathExec_IsUpdateReplan(curSt))
                {
                    updateTarget = PathExec_UpdateTarget(curSt, newEnviObj, newTarget,
                                                         false, __LINE__);
                }
                else
                {
                    updateTarget = PathExec_UpdateTarget(curSt, newEnviObj, newTarget,
                                                         true, __LINE__);
                }

                if (updateTarget)
                {
                    int trajNum = ReplanPathPre(plannedPath, newEnviObj, newTarget, 0,
                                                curSt, swell);
                    if (trajNum > 0)
                    {
                        ReplanPathPost(plannedPath, trajNum, newEnviObj, newTarget,
                                       curSt);
                        memcpy(enviObj, newEnviObj, sizeof(newEnviObj));
                    }
                    else if (PathExec_IsUpdateReplan(curSt) && curSt[0].updatePathNum > 0)
                    {
                        PRINT_SWITCH_INFO("PathExec_ReplanCal");
                        PathExec_PrintPathInfo(curSt[0].updatePath,
                                               curSt[0].updatePathNum);
                        float trajConn[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
                        int trajConnNum = PathExec_BetterTra(
                            curSt[0].curpos, curSt[0].updatePath[0], 3, &newEnviObj[1],
                            trajConn, side_swell_0, 0.1);
                        if (trajConnNum > 0)
                        {
                            PRINT_SWITCH_INFO("PathExec_ReplanCal");
                            memcpy(trajConn[trajConnNum], curSt[0].updatePath[0],
                                   sizeof(float) * TRAJITEM_LEN * curSt[0].updatePathNum);
                            trajConnNum = PK_PathMerge(
                                trajConn, trajConnNum + curSt[0].updatePathNum, 0);
                            memcpy(plannedPath[0], trajConn[0],
                                   sizeof(float) * TRAJITEM_LEN * trajConnNum);
                            ReplanPathPost(plannedPath, trajConnNum, newEnviObj,
                                           newTarget, curSt);
                            memcpy(enviObj, newEnviObj, sizeof(newEnviObj));
                            curSt[0].updatePathNum = 0;
                        }
                    }
                    else
                    {
                        curSt[0].B_ReplanResult = PATHEXECUTE_FAIL;
                    }
                }
            }
            else if (!PathExec_IsUpdateReplan(curSt) || fabs(curSt[0].cur_vel) < 0.20)
            {
                if (curSt[0].brakeSt == 0)
                {
                    curSt[0].brakeSt = 1;
                }
            }
        }
    }
}

// PathExec_ParaCal(Planned_Path,uradar,finpoint,Envi_obj,curSt,B_Danger_filter);

/**
 * @brief 路径跟踪函数
 *
 * 负责路径执行过程中的轨迹跟踪和控制计算，分析当前位置与规划路径之间的偏差，并更新车辆的控制参数
 *
 * @param Planned_Path 规划路径信息数组
 * @param curSt 当前车辆状态信息结构体
 */
static void PathExec_TraceTraj(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                               PK_Cur_AutodrPara curSt[1])
{
    const float ceps_max   = PK_EPS_ANGLE_MAX; // 最大转向角
    const float d_ceps_max = 200.0f;           // 转向角变化的最大值
    const float K2_MDB     = 11.0f;            // 基于模型的控制参数
    const float DE_GAP     = 0.025f;           // 偏差容差
    const float MAXEPS_MDB = 180.0;            // 最大电动助力转向角

    ////////////////////////////////////////////////////////////////////////////////////////
    // 提取当前轨迹点的半径和距离
    float r  = Planned_Path[curSt[0].cur_tra_nth][4]; // 当前轨迹点的半径
    float ds = Planned_Path[curSt[0].cur_tra_nth][3]; // 当前轨迹点的距离

    ////////////////////////////////////////////////////////////////////////////////////////
    // 计算位置误差
    float rtheta = curSt[0].curpos[2] -
                   Planned_Path[curSt[0].cur_tra_nth][2]; // 当前航向角与轨迹点的航向角差
    rtheta   = Round_PI(rtheta);                          // 将角度标准化到[-PI, PI]范围
    float rx = Project_PosTo1st_rx(Planned_Path[curSt[0].cur_tra_nth],
                                   curSt[0].curpos); // 投影到第一轨迹点的x偏差
    float ry = Project_PosTo1st_ry(Planned_Path[curSt[0].cur_tra_nth],
                                   curSt[0].curpos); // 投影到第一轨迹点的y偏差

    // 判断路径是直线还是曲线
    if (fabsf(r) < 2.0f) // 路径是直线
    {
        curSt[0].de      = -ry;     // y方向偏差，正值表示目标位置在当前车辆左侧
        curSt[0].rxs     = rx;      // x方向偏差，向前为正，向后为负
        curSt[0].rthetaf = -rtheta; // 转向角偏差，低通滤波去抖动
    }
    else // 路径是曲线
    {
        float d     = sqrtf(rx * rx + (r - ry) * (r - ry)); // 当前点到路径圆心的距离
        float xor_t = fabsf(r) / d * rx;                    // x方向的投影
        float yor_t = fabsf(r) / d * (ry - r) + r;          // y方向的投影
        curSt[0].de = (d - fabsf(r)) * sign(r);             // 距离误差，正值表示偏离路径

        float temp = sign(yor_t - r);
        if (fabs(yor_t) > fabs(r))
        {
            temp = -temp;
        }
        float thor   = temp * acosf((xor_t) / fabsf(r)) + sign(r) * PI / 2.0f; // 角度误差
        curSt[0].rxs = sign(rx) * fabsf((thor)*r);    // x方向偏差
        curSt[0].rthetaf = Round_PI(ds / r - rtheta); // 转向角偏差
    }
    curSt[0].rthetas = rtheta;            // 当前转向角偏差
    curSt[0].rxf     = curSt[0].rxs - ds; // x方向偏差，向前为负，向后为正

    //////////////////////////////////////////////////////////////////////////////////////
    // 计算1秒后的偏差变化率
    float temp_pos[4]; // 临时位置数组
    PK_Pre_locat(curSt[0].curpos, temp_pos, curSt[0].cur_eps, curSt[0].cur_vel,
                 1.0f); // 预测1秒后的位置
    rx = Project_PosTo1st_rx(Planned_Path[curSt[0].cur_tra_nth],
                             temp_pos); // 投影到第一轨迹点的x偏差
    ry = Project_PosTo1st_ry(Planned_Path[curSt[0].cur_tra_nth],
                             temp_pos); // 投影到第一轨迹点的y偏差

    if (fabsf(r) < 2.0f) // 路径是直线
    {
        curSt[0].de_dt = -ry; // y方向偏差变化率
    }
    else // 路径是曲线
    {
        float d        = sqrtf(rx * rx + (r - ry) * (r - ry)); // 当前点到路径圆心的距离
        curSt[0].de_dt = (d - fabsf(r)) * sign(r);             // 距离误差变化率
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // 更新累积偏差
    if (fabsf(curSt[0].cvel) < 0.01f)
    {
        curSt[0].sum_de = 0; // 车辆即将停止，重置累积偏差
    }
    else if (fabsf(curSt[0].de) < DE_GAP) // 偏差小于容差，慢慢减小累积偏差
    {
        if (curSt[0].sum_de > 1)
            curSt[0].sum_de = curSt[0].sum_de * 0.9f;
        else
            curSt[0].sum_de = 0;
    }
    else // 偏差较大，更新累积偏差
    {
        if (fabsf(curSt[0].de - curSt[0].de_last) > 0.01f)
        {
            curSt[0].sum_de =
                curSt[0].sum_de + 1.5 * fabsf(curSt[0].cur_vel) *
                                      (curSt[0].de - sign(curSt[0].de) * DE_GAP);
        }
        else
        {
            curSt[0].sum_de =
                curSt[0].sum_de +
                fabsf(curSt[0].cur_vel) * (curSt[0].de - sign(curSt[0].de) * DE_GAP) +
                fabsf(curSt[0].cur_vel) * K2_MDB * (curSt[0].de_dt - curSt[0].de);
        }
    }

    // 限制累积偏差的范围
    curSt[0].sum_de =
        rte_min(rte_max(curSt[0].sum_de, -MAXEPS_MDB / K2_MDB), MAXEPS_MDB / K2_MDB);
    curSt[0].deps    = K2_MDB * curSt[0].sum_de;
    curSt[0].de_last = curSt[0].de;

    // 更新转向请求
    if (curSt[0].B_BrakeState == 0 &&
        RTE_BSW_Get_CurrentGear() != GEAR_REAL_P) // 刹车状态为0且车辆不是P档时更新
    {
        if (ds * curSt[0].rxs < -0.0001f && fabs(r) > 2)
        {
            curSt[0].Rou_req = 1.0 / r;
        }
        else if (ds * curSt[0].rxf < -0.0001f) // 如果超过停止点则不更新
        {
            curSt[0].Rou_req = -curSt[0].rthetaf / curSt[0].rxf;
        }
    }

    curSt[0].EPS_req =
        PK_Rou_to_EPS(curSt[0].Rou_req, curSt[0].cur_vel); // 计算电动助力转向请求
    float cur_ceps = curSt[0].deps + curSt[0].EPS_req;     // 当前转向角

    // 限制转向角范围
    cur_ceps = rte_min(rte_max(cur_ceps, -ceps_max), ceps_max);
    if (fabsf(curSt[0].ceps - cur_ceps) > d_ceps_max)
    {
        curSt[0].ceps = cur_ceps;
    }
    else
    {
        curSt[0].ceps = 0.6f * cur_ceps + 0.4f * curSt[0].ceps;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // 计算控制速度
    int i = 0;
    for (i = curSt[0].cur_tra_nth; i < curSt[0].tra_num; i++)
    {
        if (i == curSt[0].tra_num - 1) // 最后一段轨迹
        {
            break;
        }
        else if (Planned_Path[i][3] * Planned_Path[i + 1][3] < 0) // 找到路径中的停车点
        {
            break;
        }
    }

    if (curSt[0].cur_tra_nth == i)
    {
        curSt[0].rx_brake = curSt[0].rxf; // 设置刹车点
    }
    else
    {
        PK_Get_Path_EndPos(Planned_Path[i], temp_pos);             // 获取路径结束点位置
        float rx = Project_PosTo1st_rx(temp_pos, curSt[0].curpos); // 投影到结束点的x偏差
        float dist =
            Cal_Dis_Pt2Pt(temp_pos, curSt[0].curpos); // 计算当前位置到结束点的距离
        curSt[0].rx_brake = (fabs(rx) < dist * 0.75) ? dist * sign(rx) : rx;
    }

    // 如果需要重新规划路径且速度大于0.25，限制刹车点
    if (PathExec_IsUpdateReplan(curSt) && fabs(curSt[0].cur_vel) > 0.25)
    {
        float dist = rte_min(fabs(curSt[0].cur_vel) * 0.8, 0.6);
        curSt[0].rx_brake =
            rte_min(fabs(curSt[0].rx_brake), dist) * sign(curSt[0].rx_brake);
    }
}

/**
 * @brief 执行路径参数计算
 *
 * 该函数计算自动驾驶过程中需要的参数，并根据当前状态调整车速和制动参数。
 *
 * @param Planned_Path 预定路径的数组，包含多个路径点，每个路径点有多个参数
 * @param curSt 当前自动驾驶参数的结构体数组，包含当前状态和控制信息
 * @return void 无返回值
 */
static void PathExec_ParaCal(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                             PK_Cur_AutodrPara curSt[1])
{
    const float cvel_base     = 0.05f; // 最低允许速度控制
    const float slot_range_ry = 1.5f;  // 靠近停车位的范围
    const float rx_danger_ds  = 4.8f;  // 危险距离阈值，临时修改为4.8以支持B过程
    const volatile float drx_brake_p1_ccp = 0.5462f;  // 制动距离曲线参数1
    const volatile float drx_brake_p2_ccp = 0.1173f;  // 制动距离曲线参数2
    const volatile float drx_brake_p3_ccp = 0.02413f; // 制动距离曲线参数3

    const float spd_control_dist = 0.8;                 // 速度控制距离
    float cfgvel                 = fabs(curSt[0].cvel); // 当前速度的绝对值

    // 如果车距超过控制距离，并且不在停车位附近或远离危险距离
    if ((fabsf(curSt[0].rx_brake) > spd_control_dist) &&
        (fabsf(curSt[0].ry_cur_to_fin) > slot_range_ry ||
         fabsf(curSt[0].rx_cur_to_fin) > rx_danger_ds))
    {
        cfgvel = 0.65f; // 设置速度为停车控制速度
    }
    else
    {
        // 如果车辆接近停车点，则减少速度，但这种方法不适用于平行泊车路径
        if (fabsf(curSt[0].rx_brake) < spd_control_dist)
        {
            if (cfgvel > 0.4)
            {
                cfgvel = 0.99 * cfgvel; // 减小速度
            }
            else
            {
                // 根据距离调整速度，确保速度不低于最低值
                cfgvel     = rte_min(0.996 * cfgvel, 0.4);
                float dist = fabs(curSt[0].rx_brake);
                cfgvel     = 0.782 * pow3(dist) +
                         1.636 * dist * pow2(spd_control_dist - dist) +
                         1.152 * pow2(dist) * (spd_control_dist - dist) +
                         cvel_base * pow3(spd_control_dist - dist);
                cfgvel = rte_max(cfgvel, cvel_base);
            }
        }
        else
        {
            // 调整速度，确保在合理范围内
            cfgvel = rte_max(rte_min(0.996 * cfgvel, 0.5), 0.4);
        }
    }

    // 设置当前状态的速度和曲率半径
    curSt[0].cvel   = sign(Planned_Path[curSt[0].cur_tra_nth][3]) * cfgvel;
    curSt[0].radius = Planned_Path[curSt[0].cur_tra_nth][4];

    // 如果当前速度大于某个阈值且速度方向一致，计算制动距离
    if (fabsf(curSt[0].cvel) > 0.001f && curSt[0].cur_vel * curSt[0].cvel > 0)
    {
        curSt[0].drx_brake = drx_brake_p3_ccp +
                             drx_brake_p1_ccp * curSt[0].cur_vel * curSt[0].cur_vel +
                             drx_brake_p2_ccp * fabsf(curSt[0].cur_vel);
    }
    else
    {
        curSt[0].drx_brake = 0.0f;
    }
}

static int CheckUpdatePathPos(const float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                              const PK_Cur_AutodrPara curSt[1])
{
    // 临近车位框不要再重规划
    auto IsStopReplanPos = [](const float pos[3]) {
        const float valid_alue = 30 * PI_RAD;
        return (pos[0] < 1.2f * VEHICLE_LEN && fabs(pos[2]) < valid_alue &&
                fabs(pos[1]) < VEHICLE_WID * 0.3f);
    };

    float rsStart[3];
    CoordinadteTransfer(curSt[0].finpos, curSt[0].curpos, rsStart);

    if (IsStopReplanPos(rsStart))
    {
        return 0;
    }

    int pathId = curSt[0].cur_tra_nth + 1;
    while (pathId < curSt[0].tra_num)
    {
        CoordinadteTransfer(curSt[0].finpos, plannedPath[pathId], rsStart);
        if (IsStopReplanPos(rsStart))
        {
            break;
        }
        pathId++;
    }

    return (pathId - curSt[0].cur_tra_nth);
}

static void PathExec_UpdatePath(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                PK_Cur_AutodrPara curSt[1])
{
    int startIdx = curSt[0].cur_tra_nth;
    if (curSt[0].tra_num - startIdx <= 0)
    {
        return;
    }

    int pathNum = curSt[0].tra_num - startIdx;
    float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    memcpy(updatePath, &Planned_Path[startIdx][0],
           pathNum * sizeof(float) * TRAJITEM_LEN);

    if (curSt[0].replanSt == PARK_NOUPDATE)
    {
        return;
    }

    if (PathExec_IsVehStill() && !PathExec_ObjDelayTimeOut(curSt))
    {
        return;
    }

    PlanInfoType replan;
    RTE_PK_PathPlan_Get_Park_PlanInfo(&replan);
    if (replan.Path_num <= 0)
    {
        return;
    }

    int count1, count2;
    count1 = count2 = 0;
    if (replan.IsDirConect == PARK_SWITCH)
    {
        count1 = PathExec_RevertCount(replan.Act_traj, replan.Path_num);
        count2 = PathExec_RevertCount(updatePath, pathNum);
        if (PathExec_IsSideUpdateTarg(curSt))
        {
            return;
        }
    }

    if ((count2 > 0 && count1 < count2) ||
        (replan.IsDirConect == PARK_DANGER_SWITCH && !PathExec_IsParkUpdate(curSt)))
    {
        if (replan.Act_traj[0][3] * updatePath[0][3] < 0)
        {
            return;
        }

        auto dangerSt = PK_ObjAvoid_Dir();
        if (replan.IsDirConect == PARK_DANGER_SWITCH && dangerSt == 0)
        {
            PRINT_SWITCH_INFO("PARK_DANGER_SWITCH Check");
            return;
        }
        // if (replanResult == 0 && (Planned_Path[startIdx][3] > 0 && dangerSt != 2) {
        // return;  } if (replanResult == 0 && (Planned_Path[startIdx][3] < 0 && dangerSt
        // != 1) {  return;  }

        char info[128];
        snprintf(info, sizeof(info),
                 "PathExec_PathSwitchOther switch %d,%d,%d,path:%d,danger:%d", count1,
                 count2, replan.IsDirConect, startIdx, dangerSt);
        PRINT_SWITCH_INFO(info);
        PathExec_PrintPathInfo(updatePath, pathNum);
        PathExec_PrintPathInfo(replan.Act_traj, replan.Path_num);
        PathExec_PrintObjInfo();

        if (cfg_updatepath_stat == 1)
        {
            int newPathNum = SmoothNewTraj(replan, curSt[0].curpos, Planned_Path);
            float enviObj[5][4];
            SlotObj_Convert_float(replan.slotObjs, enviObj); // B 过程会更新谁

            float point[8];
            Avm_Pot_T avmpoint;
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmpoint);
            PathExec_AvmSlotToPoint(avmpoint, point);
            PathExec_PrintSlotInfo(enviObj, curSt[0].finpos, point); // 日志
            ReplanPathPost(Planned_Path, newPathNum, enviObj, curSt[0].finpos, curSt);
            if (replan.IsDirConect == PARK_DANGER_SWITCH)
            {
                PathExec_SetParkUpdate(curSt);
            }
        }
    }
}

/**
 * @brief 更新泊车目标点
 *
 *
 *
 * @param curSt 当前泊车模块参数结构体
 * @param enviObj 车位边界
 * @param newTarget 新目标点
 * @param update 是否更新
 * @param swell 安全预留距离
 * @return bool 是否更新成功
 */
bool PathExec_UpdateTarget(PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                           float newTarget[3], bool update, int flag, const float swell)
{
    int sensorType = RTE_PK_DataConvt_Get_IsAVM_Slot();
    if (sensorType == 2)
    {
        return false;
    }

    if (curSt[0].slotshape >= PK_SLOT_LEFT_VERT && PathExec_IsRearDanger(curSt) &&
        fabs(curSt[0].rtheta_cur_to_fin) < 10.0f * PI_RAD)
    {
        U_RadarType uradar;
        RTE_PD_Get_U_Radar(&uradar); // 超声波实时反馈数据

        float uRadarDist = (uradar.RSR + uradar.RSL) / 1000.0f;
        if ((curSt[0].rx_cur_to_fin < REAR_SUSPENSION && uRadarDist < 0.75f) ||
            (curSt[0].rx_cur_to_fin < (VEHICLE_LEN - 0.50f) && uRadarDist < 0.65f))
        {
            PathExec_PrintPathSwitch("Narrow Slot No UpdateTarget", flag);
            return false;
        }
    }

    Avm_Pot_T avmSlot;
    float avmpoint[8];

    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);
    PathExec_GetAvmPoint(avmpoint);
    PathExec_PrintPathSwitch("PathExec_UpdateTarget", flag);
    PathExec_PrintSlotInfo(enviObj, curSt[0].finpos, avmpoint);
    if (sensorType == 1)
    {
        PathExec_UpdateAvmPoint(curSt, avmSlot);
        PathExec_AvmSlotToPoint(avmSlot, avmpoint);
    }

    SlotObj_T slotObj;
    SlotObj_Convert_struct(&slotObj, enviObj);
    // printf("PathExec_UpdateTarget: enviObj: %f %f %f %f %f %f %f %f %f %f %f %f\n",
    //     slotObj.ptA.x, slotObj.ptA.y,slotObj.ptB.x, slotObj.ptB.y, slotObj.ptC.x,
    //     slotObj.ptC.y, slotObj.ptD.x, slotObj.ptD.y,slotObj.ptE.x,
    //     slotObj.ptE.y,slotObj.ptF.x, slotObj.ptF.y);

    PathExec_PrintObjInfo();
    auto result = PathExec_UpdateSlotByObjNearSlot(curSt[0].curpos, curSt[0].finpos,
                                                   avmSlot, slotObj); // 更新目标车位信息
    if (PathExec_BYIdentify(curSt) || PathExec_EYIdentify(curSt))
    {
        SlotObj_T tempObj;
        SlotObj_Convert_struct(&tempObj, enviObj);
        if (PathExec_BYIdentify(curSt))
        {
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptA,
                             (float *)&slotObj.ptA);
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptB,
                             (float *)&slotObj.ptB);
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptC,
                             (float *)&slotObj.ptC);
        }

        if (PathExec_EYIdentify(curSt))
        {
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptD,
                             (float *)&slotObj.ptD);
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptE,
                             (float *)&slotObj.ptE);
            PathExec_UpdateY(curSt[0].finpos, (float *)&tempObj.ptF,
                             (float *)&slotObj.ptF);
        }
    }
    curSt[0].targState = result;

    VehPos_T targpos;
    memcpy(&targpos, curSt[0].finpos, sizeof(targpos));
    Calc_Target(sensorType, curSt[0].targState, slotObj, avmSlot,
                targpos); // 计算车辆停车的目标位置.

    float newEnvi[5][4], rsMid[3];
    SlotObj_Convert_float(slotObj, newEnvi);
    PathExec_PrintSlotInfo(newEnvi, (float *)&targpos, avmpoint);

    CoordinadteTransfer(curSt[0].finpos, (float *)&targpos, rsMid);
    if (fabs(rsMid[1]) < 0.05f)
    {
        auto alarmState = PathExec_SideRadarAlarmObjState();
        float line[4];
        memcpy(&line[0], (float *)&slotObj.ptB, sizeof(float) * 2);
        memcpy(&line[2], (float *)&slotObj.ptC, sizeof(float) * 2);
        float distToBC = PK_PointToLineDist(line, (float *)&targpos);

        memcpy(&line[0], (float *)&slotObj.ptD, sizeof(float) * 2);
        memcpy(&line[2], (float *)&slotObj.ptE, sizeof(float) * 2);
        float distToDE = PK_PointToLineDist(line, (float *)&targpos);
        float ery      = Project_PosTo1st_ry(curSt[0].finpos, (float *)&slotObj.ptD);
        if ((alarmState & 0x0f) != 0)
        {
            if (sign(ery) < 0.01f && distToBC < distToDE) // 左边B告警
            {
                if (distToDE > 0.50f)
                {
                    rsMid[1] = ((alarmState & 0x02) != 0 || (alarmState & 0x08) != 0)
                                   ? -0.08f
                                   : -0.055f;
                }

                if (distToDE > 0.55f)
                {
                    rsMid[1] = ((alarmState & 0x02) != 0 || (alarmState & 0x08) != 0)
                                   ? -0.10f
                                   : -0.07f;
                }
            }

            if (sign(ery) > 0.01f && distToBC > distToDE) // 左边E告警
            {
                if (distToBC > 0.50f)
                {
                    rsMid[1] = ((alarmState & 0x02) != 0 || (alarmState & 0x08) != 0)
                                   ? -0.08f
                                   : -0.055f;
                }

                if (distToBC > 0.55f)
                {
                    rsMid[1] = ((alarmState & 0x02) != 0 || (alarmState & 0x08) != 0)
                                   ? -0.10f
                                   : -0.07f;
                }
            }
        }
        else if ((alarmState & 0xf0) != 0)
        {
            if (sign(ery) > 0.01f && distToBC < distToDE) // 右边B告警
            {
                if (distToDE > 0.50f)
                {
                    rsMid[1] = ((alarmState & 0x20) != 0 || (alarmState & 0x80) != 0)
                                   ? 0.08f
                                   : 0.055f;
                }

                if (distToDE > 0.55f)
                {
                    rsMid[1] = ((alarmState & 0x20) != 0 || (alarmState & 0x80) != 0)
                                   ? 0.10f
                                   : 0.07f;
                }
            }

            if (sign(ery) < 0.01f && distToBC > distToDE) // 右边E告警
            {
                if (distToBC > 0.50f)
                {
                    rsMid[1] = ((alarmState & 0x20) != 0 || (alarmState & 0x80) != 0)
                                   ? 0.08f
                                   : 0.055f;
                }

                if (distToBC > 0.55f)
                {
                    rsMid[1] = ((alarmState & 0x20) != 0 || (alarmState & 0x80) != 0)
                                   ? 0.10f
                                   : 0.07f;
                }
            }
        }

        if (fabs(rsMid[1]) >= 0.05f)
        {
            float theta = targpos.theta;
            Convert(curSt[0].finpos, rsMid, (float *)&targpos);
            targpos.theta = theta;
        }
    }

    if (!update && fabs(rsMid[1]) < swell && fabs(rsMid[2]) < 0.015)
    {
        return false;
    }

    // printf("PathExec_UpdateTarget: enviObj: %f %f %f %f %f %f %f %f %f %f %f %f\n",
    //     slotObj.ptA.x, slotObj.ptA.y,slotObj.ptB.x, slotObj.ptB.y, slotObj.ptC.x,
    //     slotObj.ptC.y, slotObj.ptD.x, slotObj.ptD.y,slotObj.ptE.x,
    //     slotObj.ptE.y,slotObj.ptF.x, slotObj.ptF.y);

    if (sensorType == 1)
    {
        // printf("PathExec_UpdateTarget: point: %f %f %f %f %f %f %f %f\n",
        //     avmSlot.near_rear.x, avmSlot.near_rear.y, avmSlot.far_rear.x,
        //     avmSlot.far_rear.y, avmSlot.far_front.x, avmSlot.far_front.y,
        //     avmSlot.near_front.x, avmSlot.near_front.y);
    }

    printf("PathExec_UpdateTarget: old: %f %f %f new: %f %f %f offset: %f %f %f %d\n",
           curSt[0].finpos[0], curSt[0].finpos[1], curSt[0].finpos[2], targpos.x,
           targpos.y, targpos.theta, rsMid[0], rsMid[1], rsMid[2], update);

    memcpy(newTarget, (float *)&targpos, sizeof(targpos));
    SlotObj_Convert_float(slotObj, enviObj);
    PathExec_PrintSlotInfo(enviObj, newTarget, avmpoint);
    return true;
}

/**
 * @brief 执行路径切换的函数
 *
 * 此函数根据当前状态和环境信息决定是否切换路径，并更新车辆的相关参数和路径信息。
 *
 * @param plannedPath 当前规划的路径数组
 * @param curSt 当前车辆自动驾驶参数的状态数组
 * @param enviObj 环境对象数组
 * @param swell 膨胀系数
 */
static void PathExec_PathSwitch(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                                float swell)
{
    const float long_path             = 0.8f;  //  can't ignore a long path
    const float drx_brake_to_stop     = 0.06f; // 停车制动距离
    const float rxf_to_stop           = 0.06f; // 停车位置误差
    const int B_FirstPlan_StopWaitCnt = 4;     // 初始计划等待计数
    const float B_DynaPlan_de_min     = 0.2f;  // 动态规划最小距离误差
    // const float B_FirstPlan_de_min = 0.04f;  // 初始计划最小距离误差
    // const float sum_de_rep_min = 2.0f;  // 累积误差最小值

    float r  = plannedPath[curSt[0].cur_tra_nth][4];     // radias
    float ds = plannedPath[curSt[0].cur_tra_nth][3];     // distance
    float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]; // 更新后的路径

    float trajEnd[3]; // 当前路径段的终点位置
    PK_Get_Path_EndPos(plannedPath[curSt[0].cur_tra_nth],
                       trajEnd); // 获取当前路径段的终点位置
    curSt[0].leftDist =
        Cal_Dis_Pt2Pt(trajEnd, curSt[0].curpos); // 计算当前位置到路径段终点的距离

    //////////////////////////////////////////////////////////////////////
    //  2.different tra state detect -> this part should be set behind the update of cvel
    if (curSt[0].cur_tra_nth >= curSt[0].tra_num - 1) // last traject
    {
        curSt[0].drx_crou      = 0.0f;
        curSt[0].cur_tra_state = 0; // not continuous trajs
    }
    else if (plannedPath[curSt[0].cur_tra_nth][3] *
                 plannedPath[curSt[0].cur_tra_nth + 1][3] <
             0) // ds*ds_next<0 change direction
    {
        curSt[0].drx_crou      = 0.0f;
        curSt[0].cur_tra_state = 0; // not continuous trajs
    }
    else
    {
        curSt[0].cur_tra_state = 1;                             //  continuous trajs
        float rnext = plannedPath[curSt[0].cur_tra_nth + 1][4]; // 下一段路径转弯半径
        if (fabsf(curSt[0].rxf) < long_path)
        {
            curSt[0].drx_crou = PathExec_ContinueR(curSt, r, rnext);
        }
    }

    // 6. Path control
    if (curSt[0].cur_tra_nth == curSt[0].tra_num - 1) // Para parking only
    {
        if (curSt[0].slotshape == PK_SLOT_LEFT_PARA ||
            curSt[0].slotshape == PK_SLOT_RIGHT_PARA) // 平行
        {
            if ((fabsf(curSt[0].rxf) < 2.0f * dfradioGap_allow &&
                 fabsf(curSt[0].rthetaf) < thetaGap_allow * 1.5) ||
                (curSt[0].cvel * curSt[0].rxf > ZERO_FLOAT &&
                 fabsf(curSt[0].rxf) >
                     ZERO_FLOAT)) // 20181015:rtheta allow error tune big
            {
                if (sign(curSt[0].cvel) * sign(curSt[0].rxf) > ZERO_FLOAT)
                {
                    if (curSt[0].brakeSt == 0)
                    {
                        curSt[0].brakeSt = 1;
                    }
                    curSt[0].B_BrakeState++;
                }

                if (PathExec_IsVehStill()) // 如果车辆静止
                {
                    if ((fabsf(curSt[0].ry_cur_to_fin) > 2.0f * dfradioGap_allow ||
                         fabsf(curSt[0].rthetaf) > thetaGap_allow * 1.5) &&
                        (RTE_PK_SM_GetApaMode() != PARKOUT_MODE))
                    {
                        PRINT_SWITCH_INFO(
                            "PathExec_B_MoveOut_ParaRePlan last Danger_replan!");
                        PathExec_SetLastReplan(curSt, replan_start);
                        // PathExec_UpdateTarget(curSt, enviObj, curSt[0].finpos, false,
                        // __LINE__);
                        int trajNum = ReplanPathPre(plannedPath, enviObj, curSt[0].finpos,
                                                    0, curSt, swell); // 重规划
                        if (trajNum > 0)
                        {
                            ReplanPathPost(plannedPath, trajNum, enviObj, curSt[0].finpos,
                                           curSt);
                            return;
                        }
                        else
                        {
                            curSt[0].B_ReplanResult = PATHEXECUTE_FAIL; // B
                                                                        // 过程重规划失败
                        }
                    }

                    PRINT_SWITCH_INFO("PathExec_PathSwitch para slot switch to last");
                    PathExec_SkipToLastTraj(curSt); // 跳到最后一段路径
                }
            }
            return;
        }
        else // 垂直
        {
            if ((sign(curSt[0].cvel) * sign(curSt[0].rx_cur_to_fin) > ZERO_FLOAT &&
                 fabsf(curSt[0].cvel) > ZERO_FLOAT &&
                 fabsf(curSt[0].leftDist) < rte_min(rxf_to_stop * 5, 0.6)) ||
                (curSt[0].B_BrakeState > 0) ||
                (fabsf(curSt[0].rx_cur_to_fin) < 2.0f * dfradioGap_allow &&
                 fabsf(curSt[0].rthetaf) < thetaGap_allow * 1.5))
            {
                if (sign(curSt[0].cvel) * sign(curSt[0].rx_cur_to_fin) > 0.0f)
                {
                    if (curSt[0].brakeSt == 0)
                    {
                        curSt[0].brakeSt = 1;
                    };
                    curSt[0].B_BrakeState++;
                }

                if (curSt[0].B_BrakeState > B_FirstPlan_StopWaitCnt ||
                    curSt[0].stillCount > B_FirstPlan_StopWaitCnt)
                {
                    PRINT_SWITCH_INFO("PathExec_PathSwitch vert slot stop");
                    PathExec_UpdateLastCVel(curSt, plannedPath[curSt[0].cur_tra_nth]);
                    PathExec_ContinueInvTraj(curSt); // 继续倒车路径
                }

                return;
            }
        }
    }

    if (PathExec_IsStopUpdateTarg(curSt) &&
        curSt[0].rx_cur_to_fin < 1.2 * REAR_SUSPENSION &&
        fabs(curSt[0].rtheta_cur_to_fin) < 15.0f * PI_RAD && curSt[0].cvel < 0)
    {
        Avm_Obj_T stopbar;
        RTE_PD_Get_AVM_SlotObs_B(&stopbar);

        // 调节方式:
        // 使用视觉更新，配置参数AVM_Vert_TargPos_off_ldy/rdy是在视觉更新后的基础上再调整
        // AVM_Vert_TargPos_off_ldy/rdy 都是以终点位置为坐标原点, 车头方向为X轴正向.
        // 正值表示向前调节, 负值表示向后调节
        float AVM_Vert_TargPos_off_dy = 0;
        float avmTarg[2] = {0}, rsMid[3] = {0};
        if (stopbar.slot_index > 0 && PathExec_IsUpdateStopBar())
        {
            Avm_Pot_T avmSlot;
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);
            if (avmSlot.AlignIndex[0] == 0)
            {
                avmSlot.AlignIndex[0] = 1;
                curSt[0].hasStop      = 0x01;
                RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&avmSlot);
            }

            if (Slot_Dir(curSt[0].slotshape) == 1)
            {
                AVM_Vert_TargPos_off_dy = AVM_Vert_TargPos_off_ldy;
            }
            else
            {
                AVM_Vert_TargPos_off_dy = AVM_Vert_TargPos_off_rdy;
            }

            avmTarg[0] = (stopbar.ls.pt1.x + stopbar.ls.pt2.x) * 0.5;
            avmTarg[1] = (stopbar.ls.pt1.y + stopbar.ls.pt2.y) * 0.5;
            RevConvert(curSt[0].finpos, avmTarg, rsMid);
        }
        else
        {
            rsMid[0] = rsMid[1] = rsMid[2] = 0.0f;
        }

        rsMid[0] = rsMid[0] + AVM_Vert_TargPos_off_dy;
        char errorInfo[256];
        // curSt[0].rx_cur_to_fin < 1.2 * REAR_SUSPENSION 和0.9的关系
        if ((rsMid[0] > curSt[0].rx_cur_to_fin && rsMid[0] > 0) ||
            fabs(rsMid[0]) > 0.9f || fabs(rsMid[0]) < 0.001)
        {
            if (fabs(rsMid[0]) > 0.001)
            {
                sprintf(
                    errorInfo,
                    "No UpdateTarget By stopbar! Stop! rsMid[2]:%f,%f, rx_cur_to_fin %f",
                    rsMid[2], rsMid[0], curSt[0].rx_cur_to_fin);
                PRINT_SWITCH_INFO(errorInfo);
            }
        }
        else
        {
            bool flag = 0;
            // 超过最后路径的长度, 保持最后路径的方向
            if (plannedPath[curSt[0].tra_num - 1][3] *
                    (plannedPath[curSt[0].tra_num - 1][3] + rsMid[0]) <
                ZERO_FLOAT)
            {
                plannedPath[curSt[0].tra_num - 1][3] =
                    0.01f * sign(plannedPath[curSt[0].tra_num - 1][3]);
                flag = 1;
            }
            else
            {
                plannedPath[curSt[0].tra_num - 1][3] =
                    plannedPath[curSt[0].tra_num - 1][3] + rsMid[0];
            }

            PK_Get_Path_EndPos(plannedPath[curSt[0].tra_num - 1], curSt[0].finpos);
            RTE_PK_SlotDetect_Set_TargPos(curSt[0].finpos);
            PathExec_SetStopUpdateTarg(curSt);
            sprintf(errorInfo, "UpdateTarget By stopbar! rsMid %f, rx_cur_to_fin %f %d",
                    rsMid[0], curSt[0].rx_cur_to_fin, flag);
            PRINT_SWITCH_INFO(errorInfo);
        }
    }

    float newTarget[3], newEnviObj[5][4];
    PK_CopyPos(newTarget, curSt[0].finpos);
    memcpy(newEnviObj, enviObj, sizeof(newEnviObj));
    int trajNum            = 0;
    const float stop_speed = 0.15f;
    if (ds > 0.0f)
    {
        if (curSt[0].cur_tra_state > 0)
        {
            if (curSt[0].rxf > -curSt[0].drx_crou && fabsf(curSt[0].rxf) < long_path)
            {
                PRINT_SWITCH_INFO("PathExec_ParaCal slot continue forword");
                PathExec_ContinueNextTraj(curSt);
            }
        }
        else if (curSt[0].step_gone > 3) // only for static path
        {
            if ((curSt[0].rxf > -curSt[0].drx_brake) ||
                (curSt[0].drx_brake < drx_brake_to_stop &&
                 fabsf(curSt[0].rxf) < rxf_to_stop) ||
                (curSt[0].B_BrakeState > 0)) // Ready brake
            {
                curSt[0].cvel = sign(curSt[0].cvel) * rte_min(fabs(curSt[0].cvel), 0.10);
                if (curSt[0].brakeSt == 0)
                {
                    curSt[0].brakeSt = 1;
                };
                curSt[0].B_BrakeState++;

                if (curSt[0].B_BrakeState > B_FirstPlan_StopWaitCnt &&
                    fabs(curSt[0].cur_vel) < stop_speed) //{  curSt[0].ceps = 0.0;  }
                {
                    PRINT_SWITCH_INFO("PathExec_ParaCal vert slot stop forword");
                    PathExec_UpdateLastCVel(curSt, plannedPath[curSt[0].cur_tra_nth]);
                    PathExec_ContinueInvTraj(curSt);
                    curSt[0].percent = -1.0;
                    if (curSt[0].slotshape > PK_SLOT_RIGHT_PARA &&
                        RTE_PK_SM_GetApaMode() != PARKOUT_MODE)
                    {
                        int i = curSt[0].cur_tra_nth;
                        while (i < curSt[0].tra_num && plannedPath[i][3] < 0)
                        {
                            i++;
                        }; // 疑问
                        if (i < curSt[0].tra_num || PathExec_IsLastAdjust(curSt))
                        {
                            return;
                        }

                        if (PathExec_IsSideUpdateTarg(curSt))
                        {
                            PathExec_UpdateTarget(curSt, newEnviObj, newTarget, false,
                                                  __LINE__);
                        }

                        PRINT_SWITCH_INFO("PathExec_ParaCal backword update");
                        trajNum = PathExec_BetterTra(curSt[0].curpos, newTarget, 3,
                                                     &newEnviObj[1], updatePath,
                                                     side_swell_0, 0.1);
                        if (trajNum <= 0 && PathExec_IsSideUpdateTarg(curSt))
                        {
                            PRINT_SWITCH_INFO(
                                "PathExec_B_MoveOutRear vert slot stop forword");
                            trajNum = PathExec_B_MoveOutRear(updatePath, newTarget,
                                                             newEnviObj, curSt[0].curpos);
                        }

                        PathExec_ClrSideUpdateTarg(curSt);
                    }
                }
            }
        }
    }
    else
    {
        if (curSt[0].cur_tra_state > 0) //  for continuous curvature change path
        {
            if (curSt[0].rxf < curSt[0].drx_crou && fabsf(curSt[0].rxf) < long_path)
            // if (fabs(curSt[0].drx_crou1) < 0.10 && fabsf(curSt[0].rxf) < long_path)
            {
                PRINT_SWITCH_INFO("PathExec_ParaCal slot continue backword");
                PathExec_ContinueNextTraj(curSt);
            }
        }
        else if (curSt[0].step_gone > 3) // only for static path
        {
            if ((curSt[0].rxf < curSt[0].drx_brake) ||
                (curSt[0].drx_brake < drx_brake_to_stop &&
                 fabsf(curSt[0].rxf) < rxf_to_stop) ||
                (curSt[0].B_BrakeState > 0)) // Ready break
            {
                curSt[0].cvel = sign(curSt[0].cvel) * rte_min(fabs(curSt[0].cvel), 0.10);
                if (curSt[0].brakeSt == 0)
                {
                    curSt[0].brakeSt = 1;
                }
                curSt[0].B_BrakeState++;
                if (curSt[0].B_BrakeState > B_FirstPlan_StopWaitCnt &&
                    fabs(curSt[0].cur_vel) < stop_speed) //{    curSt[0].ceps = 0.0;    }
                {
                    PRINT_SWITCH_INFO("PathExec_ParaCal slot stop backword");
                    PathExec_UpdateLastCVel(curSt, plannedPath[curSt[0].cur_tra_nth]);
                    PathExec_ContinueInvTraj(curSt);

                    curSt[0].percent = -1.0;

                    if (RTE_PK_SM_GetApaMode() == PARKOUT_MODE)
                    {
                        return;
                    }

                    // if (curSt[0].slotshape > 2 && curSt[0].B_FirstPlanFlag==1)//vert
                    // only
                    if (curSt[0].slotshape > PK_SLOT_RIGHT_PARA)
                    {
                        bool update = PathExec_UpdateTarget(curSt, newEnviObj, newTarget,
                                                            false, __LINE__, 0.04f);

                        int i = curSt[0].cur_tra_nth;
                        while (i < curSt[0].tra_num && plannedPath[i][3] > 0)
                        {
                            i++;
                        };
                        bool lastTraj = (i >= curSt[0].tra_num);
                        if (!lastTraj)
                        {
                            while (i < curSt[0].tra_num && plannedPath[i][3] < 0)
                            {
                                i++;
                            }
                            lastTraj = (i >= curSt[0].tra_num);
                        }

                        if (!update && !lastTraj &&
                            (fabsf(curSt[0].de) <= B_DynaPlan_de_min ||
                             PathExec_IsLastAdjust(curSt)))
                        {
                            return;
                        }

                        PRINT_SWITCH_INFO(
                            "PathExec_B_MoveOutFront vert slot update path");
                        trajNum =
                            PathExec_B_MoveOutFront(updatePath, curSt[0].finpos, enviObj,
                                                    curSt[0].curpos, curSt[0].slotshape);
                    }
                    else if (curSt[0].slotshape == PK_SLOT_LEFT_PARA ||
                             curSt[0].slotshape == PK_SLOT_RIGHT_PARA)
                    {
                        int i = curSt[0].cur_tra_nth;
                        while (i < curSt[0].tra_num && plannedPath[i][3] > 0)
                        {
                            i++;
                        };
                        if (i < curSt[0].tra_num || PathExec_IsLastAdjust(curSt))
                        {
                            return;
                        }

                        float tempTarg[3];
                        MoveLeftTrajsByStart(&plannedPath[curSt[0].cur_tra_nth],
                                             curSt[0].tra_num - curSt[0].cur_tra_nth,
                                             curSt[0].curpos, tempTarg);

                        float ry     = Project_PosTo1st_ry(curSt[0].finpos, tempTarg);
                        float rtheta = Project_PosTo1st_rtheta(curSt[0].finpos, tempTarg);
                        if (fabsf(ry) > 0.03 || fabsf(rtheta) > 0.02)
                        {
                            PRINT_SWITCH_INFO("PathPlan_BetterTra para slot update path");
                            trajNum =
                                PathExec_BetterTraFront(curSt[0].curpos, newTarget, 3,
                                                        &newEnviObj[1], updatePath);
                        }
                    }
                }
            }
        }
    }

    if (trajNum > 0)
    {
        SlotObj_T slotObj;
        SlotObj_Convert_struct(&slotObj, newEnviObj);
        RTE_PK_SlotDetect_Set_SlotObj(&slotObj);
        RTE_PK_SlotDetect_Set_TargPos(newTarget);
        SlotObj_Convert_float(slotObj, enviObj);
        PK_CopyPos(curSt[0].finpos, newTarget);
        memcpy(plannedPath, updatePath, trajNum * sizeof(float) * TRAJITEM_LEN);
        PRINT_SWITCH_INFO("PathExec_PathSwitch");
        PathExec_PrintPathInfo(plannedPath, trajNum);

        if (curSt[0].slotshape >= PK_SLOT_LEFT_VERT &&
            PathExec_StopReplan(curSt, enviObj, curSt[0].finpos) && plannedPath[0][3] > 0)
        {
            return;
        }
        PathExec_StartNewPlanPath(curSt, trajNum, plannedPath[0][3]);
    }
}

/**
 * @brief 最后重规划函数
 *
 * 该函数用于泊车过程中的最后重规划。当检测到泊车过程中存在危险或需要进行调整时，执行重新规划路径的操作。
 *
 * @param plannedPath 规划的路径数组
 * @param curSt 当前泊车模块参数结构体
 * @param enviObj 环境障碍物数组
 * @param swell 车辆膨胀参数
 */
static void PathExec_LastReplan(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                                float swell)
{
    PRINT_SWITCH_INFO("PathExec_B_MoveOut_ParaRePlan B_Danger_replan!");

    // 当当前轨迹编号大于等于总轨迹数且车位类型不为空时
    if (curSt[0].cur_tra_nth >= curSt[0].tra_num && curSt[0].slotshape != PK_SLOT_NO &&
        PathExec_ReplanSuccess(curSt) && !PathExec_IsPreTrajNoMove(curSt) &&
        !PathExec_IsLastAdjust(curSt) && !PathExec_IsLastReplan(curSt))
    {
        PRINT_SWITCH_INFO("ReplanPathPre B_Danger_replan!");

        // 设置危险重规划标志位
        PathExec_SetLastReplan(curSt, replan_start);
        // PathExec_UpdateTarget(curSt, enviObj, curSt[0].finpos, false, __LINE__);
        int trajNum = ReplanPathPre(plannedPath, enviObj, curSt[0].finpos, 0, curSt,
                                    swell); // 重新规划路径
        if (trajNum > 0)
        {
            ReplanPathPost(plannedPath, trajNum, enviObj, curSt[0].finpos,
                           curSt); // 重新规划成功后执行后续处理
        }
        else
        {
            curSt[0].B_ReplanResult = PATHEXECUTE_FAIL; // 重规划失败
            PathExec_SkipToLastTraj(curSt);             // 跳过到最后一个轨迹
        }
    }
}

/**
 * @brief PathExec_PathSwitchOther - 处理自动泊车的路径切换逻辑
 *
 * 该函数检查各种条件以确定车辆是否应切换到不同的路径，继续当前路径或退出停车。
 *
 * @param Planned_Path 规划的轨迹点二维数组。
 * @param curSt 指向当前自动泊车参数状态的指针。
 * @param enviObj 环境对象信息数组。
 * @param swell 车辆膨胀系数。
 */
static void PathExec_PathSwitchOther(
    float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], PK_Cur_AutodrPara curSt[1],
    float enviObj[5][4], float swell)
{
    // 检查车辆是否静止、切换延迟时间是否超时以及当前模式是否不是PARKOUT_MODE
    if (PathExec_IsVehStill() &&
        PathExec_SwitchDelayTimeOut(curSt)) // && RTE_PK_SM_GetApaMode() != PARKOUT_MODE)
    {
        // 1 有障碍物
        if ((PathExec_IsForward(curSt) && PathExec_IsFrontDanger(curSt)) ||
            (PathExec_IsBackward(curSt) && PathExec_IsRearDanger(curSt)))
        {
            if (PathExec_IsRearDanger(curSt) && PathExec_IsEndLastTraj(curSt))
            {
                PRINT_SWITCH_INFO("PathExec_MainControl exit by rear object!");
                PathExec_ParkingLeverUpdate(curSt, enviObj);
                PathExec_SkipToLastTraj(curSt);
                return;
            }

            if (!PathExec_IsSuspendByObjs(curSt) ||
                RTE_PK_SM_GetApaMode() == PARKOUT_MODE)
            {
                if (PathExec_IsRearDanger(curSt) && PathExec_IsEndLastTraj(curSt))
                {
                    PRINT_SWITCH_INFO("PathExec_MainControl exit by rear object!");
                    PathExec_ParkingLeverUpdate(curSt, enviObj);
                    PathExec_SkipToLastTraj(curSt);
                    return;
                }

                if (PathExec_IsRearDanger(curSt) &&
                    curSt[0].cur_tra_nth + 1 == curSt[0].tra_num)
                {
                    PRINT_SWITCH_INFO(
                        "PathExec_MainControl updateTarget by rear object!");
                    PathExec_ParkingLeverUpdate(curSt, enviObj);
                }

                PRINT_SWITCH_INFO("PathExec_MainControl slot switch for danger!");
                PathExec_StartNextInvTraj(Planned_Path, curSt);
                if (curSt[0].cur_tra_nth >= curSt[0].tra_num &&
                    !PathExec_NearToTarget(curSt) &&
                    RTE_PK_SM_GetApaMode() != PARKOUT_MODE)
                {
                    PathExec_LastReplan(Planned_Path, curSt, enviObj, swell);
                }
                return;
            }
        }
        else if (PathExec_IsParkingLever(curSt))
        {
            PRINT_SWITCH_INFO("PathExec_MainControl vert slot switch for ParkLevel!");
            PathExec_ParkingLeverUpdate(curSt, enviObj);
            PathExec_StartNextInvTraj(Planned_Path, curSt);
            if (curSt[0].cur_tra_nth >= curSt[0].tra_num && !PathExec_NearToTarget(curSt))
            {
                PathExec_LastReplan(Planned_Path, curSt, enviObj, swell);
            }

            return;
        }
        // 2 没有障碍物,
        // 不切换到下一条路径，例如因为1中强制切换到下一条路径，由于长度没有更新，如果还是按照原来的路径行驶，
        // 由于路径长度没有更新，会导致不能够走到制定的位置，因而可能不会触发切换到下一条路径
        else if (PathExec_IsSpeedBump(curSt))
        {
            if (curSt[0].stillCount >= path_dangerquit_time * 3)
            {
                PRINT_SWITCH_INFO("PathExec_MainControl speed bump vert/para switch!");
                curSt[0].stillCount = 0;
                PathExec_ContinueNextTraj(curSt);
                return;
            }
        }
        else if (!PathExec_IsFrontDanger(curSt) && !PathExec_IsRearDanger(curSt))
        {
            if (fabsf(curSt[0].rtheta_cur_to_fin) < thetaGap_allow &&
                fabs(curSt[0].rx_cur_to_fin) < rxGap_allow &&
                fabs(curSt[0].ry_cur_to_fin) < dfradioGap_allow &&
                curSt[0].stillCount >= 2 * path_dangerquit_time)
            {
                PathExec_ContinueNextTraj(curSt);
            }
            else if ((curSt[0].ds_gone_dir > 0.10f &&
                      curSt[0].stillCount >= rte_max(2 * path_dangerquit_time,
                                                     B_DangeReplanWaitCnt_ccp + 10)) ||
                     (curSt[0].leftDist < 0.20f &&
                      curSt[0].stillCount >= 0.5f * path_dangerquit_time))
            {
                PRINT_SWITCH_INFO("PathExec_MainControl no object vert/para switch!");
                curSt[0].stillCount = path_dangerquit_time;
                PathExec_ContinueNextTraj(curSt);
                if (curSt[0].cur_tra_nth >= curSt[0].tra_num &&
                    !PathExec_NearToTarget(curSt))
                {
                    PathExec_LastReplan(Planned_Path, curSt, enviObj, swell);
                }
                else
                {
                    PRINT_SWITCH_INFO("PathExec_ParaCal vert slot stop");
                    if (curSt[0].slotshape > PK_SLOT_RIGHT_PARA)
                    {
                        int i = curSt[0].cur_tra_nth;
                        while (i < curSt[0].tra_num && Planned_Path[i][3] < 0)
                        {
                            i++;
                        }; // 疑问
                        if (i < curSt[0].tra_num || PathExec_IsLastAdjust(curSt))
                        {
                            return;
                        }

                        float newTarget[3], newEnviObj[5][4];
                        float updatePath[MAX_SINGLE_TRAJ_NUM]
                                        [TRAJITEM_LEN]; // 更新后的路径
                        PK_CopyPos(newTarget, curSt[0].finpos);
                        memcpy(newEnviObj, enviObj, sizeof(newEnviObj));

                        PRINT_SWITCH_INFO("PathExec_ParaCal update");
                        int trajNum = 0;
                        if (RTE_PK_SM_GetApaMode() != PARKOUT_MODE)
                        {
                            if (PathExec_IsSideUpdateTarg(curSt))
                            {
                                PathExec_UpdateTarget(curSt, enviObj, newTarget, false,
                                                      __LINE__);
                            }

                            trajNum = PathExec_BetterTra(curSt[0].curpos, newTarget, 3,
                                                         &newEnviObj[1], updatePath,
                                                         side_swell_0, 0.1);
                            if (trajNum <= 0 && PathExec_IsSideUpdateTarg(curSt))
                            {
                                PRINT_SWITCH_INFO(
                                    "PathExec_B_MoveOutRear vert slot stop forword");
                                trajNum = PathExec_B_MoveOutRear(
                                    updatePath, newTarget, newEnviObj, curSt[0].curpos);
                            }

                            PathExec_ClrSideUpdateTarg(curSt);
                        }
                        else
                        {
                            trajNum = PathExec_BetterTra(curSt[0].curpos, newTarget, 3,
                                                         &newEnviObj[1], updatePath,
                                                         side_swell_0, 0.1);
                        }
                        if (trajNum > 0)
                        {
                            SlotObj_T slotObj;
                            SlotObj_Convert_struct(&slotObj, newEnviObj);
                            RTE_PK_SlotDetect_Set_SlotObj(&slotObj);
                            RTE_PK_SlotDetect_Set_TargPos(newTarget);
                            SlotObj_Convert_float(slotObj, enviObj);
                            PK_CopyPos(curSt[0].finpos, newTarget);
                            memcpy(Planned_Path, updatePath,
                                   trajNum * sizeof(float) * TRAJITEM_LEN);
                            PRINT_SWITCH_INFO("PathExec_PathSwitch");
                            PathExec_PrintPathInfo(Planned_Path, trajNum);

                            if (curSt[0].slotshape >= PK_SLOT_LEFT_VERT &&
                                PathExec_StopReplan(curSt, enviObj, curSt[0].finpos) &&
                                Planned_Path[0][3] > 0)
                            {
                                return;
                            }

                            PathExec_StartNewPlanPath(curSt, trajNum, Planned_Path[0][3]);
                        }
                    }
                }

                return;
            }
        }
        // 3 驾驶员干预
    }
    else if (PathExec_IsParkingLever(curSt))
    {
        curSt[0].brakeSt = 2;
        return;
    }

    if (curSt[0].stillCount >= path_apaquit_time)
    {
        if (RTE_PK_DistCtrl_Get_ModuleState_DistCtrl() != MSTAT_NORM)
        {
            PRINT_SWITCH_INFO("PathExec_MainControl OverTime!");
            curSt[0].stillCount = 0;
        }
        else
        {
            PRINT_SWITCH_INFO("PathExec_MainControl exit parking!");
            PathExec_SkipToLastTraj(curSt);
        }
    }
}

/**
 * @brief 后处理路径计算
 *
 * 该函数用于泊车过程中的后处理路径计算，根据当前状态和环境信息调整车辆的路径。
 *
 * @param plannedPath 规划的路径数组
 * @param curSt 当前泊车模块参数结构体
 * @param enviObj 环境障碍物数组
 * @return int 执行状态，1表示继续执行，0表示结束
 */
static int PathExec_PathPostCal(float plannedPath[MAX_SINGLE_TRAJ_NUM][5],
                                PK_Cur_AutodrPara curSt[1], float enviObj[5][4])
{
    int execState = 1;
    if (RTE_PK_SM_GetApaMode() == PARKOUT_MODE) // 如果是泊出模式
    {
        execState = 0;
        if (curSt[0].slotshape <= PK_SLOT_RIGHT_PARA)
        {
            if (curSt[0].AfterTra_nth == 0 &&
                fabs(curSt[0].rtheta_cur_to_fin) < PI * 0.1f &&
                fabs(curSt[0].rx_cur_to_fin) < 0.5f * VEHICLE_LEN)
            {
                PRINT_SWITCH_INFO("PathExec_PathPostCal para slot end!");
                return execState;
            }

            if (PathExec_ParaPostParkOutTraj(curSt, curSt[0].cvel_direct_fac, enviObj) ==
                0)
            {
                execState = 1;
            }
        }
    }
    else if (curSt[0].slotshape > PK_SLOT_RIGHT_PARA) // 如果是垂直泊车
    {
        float newTarget[3];
        PK_CopyPos(newTarget, curSt[0].curpos);
        PathExec_UpdateTarget(curSt, enviObj, newTarget, true, __LINE__);
        float ryf     = Project_PosTo1st_ry(newTarget, curSt[0].curpos);
        float rxf     = Project_PosTo1st_rx(newTarget, curSt[0].curpos);
        float rthetaf = Project_PosTo1st_rtheta(newTarget, curSt[0].curpos);
        printf(
            "parking end: curpos: %04lf %04lf %04lf finpos: %04lf %04lf %04lf "
            "target:%04lf %04lf %04lf, curtotarget: %04lf %04lf %04lf\n",
            curSt[0].curpos[0], curSt[0].curpos[1], curSt[0].curpos[2],
            curSt[0].finpos[0], curSt[0].finpos[1], curSt[0].finpos[2], newTarget[0],
            newTarget[1], newTarget[2], rxf, ryf, rthetaf);

        if ((fabsf(rthetaf) > thetaGap_allow || fabs(ryf) > 0.1) &&
            curSt[0].B_ReplanResult == PATHEXECUTE_SUCCESS &&
            !PathExec_IsLastAdjust(curSt) && !IsNarrowSlot(enviObj, newTarget, 0.75f))
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal exist! too remote");
            PK_CopyPos(curSt[0].finpos, newTarget);
            curSt[0].B_ReplanResult = PATHEXECUTE_FAIL; // 重规划失败
            int trajTemp =
                PathExec_SpaceExplorer_RePlanInSlot(plannedPath, curSt[0].finpos, enviObj,
                                                    curSt[0].curpos); // 按旧数据重新规划
            if (trajTemp > 0)
            {
                PathExec_PrintPathInfo(plannedPath, trajTemp);
                curSt[0].B_ReplanResult = PATHEXECUTE_SUCCESS;
                PathExec_SetLastAdjust(curSt);
                // 根据新路径更新并初始化B过程数据
                PathExec_StartNewPlanPath(curSt, trajTemp, plannedPath[0][3]);
            }
            else
            {
                execState = 0;
            }
        }
        else
        {
            pre_nout_ccp = 44;
            execState    = 0;
        }
    }
    else // 平行泊车结束调整
    {
        curSt[0].rxf     = Project_PosTo1st_rx(curSt[0].finpos, curSt[0].curpos);
        curSt[0].rthetaf = Project_PosTo1st_rtheta(curSt[0].finpos, curSt[0].curpos);
        curSt[0].rthetaf = Round_PI(curSt[0].rthetaf);

        if (curSt[0].AfterTra_nth == 0 &&
            (fabs(curSt[0].rx_cur_to_fin) > 0.6f * WHEEL_BASE ||
             fabs(curSt[0].ry_cur_to_fin) > 0.5f * VEHICLE_WID))
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal exist! too remote");
            execState = 0;
        }
        else if (fabsf(curSt[0].rthetaf) > thetaGap_allow || curSt[0].AfterTra_nth > 0)
        {
            // 调整车辆的角度到目标角度
            if (curSt[0].AfterTra_nth == 0)
            {
                curSt[0].cvel_direct_fac = sign(plannedPath[curSt[0].tra_num - 1][3]);
                if (PathExec_IsParkingLever(curSt))
                {
                    curSt[0].cvel_direct_fac = -1.0f;
                }
                else if (PathExec_IsRearDanger(curSt) && !PathExec_IsFrontDanger(curSt))
                {
                    curSt[0].cvel_direct_fac = -1.0f;
                }
                else if (!PathExec_IsRearDanger(curSt) && PathExec_IsFrontDanger(curSt))
                {
                    curSt[0].cvel_direct_fac = 1.0f;
                }
                else if (curSt[0].rxf < 0.10f)
                {
                    curSt[0].cvel_direct_fac = -1.0f;
                }

                if (curSt[0].cvel_direct_fac > 0.01f)
                {
                    PRINT_SWITCH_INFO("PathExec_PathPostCal init forward");
                }
                else
                {
                    PRINT_SWITCH_INFO("PathExec_PathPostCal init backward");
                }
            }

            int result = PathExec_ParaPostTraj(curSt, curSt[0].cvel_direct_fac, enviObj);
            if (result != 0)
            {
                execState = 0;
            }
        }
        else if ((!PathExec_IsObjDanger(curSt) &&
                  fabsf(curSt[0].rxf) > dfradioGap_allow) ||
                 (curSt[0].B_Danger_filter > 0 &&
                  fabsf(curSt[0].rxf) > 2.0f * dfradioGap_allow))
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal backward!");
            curSt[0].ceps     = 0;
            curSt[0].cvel     = -sign(curSt[0].rxf) * 0.2f;
            curSt[0].rx_brake = sign(curSt[0].rxf) * rte_min(fabsf(curSt[0].rxf), 0.4f);
        }
        else
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal exist!");
            curSt[0].rx_brake = 0;
            execState         = 0;
        }

        if (curSt[0].AfterTra_nth > 8)
        {
            execState = 0;
        }

        if (curSt[0].stillCount > path_apaquit_time && execState != 0)
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal exist timeout!");
            execState = 0;
        }
    }

    if (execState == 0)
    {
        curSt[0].cvel = 0.0f;
        curSt[0].ceps = 0.0f;
        PRINT_SWITCH_INFO("SlotBInfo");
        PathExec_PrintSlotBInfo();
        if (fabsf(curSt[0].rthetaf) > thetaGap_allow || curSt[0].rx_cur_to_fin > 0.05)
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal adjust!");
        }
    }

    return execState;
}

/**
 * @brief 动态更新路径函数
 *
 * 该函数在泊车过程中动态更新规划路径，确保车辆能够准确到达目标位置。
 *
 * @param Planned_Path 规划的路径数组
 * @param enviObj 环境障碍物数组
 * @param curSt 当前泊车模块参数结构体
 */
void PathExec_MovePath_dyna(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            float newTarget[3], PK_Cur_AutodrPara curSt[1])
{
    // 获取当前路径的终点位置
    float Pathfinpoint[3] = {0};
    PK_Get_Path_EndPos(Planned_Path[curSt[0].tra_num - 1], Pathfinpoint);

    // 仅处理垂直泊车位
    if (curSt[0].slotshape != PK_SLOT_RIGHT_VERT &&
        curSt[0].slotshape != PK_SLOT_LEFT_VERT)
    {
        return;
    }

    float rspoint[3];
    CoordinadteTransfer(newTarget, Pathfinpoint, rspoint);
    PK_CopyPos(curSt[0].finpos, newTarget);

    // 打印目标位置更新信息
    char info[128];
    snprintf(info, 128 - 1, "PathExec_UpdateTarget old:%04f %04f %04f new:%04f %04f %04f",
             Pathfinpoint[0], Pathfinpoint[1], Pathfinpoint[2], newTarget[0],
             newTarget[1], newTarget[2]);
    PRINT_SWITCH_INFO(info);

    // 计算目标位置的角度变化
    if (fabs(rspoint[2]) < 0.005 &&
        (rspoint[0] * rspoint[0] + rspoint[1] * rspoint[1] < 0.03 * 0.03))
    {
        return;
    }

    PRINT_SWITCH_INFO("PathExec_MovePath_dyna");
    PathExec_PrintPathInfo(Planned_Path, curSt[0].tra_num);

    // 根据新的目标位置调整路径
    for (int i = curSt[0].cur_tra_nth; i < curSt[0].tra_num; i++)
    {
        float trajStart[3];
        PK_CopyPos(trajStart, Planned_Path[i]);
        Convert(rspoint, trajStart,
                Planned_Path[i]); // 将路径点从新的目标位置坐标系转换回原坐标系
        Planned_Path[i][2] = Round_PI(trajStart[2] + rspoint[2]);
    }

    // 再次打印更新后的路径信息
    PathExec_PrintPathInfo(Planned_Path, curSt[0].tra_num);
}

/*********************************************************************
    Function description : PathExec_MainControl-->>Main Control function of path execute
    Calibration state    : NO
    Edition              : 1.2 2016/06/01
    Input list:
                        B_inital: initialize once flag
                        tra_num: the number of planned path
                        Explained_Path: the decompressed path
                        Cur_state: the state of current path execute
                        drx: the pre_turning or stop dist
    Output list:
                        cvel: control speed
                        ceps: control eps angle
                        return:  B_state: //0 complete,1 running, -1 exit
**********************************************************************/

/**
 * @brief PathExec_MainControl-->>Main Control function of path execute
 *
 * @param Planned_Path 路径
 * @param enviObj 车位边界
 * @param curSt 当前泊车模块参数结构体
 *
 * @return int 返回值简介
 */
static int PathExec_MainControl(float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                float enviObj[5][4], PK_Cur_AutodrPara curSt[1])
{
    // definition of inner params
    const int B_StopCnt_max       = 5;
    const float big_follow_de     = 0.8f;
    const float big_follow_sum_de = 16.0f; // MAXEPS_MDB / K2_MDBf;
    // const float big_eps_angle_gap = 10.0f; //  deg: the gap bewteen ceps and cureps

    int execState = 1;

    curSt[0].rx_cur_to_fin = Project_PosTo1st_rx(curSt[0].finpos, curSt[0].curpos);
    curSt[0].ry_cur_to_fin = Project_PosTo1st_ry(curSt[0].finpos, curSt[0].curpos);
    curSt[0].rtheta_cur_to_fin =
        Project_PosTo1st_rtheta(curSt[0].finpos, curSt[0].curpos);

    curSt[0].cur_vel     = RTE_PK_Location_Get_Vel_Spd();
    curSt[0].cur_eps     = RTE_BSW_Get_EPS_Angle();
    curSt[0].ds_gone_dir = curSt[0].curpos[3] - curSt[0].ds_start_dir;
    curSt[0].brakeSt     = 0;

    PathExec_DangerObj(curSt, enviObj); // 紧急障碍物

    PathExec_SpeedBump(curSt); // 减速带

    PathExec_ParkingLever(curSt); // 限位杆

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanInfoType curParkPath;
    if (fabsf(curSt[0].de) > big_follow_de && fabsf(curSt[0].sum_de) > big_follow_sum_de)
    {
        RTE_PK_PathPlan_Get_Park_PlanInfo(&curParkPath);
        if (curParkPath.Path_num <= 0 && PathExec_IsVehStill())
        {
            PRINT_SWITCH_INFO("PathExec_MainControl sum_de/de end!");
            curSt[0].cvel = 0.0f;
            curSt[0].ceps = 0.0f;
            pre_nout_ccp  = 94;
            execState     = -1;
        }
        else if (curParkPath.Path_num > 0 &&
                 (sign(curParkPath.Act_traj[0][3]) *
                          sign(Planned_Path[curSt[0].cur_tra_nth][3]) >
                      0 ||
                  PathExec_IsVehStill()) &&
                 !PathExec_IsParkUpdate(curSt))
        {
            if (RTE_PK_PathExecute_Get_BrkSt() == 3)
            {
                curSt[0].brakeSt = 0;
            }
            PRINT_SWITCH_INFO("PathExec_MainControl sum_de/de switch!");
            int newPathNum = SmoothNewTraj(curParkPath, curSt[0].curpos, Planned_Path);
            ReplanPathPost(Planned_Path, newPathNum, enviObj, curSt[0].finpos, curSt);
            PathExec_SetParkUpdate(curSt);
        }
        else
        {
            if (RTE_PK_PathExecute_Get_BrkSt() != 3)
            {
                PRINT_SWITCH_INFO("PathExec_MainControl sum_de/de switch wait!");
            }
            curSt[0].brakeSt = 3;
        }
    }
    else if (curSt[0].B_Stop_Counter > B_StopCnt_max || curSt[0].B_DynaPlanCounter > 10 ||
             curSt[0].AfterTra_nth > CONFIG_POSREPLAN_NUM)
    {
        PRINT_SWITCH_INFO("PathExec_MainControl End!");
        curSt[0].cvel = 0.0f;
        curSt[0].ceps = 0.0f;
        pre_nout_ccp  = 94;
        execState     = -1;
    }
    else if (curSt[0].cur_tra_nth < curSt[0].tra_num) // follow the planned path
    {
        curSt[0].AfterTra_nth = 0; //  init

        float swell = 0.01f;
        if (PathExec_ReplanSlotShape(curSt))
        {
            if (!PathExec_IsRearDanger(curSt) ||
                !PathExec_IsSuspendByObjs(
                    curSt)) // 如果有后方危险障碍物且车辆被障碍物迫停则不进行重规划
            {
                PathExec_ReplanCal(Planned_Path, enviObj, curSt,
                                   swell); // 重新计算规划路径
            }
        }

        PathExec_TraceTraj(Planned_Path, curSt); // 路径跟踪，误差控制

        PathExec_ParaCal(Planned_Path, curSt); // 车速、制动

        PathExec_PathSwitch(Planned_Path, curSt, enviObj, swell);

        PathExec_PathSwitchOther(Planned_Path, curSt, enviObj, swell);

        PathExec_UpdatePath(Planned_Path, curSt);

        PathExec_CompleteRate(Planned_Path, curSt);

        if (PathExec_IsParkUpdate(curSt))
        {
            if (curSt[0].cur_tra_nth > 0 || curSt[0].ds_gone_dir > 1.0)
            {
                PathExec_ClearParkUpdate(curSt);
            }
        }

        curParkPath.slotshape  = curSt[0].slotshape;
        curParkPath.Slot_index = curSt[0].Slot_index;
        RTE_PK_SlotDetect_Get_SlotObj(&curParkPath.slotObjs);
        int checkPathNum = CheckUpdatePathPos(Planned_Path, curSt);
        if (checkPathNum > 0)
        {
            if (curSt[0].tra_num > curSt[0].cur_tra_nth && curSt[0].cur_tra_nth >= 0 &&
                curSt[0].tra_num < MAX_SINGLE_TRAJ_NUM)
            {
                memcpy(curParkPath.Act_traj, Planned_Path[curSt[0].cur_tra_nth],
                       checkPathNum * TRAJITEM_LEN * sizeof(float));
                RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&curParkPath.avm_point);
                curParkPath.Path_num    = checkPathNum;
                curParkPath.IsDirConect = PARK_UPDATE;
                curSt[0].replanSt       = PARK_UPDATE;
            }
        }
        else
        {
            if (curSt[0].brakeSt == 0)
            {
                curSt[0].brakeSt = 4;
            }
            curParkPath.Path_num    = 0;
            curParkPath.IsDirConect = PARK_NOUPDATE;
            curSt[0].replanSt       = PARK_NOUPDATE;
        }

        RTE_PK_PathExecute_Set_CurTraj(&curParkPath);
    }
    else
    {
        curParkPath.slotshape  = curSt[0].slotshape;
        curParkPath.Slot_index = curSt[0].Slot_index;
        RTE_PK_SlotDetect_Get_SlotObj(&curParkPath.slotObjs);

        curParkPath.Path_num    = 0;
        curParkPath.IsDirConect = PARK_NOUPDATE;
        curSt[0].replanSt       = PARK_NOUPDATE;
        RTE_PK_PathExecute_Set_CurTraj(&curParkPath);
        execState = PathExec_PathPostCal(Planned_Path, curSt, enviObj);
    }

    // if cur eps angle far from target eps,wait
    if ((fabsf(curSt[0].cvel_last) < 0.01f && fabsf(curSt[0].cvel) > 0.1f) // 启动
        || (curSt[0].cvel_last * curSt[0].cvel < -0.01f) ||
        curSt[0].AfterTra_nth > 0) // D/R档位切换
    {
        if (fabsf(curSt[0].ceps - curSt[0].cur_eps) > big_eps_angle_gap)
        {
            curSt[0].cvel = sign(curSt[0].cvel) * 0.1;
            curSt[0].rx_brake =
                sign(curSt[0].rx_brake) * rte_min(0.30, fabs(curSt[0].rx_brake));
        }
        else if (fabsf(curSt[0].ceps - curSt[0].cur_eps) >
                     rte_max(big_eps_angle_gap * 0.3, 10.0f) &&
                 fabs(curSt[0].rxf) < 1.0)
        {
            curSt[0].cvel = sign(curSt[0].cvel) * 0.15;
            curSt[0].rx_brake =
                sign(curSt[0].rx_brake) * rte_min(0.50, fabs(curSt[0].rx_brake));
        }
    }

    return execState;
}

static pthread_mutex_t rte_mutex = PTHREAD_MUTEX_INITIALIZER;

void PathExec_Lock() { pthread_mutex_lock(&rte_mutex); }

void PathExec_UnLock() { pthread_mutex_unlock(&rte_mutex); }

void PK_InitPathExecute(void)
{
    char config[64];
    if (PK_Calibration_GetDebugValue("cfg_use_dynreplan", config) > 0)
    {
        cfg_updatepath_stat = atoi(config);
    }

    if (PK_Calibration_GetDebugValue("cfg_sideupdate_dist", config) > 0)
    {
        cfg_sideupdate_dist = atof(config);
    }

    // auto taskFunc = [](void *args) -> void * {
    //     while (1)
    //     {
    //         usleep(path_execute_period * 1000);
    //         PK_PathExecuteRun();
    //     }

    //     PathExec_PrintPathEnd();
    //     return NULL;
    // };

    // ASyncRun(taskFunc, NULL, "apa_track");
}
