/**
 * @file PK_StateManage.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-15
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-15 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "PK_StateManage.h"
#include "PK_Calibration.h"
#include "PK_PathExecute.h"
#include "Rte_BSW.h"
#include "CanIf.h"
#include "Record_Log.h"
#include "PK_ObjAvoid.h"
#include "SignalConversion.h"

const char *kCarWorkingStaStr[] = {"Passive",
                                   "Serching",
                                   "GuidanceActive",
                                   "GuidanceSuspend",
                                   "GuidanceTerminated",
                                   "GuidanceCompleted",
                                   "Failure",
                                   "ParkAssist_Standby",
                                   "GuidanceInitParkingData",
                                   "Passive_SpdOver",
                                   "PowerOn",
                                   "GuidanceAvpActive"};

static PK_ModuleStateType m_State_PathExecute; // 路径追踪状态
static Apa_WorkStateType m_CurrApaWorkState;   // APA状态机状态
static Apa_WorkStateType m_PreApaWorkState;    // APA状态机前状态
static uint32_t m_ApaStaKeepCount;             // APA状态机状态保持周期计数，不考虑溢出
static HMI_ApaModeType m_preApaMode;           // APA前模式
static HMI_StateType m_HMI_State;              // HMI模式
static HMI_WorkStateType m_HMI_WorkState;      // HMI工作状态
static HMI_WorkTextInfo m_HMI_CurrApaText;     // HMI显示文字
static HMI_ButtonInfo m_HmiButtonInfo;         // HMI按键信息
static bool m_ApaModeSwitchStatus;             /// 泊车功能总开关
static uint32_t m_StateMachineCnt;             // 状态机周期计数，不考虑溢出

static inline uint32_t TimeMsToFrame(uint32_t time_ms)
{
    return (time_ms + STATE_MACHINE_PERIOD_MS - 1) / STATE_MACHINE_PERIOD_MS;
}
/**
 * @brief 检查 IPB 模块是否处于 Standby 状态。
 *
 * @note 信号值因不同车型的 CAN 矩阵而适配。
 *
 * @return true
 * @return false
 */
static bool m_IpbModulesShakehandIsStandby(void)
{
    const uint8_t IPB2Park1_Lsm12ComfortAvl_S   = 0x3;
    const uint8_t IPB2Park1_Lsm12EmergencyAvl_S = 0x3;
    const uint8_t IPB2Park1_LSMPosLongACapbl_S  = 0x0;

    bool l_ret = false;
    l_ret      = (IPB2Park1_Lsm12ComfortAvl_S == RTE_BSW_Get_Lsm12ComfortAvl() &&
             IPB2Park1_Lsm12EmergencyAvl_S == RTE_BSW_Get_Lsm12EmergencyAvl() &&
             IPB2Park1_LSMPosLongACapbl_S == RTE_BSW_Get_LSMPosLongACapbl());
    return l_ret;
}

/**
 * @brief 检查 IPB 模块握手是否成功。
 *
 * 校验逻辑：
 * 1. ReqEcho == 0x2 且 LongEcho == 0x1；
 * 2. 根据模式不同：
 *    - APA_MODE: LevelEcho == 0x1
 *    - RPA_MODE: LevelEcho == 0x2
 *    - AVP_MODE: LevelEcho == 0x3
 *
 * @note 信号值可能因不同车型的 CAN 矩阵而适配。
 *
 * @return true  握手成功
 * @return false 握手失败
 */
static bool m_IpbModulesShakehandIsReady(void)
{
    const uint8_t IPB2Park1_LSMSubMTLevelEcho_S_APA = 0x1;
    const uint8_t IPB2Park1_LSMSubMTLevelEcho_S_RPA = 0x2;
    const uint8_t IPB2Park1_LSMSubMTLevelEcho_S_AVP = 0x3;
    const uint8_t IPB2Park1_LSMSubMTReqEcho_S       = 0x2;
    const uint8_t IPB2Park1_LSMSubMTLongEcho_S      = 0x1;

    bool l_handshake_ok = false;
    uint8_t apaMode     = APA_MODE;

    if (IPB2Park1_LSMSubMTReqEcho_S != RTE_BSW_Get_LSMSubMTReqEcho() ||
        IPB2Park1_LSMSubMTLongEcho_S != RTE_BSW_Get_LSMSubMTLongEcho())
    {
        l_handshake_ok = false;
    }
    else if (apaMode == APA_MODE &&
             IPB2Park1_LSMSubMTLevelEcho_S_APA == RTE_BSW_Get_LSMSubMTLevelEcho())
    {
        l_handshake_ok = true;
    }
    else if (apaMode == RPA_MODE &&
             IPB2Park1_LSMSubMTLevelEcho_S_RPA == RTE_BSW_Get_LSMSubMTLevelEcho())
    {
        l_handshake_ok = true;
    }
    else if (apaMode == AVP_MODE &&
             IPB2Park1_LSMSubMTLevelEcho_S_AVP == RTE_BSW_Get_LSMSubMTLevelEcho())
    {
        l_handshake_ok = true;
    }
    else
    {
        l_handshake_ok = false;
    }

    return l_handshake_ok;
}

/**
 * @brief APA当前泊车模式
 *
 * @return ApaTypeCmd  0-正常车内泊车  1-遥控泊车
 */
static bool m_isRemoteMode(void)
{
    bool l_ret = false;
    if (RTE_PK_SM_GetApaMode() != APA_MODE) // 遥控泊车触发条件
    {
        l_ret = true;
    }
    return l_ret;
}

/**
 * @brief  模块内部状态检查
 *
 * @note  需要故障诊断功能，未实现
 *
 * @return true
 * @return false
 */
static bool m_InternalStaIsOk(void)
{
    bool l_ret = true;
    return l_ret;
}

/**
 * @brief 外部关联件状态
 *
 * @return true
 * @return false
 */
static bool m_ExternalStaIsOk(void)
{
    bool l_ret           = false;
    uint32_t l_FaultCode = RTE_BSW_Get_ModuleCheck();

    // EPS 状态检测
    RTE_BSW_EpsCtrlStatType EpsSta = RTE_BSW_Get_EpsCtrlStat();
    if (EpsSta == EPS_CTRL_PERM_INHIBITED)
    {
        l_FaultCode |= kExternalEps;
    }

    // ESC 状态检测
    RTE_BSW_EscCtrlStatType EscSta = RTE_BSW_Get_EscApaStat();
    if (EscSta == ESC_CTRL_PERM_INHIBITED)
    {
        l_FaultCode |= kExternalEpb;
        l_FaultCode |= kExternalTcu;
    }

    // EPB 状态检测
    RTE_BSW_EpbModeType EpbSta = RTE_BSW_Get_EpbStat();
    if (EpbSta == EPB_Fault)
    {
        l_FaultCode |= kExternalEpb;
    }

    // Camera 状态检测
    VisionState VisiSta = RTE_PK_SM_GetVisiInfo();
    if (VisiSta == VisionState_Invalid)
    {
        l_FaultCode |= kExternalCamera;
    }
    // 更新故障寄存
    RTE_BSW_Set_ModuleCheck(l_FaultCode);
    if (l_FaultCode == 0)
    {
        l_ret = true;
    }

    return l_ret;
}

/**
 * @brief 检查控制关联件是否正常握手
 *
 * @return true
 * @return false
 */
static RTE_BSW_RelateCtrlStatType OtherRelateModulesShakehandIsOk(void)
{
    RTE_BSW_RelateCtrlStatType ret_state = CTRL_TEMP_INHIBITED;
    if (ESC_CTRL_PERM_INHIBITED == RTE_BSW_Get_EscApaStat() ||
        EPS_CTRL_PERM_INHIBITED == RTE_BSW_Get_EpsCtrlStat())
    {
        ret_state = CTRL_PERM_INHIBITED;
    }
    else if (ESC_CTRL_ACTIVE == RTE_BSW_Get_EscApaStat() &&
             EPS_CTRL_ACTIVE == RTE_BSW_Get_EpsCtrlStat())
    {
        ret_state = CTRL_ACTIVE;
    }
    else if (ESC_CTRL_AVAILABLE == RTE_BSW_Get_EscApaStat() ||
             EPS_CTRL_AVAILABLE == RTE_BSW_Get_EpsCtrlStat())
    {
        ret_state = CTRL_AVAILABLE;
    }
    else
    {
        ret_state = CTRL_TEMP_INHIBITED;
    }
    return ret_state;
}

/**
 * @brief 设置泊车位（Slot），更新路径规划和相关状态信息
 *
 * @param data CAN数据，data[1..4]为目标泊车位ID（uint32_t）
 */
static StartFailResultType SetParkSlot(const int targetId)
{
    StartFailResultType l_ret_result = StartPark_OK;
    int targetIndex                  = -1;
    SlotInfo_T targetSlot;        // 目标车位信息
    SlotInfo_T selfSlot;          // 自选车位信息
    Multi_Slot_Array_T slotsInfo; // 车位管理信息
    MultiPlanInfo plansInfo;      // 路径规划结果
    RTE_PK_DataConv_Get_SlotInfo(&selfSlot);
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slotsInfo);
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&plansInfo);
    memset(&targetSlot, 0, sizeof(targetSlot));

    // 路径规划结果是空的
    if (0 == plansInfo.slotNum)
    {
        l_ret_result = StartFail_PlanIsEmpty;
    }

    // 检查该车位ID是否在被管理
    if (StartPark_OK == l_ret_result)
    {
        if (targetId == selfSlot.slot_index)
        {
            targetSlot = selfSlot;
        }
        else
        {
            int index = 0;
            // 遍历检测到的车位，找到与目标slotId匹配的
            for (index = 0; index < slotsInfo.multiNum; index++)
            {
                if (targetId == slotsInfo.multiArray[index].slot_index)
                {
                    memcpy(&targetSlot, &slotsInfo.multiArray[index], sizeof(SlotInfo_T));
                    break;
                }
            }

            // 如果没找到完全匹配，则再尝试通过compareSlotIndex模糊匹配
            if (0 == targetSlot.slot_index)
            {
                for (index = 0; index < slotsInfo.multiNum; index++)
                {
                    if (CompareSlotIndex(slotsInfo.multiArray[index].slot_index,
                                         targetId))
                    {
                        memcpy(&targetSlot, &slotsInfo.multiArray[index],
                               sizeof(SlotInfo_T));
                        break;
                    }
                }
            }

            if (0 == targetSlot.slot_index)
            {
                // 目标车位不是被管理
                l_ret_result = StartFail_NotManagedSlot;
            }
            else if (PK_SLOT_NO == targetSlot.slotshap)
            {
                // 无效的车位形状
                l_ret_result = StartFail_InvalidShape;
            }
        }
    }

    // 在路径规划信息中查找目标泊车位
    if (StartPark_OK == l_ret_result)
    {
        int index = 0;
        for (index = 0; index < plansInfo.slotNum; index++)
        {
            if (targetId == plansInfo.slotPlans[index].Slot_index)
            {
                targetIndex = index;
                break;
            }
        }

        // 如果没找到完全匹配，则再尝试通过compareSlotIndex模糊匹配
        if (0 == targetSlot.slot_index)
        {
            for (index = 0; index < plansInfo.slotNum; index++)
            {
                if (CompareSlotIndex(plansInfo.slotPlans[index].Slot_index,
                                     static_cast<int>(targetId)))
                {
                    targetIndex = index;
                    break;
                }
            }
        }

        // 目标车位没有路径规划过
        if (targetIndex < 0)
        {
            l_ret_result = StartFail_NotInPlan;
        }
    }

    // 目标车位路径无效
    if (StartPark_OK == l_ret_result)
    {
        if (plansInfo.slotPlans[targetIndex].Path_num <= 0)
        {
            l_ret_result = StartFail_NoValidPath;
        }
    }

    // 设置车位
    if (StartPark_OK == l_ret_result)
    {
        // 设置车位形状
        RTE_PK_SlotDetect_Set_SlotShape(
            (PK_SlotShapeType)plansInfo.slotPlans[targetIndex].slotshape);

        // 设置目标路径规划信息
        RTE_PK_DataConvt_Set_Target_PlanInfo(&plansInfo.slotPlans[targetIndex]);

        // 设置是否为AVM（全景影像）识别车位
        RTE_PK_DataConvt_Set_IsAVM_Slot(targetSlot.is_vision_slot);

        // 获取规划路径的末端坐标，作为泊车目标点
        float target[3];
        int lastTraj = plansInfo.slotPlans[targetIndex].Path_num - 1;
        PK_Get_Path_EndPos(plansInfo.slotPlans[targetIndex].Act_traj[lastTraj], target);
        RTE_PK_SlotDetect_Set_TargPos(target);

        // 设置车位附加信息（如限位杆）
        plansInfo.slotPlans[targetIndex].avm_point.AlignIndex[0] = targetSlot.has_stopper;
        RTE_PK_DataConvt_Set_TargAVM_SlotInfo(
            &plansInfo.slotPlans[targetIndex].avm_point);

        // 设置车位边界障碍物信息
        RTE_PK_SlotDetect_Set_SlotObj(&plansInfo.slotPlans[targetIndex].slotObjs);

        // 特殊处理：如果为AVM车位，还需根据当前位置和车位中心点关系判断有效性
        if (targetSlot.is_vision_slot == 1)
        {
            float curpos[4];
            float center[2];

            // 计算车位中心点
            center[0] = 0.5f * (plansInfo.slotPlans[targetIndex].avm_point.near_front.x +
                                plansInfo.slotPlans[targetIndex].avm_point.near_rear.x);
            center[1] = 0.5f * (plansInfo.slotPlans[targetIndex].avm_point.near_front.y +
                                plansInfo.slotPlans[targetIndex].avm_point.near_rear.y);

            // 获取车辆当前位置
            RTE_PK_Location_Get_CurPos(curpos);

            // 投影计算车辆与车位的相对位置
            float rx = Project_PosTo1st_rx(curpos, center);

            // 如果车辆距离车位过远，则标记为特殊状态（3）
            if (rx > 3.0) // 1.9 + 1.2
            {
                RTE_PK_DataConvt_Set_IsAVM_Slot(3);
            }
        }
    }
    return l_ret_result;
}

/**
 * @brief 检查车位是否被选中确认
 *
 * @return true 有车位选中
 * @return false
 */
static bool SlotIsSelected(void)
{
    bool ret                         = true;
    int rawIndx                      = m_HmiButtonInfo.HmiSelectSlotInx;
    PK_ModuleComType ExeState        = RTE_PK_StateManage_Get_ModuleCom_PathExecute();
    StartFailResultType l_ret_result = StartPark_OK;

    // 泊车功能未启用
    if (false == m_ApaModeSwitchStatus)
    {
        ret = false;
    }
    // 用户未选中车位
    else if (rawIndx == -1)
    {
        ret = false;
    }
    // 泊车已经开始，选中车位无效
    else if (ExeState == MCOM_ON_PARKING)
    {
        ret = false;
    }
    else
    {
        // 启动泊车
        if (SetParkSlot(rawIndx) != StartPark_OK)
        {
            ret = false;
        }
    }

    return ret;
}

/**
 * @brief 检查四门两盖状态
 *
 * @return uint8_t  0-四门两盖全部关闭       1-任意门开启
 */
static CarDoorStatus m_CarDoorCurrStatus(void)
{
    RTE_BSW_BCM_StateType bcmSt;
    RTE_BSW_Get_BCM_VehicleState(&bcmSt);

    uint8_t door_LD = bcmSt.BCM_DriverDoorAjarSt;
    uint8_t door_RD = bcmSt.BCM_PsngrDoorAjarSt;
    uint8_t door_LR = bcmSt.BCM_RLDoorAjarSt;
    uint8_t door_RR = bcmSt.BCM_RRDoorAjarSt;

    uint8_t former_hatch = bcmSt.BCM_BonnetAjarSt;
    uint8_t back_door    = bcmSt.RDM_RearDoorStatus;

    CarDoorStatus ret = AllClosed;
    if (door_LD || door_RD || door_LR || door_RR)
    {
        ret = DoorOpen;
    }
    else if (former_hatch)
    {
        ret = BonnetOpen;
    }
    else if (back_door)
    {
        ret = TrunkOpen;
    }
    else
    {
        ret = AllClosed;
    }

    return ret;
}

/**
 * @brief 获取安全带状态
 *
 * @return CarSafeBeltStatus
 */
static CarSafeBeltStatus CarSafeBeltCurrStatus(void)
{
    RTE_BSW_BCM_StateType bcmSt;
    RTE_BSW_Get_BCM_VehicleState(&bcmSt);
    CarSafeBeltStatus l_rte = CarSafeBeltFastened;
    if (bcmSt.SRS_DriverSeatBeltWarning != 0x00)
    {
        l_rte = CarSafeBeltDriverUnfastened;
    }
    return l_rte;
}

/**
 * @brief EPB是否被拉起
 *
 * @return true
 * @return false
 */
static bool EpbIsPullup(void)
{
    bool ret                    = false;
    static uint32_t timer_start = 0;                   // 静态计时起点
    uint32_t curr_cnt           = m_StateMachineCnt;   // 状态机全局计数器
    const uint32_t k3s          = TimeMsToFrame(3000); // 3秒对应帧数

    // EPB拉起持续3s
    if (EPB_Applied == RTE_BSW_Get_EpbStat())
    {
        if ((curr_cnt - timer_start) >= k3s)
        {
            ret = true;
        }
    }
    else
    {
        timer_start = curr_cnt;
    }

    return ret;
}

/**
 * @brief 检查后视镜是否折叠
 *
 * @return true  未折叠
 * @return false  折叠
 */
static bool OutsideRearviewMirrorIsFolded(void)
{
    //  (RTE_BSW_Get_FldigExtrrMrorsExpand() == 0x1)
    return 0;
}

/**
 * @brief 检查障碍物是否在路径上
 *
 * @note 当前是判断了是否为危险状态，只采用紧急制动模式
 *
 * @return true  路径上有障碍物
 * @return false 路径上无障碍物
 */
static bool ObstacleIsInPath(void)
{
    bool l_ret = false;

    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();

    if (dangerSt != 0U && (dangerSt & 0x0FU) != 0x00U)
    {
        l_ret = true;
    }

    return l_ret;
}

/**
 * @brief 获取当前车速值
 *
 * @return float
 */
static float CurrCarSpeedValue(void)
{
    // if (RTE_BSW_Get_ESC_VehSpdVD())
    return RTE_BSW_Get_ESC_VehSpd();
}

/**
 * @brief 获取来自 HMI 的用户按钮状态（带防抖处理）
 *
 * 对 HMI 上报的按钮状态进行防抖判断，防止短时抖动触发错误动作。
 * 特殊处理“退出泊车”按钮，不要立即退出，使用更长的计数做延时。
 *
 * @return HMI_ButtonStateType 当前稳定的按钮状态，
 *         如果未达到防抖阈值，则返回 ButtonState_Invalid
 */
static HMI_ButtonStateType GetUserButtonSta(void)
{

    static HMI_ButtonStateType preButton = ButtonState_Invalid; ///< 上一次按钮状态
    static int buttonStaCount            = 0;                   ///< 连续相同状态计数
    HMI_ButtonStateType currButton       = m_HmiButtonInfo.HmiButtonSta;
    HMI_ButtonStateType retButton        = ButtonState_Invalid;

    // 根据按钮类型选择防抖阈值，不要立即退出
    int debounceCnt = (currButton == ButtonState_OutPark) ? 3 : 60;

    if (preButton == currButton)
    {
        buttonStaCount++;
    }
    else
    {
        buttonStaCount = 0;
    }

    if (buttonStaCount > debounceCnt)
    {
        retButton = currButton;
    }

    preButton = currButton;
    return retButton;
}

/**
 * @brief 获取来自HMI的用户按钮状态
 *
 * @return true 自动泊车功能开启
 * @return false 无动作
 */
static bool GetApaSwitchSta(void)
{
    bool isApaOn             = false;
    static bool preSwitchSta = false;

    // APA 开启条件：当前开关是使能 & （之前开关是禁止或 APA 模式未开启）
    if ((true == m_HmiButtonInfo.HmiSwitchSta) &&
        ((false == preSwitchSta) ||
         (m_ApaModeSwitchStatus == false))) // 获取进入泊车模式总开关信号
    {
        m_ApaModeSwitchStatus = true;
        isApaOn               = true;
        log_info("--------- APA switch ON --------\r\n");
    }
    // APA 关闭条件
    else if (false == m_HmiButtonInfo.HmiSwitchSta) // 点击泊车退出按钮，退出泊车
    {
        m_ApaModeSwitchStatus = false;
        isApaOn               = false;
    }
    preSwitchSta = m_HmiButtonInfo.HmiSwitchSta;

    return isApaOn;
}

/**
 * @brief 获取当前刹车踏板状态
 *
 * @return true  已踩下刹车踏板
 * @return false 未踩刹车踏板
 */
static bool GetCurrPressBrakePedalStatus(void)
{
    bool isPressed = false;

    // TODO: 用真实信号替换占位条件，例如：
    // isPressed = (ESP_BRAKE_PEDAL_STATUS == 1);
    if (1) // 占位条件，ESP_BRAKE_PEDAL_STATUS
    {
        isPressed = true;
    }

    return isPressed;
}

/**
 * @brief 检测驾驶员是否干预方向盘
 *
 * @return true  驾驶员干预方向盘
 * @return false 驾驶员未干预方向盘
 */
static bool DriverInterventSteeringWheel(void)
{
    const float kTorqueThreshold  = 2.0f; // Nm
    const uint8_t kCountThreshold = 5;    // 100ms @ 20ms周期
    static uint8_t epsCount       = 0;
    bool isIntervened             = false;
    uint8_t epsTorqueValid        = RTE_BSW_Get_EpsStrWhlTorqVD();
    float epsTorqueVal            = RTE_BSW_Get_EpsStrWhlTorqVal();

    // 检测驾驶员手力矩是否超过阈值
    if (epsTorqueValid && fabs(epsTorqueVal) > kTorqueThreshold)
    {
        if (++epsCount > kCountThreshold)
        {
            epsCount     = kCountThreshold; // 防止溢出
            isIntervened = true;
        }
    }
    else
    {
        epsCount = 0;
    }

    return isIntervened;
}
/**
 * @brief 检测驾驶员是否干预退出泊车
 *
 * @return true  驾驶员干预
 * @return false 未干预
 */
static bool DriverOverrideCheck(void)
{
    static uint8_t accCount  = 0; // 油门防抖计数
    static uint8_t gearCount = 0; // 档位防抖计数

    bool isOverridden = DriverInterventSteeringWheel(); // 方向盘干预

    // 油门干预
    uint8_t throttleValid                 = RTE_BSW_Get_VcuAccelerographDepthVD();
    float throttleValue                   = RTE_BSW_Get_VcuAccelerographDepth();
    const float kThrottleThreshold        = 0.3f; ///< N
    const uint8_t kThrottleCountThreshold = 3;

    if (throttleValid && throttleValue > kThrottleThreshold)
    {
        if (++accCount > kThrottleCountThreshold)
        {
            accCount     = kThrottleCountThreshold;
            isOverridden = true;
        }
    }
    else
    {
        accCount = 0;
    }

    // 档位干预
    uint8_t gearStatus                     = RTE_BSW_Get_GearSystemStatus();
    RTE_TCU_ApaStatusType scuApaStatusInfo = RTE_BSW_Get_TcuApaStat();
    const uint8_t kGearCountThreshold      = 3;

    if ((gearStatus == 0) && (scuApaStatusInfo == TCU_DriverInterference))
    {
        if (++gearCount > kGearCountThreshold)
        {
            gearCount    = kGearCountThreshold;
            isOverridden = true;
        }
    }
    else
    {
        gearCount = 0;
    }

    return isOverridden;
}

/**
 * @brief 检查充电枪是否已连接
 *
 * @return true  充电枪已启用/连接
 * @return false 充电枪未启用/未连接
 */
static bool ChargingGunIsConnected(void)
{
    bool isConnected = false;

    // TODO: 用真实信号替换占位条件，例如：
    // isConnected = (RTE_BSW_Get_ChargingGunStatus() == 1);
    if (0) // 占位条件，充电枪状态信号 0x055
    {
        isConnected = true;
    }

    return isConnected;
}

/**
 * @brief 检查 APA 是否处于暂停状态超过 30s
 *
 * @return true  已超过 30s
 * @return false 未超过
 */
static bool ApaSuspendStatusOver30s(void)
{
    bool ret                    = false;
    static uint32_t timer_start = 0;                    // 计时起点
    uint32_t curr_cnt           = m_StateMachineCnt;    // 状态机全局计数器
    const uint32_t k30s         = TimeMsToFrame(30000); // 30秒对应帧数（状态机周期 20ms）

    if (CarSta_GuidanceSuspend == m_CurrApaWorkState)
    {
        if ((curr_cnt - timer_start) >= k30s)
        {
            ret = true;
        }
    }
    else
    {
        timer_start = curr_cnt;
    }

    return ret;
}

/**
 * @brief 检查 超速是否超时
 *
 * @return true  已超过 10s
 * @return false 未超过
 */
static bool OverSpeedStatusOverOnSpdOver(void)
{
    bool ret                  = false;
    static uint32_t start_cnt = 0;                 // 计时起点
    uint32_t curr_cnt         = m_StateMachineCnt; // 状态机全局计数器
    const uint32_t keep_cnt   = TimeMsToFrame(10000);

    if (CarSta_Passive_SpdOver != m_CurrApaWorkState)
    {
        start_cnt = curr_cnt;
    }
    else
    {
        if (CurrCarSpeedValue() > APA_SPEE_LIMIT_MAX_WARNING)
        {
            if ((curr_cnt - start_cnt) >= keep_cnt)
            {
                ret = true;
            }
        }
    }
    return ret;
}
/**
 * @brief 检查 低速是否保持
 *
 * @return true  已超过 100ms
 * @return false 未超过
 */
static bool LowSpeedStatusKeepOnPassive(void)
{
    bool ret                  = false;
    static uint32_t start_cnt = 0;                  // 计时起点
    uint32_t curr_cnt         = m_StateMachineCnt;  // 状态机全局计数器
    const uint32_t keep_cnt   = TimeMsToFrame(100); // 保持100ms

    if (CarSta_Passive != m_CurrApaWorkState)
    {
        start_cnt = curr_cnt;
    }
    else
    {
        if (CurrCarSpeedValue() < APA_SPEE_LIMIT_MAX_WARNING)
        {
            if ((curr_cnt - start_cnt) >= keep_cnt)
            {
                ret = true;
            }
        }
    }
    return ret;
}

/**
 * @brief 检查 退出状态是否保持
 *
 * @return true  已超过 100ms
 * @return false 未超过
 */
static bool OutParkStatusKeepOnPassive(void)
{
    static uint32_t start_cnt = 0;
    const uint32_t keep_cnt   = TimeMsToFrame(200);

    bool ret          = false;
    uint32_t curr_cnt = m_StateMachineCnt; // 状态机全局计数器

    if (CarSta_Passive != m_CurrApaWorkState)
    {
        start_cnt = curr_cnt;
    }
    else
    {
        if (true == m_HmiButtonInfo.HmiSwitchSta)
        {
            if ((curr_cnt - start_cnt) >= keep_cnt)
            {
                ret = true;
            }
        }
    }

    return ret;
}

/**
 * @brief 获取各模块的状态信息并更新全局状态
 */
static void GetOtherModuleStatus(void)
{
    // input status data
    m_State_PathExecute = RTE_PK_PathExecute_Get_ModuleState_PathExecute();
}

/**
 * @brief
 *
 * @param buff
 */
static bool ReportRadarInfo()
{
    U_RadarType radarData;
    RTE_PD_Get_U_Radar(&radarData);
    float minDist = 5000.0;

    minDist = rte_min(radarData.FOL, minDist);
    minDist = rte_min(radarData.FCL, minDist);
    minDist = rte_min(radarData.FCR, minDist);
    minDist = rte_min(radarData.FOR, minDist);
    minDist = rte_min(radarData.ROL, minDist);
    minDist = rte_min(radarData.RCL, minDist);
    minDist = rte_min(radarData.RCR, minDist);
    minDist = rte_min(radarData.ROR, minDist);
    minDist = rte_min(radarData.FSL, minDist);
    minDist = rte_min(radarData.FSR, minDist);
    minDist = rte_min(radarData.RSL, minDist);
    minDist = rte_min(radarData.RSR, minDist);

    bool RadarAlarm = (minDist < 300.0) ? 1 : 0;

    return RadarAlarm;
}
/**
 * @brief 判断是否有车位已被路径规划覆盖
 *
 * 遍历检测到的车位信息和路径规划中的车位，
 * 如果至少有一个车位在规划路径中，则返回 true。
 *
 * @return true 已搜索到规划过的车位
 * @return false 没有搜索到规划过的车位
 */
static bool SlotIsSearched()
{
    bool ret = false;

    MultiPlanInfo plansInfo;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&plansInfo);

    Multi_Slot_Array_T slotsInfo;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slotsInfo);

    for (int slot = 0; slot < MAX_PARKED_SLOTS; slot++)
    {
        // 车位未检测到，提前结束
        if (slotsInfo.multiArray[slot].slot_index < 0)
        {
            break;
        }

        for (int planId = 0; planId < plansInfo.slotNum; planId++)
        {
            if (CompareSlotIndex(slotsInfo.multiArray[slot].slot_index,
                                 plansInfo.slotPlans[planId].Slot_index))
            {
                ret = true;
                break;
            }
        }

        if (ret)
        {
            break; // 找到一个匹配即可，不用继续遍历
        }
    }

    return ret;
}

/**
 * @brief 判断 APA 车位数据是否为空
 *
 * @return true  APA 车位数据为空
 * @return false APA 车位数据有效
 */
static bool IsApaSlotDataEmpty()
{
    bool isEmpty = false;

    float curPos[4] = {0};
    AVM_Buff_Info leftBuffInfo{}, rightBuffInfo{};

    // 获取当前位置
    RTE_PK_Location_Get_CurPos(curPos);

    // 获取左右车位缓存
    RTE_PK_SF_Get_AVM_Buff_Left(&leftBuffInfo);
    RTE_PK_SF_Get_AVM_Buff_Right(&rightBuffInfo);

    int leftSlotNum  = leftBuffInfo.AccNum;
    int rightSlotNum = rightBuffInfo.AccNum;

    // 判断车位数据是否为空
    if (leftSlotNum == 0 && rightSlotNum == 0 &&
        fabs(curPos[0]) <= 0.5f * vehicle_travel_dist_min &&
        fabs(curPos[1]) <= 0.5f * vehicle_travel_dist_min &&
        fabs(Round_PI(curPos[2])) <= 2 * PI_RAD)
    {
        isEmpty = true;
    }

    return isEmpty;
}

/**
 * @brief 获取当前档位信息
 *
 * @return uint8_t
 */
static uint8_t GetCurrentGear()
{
    uint8_t curGear = RTE_BSW_Get_CurrentGear();
    if (curGear == GEAR_REAL_D)
    {
        curGear = GEAR_D;
    }
    else if (curGear == GEAR_REAL_R)
    {
        curGear = GEAR_R;
    }
    else
    {
        curGear = GEAR_P;
    }

    return curGear;
}

/**
 * @brief APA 状态管理模块数据初始化
 *
 */
void StateManage_DataInit(void)
{
    m_State_PathExecute   = MSTAT_NORM;
    m_HMI_State           = HmiState_Invalid;
    m_HMI_WorkState       = HmiWorkState_Passive;
    m_HMI_CurrApaText     = ApaWorkingText_NoInfo;
    m_CurrApaWorkState    = CarSta_PowerOn;
    m_ApaModeSwitchStatus = false;
    m_HmiButtonInfo       = {};
    m_StateMachineCnt     = 0;
    m_preApaMode          = APA_MODE;
    log_info("--------- StatusManage Init --------\r\n");
}

/**
 * @brief 检查是否可以清理 APA 数据
 *
 * @return true  数据可以清理
 * @return false 数据不可清理
 */
static bool ClearApaData()
{
    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    if (curPos[3] > 255.0f)
    {
        Multi_Slot_Array_T slots;
        RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slots);
        if (slots.multiNum != 0)
        {
            return false;
        }

        AVM_Buff_Info avms;
        RTE_PK_SF_Get_AVM_Buff_Left(&avms);
        if (avms.Num != 0)
        {
            return false;
        }

        RTE_PK_SF_Get_AVM_Buff_Right(&avms);
        if (avms.Num != 0)
        {
            return false;
        }

        return true;
    }

    return false;
}
/**
 * @brief 更新各个模块状态
 *
 * @param curState
 */
void UpdateModComState(const Apa_WorkStateType curState)
{
    static PK_ModuleComType preCom_Location    = MCOM_INIT;
    static PK_ModuleComType preCom_PathPlan    = MCOM_INIT;
    static PK_ModuleComType preCom_PathExecute = MCOM_INIT;
    static PK_ModuleComType preCom_SpeedCtrl   = MCOM_INIT;
    static PK_ModuleComType preCom_ObjAvoid    = MCOM_INIT;
    static Apa_WorkStateType last_CarState     = CarSta_PowerOn;
    static float clearCount                    = 0;

    PK_ModuleComType Com_Location    = MCOM_INIT;
    PK_ModuleComType Com_PathPlan    = MCOM_INIT;
    PK_ModuleComType Com_PathExecute = MCOM_INIT;
    PK_ModuleComType Com_SpeedCtrl   = MCOM_INIT;
    PK_ModuleComType Com_ObjAvoid    = MCOM_INIT;

    switch (curState)
    {
        case CarSta_PowerOn:
            break;
        case CarSta_Passive:
            if (last_CarState != CarSta_Passive)
            {
                Com_Location    = MCOM_DATCLC;
                Com_PathPlan    = MCOM_DATCLC;
                Com_PathExecute = MCOM_DATCLC;
                Com_SpeedCtrl   = MCOM_DATCLC;
                Com_ObjAvoid    = MCOM_DATCLC;
            }
            else
            {
                Com_Location    = MCOM_OFF;
                Com_PathPlan    = MCOM_OFF;
                Com_PathExecute = MCOM_OFF;
                Com_SpeedCtrl   = MCOM_OFF;
                Com_ObjAvoid    = MCOM_OFF;
            }
            break;

        case CarSta_Serching:
            if (ClearApaData() && clearCount == 0)
            {
                clearCount = 1;
            }
            else
            {
                clearCount = 0;
            }

            if (clearCount > 20 || clearCount == 0)
            {
                clearCount      = 0;
                Com_Location    = MCOM_ON;
                Com_PathPlan    = MCOM_ON;
                Com_PathExecute = MCOM_OFF;
                Com_SpeedCtrl   = MCOM_ON;
                Com_ObjAvoid    = MCOM_OFF;
            }
            else
            {
                clearCount++;
                Com_Location    = MCOM_OFF;
                Com_PathPlan    = MCOM_OFF;
                Com_PathExecute = MCOM_OFF;
                Com_SpeedCtrl   = MCOM_OFF;
                Com_ObjAvoid    = MCOM_OFF;
            }

            break;

        case CarSta_GuidanceAvpActive:
            Com_Location    = MCOM_ON;
            Com_PathPlan    = MCOM_ON;
            Com_PathExecute = MCOM_ON_AVMPARKING;
            Com_SpeedCtrl   = MCOM_ON;
            Com_ObjAvoid    = MCOM_ON_PARKING;
            break;

        case CarSta_GuidanceActive:
            Com_Location    = MCOM_ON;
            Com_PathPlan    = MCOM_ON_PARKING;
            Com_PathExecute = MCOM_ON_PARKING;
            Com_SpeedCtrl   = MCOM_ON;
            Com_ObjAvoid    = MCOM_ON_PARKING;
            break;

        case CarSta_GuidanceSuspend:
            Com_Location    = MCOM_ON;
            Com_PathPlan    = MCOM_ON_PARKING;
            Com_PathExecute = MCOM_ON_PARKING;
            Com_SpeedCtrl   = MCOM_ON;
            Com_ObjAvoid    = MCOM_ON_PARKING;
            break;

        case CarSta_GuidanceTerminated:
            Com_Location    = MCOM_DATCLC;
            Com_PathPlan    = MCOM_DATCLC;
            Com_PathExecute = MCOM_DATCLC;
            Com_SpeedCtrl   = MCOM_DATCLC;
            Com_ObjAvoid    = MCOM_DATCLC;
            break;

        case CarSta_GuidanceCompleted:
            Com_Location    = MCOM_DATCLC;
            Com_PathPlan    = MCOM_DATCLC;
            Com_PathExecute = MCOM_DATCLC;
            Com_SpeedCtrl   = MCOM_DATCLC;
            Com_ObjAvoid    = MCOM_DATCLC;
            break;

        case CarSta_Failure:
            Com_Location    = MCOM_DATCLC;
            Com_PathPlan    = MCOM_DATCLC;
            Com_PathExecute = MCOM_DATCLC;
            Com_SpeedCtrl   = MCOM_DATCLC;
            Com_ObjAvoid    = MCOM_DATCLC;
            break;

        case CarSta_ParkAssist_Standby:
            Com_Location    = MCOM_ON;
            Com_PathPlan    = MCOM_ON;
            Com_PathExecute = MCOM_ON;
            Com_SpeedCtrl   = MCOM_ON;
            Com_ObjAvoid    = MCOM_ON;
            break;

        case CarSta_GuidanceInitParkingData:
            if (last_CarState != CarSta_GuidanceInitParkingData)
            {
                Com_Location    = MCOM_DATCLC;
                Com_PathPlan    = MCOM_DATCLC;
                Com_PathExecute = MCOM_DATCLC;
                Com_SpeedCtrl   = MCOM_DATCLC;
                Com_ObjAvoid    = MCOM_DATCLC;
            }
            else
            {
                Com_Location    = MCOM_OFF;
                Com_PathPlan    = MCOM_OFF;
                Com_PathExecute = MCOM_OFF;
                Com_SpeedCtrl   = MCOM_OFF;
                Com_ObjAvoid    = MCOM_OFF;
            }
            break;

        case CarSta_Passive_SpdOver:
            Com_Location    = MCOM_OFF;
            Com_PathPlan    = MCOM_OFF;
            Com_PathExecute = MCOM_OFF;
            Com_SpeedCtrl   = MCOM_OFF;
            Com_ObjAvoid    = MCOM_OFF;
            break;

        default:
            Com_Location    = MCOM_DATCLC;
            Com_PathPlan    = MCOM_DATCLC;
            Com_PathExecute = MCOM_DATCLC;
            Com_SpeedCtrl   = MCOM_DATCLC;
            Com_ObjAvoid    = MCOM_DATCLC;
            break;
    }

    if (preCom_Location != Com_Location)
    {
        RTE_PK_StateManage_Set_ModuleCom_Location(Com_Location);
    }

    if (preCom_PathPlan != Com_PathPlan)
    {
        RTE_PK_StateManage_Set_ModuleCom_PathPlan(Com_PathPlan);
    }
    if (preCom_PathExecute != Com_PathExecute)
    {
        RTE_PK_StateManage_Set_ModuleCom_PathExecute(Com_PathExecute);
    }
    if (preCom_SpeedCtrl != Com_SpeedCtrl)
    {
        RTE_PK_StateManage_Set_ModuleCom_SpeedCtrl(Com_SpeedCtrl);
    }
    if (preCom_ObjAvoid != Com_ObjAvoid)
    {
        RTE_PK_StateManage_Set_ModuleCom_ObjAvoid(Com_ObjAvoid);
    }

    preCom_Location    = Com_Location;
    preCom_PathPlan    = Com_PathPlan;
    preCom_PathExecute = Com_PathExecute;
    preCom_SpeedCtrl   = Com_SpeedCtrl;
    preCom_ObjAvoid    = Com_ObjAvoid;
    last_CarState      = curState;
}

/**
 * @brief 更新状态机
 *
 */
void CarStateManage(void)
{
    static bool PowerInitFlg = false; // 上电初始化标志位

    m_StateMachineCnt++;
    if (false == PowerInitFlg)
    {
        PowerInitFlg = true;
        StateManage_DataInit(); // data init
        return;
    }

    // 状态切换时，至少保持10帧（猜测，1.保证HMI接收，2.异步线程，保证其他模块切换正常）
    // 重构应考虑与其他异步任务建立响应模式
    if (m_PreApaWorkState != m_CurrApaWorkState)
    {
        m_ApaStaKeepCount = 0;
        if (m_CurrApaWorkState == CarSta_Passive)
        {
            m_HMI_State       = HmiState_Exit;
            m_HMI_WorkState   = HmiWorkState_Passive;
            m_HMI_CurrApaText = ApaWorkingText_NoInfo;
            log_info("--------- APA change to CarSta_Passive --------\r\n");
        }
        log_warn("Apa Status: %s -----> %s\n", kCarWorkingStaStr[m_PreApaWorkState],
                 kCarWorkingStaStr[m_CurrApaWorkState]);
        m_PreApaWorkState = m_CurrApaWorkState;
    }
    else
    {
        m_ApaStaKeepCount++;
        if (m_ApaStaKeepCount <= 10)
        {
            return;
        }
    }

    // 更新外部数据
    // 获取其它模块的泊车状态
    GetOtherModuleStatus();
    // 获取来自HMI的用户按钮状态
    RTE_PK_SM_GetHmiBtInfo(&m_HmiButtonInfo);
    // 设置 APA 模式
    RTE_PK_SM_SetApaMode((HMI_ApaModeType)(m_HmiButtonInfo.HmiApaMode));
    // 获取HMI状态信息
    GetApaSwitchSta();

    // 状态管理条件
    CarDoorStatus door_state            = m_CarDoorCurrStatus();
    HMI_ApaModeType apaMode             = RTE_PK_SM_GetApaMode();
    RTE_BSW_RelateCtrlStatType ctrlStat = OtherRelateModulesShakehandIsOk();
    HMI_ButtonStateType UserButtonSta   = GetUserButtonSta();
    CarSafeBeltStatus SafeBeltStatus    = CarSafeBeltCurrStatus();

    bool isRemoteMode       = m_isRemoteMode();
    bool isObjInPath        = ObstacleIsInPath();
    bool isDriverOverride   = DriverOverrideCheck();
    bool isSearched         = SlotIsSearched();
    bool isSelected         = SlotIsSelected();
    bool InternalStaIsOk    = m_InternalStaIsOk();
    bool ExternalStaIsOk    = m_ExternalStaIsOk();
    bool isBraked           = RTE_BSW_Get_BrakeSt();
    bool SystemStaIsOk      = (ExternalStaIsOk && InternalStaIsOk);
    bool isStop             = (CurrCarSpeedValue() < RTE_BSW_Get_ESC_MinVehSpd());
    bool isOverSpeed        = (CurrCarSpeedValue() > APA_SPEE_LIMIT_MAX_WARNING);
    bool isOverSpeedExit    = (CurrCarSpeedValue() > APA_SPEE_LIMIT_MAX_STOP);
    bool isPauseG           = (RTE_BSW_Get_CurrentGear() == GEAR_REAL_P);
    bool isDoorClosed       = (AllClosed == door_state);
    bool isOrvmFolded       = OutsideRearviewMirrorIsFolded();
    bool isSoltEmpty        = IsApaSlotDataEmpty();
    bool isIPBStandby       = m_IpbModulesShakehandIsStandby();
    bool isOutKeep          = OutParkStatusKeepOnPassive();
    bool isLowSpeedKeep     = LowSpeedStatusKeepOnPassive();
    bool isOverSpeedTimeout = OverSpeedStatusOverOnSpdOver();
    bool isRadarAlarm       = ReportRadarInfo();
    // 状态管理
    switch (m_CurrApaWorkState)
    {
        case CarSta_PowerOn:
            m_HMI_State       = HmiState_Invalid;
            m_HMI_WorkState   = HmiWorkState_Passive;
            m_HMI_CurrApaText = ApaWorkingText_NoInfo;
            if (!SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            else
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            break;

        case CarSta_Passive:

            // 用户退出泊车
            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                // 退出状态维持5帧
                if (true == isOutKeep)
                {
                    m_HMI_State = HmiState_Exit;
                }
                else
                {
                    m_HMI_State = HmiState_Invalid;
                }
            }
            else
            {
                m_HMI_State = HmiState_Invalid;
            }

            m_HMI_WorkState   = HmiWorkState_Passive;
            m_HMI_CurrApaText = ApaWorkingText_NoInfo;

            // 系统自检异常
            if (false == SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            // 车速小于25kph保持5帧，同时用户开启APA功能
            else
            {
                if ((true == isLowSpeedKeep) && (true == m_ApaModeSwitchStatus))
                {
                    m_CurrApaWorkState = CarSta_Serching;
                }
            }
            break;

        case CarSta_Serching:
            // 默认赋值
            m_HMI_State     = HmiState_Active;
            m_HMI_WorkState = HmiWorkState_Serching;

            // 1. 系统/APA 故障 → Failure
            if (!SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            // 2. 模式开关关闭 / 点击退出 → Passive
            else if (!m_ApaModeSwitchStatus || ButtonState_OutPark == UserButtonSta)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            // 3. 超速 → Passive_SpdOver
            else if (isOverSpeed)
            {
                m_CurrApaWorkState = CarSta_Passive_SpdOver;
                m_HMI_CurrApaText  = ApaWorkingText_ReduceSpdInSerch;
            }
            // 4. 从 APA_MODE 切换到其他模式 → InitParkingData
            else if (m_preApaMode == APA_MODE && apaMode != APA_MODE &&
                     isSoltEmpty == false)
            {
                m_CurrApaWorkState = CarSta_GuidanceInitParkingData;
                m_HMI_CurrApaText  = ApaWorkingText_Resever;
            }
            // 5. 满足 Standby 条件
            else
            {
                // 雷达安全警告
                if (true == isRadarAlarm)
                {
                    m_HMI_CurrApaText = ApaWorkingText_RadarToNearAlarm;
                }
                // 5.1 车门未关
                else if (!isDoorClosed) //
                {
                    switch (door_state)
                    {
                        case CarDoorStatus::BonnetOpen:
                            m_HMI_CurrApaText = ApaWorkingText_CloseFrontHatchCover;
                            break;
                        case CarDoorStatus::TrunkOpen:
                            m_HMI_CurrApaText = ApaWorkingText_CloseTrunk;
                            break;
                        case CarDoorStatus::DoorOpen:
                            m_HMI_CurrApaText = ApaWorkingText_CloseDoor;
                            break;
                        default:
                            break;
                    }
                }
                // 5.2 后视镜
                else if (isOrvmFolded)
                {
                    m_HMI_CurrApaText = ApaWorkingText_OpenRearviewMirror;
                }
                // 5.2 安全带
                else if (SafeBeltStatus != CarSafeBeltFastened && false == isRemoteMode)
                {
                    m_HMI_CurrApaText = ApaWorkingText_SeatBeltNotFastened;
                }
                // 5.3 找到车位,或者自选车位(isSearched == false)
                else if (true == isSearched || true == isSelected)
                {
                    // 有车位选中，并且车辆处于安全静止的状态
                    if ((isStop && (isPauseG || isBraked)) && isSelected)
                    {
                        // 所有条件满足 → 进入 Standby
                        m_CurrApaWorkState = CarSta_ParkAssist_Standby;
                    }
                    // 车辆不是完全安全静止的，提示踩刹车
                    else
                    {
                        if (apaMode == APA_MODE && isStop && isBraked == false)
                        {
                            m_HMI_CurrApaText = ApaWorkingText_Serching_Park;
                        }
                        else if (isStop && apaMode == PARKOUT_MODE && !isSelected &&
                                 (true == isSearched))
                        {
                            m_HMI_CurrApaText = ApaWorkingText_ParkOut_Select;
                        }
                        else
                        {
                            m_HMI_CurrApaText = ApaWorkingText_Serching_NoticePG_Brake;
                        }
                    }
                }
                else
                {
                    m_HMI_CurrApaText = ApaWorkingText_Serching_NoPark;
                }
            }
            break;

        case CarSta_ParkAssist_Standby:
            // 泊车界面显现
            m_HMI_State     = HmiState_Active;
            m_HMI_WorkState = HmiWorkState_ParkAssist_Standby;

            // 系统故障
            if (SystemStaIsOk == false)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            // 用户退出泊车
            else if (ButtonState_OutPark == UserButtonSta ||
                     false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            // 底盘关联件握手之前，车辆不是完全安全静止的，退回到搜车位状态
            else if ((isStop == false || (isPauseG == false && isBraked == false)) &&
                     (CTRL_AVAILABLE == ctrlStat))
            {
                m_CurrApaWorkState = CarSta_Serching;
            }
            // 底盘关联件成功握手
            else if (ctrlStat == CTRL_ACTIVE)
            {
                // 雷达安全警告
                if (true == isRadarAlarm)
                {
                    m_HMI_CurrApaText = ApaWorkingText_RadarToNearAlarm;
                }
                else if (isBraked == true)
                {
                    m_HMI_CurrApaText = ApaWorkingText_Parking_RealseBrakeToStart;
                }
                // 等待关联件换挡启动，松手刹启动
                // else if (false == EpbIsPullup() && GEAR_P != GetCurrentGear())
                // {
                //     m_HMI_CurrApaText = ApaWorkingText_Resever;
                // }
                else
                {
                    m_CurrApaWorkState = CarSta_GuidanceActive;
                    m_HMI_CurrApaText  = ApaWorkingText_IsParking;
                    if (apaMode == AVP_MODE)
                    {
                        m_CurrApaWorkState = CarSta_GuidanceAvpActive;
                    }
                }
            }
            // 等待握手成功
            else
            {
                m_HMI_CurrApaText = ApaWorkingText_Resever;
            }

            break;

        case CarSta_GuidanceAvpActive:
        case CarSta_GuidanceActive:

            // 泊车界面显现
            m_HMI_State     = HmiState_Active;
            m_HMI_WorkState = HmiWorkState_GuidanceActive;

            // 关联系统故障
            if (!SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            // 用户退出泊车
            else if (ButtonState_OutPark == UserButtonSta ||
                     false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            // 不可恢复中断：
            // 1.驾驶员干预
            // 2.关联件处于非握手状态
            // 3.路径追踪状态异常
            else if (true == isDriverOverride || CTRL_AVAILABLE == ctrlStat ||
                     (MSTAT_ABNORM == m_State_PathExecute))
            {
                m_CurrApaWorkState = CarSta_GuidanceTerminated;
                m_HMI_WorkState    = HmiWorkState_GuidanceTerminated;
                if (MSTAT_ABNORM == m_State_PathExecute) // 泊车失败
                {
                    m_HMI_CurrApaText = ApaWorkingText_GuidanceStop;
                }
                else if (isDriverOverride)
                {
                    m_HMI_CurrApaText = ApaWorkingText_DriverIntervent;
                }
                else if (CTRL_AVAILABLE == ctrlStat)
                {
                    m_HMI_CurrApaText = ApaWorkingText_UltronicFail;
                }
            }
            // 泊车完成
            else if (MSTAT_COMPLETE == m_State_PathExecute)
            {
                m_CurrApaWorkState = CarSta_GuidanceCompleted;
                m_HMI_WorkState    = HmiWorkState_GuidanceCompleted;
                m_HMI_CurrApaText  = ApaWorkingText_GuidanceCompleted;
            }
            // 可恢复中断
            // 1.四门两盖
            // 2.后视镜
            // 3.安全带
            // 4.用户主动暂停
            // 5.路径中存在障碍物
            else if ((false == isDoorClosed) || (true == isOrvmFolded) ||
                     (SafeBeltStatus != CarSafeBeltFastened && false == isRemoteMode) ||
                     (ButtonState_Suspend == UserButtonSta) || (true == isObjInPath))
            {
                m_CurrApaWorkState = CarSta_GuidanceSuspend;
            }
            // 泊车中
            else
            {
                // 雷达安全警告
                if (true == isRadarAlarm)
                {
                    m_HMI_CurrApaText = ApaWorkingText_RadarToNearAlarm;
                }
                else if (isBraked)
                {
                    m_HMI_CurrApaText = ApaWorkingText_Parking_RealseBrake;
                }
                else
                {
                    m_HMI_CurrApaText = ApaWorkingText_IsParking;
                }
            }
            break;

        case CarSta_GuidanceSuspend:
            // 泊车界面显现
            m_HMI_State     = HmiState_Active;
            m_HMI_WorkState = HmiWorkState_GuidanceSuspend;

            // 关联系统故障
            if (!SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            // 用户退出泊车
            else if (ButtonState_OutPark == UserButtonSta ||
                     false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            // 不可恢复中断：
            // 1.驾驶员干预
            // 2.关联件处于非握手状态
            // 3.路径追踪状态异常
            else if ((MSTAT_ABNORM == m_State_PathExecute) || true == isDriverOverride ||
                     CTRL_AVAILABLE == ctrlStat)
            {
                m_CurrApaWorkState = CarSta_GuidanceTerminated;
                m_HMI_WorkState    = HmiWorkState_GuidanceTerminated;
            }
            // 可恢复中断
            // 1.四门两盖
            // 2.后视镜
            // 3.安全带
            // 4.用户主动暂停
            // 5.路径中存在障碍物
            else if ((true == isDoorClosed) && (false == isOrvmFolded) &&
                     (SafeBeltStatus == CarSafeBeltFastened || true == isRemoteMode) &&
                     (ButtonState_Continue == UserButtonSta ||
                      ButtonState_IntoAct == UserButtonSta) &&
                     (false == isObjInPath))
            {
                m_CurrApaWorkState = CarSta_GuidanceActive;
            }
            else
            {
                m_CurrApaWorkState = CarSta_GuidanceSuspend;
                if (false == isDoorClosed)
                {
                    m_HMI_CurrApaText =
                        door_state == BonnetOpen
                            ? ApaWorkingText_CloseFrontHatchCover
                            : (door_state == TrunkOpen ? ApaWorkingText_CloseTrunk
                                                       : ApaWorkingText_CloseDoor);
                }
                else if (true == isOrvmFolded)
                {
                    m_HMI_CurrApaText = ApaWorkingText_OpenRearviewMirror;
                }
                else if (SafeBeltStatus != CarSafeBeltFastened && false == isRemoteMode)
                {
                    m_HMI_CurrApaText = ApaWorkingText_SeatBeltNotFastened;
                }
                else if (ButtonState_Suspend == UserButtonSta)
                {
                    m_HMI_CurrApaText = ApaWorkingText_GuidanceSuspend;
                }
                else if (true == isObjInPath)
                {
                    m_HMI_CurrApaText = ApaWorkingText_GuidanceSuspend;
                }
                else
                {
                    ///< 这里是否是异常
                    m_HMI_CurrApaText = ApaWorkingText_GuidanceSuspend;
                }
            }
            break;

        case CarSta_GuidanceTerminated:
            m_HMI_State     = HmiState_Active;
            m_HMI_WorkState = HmiWorkState_GuidanceTerminated;

            // 点击泊车退出按钮，退出泊车
            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
                m_HMI_State        = HmiState_Exit;
            }
            else
            {
                if (MSTAT_ABNORM == m_State_PathExecute) // 泊车界面显现泊车失败
                {
                    m_HMI_CurrApaText = ApaWorkingText_GuidanceStop;
                }
                else if (isDriverOverride)
                {
                    m_HMI_CurrApaText = ApaWorkingText_DriverIntervent;
                }
                else if (CTRL_AVAILABLE == ctrlStat)
                {
                    m_HMI_CurrApaText = ApaWorkingText_UltronicFail;
                }
                else
                {
                    m_HMI_CurrApaText = ApaWorkingText_Resever;
                }
            }

            break;

        case CarSta_GuidanceCompleted:

            m_HMI_State       = HmiState_Active;
            m_HMI_WorkState   = HmiWorkState_GuidanceCompleted;
            m_HMI_CurrApaText = ApaWorkingText_GuidanceCompleted;

            // 点击泊车退出按钮，退出泊车
            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
                m_HMI_State        = HmiState_Exit;
            }
            break;

        case CarSta_Failure:
            // 泊车界面
            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                m_HMI_State = HmiState_Exit;
            }
            else
            {
                m_HMI_State = HmiState_Active;
            }
            m_HMI_WorkState   = HmiWorkState_Failure;
            m_HMI_CurrApaText = ApaWorkingText_GuidanceStop;

            // 系统故障不允许跳出该状态
            if (!SystemStaIsOk)
            {
                m_CurrApaWorkState = CarSta_Failure;
            }
            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }

            break;

        case CarSta_GuidanceInitParkingData:

            m_HMI_State       = HmiState_Active;
            m_HMI_WorkState   = HmiWorkState_Serching;
            m_HMI_CurrApaText = ApaWorkingText_NoInfo;

            if (ButtonState_OutPark == UserButtonSta || false == m_ApaModeSwitchStatus)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }
            // 车位为空作为初始化完成的判断条件？
            else if (true == isSoltEmpty)
            {
                m_CurrApaWorkState = CarSta_Serching;
            }
            else
            {
                // 等待模块初始化完成
                // 雷达安全警告
                if (true == isRadarAlarm)
                {
                    m_HMI_CurrApaText = ApaWorkingText_RadarToNearAlarm;
                }
            }

            break;

        case CarSta_Passive_SpdOver:
            m_HMI_State       = HmiState_Active;
            m_HMI_WorkState   = HmiWorkState_Serching;
            m_HMI_CurrApaText = ApaWorkingText_ReduceSpdInSerch;

            // 点击泊车退出按钮或者10s超时或者退出超速模式，退出泊车
            if (false == isOverSpeed || true == isOverSpeedTimeout ||
                true == isOverSpeedExit || ButtonState_OutPark == UserButtonSta)
            {
                m_CurrApaWorkState = CarSta_Passive;
            }

            break;

        default:
            m_CurrApaWorkState = CarSta_Passive;

            break;
    }

    RTE_SM_Set_HMI_State(m_HMI_State);
    RTE_SM_Set_HMI_WorkState(m_HMI_WorkState);
    RTE_SM_Set_HMI_WorkTextInfo(m_HMI_CurrApaText);
    RTE_SM_Set_ApaWorkState(m_CurrApaWorkState);
    m_preApaMode = apaMode;
}

/**
 * @brief 状态机模块主函数
 *
 */
void PK_SM_Main(void)
{
    CarStateManage(); // 整车状态管理
    UpdateModComState(m_CurrApaWorkState);
}
