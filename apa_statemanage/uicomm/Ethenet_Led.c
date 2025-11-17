#include "EthIf.h"
#include "CanIf.h"
#include "SystemPara.h"
#include "PK_Calibration.h"
#include "Record_Log.h"
#include "MathFunc.h"
#include "Rte.h"
#include "Rte_Func.h"
#include "Debug_Cmd.h"
#include "json/json.h"
#include "PK_StateManage.h"
#include "PK_PathExecute.h"
#include "Ethenet_HMICanData.h"
#include "Rte_ComIF.h"
#include <fstream>

#define APA2HMI_PORT 18805
#define HMI2APA_PORT 18804

static FILE *sendHMIlog = NULL;

static inline FILE *OpenLogFile(const char *name)
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    if (absStamp.tm_year < 2000)
    {
        return NULL;
    }

#ifdef ANDROID_PLATFORM
    snprintf(fileName, sizeof(fileName) - 1, "./log/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, name);
#else
    snprintf(fileName, sizeof(fileName) - 1, "/tmp/apalog/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, name);
#endif
    FILE *logFile = fopen(fileName, "w");
    return logFile;
}

static uint8_t cfg_amvobj_report = 0;
static void UpdateSlotInfo(SlotInfo_T *slots, const int counter)
{
    float newtarg[3] = {0.0, 0.0, PI * 3};
    float rspoint[3], oldtarg[3];

#define adjustAvm(avmpoint)                     \
    {                                           \
        RevConvert(oldtarg, avmpoint, rspoint); \
        Convert(newtarg, rspoint, avmpoint);    \
    }

    for (int i = 0; i < counter - 1; i++)
    {
        if (slots[i].is_vision_slot == 0)
        {
            continue;
        }
        if (slots[i].slotshap != slots[i + 1].slotshap)
        {
            continue;
        }
        if (slots[i].slotshap != PK_SLOT_LEFT_VERT &&
            slots[i].slotshap != PK_SLOT_RIGHT_VERT)
        {
            continue;
        }
        if (fabs(slots[i].targpos.theta - slots[i + 1].targpos.theta) > PI_RAD * 15)
        {
            continue;
        }
        if (Cal_Dis_Pt2Pt(slots[i].targpos, slots[i + 1].targpos) > 3)
        {
            continue;
        }
        if (fabs(Project_PosTo1stPos_rx(slots[i].targpos, slots[i + 1].targpos)) > 0.5)
        {
            continue;
        }

        // adjust direction
        if (newtarg[2] > PI * 2)
        {
            newtarg[0] = slots[i].targpos.x;
            newtarg[1] = slots[i].targpos.y;
            newtarg[2] = (slots[i].targpos.theta + slots[i + 1].targpos.theta) / 2;
            memcpy(oldtarg, (float *)&slots[i].targpos, sizeof(oldtarg));
            adjustAvm((float *)&slots[i].avm_point.far_front);
            adjustAvm((float *)&slots[i].avm_point.far_rear);
            adjustAvm((float *)&slots[i].avm_point.near_rear);
            adjustAvm((float *)&slots[i].avm_point.near_front);
        }

        newtarg[0] = slots[i + 1].targpos.x;
        newtarg[1] = slots[i + 1].targpos.y;
        memcpy(oldtarg, (float *)&slots[i + 1].targpos, sizeof(oldtarg));
        adjustAvm((float *)&slots[i + 1].avm_point.far_front);
        adjustAvm((float *)&slots[i + 1].avm_point.far_rear);
        adjustAvm((float *)&slots[i + 1].avm_point.near_rear);
        adjustAvm((float *)&slots[i + 1].avm_point.near_front);

        newtarg[0] = slots[i].targpos.x;
        newtarg[1] = slots[i].targpos.y;
        RevConvert(newtarg, (float *)&slots[i].avm_point.near_rear, rspoint);
        auto distBx = fabs(rspoint[0]);
        auto distBy = fabs(rspoint[1]);

        RevConvert(newtarg, (float *)&slots[i + 1].avm_point.near_front, rspoint);
        auto distEx = fabs(rspoint[0]);
        auto distEy = fabs(rspoint[1]);

        // adjust position
        if (distEy > 2.50f && fabs(distEy - distBy) > 0.20f)
        {
            auto disty = distBy;
            RevConvert(newtarg, (float *)&slots[i + 1].avm_point.far_front, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i + 1].avm_point.far_front);

            RevConvert(newtarg, (float *)&slots[i + 1].avm_point.near_front, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i + 1].avm_point.near_front);
        }
        else if (distBy > 2.50f && fabs(distEy - distBy) > 0.20f)
        {
            auto disty = distEy;
            RevConvert(newtarg, (float *)&slots[i].avm_point.far_rear, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i].avm_point.far_rear);

            RevConvert(newtarg, (float *)&slots[i].avm_point.near_rear, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i].avm_point.near_rear);
        }
        else if (fabs(distEy - distBy) < 0.2f)
        {
            auto disty = (distEy + distBy) * 0.5f;
            RevConvert(newtarg, (float *)&slots[i].avm_point.far_rear, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i].avm_point.far_rear);

            RevConvert(newtarg, (float *)&slots[i].avm_point.near_rear, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i].avm_point.near_rear);

            RevConvert(newtarg, (float *)&slots[i + 1].avm_point.far_front, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i + 1].avm_point.far_front);

            RevConvert(newtarg, (float *)&slots[i + 1].avm_point.near_front, rspoint);
            rspoint[1] = sign(rspoint[1]) * disty;
            Convert(newtarg, rspoint, (float *)&slots[i + 1].avm_point.near_front);
        }

        if (fabs(distEx - distBx) < 0.1f)
        {
            memcpy(oldtarg, (float *)&slots[i + 1].targpos, sizeof(oldtarg));
            oldtarg[2] = newtarg[2];

            float temptarg[3];
            RevConvert(newtarg, oldtarg, rspoint);
            rspoint[0] = 0;
            Convert(newtarg, rspoint, temptarg);
            memcpy(newtarg, temptarg, sizeof(newtarg));
            newtarg[2] = oldtarg[2];

            adjustAvm((float *)&slots[i + 1].avm_point.far_front);
            adjustAvm((float *)&slots[i + 1].avm_point.far_rear);
            adjustAvm((float *)&slots[i + 1].avm_point.near_rear);
            adjustAvm((float *)&slots[i + 1].avm_point.near_front);
        }
    }
}

static void TextSlotInfo(std::string &buff, const SlotInfo_T *slots, const int counter,
                         const float curPos[4])
{
    auto pointStrFunc = [&curPos](std::string &str, const float *point, int index) {
        float rPoint[3];
        RevConvert(curPos, point, rPoint);
        str += std::to_string(index);
        str += ' ';
        str += std::to_string(rPoint[0]);
        str += ' ';
        str += std::to_string(rPoint[1]);
        str += ',';
    };

    for (int slot = 0; slot < counter; slot++)
    {
        if (slots[slot].is_vision_slot == 0)
        {
            pointStrFunc(buff, (const float *)&slots[slot].slotobj.ptB,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].slotobj.ptC,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].slotobj.ptD,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].slotobj.ptE,
                         slots[slot].slot_index);
        }
        else
        {
            pointStrFunc(buff, (const float *)&slots[slot].avm_point.near_rear,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].avm_point.far_rear,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].avm_point.far_front,
                         slots[slot].slot_index);
            pointStrFunc(buff, (const float *)&slots[slot].avm_point.near_front,
                         slots[slot].slot_index);
        }
    }
}
/**
 * @brief   上报车辆车门状态到字符串缓冲区
 * @details 从 BCM（车身控制模块）读取车门、引擎盖、后备箱等开关状态，
 *          将状态封装为 CAN 数据结构，并计算校验码后写入 buff。
 *
 * @param[out] buff 字符串缓冲区，用于存放封装好的车门状态数据（包含 CAN ID 与数据体）
 */
static void ReportCarDoorStatus(std::string &buff)
{
    // 读取 BCM 车门状态数据
    RTE_BSW_BCM_StateType carDoorStatus;
    RTE_BSW_Get_BCM_VehicleState(&carDoorStatus);

    // 定义并初始化 CAN 数据结构
    CarDoorCanData carDoorData;
    carDoorData.canId = CanId_CarDoorStatus; // 车门状态 CAN ID

    // 填充各个门的状态（0 关，1 开）
    carDoorData.carDoorStatus.car_door_fl = carDoorStatus.BCM_DriverDoorAjarSt; // 主驾门
    carDoorData.carDoorStatus.car_door_fr = carDoorStatus.BCM_PsngrDoorAjarSt;  // 副驾门
    carDoorData.carDoorStatus.car_door_rl = carDoorStatus.BCM_RLDoorAjarSt;     // 左后门
    carDoorData.carDoorStatus.car_door_rr = carDoorStatus.BCM_RRDoorAjarSt;     // 右后门
    carDoorData.carDoorStatus.car_hood    = carDoorStatus.BCM_BonnetAjarSt;     // 引擎盖
    carDoorData.carDoorStatus.car_trunk   = carDoorStatus.RDM_RearDoorStatus;   // 后备箱

    // 保留字段置零
    carDoorData.carDoorStatus.empty_data  = 0;
    carDoorData.carDoorStatus.empty_data1 = 0;
    carDoorData.carDoorStatus.empty_data2 = 0;

    // 计算校验码（基于车门状态结构体内容）
    carDoorData.carDoorStatus.checksum =
        calculateChecksum(reinterpret_cast<const uint8_t *>(&carDoorData.carDoorStatus));

    // 将结构体序列化到输出缓冲区
    buff.clear();
    buff.append(reinterpret_cast<const char *>(&carDoorData), VEH_POSITION_SIZE);
}

static int DataSlotInfo(slotPosition &slot_position, int point_num, const float curPos[4],
                        const SlotInfo_T *slots, const int counter, const int effective)
{
    for (int i = 0; i < counter; i++)
    {
        int stopflags[4] = {0};
        stopflags[0]     = slots[i].is_vision_slot > 0 ? slots[i].has_stopper : 0;
        stopflags[1]     = effective;
        // 0-no lock 1 - lock on 2 - lock off 3 - occupied;
        if (slots[i].occupied == 1)
        {
            stopflags[2] = 1;
            stopflags[3] = 1;
            stopflags[1] = 0;
        }
        else if (slots[i].occupied == 2)
        {
            stopflags[2] = 1;
            stopflags[3] = 0;
        }
        else
        {
            stopflags[2] = 0;
            stopflags[3] = 0;
            if (slots[i].occupied == 3)
            {
                stopflags[1] = 0;
            }
        }

        float rPoint[3];
        RevConvert(curPos, (const float *)&slots[i].slotobj.ptB, rPoint);
        // printf("report:%d slotid: %d occupied:%d stopflag:%d %d %d %d\n", report_index,
        // slots[i].slot_index, slots[i].occupied, stopflags[0], stopflags[1],
        // stopflags[2], stopflags[3]);

        BuildSlotPosition(&slot_position.slot_position[point_num],
                          (const float *)&slots[i].slotobj.ptB, curPos,
                          slots[i].slot_index, stopflags, point_num + 1);
        if (sendHMIlog != NULL)
            fprintf(sendHMIlog, "%d, %d, %d, %06f",
                    slot_position.slot_position[point_num].slot_index, stopflags[1],
                    slot_position.slot_position[point_num].point_x, rPoint[0]);

        point_num++;

        BuildSlotPosition(&slot_position.slot_position[point_num],
                          (const float *)&slots[i].slotobj.ptC, curPos,
                          slots[i].slot_index, stopflags, point_num + 1);
        point_num++;

        BuildSlotPosition(&slot_position.slot_position[point_num],
                          (const float *)&slots[i].slotobj.ptD, curPos,
                          slots[i].slot_index, stopflags, point_num + 1);
        point_num++;

        BuildSlotPosition(&slot_position.slot_position[point_num],
                          (const float *)&slots[i].slotobj.ptE, curPos,
                          slots[i].slot_index, stopflags, point_num + 1);
        point_num++;
    }
    return point_num;
}

static void SwepSlot0(SlotInfo_T *slots, int slotId)
{
    SlotInfo_T temp;
    if (slotId <= 0)
    {
        return;
    }
    memcpy((SlotInfo_T *)&temp, (SlotInfo_T *)&slots[slotId], sizeof(SlotInfo_T));
    memcpy((SlotInfo_T *)&slots[slotId], (SlotInfo_T *)&slots[0], sizeof(SlotInfo_T));
    memcpy((SlotInfo_T *)&slots[0], (SlotInfo_T *)&temp, sizeof(SlotInfo_T));
}

static int IdxInPathPlan(const PlanInfoType *slotPlans, int slotNum,
                         const SlotInfo_T &slot)
{
    for (int plan = 0; plan < slotNum; plan++)
    {
        if (CompareSlotIndex(slot.slot_index, slotPlans[plan].Slot_index))
        {
            return plan;
        }
    }
    return -1;
}

static int IdxInSlots(const SlotInfo_T *slots, int slotNum, const SlotInfo_T &slot)
{
    for (int plan = 0; plan < slotNum; plan++)
    {
        if (CompareSlotIndex(slot.slot_index, slots[plan].slot_index))
        {
            return plan;
        }
    }
    return -1;
}

static int SlotSelect(const MultiPlanInfo &plansInfo, const Multi_Slot_Array_T &slotsInfo,
                      const float curPos[3], SlotInfo_T slots[2][MAX_PARKED_SLOTS / 2],
                      int counter[2], SlotInfo_T slotsOff[2][MAX_PARKED_SLOTS / 2],
                      int OffCounter[2])
{
    int defaultSlot = -1;
    counter[0] = counter[1] = 0;
    OffCounter[0] = OffCounter[1] = 0;

    float nearDist = 100.0f;
    for (int slot = 0; slot < slotsInfo.multiNum; slot++)
    {
        int slotId = IdxInPathPlan(plansInfo.slotPlans, plansInfo.slotNum,
                                   slotsInfo.multiArray[slot]);
        if (slotId >= 0)
        {
            float dist =
                Cal_Dis_Pt2Pt((float *)&slotsInfo.multiArray[slot].targpos, curPos);
            if (Slot_Dir(slotsInfo.multiArray[slot].slotshap) == 1 &&
                counter[0] < MAX_PARKED_SLOTS / 2)
            {
                memcpy(&slots[0][counter[0]], &slotsInfo.multiArray[slot],
                       sizeof(SlotInfo_T));
                if (dist < nearDist)
                {
                    defaultSlot = 1;
                    nearDist    = dist;
                    SwepSlot0(slots[0], counter[0]);
                }
                counter[0] = counter[0] + 1;
            }
            else if (Slot_Dir(slotsInfo.multiArray[slot].slotshap) == 2 &&
                     counter[1] < MAX_PARKED_SLOTS / 2)
            {
                memcpy(&slots[1][counter[1]], &slotsInfo.multiArray[slot],
                       sizeof(SlotInfo_T));
                if (dist < nearDist)
                {
                    defaultSlot = 2;
                    nearDist    = dist;
                    SwepSlot0(slots[1], counter[1]);
                }
                counter[1] = counter[1] + 1;
            }

            // if (slotsInfo.multiArray[slot].slotshap <= 2)
            {
                continue;
            } // 路线规划成功或平行车位
        }

        if (Slot_Dir(slotsInfo.multiArray[slot].slotshap) == 1 &&
            OffCounter[0] < MAX_PARKED_SLOTS / 2)
        {
            memcpy(&slotsOff[0][OffCounter[0]], &slotsInfo.multiArray[slot],
                   sizeof(SlotInfo_T));
            OffCounter[0] = OffCounter[0] + 1;
        }
        else if (Slot_Dir(slotsInfo.multiArray[slot].slotshap) == 2 &&
                 counter[1] < MAX_PARKED_SLOTS / 2)
        {
            memcpy(&slotsOff[1][OffCounter[1]], &slotsInfo.multiArray[slot],
                   sizeof(SlotInfo_T));
            OffCounter[1] = OffCounter[1] + 1;
        }
    }

    SlotInfo_T manuSlot;
    RTE_PK_DataConv_Get_SlotInfo(&manuSlot);
    if (manuSlot.slot_index != 0 && manuSlot.slotshap != PK_SLOT_NO)
    {
        int slotId  = IdxInPathPlan(plansInfo.slotPlans, plansInfo.slotNum, manuSlot);
        defaultSlot = (Slot_Dir(manuSlot.slotshap) == 1) ? 1 : 2;
        if (slotId >= 0)
        {
            if (counter[defaultSlot - 1] >= MAX_PARKED_SLOTS / 2)
            {
                memcpy(&slots[defaultSlot - 1][MAX_PARKED_SLOTS / 2 - 1], &manuSlot,
                       sizeof(SlotInfo_T));
            }
            else
            {
                memcpy(&slots[defaultSlot - 1][counter[defaultSlot - 1]], &manuSlot,
                       sizeof(SlotInfo_T));
                counter[defaultSlot - 1] = counter[defaultSlot - 1] + 1;
            }

            SwepSlot0(slots[defaultSlot - 1], counter[defaultSlot - 1]);
            defaultSlot = defaultSlot + 2;
            // printf("report tempslot index %d %d\n", tempSlot.slot_index,
            // manuSlot.slot_index);
        }
        else
        {
            if (OffCounter[defaultSlot - 1] < MAX_PARKED_SLOTS / 2)
            {
                memcpy(&slotsOff[defaultSlot - 1][OffCounter[defaultSlot - 1]], &manuSlot,
                       sizeof(SlotInfo_T));
                OffCounter[defaultSlot - 1] = OffCounter[defaultSlot - 1] + 1;
            }
        }
    }

    return defaultSlot;
}
/**
 * @brief  向 HMI（人机交互界面）上报可搜索到的车位信息
 *
 * 主要功能：
 * 1. 获取当前路径规划、车位检测和定位信息。
 * 2. 将车位按照左右分组，并选取一个默认最近车位。
 * 3. 根据逻辑判断是否保留上一次的车位信息，防止车位闪烁。
 * 4. 将整理后的车位信息打包成 slotPosition 数据，通过 buff 传递给上层。
 *
 * @param[out] buff      输出字符串缓冲区，存放打包后的车位信息（原始二进制形式）。
 * @param[in,out] lastSlot 上一次使用的车位信息（会被更新）。
 */
static void ReportSearchSlotInfo(std::string &buff, SlotInfo_T &lastSlot)
{
    // slot_position: CAN 报文格式的车位信息结构体
    slotPosition slot_position;
    memset(&slot_position, 0, sizeof(slotPosition));
    slot_position.canId = CanId_SlotPosition; // 填写车位信息的 CAN ID

    // 1. 获取多路径规划信息
    MultiPlanInfo plansInfo;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&plansInfo);

    // 2. 获取多车位检测信息
    Multi_Slot_Array_T slotsInfo;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slotsInfo);

    // 3. 获取当前车辆位置（curPos[0] = X, curPos[1] = Y, curPos[2] = 车头朝向, curPos[3]
    // 备用）
    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);

    // slots[左右车道][车位数]：检测到的可用车位
    // slotsOff[左右车道][车位数]：离线/不可用车位
    SlotInfo_T slots[2][MAX_PARKED_SLOTS / 2], slotsOff[2][MAX_PARKED_SLOTS / 2];
    int counter[2], offCounter[2];

    // 4. 将车位按左右分组，并返回默认最近车位的方向（1=左，2=右）
    int defaultSlot =
        SlotSelect(plansInfo, slotsInfo, curPos, slots, counter, slotsOff, offCounter);

    // 5. 调整车位位置（坐标微调）
    UpdateSlotInfo(slots[0], counter[0]);
    UpdateSlotInfo(slots[1], counter[1]);
    UpdateSlotInfo(slotsOff[0], offCounter[0]);
    UpdateSlotInfo(slotsOff[1], offCounter[1]);

    // 调试日志（HMI）
    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "ReportSearchSlotInfo, %06ld:%06ld, curX: %06f, ",
                stamp.tv_sec, stamp.tv_usec, curPos[0]);

    // 7. 判断是否继续使用上次车位（防止 HMI 闪烁）
    for (int i = 0; i < 1; i++)
    {
        // 如果不是指定车位
        if (defaultSlot != 1 && defaultSlot != 2)
        {
            defaultSlot = defaultSlot - 2;
            break;
        }

        // 如果上一次没有有效车位
        if (lastSlot.slotshap == PK_SLOT_NO)
        {
            break;
        }

        // dist0: 上次车位与本次默认车位的距离
        // dist1: 本次车位与当前位置的距离
        // dist2: 上次车位与当前位置的距离
        float dist0 = Cal_Dis_Pt2Pt(slots[defaultSlot - 1][0].targpos, lastSlot.targpos);
        float dist1 = Cal_Dis_Pt2Pt((float *)&slots[defaultSlot - 1][0].targpos, curPos);
        float dist2 = Cal_Dis_Pt2Pt((float *)&lastSlot.targpos, curPos);

        // 如果新旧车位位置差异不大（或距离变化超过0.5m）则继续用老车位
        if (dist0 < VEHICLE_WID * 0.5f || dist2 - dist1 > 0.5f)
        {
            break;
        }

        // 查找 lastSlot 在本次车位列表的位置
        int index1 = IdxInSlots(slots[0], counter[0], lastSlot);
        int index2 = IdxInSlots(slots[1], counter[1], lastSlot);

        // 如果两边都没找到，说明原车位无效
        if (index1 < 0 && index2 < 0)
        {
            break;
        }

        // 如果在左侧找到则置为默认左侧车位
        if (index1 >= 0)
        {
            defaultSlot = 1;
            SwepSlot0(slots[0], index1); // 将该车位移到列表首位
        }

        // 如果在右侧找到则置为默认右侧车位
        if (index2 >= 0)
        {
            defaultSlot = 2;
            SwepSlot0(slots[1], index2);
        }
    }

    // 8. 打包车位数据（优先打包默认侧的车位，再打另一侧，再打离线车位）
    if (defaultSlot == 1 || defaultSlot == 2)
    {
        int pointnum = 0;
        pointnum = DataSlotInfo(slot_position, pointnum, curPos, slots[defaultSlot - 1],
                                counter[defaultSlot - 1], 1);
        pointnum = DataSlotInfo(slot_position, pointnum, curPos, slots[2 - defaultSlot],
                                counter[2 - defaultSlot], 1);
        pointnum = DataSlotInfo(
            slot_position, pointnum, curPos, slotsOff[defaultSlot - 1],
            rte_min(4 - counter[defaultSlot - 1], offCounter[defaultSlot - 1]), 0);
        pointnum = DataSlotInfo(
            slot_position, pointnum, curPos, slotsOff[2 - defaultSlot],
            rte_min(4 - counter[2 - defaultSlot], offCounter[2 - defaultSlot]), 0);

        // 更新 lastSlot 为当前默认车位
        memcpy(&lastSlot, &slots[defaultSlot - 1][0], sizeof(lastSlot));
    }

    // 如果没有可用车位，但有离线车位，则只发离线车位
    if (counter[0] + counter[1] == 0 && offCounter[0] + offCounter[1] != 0)
    {
        int pointnum = 0;
        pointnum =
            DataSlotInfo(slot_position, pointnum, curPos, slotsOff[0], offCounter[0], 0);
        pointnum =
            DataSlotInfo(slot_position, pointnum, curPos, slotsOff[1], offCounter[1], 0);
    }

    // 9. 添加时间戳到最后一个 slot_position[32]
    uint64_t msStamp = RTE_BSW_Get_CurTime();
    uint8_t stampbuf[8];
    for (int i = 0; i < 8; i++)
    {
        stampbuf[i] = (msStamp >> (i * 8)) & 0xff;
    }
    slot_position.slot_position[32].canId = 32;
    uint8_t *dataState =
        (uint8_t *)&slot_position.slot_position[32] + sizeof(slotPosition::canId);
    memcpy(dataState, stampbuf, sizeof(stampbuf));

    if (sendHMIlog != NULL)
    {
        fprintf(sendHMIlog, "\n");
    }

    // 10. 将 slot_position 二进制数据写入 buff
    buff.clear();
    buff.append(reinterpret_cast<const char *>(&slot_position), sizeof(slot_position));
}

static void ReportActiveSlotInfo(std::string &buff)
{
    SlotObj_T slotobj;
    int stopflags[4] = {0, 1, 0, 0};

    SlotInfo_T manuSlot;
    RTE_PK_DataConv_Get_SlotInfo(&manuSlot);
    if (manuSlot.slot_index != 0 && manuSlot.slotshap != PK_SLOT_NO)
    {
        memcpy((float *)&slotobj.ptB, (float *)&manuSlot.avm_point.near_rear,
               sizeof(Point_T));
        memcpy((float *)&slotobj.ptC, (float *)&manuSlot.avm_point.far_rear,
               sizeof(Point_T));
        memcpy((float *)&slotobj.ptD, (float *)&manuSlot.avm_point.far_front,
               sizeof(Point_T));
        memcpy((float *)&slotobj.ptE, (float *)&manuSlot.avm_point.near_front,
               sizeof(Point_T));
    }
    else
    {
        PlanInfoType planInf;
        RTE_PK_DataConvt_Get_Target_PlanInfo(&planInf);
        if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 0)
        {
            RTE_PK_SlotDetect_Get_SlotObj(&slotobj);
        }
        else
        {
            Avm_Pot_T avm;
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avm);
            memcpy((float *)&slotobj.ptB, (float *)&avm.near_rear, sizeof(Point_T));
            memcpy((float *)&slotobj.ptC, (float *)&avm.far_rear, sizeof(Point_T));
            memcpy((float *)&slotobj.ptD, (float *)&avm.far_front, sizeof(Point_T));
            memcpy((float *)&slotobj.ptE, (float *)&avm.near_front, sizeof(Point_T));
            stopflags[0] = avm.AlignIndex[0];
        }
    }

    slotPosition slot_position;
    memset(&slot_position, 0, sizeof(slotPosition));
    slot_position.canId = CanId_SlotPosition;

    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    BuildSlotPosition(&slot_position.slot_position[0], (const float *)&slotobj.ptB,
                      curPos, 0, stopflags, 1);
    BuildSlotPosition(&slot_position.slot_position[1], (const float *)&slotobj.ptC,
                      curPos, 0, stopflags, 2);
    BuildSlotPosition(&slot_position.slot_position[2], (const float *)&slotobj.ptD,
                      curPos, 0, stopflags, 3);
    BuildSlotPosition(&slot_position.slot_position[3], (const float *)&slotobj.ptE,
                      curPos, 0, stopflags, 4);
    // printf("report active slot: stop:%d point:%f,%f,%f,%f,%f,%f,%f,%f\n", stopflags[0],
    //     slotobj.ptB.x, slotobj.ptB.y, slotobj.ptC.x, slotobj.ptC.y,
    //     slotobj.ptD.x, slotobj.ptD.y, slotobj.ptE.x, slotobj.ptE.y);

    buff.clear();
    buff.append(reinterpret_cast<const char *>(&slot_position), sizeof(slot_position));
}

/**
 * @brief 发送障碍物数据
 *
 * 该函数从传感器融合模块获取障碍物信息，并将交通锥或行人类型的障碍物数据打包后，通过指定的套接字发送到远程地址。
 *
 * @param sockfd 套接字描述符，用于发送数据
 * @param remote_addr 远程地址结构体，指定数据发送的目标地址
 */
static int ReportConeHdmlData(ObstaclePacket &objPacket)
{
    int obstacleNumMax = 8;    // 最大障碍物数量
    int obstacleNum    = 0;    // 当前已处理的障碍物数量
    AvmObstacle_T avmObstacle; // 障碍物信息结构体

    // 获取障碍物数据
    RTE_PK_SensorFusion_Get_Avm_Obstacle(&avmObstacle);
    if (avmObstacle.num < 0)
    {
        return 0;
    }

    memset(&objPacket, 0, OBSTACLE_PACKET_SIZE); // 初始化数据包
    objPacket.canId = CanId_Obstacle;            // 设置CAN ID

    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);

    // 遍历所有可能的障碍物
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        // 如果没有障碍物或已达到最大障碍物数量，则退出循环
        if (avmObstacle.num <= 0 || obstacleNum >= obstacleNumMax ||
            obstacleNum >= avmObstacle.num)
        {
            break;
        }

        // 仅处理交通锥和行人类型的障碍物
        if (avmObstacle.type[i] == 0)
        {
            continue;
        }

        float rPoint[3];
        RevConvert(curPos, (const float *)&avmObstacle.central_point[i], rPoint);

        // printf("%d ---- num: %d i: %d type = %d\n", __LINE__, avmObstacle.num, i,
        // avmObstacle.type[i]);
        //  设置障碍物位置信息
        objPacket.objData[obstacleNum].objPosition.canId         = obstacleNum;
        objPacket.objData[obstacleNum].objPosition.obstacle_type = avmObstacle.type[i];
        objPacket.objData[obstacleNum].objPosition.obstacle_center_x =
            (int16_t)(rPoint[0] * 100);
        objPacket.objData[obstacleNum].objPosition.obstacle_center_y =
            (int16_t)(rPoint[1] * 100);
        objPacket.objData[obstacleNum].objPosition.checksum =
            calculateChecksum((const uint8_t *)&objPacket.objData[i].objPosition);

        // 设置障碍物形状信息
        objPacket.objData[obstacleNum].objShape.canId = obstacleNum;
        objPacket.objData[obstacleNum].objShape.obstacle_angle =
            (int16_t)(avmObstacle.angle[i] * 180 / PI);
        objPacket.objData[obstacleNum].objShape.obstacle_length =
            (int16_t)(avmObstacle.length[i] * 100);
        objPacket.objData[obstacleNum].objShape.obstacle_width =
            (int16_t)(avmObstacle.width[i] * 100);
        objPacket.objData[obstacleNum].objShape.checksum =
            calculateChecksum((const uint8_t *)&objPacket.objData[i].objShape);
#if 0
        printf("%d ----i: %d type: %d, cPt: %d, %d, angle: %d, l: %d, w: %d\n",
           __LINE__,obstacleNum,
           objPacket.objData[obstacleNum].objPosition.obstacle_type,
           objPacket.objData[obstacleNum].objPosition.obstacle_center_x,
           objPacket.objData[obstacleNum].objPosition.obstacle_center_y,
           objPacket.objData[obstacleNum].objShape.obstacle_angle,
           objPacket.objData[obstacleNum].objShape.obstacle_length,
           objPacket.objData[obstacleNum].objShape.obstacle_width);
#endif
        obstacleNum++; // 增加已处理的障碍物数量
    }

    memset(&avmObstacle, 0, sizeof(avmObstacle));
    avmObstacle.num = -1;
    RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
    return 1;
}

static void ReportRadarInfo(std::string &buff)
{
    U_RadarType radarData;
    RTE_PD_Get_U_Radar(&radarData);

    raderDataArray rader_data;
    rader_data.canId = CanId_RaderData;

    // radar data:前1，前2，前3，前4，后1，后2，后3，后4，长距1，长距2，长距3，长距4

    float minDist = 2550.0;

    BuildRaderData(&rader_data.rader_data[0], 0x01, (int)(radarData.FOL),
                   (int)(radarData.FCL), (int)(radarData.FCR));
    BuildRaderData(&rader_data.rader_data[1], 0x02, (int)(radarData.FOR),
                   (int)(radarData.ROL), (int)(radarData.RCL));
    BuildRaderData(&rader_data.rader_data[2], 0x03, (int)(radarData.RCR),
                   (int)(radarData.ROR), (int)(radarData.FSL));
    BuildRaderData(&rader_data.rader_data[3], 0x04, (int)(radarData.FSR),
                   (int)(radarData.RSL), (int)(radarData.RSR));

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

    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "ReportRadarInfo, %06ld:%06ld, %f\n", stamp.tv_sec,
                stamp.tv_usec, minDist);

    buff.clear();
    // buff.append("radar data:");
    buff.append(reinterpret_cast<const char *>(&rader_data), RADER_DATA_SIZE);
}

static bool CheckParkSlotId(int temp)
{
    // printf("SetParkSlotId: recv input %s slotid = %d \n", str, temp);
    PK_ModuleComType curState = RTE_PK_StateManage_Get_ModuleCom_PathExecute();
    if (curState == MCOM_ON_PARKING)
    {
        return false;
    }

    int index  = 0;
    int slotId = temp;
    MultiPlanInfo plansInfo;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&plansInfo);
    for (index = 0; index < plansInfo.slotNum; index++)
    {
        if (plansInfo.slotPlans[index].Slot_index == slotId)
        {
            break;
        }
    }

    if (index >= plansInfo.slotNum)
    {
#ifdef SIMULATE
        for (index = 0; index < plansInfo.slotNum; index++)
        {
            if (CompareSlotIndex(temp, plansInfo.slotPlans[index].Slot_index) == 1)
            {
                slotId = plansInfo.slotPlans[index].Slot_index;
                break;
            }
        }
#endif
    }
    else
    {
        slotId = plansInfo.slotPlans[index].Slot_index;
    }

    Multi_Slot_Array_T slotsInfo;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slotsInfo);
    for (index = 0; index < slotsInfo.multiNum; index++)
    {
        if (slotsInfo.multiArray[index].slot_index == slotId)
        {
            break;
        }
    }

    if (index >= slotsInfo.multiNum)
    {
        SlotInfo_T manuSlot;
        RTE_PK_DataConv_Get_SlotInfo(&manuSlot);
        if (manuSlot.slot_index != slotId)
        {
            return false;
        }
    }

    return true;
}

static void ReportPosInfo(std::string &buff, const int startPark)
{
    vehPosition veh_position;
    veh_position.canId     = CanId_VehPosition;
    int run_rate           = 0;
    static float pointO[4] = {0.0, 0.0, 0.0, 0.0};
    float curPos[4], result[3];
    if (startPark == 0)
    {
        RTE_PK_Location_Get_CurPos(pointO);
        memcpy(curPos, pointO, sizeof(curPos));
    }
    else
    {
        RTE_PK_Location_Get_CurPos(curPos);
    }

    CoordinadteTransfer(pointO, curPos, result);

    if (RTE_SM_Get_ApaWorkState() == CarSta_GuidanceActive)
    {
        float paras[8] = {0};
        PK_PathExecute_RunRate(paras); // paras[0]是进度；  paras[1]是转弯半径，左正右负
        run_rate = paras[0];
    }
    else
    {
        run_rate = 0;
    }

    BuildVehPosition(&veh_position, result[0], result[1], result[2], run_rate);

    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "ReportPosInfo, %06ld:%06ld, %03f, %03f. %03f, %d\n",
                stamp.tv_sec, stamp.tv_usec, result[0], result[1], result[2], run_rate);

    buff.clear();
    buff.append(reinterpret_cast<const char *>(&veh_position), VEH_POSITION_SIZE);
}

/**
 * @brief  解析并处理来自 HMI 的泊车命令
 *
 * 从 `command` 数据中解析 HMI 指令，填充到 `apSwh` 数组，
 * 并更新泊车HMI输入。
 *
 * 处理逻辑：
 * 1. 将 HMI 指令字段映射到整型数组 apSwh[]（长度 10，实际使用前 5 个）。
 * 2. 记录日志（sendHMIlog）。
 * 3. 如果满足条件（车位有效、当前不在泊车中、车位 ID 检查通过），则启动泊车：
 *    - 调用 PK_StartPark() 开启泊车
 *    - 获取规划信息并检查车位一致性
 * 4. 更新泊车HMI输入。
 *
 * @param[in] command  指向 HMI 命令数据的指针（commandHMI 结构体）
 */
static void SetApaState(const uint8_t *command)
{
    int apSwh[10] = {0};
    apSwh[0]      = -1; // 初始化车位 ID 为无效值

    // 将原始字节流转换为 commandHMI 结构
    commandHMI *command_HMI = (commandHMI *)command;
    // 映射 HMI 命令字段到整型数组
    apSwh[0] = (int)command_HMI->slot_index == 0xFFFF
                   ? -1
                   : (int)command_HMI->slot_index; // 车位 ID
    apSwh[1] = (int)command_HMI->apa_start;        // 泊车总开关
    apSwh[2] = (int)command_HMI->apa_state;        // 按钮状态
    apSwh[3] = (int)command_HMI->single_boundary;  // 单边车位
    apSwh[4] = (int)command_HMI->parking_pattern;  // 泊入/泊出模式

    // 记录 HMI 命令日志
    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "RecvHMICommand, %d, %d, %d, %d, %d\n", apSwh[1], apSwh[2],
                apSwh[3], apSwh[4], apSwh[0]);

    HMI_ButtonInfo l_HmiButtonInfo = {0};
    RTE_PK_SM_GetHmiBtInfo(&l_HmiButtonInfo);
    // 被选中车位序号 0-无车位被选中 其它-选中车位序号
    l_HmiButtonInfo.HmiSelectSlotInx = apSwh[0];
    /// 泊车总开关 0-APA禁止        1-使能APA
    if (apSwh[1] >= 0)
    {
        l_HmiButtonInfo.HmiSwitchSta = (bool)apSwh[1];
    }
    /// 泊车按钮     0-无效    1-暂停    2-继续    3-退出
    if (apSwh[2] >= 0)
    {
        switch (apSwh[2])
        {
            case 1:
                l_HmiButtonInfo.HmiButtonSta = ButtonState_Suspend;
                break;
            case 2:
                l_HmiButtonInfo.HmiButtonSta = ButtonState_Continue;
                break;
            case 3:
                l_HmiButtonInfo.HmiButtonSta = ButtonState_OutPark;
                break;
            default:
                l_HmiButtonInfo.HmiButtonSta = ButtonState_IntoAct;
                break;
        }
    }
    /// 单边车位状态（预留）
    if (apSwh[3] >= 0)
    {
        l_HmiButtonInfo.HmiSinglrSlotSta = apSwh[3];
    }
    // 泊入/泊出模式（可选字段）
    if (apSwh[4] < INVALID_MODE)
    {
        l_HmiButtonInfo.HmiApaMode = (HMI_ApaModeType)apSwh[4];
    }
    RTE_PK_SM_SetHmiBtInfo(&l_HmiButtonInfo);
}

static void UpdateFrontVertSlot(float points[4][2])
{
    float temp[2];
    if (points[0][0] > 0 && points[1][0] > 0)
    {
        memcpy(temp, points[0], sizeof(temp));
        memcpy(points[0], points[1], sizeof(temp));
        memcpy(points[1], temp, sizeof(temp));

        memcpy(temp, points[2], sizeof(temp));
        memcpy(points[2], points[3], sizeof(temp));
        memcpy(points[3], temp, sizeof(temp));
    }
}

static void AddParkSlot(const uint8_t *command)
{
    float points[4][2];
    SlotInfo_T manuSlot;

    auto ClearManuSlot = [&manuSlot]() {
        memset(&manuSlot, 0, sizeof(manuSlot));
        RTE_PK_DataConv_Set_SlotInfo(&manuSlot);
    };

    slotDesignated slot_designated;
    memcpy(&slot_designated, command, SLOT_DESIGNATED_SIZE);
    if ((int)slot_designated.data_partial[0].canId == 0x01)
    {
        points[0][0] = (float)slot_designated.data_partial[0].point_a * 0.01;
        points[0][1] = (float)slot_designated.data_partial[0].point_b * 0.01;
        points[1][0] = (float)slot_designated.data_partial[0].point_c * 0.01;
#ifndef SIMULATE
        Can_PushData(0x6412, (uint8_t *)&slot_designated.data_partial[0]);
#endif
    }
    else
    {
        printf("slot_designated.data_partial[0] is Err !");
    }

    if ((int)slot_designated.data_partial[1].canId == 0x02)
    {
        points[1][1] = (float)slot_designated.data_partial[1].point_a * 0.01;
        points[2][0] = (float)slot_designated.data_partial[1].point_b * 0.01;
        points[2][1] = (float)slot_designated.data_partial[1].point_c * 0.01;
#ifndef SIMULATE
        Can_PushData(0x6412, (uint8_t *)&slot_designated.data_partial[1]);
#endif
    }
    else
    {
        printf("slot_designated.data_partial[1] is Err !");
    }

    if ((int)slot_designated.data_last.canId == 0x03)
    {
        points[3][0] = (float)slot_designated.data_last.point_x * 0.01;
        points[3][1] = (float)slot_designated.data_last.point_y * 0.01;
#ifndef SIMULATE
        Can_PushData(0x6412, (uint8_t *)&slot_designated.data_last);
#endif
    }
    else
    {
        printf("slot_designated.data_last is Err !");
    }

    PK_SlotShapeType slotShap = PK_SLOT_NO;
    if (Cal_Dis_Pt2Pt(points[3], points[0]) < 0.1)
    {
        // printf("json data invalid %s\n", command);
        ClearManuSlot();
        return;
    }

    if (Cal_Dis_Pt2Pt(points[3], points[0]) < 4.1)
    {
        if (points[3][1] < ZERO_FLOAT && points[0][1] < ZERO_FLOAT)
        {
            if ((points[3][1] >= points[2][1] && points[0][1] >= points[1][1]) ||
                (points[3][1] <= points[2][1] && points[0][1] <= points[1][1]))
            {
                // UpdateFrontVertSlot(points);
                slotShap = PK_SLOT_RIGHT_VERT;
            }
        }
        else
        {
            if ((points[3][1] <= points[2][1] && points[0][1] <= points[1][1]) ||
                (points[3][1] >= points[2][1] && points[0][1] >= points[1][1]))
            {
                // UpdateFrontVertSlot(points);
                slotShap = PK_SLOT_LEFT_VERT;
            }
        }
    }
    else
    {
        if (points[3][1] < ZERO_FLOAT && points[0][1] < ZERO_FLOAT)
        {
            if ((points[3][1] >= points[2][1] && points[0][1] >= points[1][1]) ||
                (points[3][1] <= points[2][1] && points[0][1] <= points[1][1]))
            {
                slotShap = PK_SLOT_RIGHT_PARA;
            }
        }
        else
        {
            if ((points[3][1] <= points[2][1] && points[0][1] <= points[1][1]) ||
                (points[3][1] >= points[2][1] && points[0][1] >= points[1][1]))
            {
                slotShap = PK_SLOT_LEFT_PARA;
            }
        }
    }

    if (slotShap == PK_SLOT_NO)
    {
        ClearManuSlot();
        return;
    }

    float x = (points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4;
    float y = (points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4;
    if (sqrt(x * x + y * y) > 15)
    {
        ClearManuSlot();
        return;
    }

    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);
    float rspoint[2];
    Point_T avmPoints[4];
    Convert(curPos, points[0], rspoint);
    memcpy(&avmPoints[0], rspoint, sizeof(rspoint));
    Convert(curPos, points[1], rspoint);
    memcpy(&avmPoints[1], rspoint, sizeof(rspoint));
    Convert(curPos, points[2], rspoint);
    memcpy(&avmPoints[2], rspoint, sizeof(rspoint));
    Convert(curPos, points[3], rspoint);
    memcpy(&avmPoints[3], rspoint, sizeof(rspoint));

    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "CurPos,  %f, %f, %f, %f\n", curPos[0], curPos[1], curPos[2],
                curPos[3]);

    if (sendHMIlog != NULL)
        fprintf(sendHMIlog, "RecvHMISlotInfo, %f, %f, %f, %f, %f, %f, %f, %f\n",
                points[0][0], points[0][1], points[1][0], points[1][1], points[2][0],
                points[2][1], points[3][0], points[3][1]);

    PK_BuildAvmSlotInfo(avmPoints, slotShap, manuSlot);

    // log_info("config slots %s %d\n", command, manuSlot.slot_index);
    RTE_PK_DataConv_Set_SlotInfo(&manuSlot);
}

void Led_ProcessCmd(const uint8_t *command, int len)
{
    CADID_HMI2APA canId = (CADID_HMI2APA)command[0];
    switch (canId)
    {
        case CanId_CommandHMI:
            if (len != 9)
            {
                break;
            }
            SetApaState(command);
#ifndef SIMULATE
            Can_PushData(0x6411 + canId, &command[1]);
#endif
            break;

        case CanId_SlotDesignated:
            AddParkSlot(command);
            break;

        default:
            break;
    }
}

/**
 * @brief 向 Enet 发送当前 APA 状态信息
 *
 * @param[in] data  长度为 9 的字节数组，用于存放 APA 状态数据：
 *                  data[0]  : canId
 *                  data[1]  : apa_mode(4bit) + apa_state(4bit)
 *                  data[2]  : apa_info
 *                  data[3]  : apa_avp(4bit) + apa_gear(4bit)
 *                  data[4-5]: apa_radius (int16, cm)
 *                  data[6]  : spd
 *                  data[7]  : empty_data
 *                  data[8]  : checksum (异或校验)
 */
void PK_SM_ReportApaStatusInfo(uint8_t data[9])
{
    // -------------------------------
    // 定义 APA 数据结构体（与输出格式一一对应）
    // -------------------------------
    struct ApaManageData
    {
        uint8_t canId;          // [0] CAN 报文 ID
        unsigned apa_mode : 4;  // [1] 高 4bit：泊车模式
        unsigned apa_state : 4; // [1] 低 4bit：泊车状态
        uint8_t apa_info;       // [2] 文字提示信息
        unsigned apa_avp : 4;   // [3] 高 4bit：AVP 状态
        unsigned apa_gear : 4;  // [3] 低 4bit：档位
        int16_t apa_radius;     // [4-5] 转弯半径（cm）
        uint8_t spd;            // [6] 车速
        uint8_t empty_data;     // [7] 预留空数据
        uint8_t checksum;       // [8] 校验码
    } apaStatusManage;

    // 获取 APA 路径执行相关数据
    // paras[0]: APA 进度
    // paras[1]: 转弯半径（左正右负）
    // paras[2]: 路径方向（<0 后退，>0 前进）
    float paras[8] = {0};
    PK_PathExecute_RunRate(paras);
    uint8_t pathGEAR = fabs(paras[2]) < ZERO_FLOAT
                           ? GEAR_REAL_P
                           : (paras[2] < 0 ? GEAR_REAL_R : GEAR_REAL_D);

    HMI_WorkStateType l_HMI_CurrApaSta = RTE_SM_Get_HMI_WorkState();
    // 填充 canId
    apaStatusManage.canId = 0x00;

    // 处理泊车模式与状态
    apaStatusManage.apa_mode = RTE_SM_Get_HMI_State();

    if (RTE_SM_Get_ApaWorkState() == CarSta_GuidanceAvpActive &&
        RTE_PK_SM_GetApaMode() == AVP_MODE)
    {
        apaStatusManage.apa_state = HmiWorkState_Serching;
    }
    else if (RTE_SM_Get_ApaWorkState() == CarSta_GuidanceAvpActive &&
             RTE_PK_SM_GetApaMode() == PARKOUT_MODE)
    {
        apaStatusManage.apa_state = HmiWorkState_GuidanceActive;
    }
    else
    {
        apaStatusManage.apa_state = l_HMI_CurrApaSta;
    }

    apaStatusManage.apa_info = RTE_SM_Get_HMI_WorkTextInfo();

    // AVP 状态
    apaStatusManage.apa_avp = RTE_PK_SM_GetApaMode();

    // 档位转换（映射到 1~4）
    uint8_t curGear = RTE_BSW_Get_CurrentGear();
    if (curGear == GEAR_REAL_D)
    {
        apaStatusManage.apa_gear = 1;
    }
    else if (curGear == GEAR_REAL_R)
    {
        apaStatusManage.apa_gear = 2;
    }
    else if (curGear == GEAR_REAL_P)
    {
        apaStatusManage.apa_gear = 3;
    }
    else
    {
        apaStatusManage.apa_gear = 4;
    }

    // 转弯半径（仅在 APA 执行状态下有效）
    if (RTE_SM_Get_ApaWorkState() == CarSta_GuidanceActive && pathGEAR == curGear)
    {
        apaStatusManage.apa_radius = (int16_t)(paras[1] * 100);
    }
    else
    {
        apaStatusManage.apa_radius = 0;
    }

    // 车速
    apaStatusManage.spd = RTE_BSW_Get_ESC_VehSpd();
    // printf("APA: %d, %d, %d, %d, %d,
    // %d\n",apaStatusManage.apa_mode,apaStatusManage.apa_state,apaStatusManage.apa_info,apaStatusManage.apa_avp,apaStatusManage.apa_gear,apaStatusManage.apa_radius
    // );

    // 预留字段与校验码初始化
    apaStatusManage.empty_data = 0;
    apaStatusManage.checksum   = 0;

    // 将结构体内容写入 data 数组（先不带 checksum）
    memcpy(data, &apaStatusManage, sizeof(uint8_t) * 9);

    // 计算 checksum：data[1]~data[7] 异或 0xFF
    apaStatusManage.checksum =
        (data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) ^ 0XFF;

    // 回写 checksum 到 data[8]
    memcpy(&data[8], &apaStatusManage.checksum, sizeof(uint8_t));
}

static void *LedDataLoop(void *paras)
{
    const char *input   = (char *)paras;
    const char *portPos = strstr((char *)input, ":");

    int reportFd = socket(PF_INET, SOCK_DGRAM, 0);
    if (reportFd < 0)
    {
        return NULL;
    }

    struct sockaddr_in servaddr, localaddr;

    if (portPos == NULL)
    {
        servaddr.sin_addr.s_addr = 0;
    }
    else
    {
        char servStr[32], portStr[16];
        memset(servStr, 0, sizeof(servStr));
        memset(portStr, 0, sizeof(portStr));
        strncpy(servStr, input, rte_min((size_t)(portPos - input), sizeof(servStr)));
        strncpy(portStr, portPos + 1,
                rte_min((size_t)(strlen(input) - strlen(servStr) - 1), sizeof(portStr)));

        printf("server ip %s port %s\r\n", servStr, portStr);
        servaddr.sin_addr.s_addr = inet_addr(servStr);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port   = htons(APA2HMI_PORT); // 18805

    char localIp[16] = {0};
    RTE_BSW_Get_LocalIPAddr(localIp);

    localaddr.sin_family      = AF_INET;
    localaddr.sin_port        = htons(HMI2APA_PORT); // 18804
    localaddr.sin_addr.s_addr = inet_addr(localIp);

    int flag = 1;
    setsockopt(reportFd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
    if (bind(reportFd, (struct sockaddr *)&localaddr, sizeof(localaddr)) < 0)
    {
        printf("bind errno=%d %s\n", errno, strerror(errno));
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(reportFd, &rfds);
    int maxfd              = reportFd;
    struct timeval timeout = {1, 0};

    struct timeval lastStamp;
    gettimeofday(&lastStamp, NULL);

    SlotInfo_T lastSlot;
    int startPark = 0;
    uint8_t apastate[9];
    while (1)
    {
        FD_SET(reportFd, &rfds);
        timeout.tv_sec  = 0;
        timeout.tv_usec = 25000;

        int result = select(maxfd + 1, &rfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            printf("select errno=%d %s\n", errno, strerror(errno));
            continue;
        }
        else if (result == 0)
        {
            if (servaddr.sin_addr.s_addr == 0)
            {
                // printf("servaddr.sin_addr.s_addr == 0\n");
                continue;
            }
            servaddr.sin_port = htons(APA2HMI_PORT); // 18805

            static int EthSendTimeCnt = 0;
            std::string slotinfo;
            EthSendTimeCnt++;

            switch (EthSendTimeCnt % 2)
            {
                case 0:
                    if (CarSta_Serching == RTE_SM_Get_ApaWorkState() ||
                        CarSta_ParkAssist_Standby == RTE_SM_Get_ApaWorkState() ||
                        CarSta_GuidanceAvpActive == RTE_SM_Get_ApaWorkState())
                    {
                        // 0x01:车位数据
                        ReportSearchSlotInfo(slotinfo, lastSlot);
                        startPark = 0;
                        result = sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                        (struct sockaddr *)&servaddr, sizeof(servaddr));

                        // 0x03:雷达数据
                        ReportRadarInfo(slotinfo);
                        if (slotinfo.length() == RADER_DATA_SIZE)
                        {
                            result =
                                sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                       (struct sockaddr *)&servaddr, sizeof(servaddr));
                        }
                    }
                    else if (CarSta_GuidanceActive == RTE_SM_Get_ApaWorkState())
                    {
                        // 0x01:车位数据
                        ReportActiveSlotInfo(slotinfo);
                        if (slotinfo.length() > 0)
                        {
                            result =
                                sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                       (struct sockaddr *)&servaddr, sizeof(servaddr));
                        }

                        // 0x02：泊车轨迹
                        ReportPosInfo(slotinfo, startPark);
                        startPark = 1;
                        if (slotinfo.length() > 0)
                        {
                            result =
                                sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                       (struct sockaddr *)&servaddr, sizeof(servaddr));
                        }

                        // 0x03:雷达数据
                        ReportRadarInfo(slotinfo);
                        if (slotinfo.length() == RADER_DATA_SIZE)
                        {
                            result =
                                sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                       (struct sockaddr *)&servaddr, sizeof(servaddr));
                        }
                    }
                    else
                    {
                        lastSlot.slotshap = PK_SLOT_NO;
                        SlotInfo_T manuSlot;
                        manuSlot.slot_index = 0;
                        manuSlot.slotshap   = PK_SLOT_NO;
                        RTE_PK_DataConv_Set_SlotInfo(&manuSlot);
                    }

                    if (cfg_amvobj_report != 0)
                    {
                        // 0x04:障碍物信息
                        ObstaclePacket objPacket;
                        if (ReportConeHdmlData(objPacket) > 0)
                        {
                            result =
                                sendto(reportFd, &objPacket, OBSTACLE_PACKET_SIZE, 0,
                                       (struct sockaddr *)&servaddr, sizeof(servaddr));
                        }
                    }
                    break;

                case 1:
                    // 0x00:apa状态
                    PK_SM_ReportApaStatusInfo(apastate);
                    sendto(reportFd, apastate, sizeof(apastate), 0,
                           (struct sockaddr *)&servaddr, sizeof(servaddr));
                    Can_PushData(0x6410 + apastate[0], &apastate[1]);

                    // 0x05:车身数据
                    ReportCarDoorStatus(slotinfo);
                    if (slotinfo.length() == CAR_DOORSTATUS_SIZE)
                    {
                        result = sendto(reportFd, slotinfo.c_str(), slotinfo.length(), 0,
                                        (struct sockaddr *)&servaddr, sizeof(servaddr));
                    }
                    break;

                default:
                    break;
            }
        }
        else
        {
            if (sendHMIlog == NULL && is_save_log)
            {
                sendHMIlog = OpenLogFile("sendHMIlog.txt");
            }

            uint8_t buf[256];
            memset(buf, 0, sizeof(buf));
            int len = sizeof(servaddr);
            result = recvfrom(reportFd, buf, sizeof(buf), 0, (struct sockaddr *)&servaddr,
                              (socklen_t *)&len);
            log_info("recvfrom errno=%d %s %s\n", errno, strerror(errno), buf);
            if (result > 0)
            {
                Led_ProcessCmd(buf, result);
            }
        }
    }

    if (sendHMIlog != NULL)
    {
        fclose(sendHMIlog);
        sendHMIlog = NULL;
    }
    close(reportFd);
    reportFd = -1;
    return NULL;
}

void Eth_InitLedDisplay()
{
    static char buf[32];
    std::ifstream fin("server.cfg");
    if (fin.is_open())
    {
        fin.getline(buf, sizeof(buf));
        fin.close();
    }

    char config[64];
    if (PK_Calibration_GetDebugValue("cfg_use_avm_obj", config) > 0)
    {
        int value         = atoi(config);
        cfg_amvobj_report = ((value & 0x01) != 0) ? 1 : 0;
    }

    ASyncRun(LedDataLoop, (void *)buf, "apa_eth_hmi");
}
