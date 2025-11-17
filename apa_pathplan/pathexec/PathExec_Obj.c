#include "PK_PathExecute.h"
#include "PathExec_Obj.h"
#include "PK_ObjAvoid.h"
#include "PK_SensorT2S.h"
#include "PathExec_Func.h"
#include "PathExec_Debug.h"
#include "PK_Utility.h"

static FusionObj_T ObsObj_AnotherSide;
static FusionObj_T obsObj_nearSlot;
static FusionObj_T obsObj_parking;

static float slotObj_Conference[2][4];

void PK_PathExec_GetParkObj(FusionObj_T &fusion)
{
    PathExec_Lock();
    memcpy((float *)&fusion, (float *)&obsObj_parking, sizeof(FusionObj_T));
    PathExec_UnLock();
}

void PathExec_ClearParkObj()
{
    PathExec_Lock();
    memset((float *)&obsObj_parking, 0, sizeof(obsObj_parking));
    obsObj_parking.num      = 0;
    obsObj_parking.num_accu = 0;
    PathExec_UnLock();
}

void PathExec_AddParkObj(const float obj[4], const float finpoint[3])
{
    PathExec_Lock();
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        if (obsObj_parking.attr[i] == 0)
        {
            memcpy((float *)&obsObj_parking.obj[i], obj, sizeof(LineSeg_T));
            obsObj_parking.attr[i] = 1;
            obsObj_parking.num     = obsObj_parking.num + 1;
            break;
        }
    }
    PathExec_UnLock();
}

void PathExec_GetBeConference(float be[2][4])
{
    memcpy(be, slotObj_Conference, sizeof(slotObj_Conference));
}

void PathExec_SetBeConference(const float be[2][4])
{
    memcpy(slotObj_Conference, be, sizeof(slotObj_Conference));
}

void PathExec_GetObsObj_AnotherSide(FusionObj_T &fusion)
{
    memcpy((float *)&fusion, (float *)&ObsObj_AnotherSide, sizeof(FusionObj_T));
}

FusionObj_T &PathExec_Init_NearSlotObjs()
{
    memset((float *)&obsObj_nearSlot, 0, sizeof(obsObj_nearSlot));
    obsObj_nearSlot.num      = 0;
    obsObj_nearSlot.num_accu = 0;
    return obsObj_nearSlot;
}

inline void SetSlotFlag(uint8_t &result, uint8_t flag, int index)
{
    result = result | (flag << index);
}

inline bool IsSetSlotFlag(const uint8_t &flag, int index)
{
    return (((flag >> index) & 0x03) != 0);
}

static void MergeObsObjs(FusionObj_T &fusion)
{
    int cur  = 0;
    int next = 1;
    while (next < SF_OBJ_NUM - 1)
    {
        if (fusion.attr[cur] == 0)
        {
            cur++;
            next++;
            continue;
        }
        if (fusion.attr[next] == 0)
        {
            next++;
            continue;
        }

        auto lenCur  = Get_Segment_Len(fusion.obj[cur]);
        auto lenNext = Get_Segment_Len(fusion.obj[next]);
        auto dist    = Cal_Dis_Pt2Pt(fusion.obj[cur].pt2, fusion.obj[next].pt1);
        if (dist < 0.10 && lenCur > 0.35 && lenNext < 0.35)
        {
            fusion.obj[cur].pt2.x =
                (fusion.obj[next].pt1.x + fusion.obj[next].pt2.x) * 0.5;
            fusion.obj[cur].pt2.y =
                (fusion.obj[next].pt1.y + fusion.obj[next].pt2.y) * 0.5;
            fusion.attr[next] = 0;
        }
        else if (dist < 0.10 && lenCur < 0.35 && lenNext > 0.35)
        {
            fusion.obj[next].pt1.x =
                (fusion.obj[cur].pt1.x + fusion.obj[cur].pt2.x) * 0.5;
            fusion.obj[next].pt1.y =
                (fusion.obj[cur].pt1.y + fusion.obj[cur].pt2.y) * 0.5;
            fusion.attr[cur] = 0;
            cur              = next;
        }
        else
        {
            cur++;
        }
        next++;
    }
}

/**
 * @brief 添加靠近车位的障碍物对象
 *
 * 该函数用于在泊车过程中添加靠近车位的障碍物对象，通过计算障碍物两点间的距离来判断是否有效。
 * 如果障碍物有效，函数会将其转换到相对于最终停车点的坐标系中，并根据障碍物的位置更新相关属性。
 *
 * @param obj 障碍物的四个坐标点（两个端点）
 * @param finpoint 最终停车点的坐标
 */
void PathExec_AddObsObjNearSlot(const float obj[4], const float finpoint[3])
{
    float rspoint[2][2];                          // 存储障碍物点转换后的坐标
    float dist = Cal_Dis_Pt2Pt(&obj[0], &obj[2]); // 计算障碍物两点之间的距离
    if (dist < 0.1)
    {
        // 如果距离小于阈值，认为障碍物无效，重置相关属性
        obsObj_nearSlot.attr[0]              = 0;
        obsObj_nearSlot.attr[SF_OBJ_NUM / 2] = 0;
        return;
    }

    // 将障碍物的两个端点转换到相对于最终停车点的坐标系中
    RevConvert(finpoint, &obj[0], rspoint[0]);
    RevConvert(finpoint, &obj[2], rspoint[1]);

    // 如果障碍物的两个端点在车宽范围外，返回
    if (fabs(rspoint[0][1]) > VEHICLE_WID && fabs(rspoint[1][1]) > VEHICLE_WID)
    {
        return;
    }

    // 根据障碍物位置设置属性值，左侧为30001，右侧为30002
    int objAttr =
        (rspoint[0][1] > ZERO_FLOAT && rspoint[1][1] > ZERO_FLOAT) ? 30001 : 30002;

    if (objAttr == 30001)
    {
        // 将障碍物数据存储到对应位置，并更新属性和计数
        memcpy((float *)&obsObj_nearSlot.obj[0], obj, sizeof(LineSeg_T));
        if (objAttr != obsObj_nearSlot.attr[0])
        {
            obsObj_nearSlot.num++;
        }
        obsObj_nearSlot.attr[0] = objAttr;
    }
    else
    {
        // 将障碍物数据存储到另一位置，并更新属性和累计计数
        memcpy((float *)&obsObj_nearSlot.obj[SF_OBJ_NUM / 2], obj, sizeof(LineSeg_T));
        if (objAttr != obsObj_nearSlot.attr[SF_OBJ_NUM / 2])
        {
            obsObj_nearSlot.num_accu++;
        }
        obsObj_nearSlot.attr[SF_OBJ_NUM / 2] = objAttr;
    }
}

/**
 * @brief 更新目标车位附近的障碍物。
 *
 * 以目标点作为原点，将目标车位附近的障碍物信息存入全局变量obsObj_nearSlot中。
 *
 * @param finpoint 目标点位置信息
 */
static void UpdateBObjNearSlot(const float finpoint[3], FusionObj_T &nearObj)
{
    FusionObj_T fusion[2];
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&fusion[0]);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&fusion[1]);

    MergeObsObjs(fusion[0]);
    MergeObsObjs(fusion[1]);

    int left  = 1;
    int right = 1;

    for (int k = 0; k < 2; k++)
    {
        for (int i = 0; i < fusion[k].num; i++)
        {
            float rspoint[2][2];
            if (fusion[k].attr[i] == 0)
            {
                continue;
            }
            RevConvert(finpoint, (float *)&fusion[k].obj[i].pt1, rspoint[0]);
            RevConvert(finpoint, (float *)&fusion[k].obj[i].pt2, rspoint[1]);

            float objLen = Cal_Dis_Pt2Pt(fusion[k].obj[i].pt1, fusion[k].obj[i].pt2);
            if (objLen < 0.30f)
            {
                continue;
            }

            if (fabs(rspoint[0][0]) > VEHICLE_LEN && fabs(rspoint[1][0]) > VEHICLE_LEN)
            {
                continue;
            }
            if (fabs(rspoint[0][1]) > VEHICLE_WID && fabs(rspoint[1][1]) > VEHICLE_WID)
            {
                continue;
            }
            if (rspoint[0][1] * rspoint[1][1] <= 0)
            {
                continue;
            }

            if (rspoint[0][1] > 0 && rspoint[1][1] > 0)
            {
                nearObj.attr[left] = fusion[k].attr[i];
                memcpy((float *)&nearObj.obj[left], (float *)&fusion[k].obj[i],
                       sizeof(LineSeg_T));
                left++;
            }
            else
            {
                nearObj.attr[right + SF_OBJ_NUM / 2] = fusion[k].attr[i];
                memcpy((float *)&nearObj.obj[right + SF_OBJ_NUM / 2],
                       (float *)&fusion[k].obj[i], sizeof(LineSeg_T));
                right++;
            }
        }
    }

    nearObj.num      = nearObj.num + (left - 1);
    nearObj.num_accu = nearObj.num_accu + (right - 1);

    PRINT_SWITCH_INFO("UpdateBObjNearSlot");
    PathExec_PrintSlotObjInfo(nearObj);
}

/**
 * @brief 根据当前车辆位置和目标点生成后方雷达的报警对象。
 *
 * 该函数根据车辆当前位置和目标点，结合危险状态标志，计算并生成对应的后方雷达报警对象。
 *
 * @param curpos 当前车辆位置的数组，包含x, y, 方向角等信息。
 * @param finpoint 目标点位置的数组，包含x, y, 方向角等信息。
 * @param left 指定生成左侧或右侧或左偏移或右偏移的雷达对象。
 *             1: 左侧
 *             2: 右侧
 *             3: 左偏移
 *             4: 右偏移
 * @param objs 用于存储生成的雷达对象的线段结构体。
 * @return int 返回1表示生成成功，返回0表示未生成。
 */
static int BuildRearUradarObj(const float curpos[4], const float finpoint[3],
                              const int left, LineSeg_T &objs)
{
    // 定义危险状态掩码
    const uint8_t leftside_mask    = 0x10;
    const uint8_t rightside_mask   = 0x20;
    const uint8_t leftoffset_mask  = 0x40;
    const uint8_t rightoffset_mask = 0x80;

    // 计算当前车辆方向角与目标方向角的差值
    float angle = Round_PI(curpos[2] - finpoint[2]);
    // 获取危险状态
    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();

    // 根据左侧、右侧、左偏移、右偏移的标志和危险状态掩码生成对应的雷达对象
    if (left == 1 && (dangerSt & leftside_mask) != 0)
    {
        float offset  = rte_min(angle * PI_DEG * 0.005, 0.07);
        float temp[4] = {-0.25174, 1.2345, -0.65174, 1.2345};
        temp[1]       = temp[1] + fabs(offset);
        temp[3]       = temp[3] + fabs(offset);
        if ((dangerSt >> 8) & leftside_mask)
        {
            temp[1] = temp[1] - sign(temp[1]) * 0.05;
            temp[3] = temp[3] - sign(temp[3]) * 0.05;
        }

        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 2 && (dangerSt & rightside_mask) != 0)
    {
        float offset  = rte_min(angle * PI_DEG * 0.005, 0.07);
        float temp[4] = {-0.25174, -1.2345, -0.65174, -1.2345};
        temp[1]       = temp[1] - fabs(offset);
        temp[3]       = temp[3] - fabs(offset);

        if ((dangerSt >> 8) & rightside_mask)
        {
            temp[1] = temp[1] - sign(temp[1]) * 0.05;
            temp[3] = temp[3] - sign(temp[3]) * 0.05;
        }

        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 3 && (dangerSt & leftoffset_mask) != 0)
    {
        float temp[4] = {-(REAR_SUSPENSION + 0.45f), 0.60f, -(REAR_SUSPENSION + 0.50f),
                         0.25f};
        if ((dangerSt >> 8) & leftoffset_mask)
        {
            temp[0] = temp[0] - sign(temp[0]) * 0.06;
            temp[2] = temp[2] - sign(temp[2]) * 0.06;
        }
        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 4 && (dangerSt & rightoffset_mask) != 0)
    {
        float temp[4] = {-(REAR_SUSPENSION + 0.45f), -0.60, -(REAR_SUSPENSION + 0.50f),
                         -0.25};
        if ((dangerSt >> 8) & rightoffset_mask)
        {
            temp[0] = temp[0] - sign(temp[0]) * 0.06;
            temp[2] = temp[2] - sign(temp[2]) * 0.06;
        }
        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 根据当前车辆位置和目标点生成雷达报警对象。
 *
 * 该函数根据车辆当前位置和目标点，结合危险状态标志，调用 BuildRearUradarObj
 * 函数生成并添加后方雷达报警对象。
 *
 * @param curpos 当前车辆位置的数组，包含x, y, 方向角等信息。
 * @param finpoint 目标点位置的数组，包含x, y, 方向角等信息。
 */
int PathExec_BuildSideRadarAlarmObj(const float curpos[4], const float finpoint[3],
                                    float objs[4][4])
{
    // 获取危险状态
    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();
    if (dangerSt == 0)
    {
        return 0;
    }
    printf("BuildRadarAlarmObj dangerSt %d\n", dangerSt);

    LineSeg_T line;

    // 生成左侧雷达对象
    int count = 0;
    if (BuildRearUradarObj(curpos, finpoint, 1, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    // 生成右侧雷达对象
    if (BuildRearUradarObj(curpos, finpoint, 2, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    float rx_cur_to_fin     = Project_PosTo1st_rx(finpoint, curpos);
    float rtheta_cur_to_fin = Project_PosTo1st_rtheta(finpoint, curpos);
    if (rx_cur_to_fin < 0.6f && fabs(rtheta_cur_to_fin) < 5.0f * PI_RAD)
    {
        return count;
    }

    // 生成左偏移雷达对象
    if (BuildRearUradarObj(curpos, finpoint, 3, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    // 生成右偏移雷达对象
    if (BuildRearUradarObj(curpos, finpoint, 4, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    return count;
}

/**
 * @brief 根据当前车辆位置和目标点生成雷达报警状态。
 *
 * 该函数根据车辆当前位置和目标点，结合危险状态标志，调用 BuildRearUradarObj
 * 函数生成并添加后方雷达报警对象。
 */
uint16_t PathExec_SideRadarAlarmObjState()
{
    // 定义危险状态掩码
    const uint8_t leftside_mask    = 0x10;
    const uint8_t rightside_mask   = 0x20;
    const uint8_t leftoffset_mask  = 0x40;
    const uint8_t rightoffset_mask = 0x80;

    // 获取危险状态
    uint16_t result   = 0;
    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();
    if (dangerSt == 0)
    {
        return result;
    }

    // 根据左侧、右侧、左偏移、右偏移的标志和危险状态掩码生成对应的雷达对象
    if ((dangerSt & leftside_mask) != 0)
    {
        result = result | 0x01;
        if ((dangerSt >> 8) & leftside_mask)
        {
            result = result | 0x02;
        }
    }

    if ((dangerSt & rightside_mask) != 0)
    {
        result = result | 0x10;
        if ((dangerSt >> 8) & rightside_mask)
        {
            result = result | 0x20;
        }
    }

    if ((dangerSt & leftoffset_mask) != 0)
    {
        result = result | 0x04;
        if ((dangerSt >> 8) & leftoffset_mask)
        {
            result = result | 0x08;
        }
    }

    if ((dangerSt & rightoffset_mask) != 0)
    {
        result = result | 0x40;
        if ((dangerSt >> 8) & rightoffset_mask)
        {
            result = result | 0x80;
        }
    }

    return result;
}

/**
 * @brief 根据车位对象更新目标点的方向角度。
 *
 * 该函数根据给定的车位对象和目标点位置，计算目标点附近的障碍物信息，并据此调整目标点的方向角度。
 *
 * @param slotObj 车位对象（ABCDEF），包含起始点（B点）和结束点（E点）信息。
 * @param finpoint 目标点位置的数组，包含x, y, 方向角等信息，函数将更新其方向角信息。
 */
static void UpdateSlotThetaByObj(const SlotObj_T &slotObj, float finpoint[3])
{
    // 定义有效障碍物长度
    const float valid_obj_len = 0.6;

    // 将目标点坐标从车体坐标系转换到全局坐标系
    float rspointB[2], rspointE[2];
    RevConvert(finpoint, (float *)&slotObj.ptB, rspointB);
    RevConvert(finpoint, (float *)&slotObj.ptE, rspointE);

    // 备份转换后的目标点坐标
    float oldrsB[2], oldrsE[2];
    memcpy(oldrsB, rspointB, sizeof(oldrsB));
    memcpy(oldrsE, rspointE, sizeof(oldrsE));

    // 临时存储障碍物坐标和相关信息的数组
    float rspoint[2][2];
    int objIdx[2][SF_OBJ_NUM / 2];
    int objCount[2]      = {0, 0};
    float objTotalLen[2] = {0.0, 0.0};

    // 遍历所有障碍物
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        // 如果障碍物属性为0，表示不存在障碍物，跳过
        if (obsObj_nearSlot.attr[i] == 0)
        {
            continue;
        }
        // 将障碍物的起始点和结束点坐标从车体坐标系转换到全局坐标系
        RevConvert(finpoint, (float *)&obsObj_nearSlot.obj[i].pt1, rspoint[0]);
        RevConvert(finpoint, (float *)&obsObj_nearSlot.obj[i].pt2, rspoint[1]);

        // 计算障碍物的方向角度和长度
        float angle = atan2(obsObj_nearSlot.obj[i].pt2.y - obsObj_nearSlot.obj[i].pt1.y,
                            obsObj_nearSlot.obj[i].pt2.x - obsObj_nearSlot.obj[i].pt1.x);
        float angleDiff = Round_PI(Round_PI(angle) - finpoint[2]);
        float objLen =
            Cal_Dis_Pt2Pt(obsObj_nearSlot.obj[i].pt2, obsObj_nearSlot.obj[i].pt1);

        // 如果障碍物位于车辆左侧或右侧，则继续处理，否则跳过
        if (rspoint[0][1] * rspoint[1][1] < 0)
        {
            continue;
        }

        // 根据障碍物位置和角度判断其是否在车位的范围内
        if (rspoint[0][1] * rspointB[1] > 0)
        {
            // 如果障碍物与车位起始点在同一侧且在一定角度范围内，并且障碍物长度大于有效长度，则记录相关信息
            if (fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) && objLen > valid_obj_len)
            {
                objIdx[0][objCount[0]] = i;
                objCount[0]            = objCount[0] + 1;
                objTotalLen[0]         = objTotalLen[0] + objLen;
            }
        }

        // 类似地，判断障碍物是否在车位结束点的范围内
        if (rspoint[0][1] * rspointE[1] > 0)
        {
            if (fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) && objLen > valid_obj_len)
            {
                objIdx[1][objCount[1]] = i;
                objCount[1]            = objCount[1] + 1;
                objTotalLen[1]         = objTotalLen[1] + objLen;
            }
        }
    }

    // 计算参考方向角度
    float refTheta[2];
    for (int j = 0; j < 2; j++)
    {
        refTheta[j] = 0.0;
        for (int i = 0; i < objCount[j]; i++)
        {
            int idx = objIdx[j][i];
            RevConvert(finpoint, (float *)&obsObj_nearSlot.obj[idx].pt1, rspoint[0]);
            RevConvert(finpoint, (float *)&obsObj_nearSlot.obj[idx].pt2, rspoint[1]);

            float angle =
                atan2(rspoint[1][1] - rspoint[0][1], rspoint[1][0] - rspoint[0][0]);
            if (rspoint[0][0] > rspoint[1][0])
            {
                angle = PI + angle;
            }
            float objLen = Cal_Dis_Pt2Pt(rspoint[0], rspoint[1]);
            float factor = objLen / objTotalLen[j];
            refTheta[j] += factor * Round_PI(angle);
        }
    }

    // 根据参考方向角度调整目标点方向角度
    float tempAngle = 0;
    if (objCount[0] > 0 && objCount[1] == 0)
    {
        tempAngle = refTheta[0] * 0.8;
    }
    else if (objCount[0] == 0 && objCount[1] > 0)
    {
        tempAngle = refTheta[1] * 0.8;
    }
    else if (objCount[0] > 0 && objCount[1] > 0)
    {
        tempAngle = (refTheta[0] + refTheta[1]) * 0.5;
    }
    else
    {
        tempAngle = 0;
    }

    // 更新目标点方向角度
    finpoint[2] = Round_PI(tempAngle + finpoint[2]);
}

/**
 * @brief 更新目标车位信息。
 *
 * 该函数通过传感器数据更新目标车位信息，计算车位的各个顶点位置。
 * 通过遍历障碍物对象，计算各个障碍物的角度和长度，根据这些信息更新车位的B点和E点位置，并调整车位的角度和位置。
 *
 * @param curpos 当前车辆位置
 * @param finpoint 目标点位置信息
 * @param avmSlot AVM（环视系统）提供的车位信息
 * @param slotObj 需要更新的车位对象
 */
uint8_t UpdateSlotByObjNearSlot(const float finpoint[3], const Avm_Pot_T &avmSlot,
                                FusionObj_T &nearObj, SlotObj_T &slotObj)
{
    const float valid_obj_len   = 0.6;  // 有效对象的最小长度
    const float valid_obj_angle = 75.0; // 有效对象的最小角度

    uint8_t confidenceB[2], confidenceE[2];
    confidenceB[0] = confidenceB[1] = 0;
    confidenceE[0] = confidenceE[1] = 0;

    float reffinpoint[3];
    memcpy(reffinpoint, finpoint, sizeof(reffinpoint));

    int sensorType = RTE_PK_DataConvt_Get_IsAVM_Slot();
    if (sensorType == 0) // 如果是超声波车位，根据障碍物更新车位姿态
    {
        UpdateSlotThetaByObj(slotObj, reffinpoint);
    }

    if (sensorType == 1) // 如果是视觉车位，根据AVM（环视系统）提供的车位信息更新车位姿态
    {
        Vec2_T slotVec;
        slotVec.vx = avmSlot.near_front.x + avmSlot.near_rear.x - avmSlot.far_front.x -
                     avmSlot.far_rear.x;
        slotVec.vy = avmSlot.near_front.y + avmSlot.near_rear.y - avmSlot.far_front.y -
                     avmSlot.far_rear.y;
        slotVec.vx = 0.5 * slotVec.vx;
        slotVec.vy = 0.5 * slotVec.vy;
        Normalize_Vec2(&slotVec);
        float newtheta = atan2(slotVec.vy, slotVec.vx);
        reffinpoint[2] = reffinpoint[2] * 0.0f + newtheta * 1.0f;
        printf("%d---change theta : oldtheta: %f, newtheta: %f,\n", __LINE__, finpoint[2],
               reffinpoint[2]);
    }

    float rspointB[2], rspointE[2], oldptB[2], oldptE[2];
    RevConvert(reffinpoint, (float *)&slotObj.ptB, rspointB);
    RevConvert(reffinpoint, (float *)&slotObj.ptE, rspointE);
    memcpy(oldptB, rspointB, sizeof(oldptB));
    memcpy(oldptE, rspointE, sizeof(oldptE));

    float bewide = fabs(rspointB[1]) + fabs(rspointE[1]);
    rspointB[1]  = rspointB[1] + 0.5 * sign(rspointB[1]); // 外扩50cm
    rspointE[1]  = rspointE[1] + 0.5 * sign(rspointE[1]); // 外扩50cm

    float oldrsB[2], oldrsE[2];
    memcpy(oldrsB, rspointB, sizeof(oldrsB));
    memcpy(oldrsE, rspointE, sizeof(oldrsE));

    float rspoint[2][2];

    int objIdx[2][SF_OBJ_NUM / 2];
    int objCount[2]      = {0, 0};
    float objTotalLen[2] = {0.0, 0.0};

    // 遍历障碍物对象，根据条件更新B点和E点的位置
    reffinpoint[2] = Round_PI(reffinpoint[2]);
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        if (nearObj.attr[i] == 0)
        {
            continue;
        }
        RevConvert(reffinpoint, (float *)&nearObj.obj[i].pt1, rspoint[0]);
        RevConvert(reffinpoint, (float *)&nearObj.obj[i].pt2, rspoint[1]);

        float angle     = atan2(nearObj.obj[i].pt2.y - nearObj.obj[i].pt1.y,
                                nearObj.obj[i].pt2.x - nearObj.obj[i].pt1.x);
        float angleDiff = Round_PI(Round_PI(angle) - reffinpoint[2]);
        float objLen    = Cal_Dis_Pt2Pt(nearObj.obj[i].pt2, nearObj.obj[i].pt1);

        if (rspoint[0][1] * rspoint[1][1] < 0)
        {
            continue;
        }

        if (rspoint[0][1] * rspointB[1] > 0)
        {
            rspointB[0] =
                rte_max(fabs(rspointB[0]), fabs(rspoint[0][0])) * sign(rspointB[0]);
            if ((fabs(sinf(angleDiff)) > sinf(valid_obj_angle * PI_RAD) &&
                 objLen > valid_obj_len) ||
                (i == 0 || i == SF_OBJ_NUM / 2))
            {
                rspointB[0] = fabs(rspoint[0][0]) * sign(rspointB[0]);
                confidenceB[1] =
                    (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1; // rspointB[0] 的可行度
            }

            if (rspoint[0][0] * rspointB[0] > 0)
            {
                rspointB[1] =
                    rte_min(fabs(rspointB[1]), fabs(rspoint[0][1])) * sign(rspointB[1]);
                if ((fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) &&
                     objLen > valid_obj_len) ||
                    (i == 0 || i == SF_OBJ_NUM / 2))
                {
                    rspointB[1]    = fabs(rspoint[0][1]) * sign(rspointB[1]);
                    confidenceB[0] = (i != 0 && i != SF_OBJ_NUM / 2)
                                         ? 0x11
                                         : 0x1; // rspointB[1] 的可行度

                    objIdx[0][objCount[0]] = i;
                    objCount[0]            = objCount[0] + 1;
                    objTotalLen[0]         = objTotalLen[0] + objLen;
                }
            }
        }

        if (rspoint[1][1] * rspointB[1] > 0)
        {
            rspointB[0] =
                rte_max(fabs(rspointB[0]), fabs(rspoint[1][0])) * sign(rspointB[0]);

            if ((fabs(sinf(angleDiff)) > sinf(valid_obj_angle * PI_RAD) &&
                 objLen > valid_obj_len) ||
                (i == 0 || i == SF_OBJ_NUM / 2))
            {
                rspointB[0]    = fabs(rspoint[0][0]) * sign(rspointB[0]);
                confidenceB[1] = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
            }

            if (rspoint[1][0] * rspointB[0] > 0)
            {
                rspointB[1] =
                    rte_min(fabs(rspointB[1]), fabs(rspoint[1][1])) * sign(rspointB[1]);
                if ((fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) &&
                     objLen > valid_obj_len) ||
                    (i == 0 || i == SF_OBJ_NUM / 2))
                {
                    rspointB[1]            = fabs(rspoint[0][1]) * sign(rspointB[1]);
                    confidenceB[0]         = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
                    objIdx[0][objCount[0]] = i;
                    objCount[0]            = objCount[0] + 1;
                    objTotalLen[0]         = objTotalLen[0] + objLen;
                }
            }
        }

        if (rspoint[0][1] * rspointE[1] > 0)
        {
            rspointE[0] =
                rte_max(fabs(rspointE[0]), fabs(rspoint[0][0])) * sign(rspointE[0]);
            if ((fabs(sinf(angleDiff)) > sinf(valid_obj_angle * PI_RAD) &&
                 objLen > valid_obj_len) ||
                (i == 0 || i == SF_OBJ_NUM / 2))
            {
                rspointE[0]    = fabs(rspoint[0][0]) * sign(rspointE[0]);
                confidenceE[1] = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
            }

            if (rspoint[0][0] * rspointE[0] > 0)
            {
                rspointE[1] =
                    rte_min(fabs(rspointE[1]), fabs(rspoint[0][1])) * sign(rspointE[1]);
                if ((fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) &&
                     objLen > valid_obj_len) ||
                    (i == 0 || i == SF_OBJ_NUM / 2))
                {
                    rspointE[1]            = fabs(rspoint[0][1]) * sign(rspointE[1]);
                    confidenceE[0]         = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
                    objIdx[1][objCount[1]] = i;
                    objCount[1]            = objCount[1] + 1;
                    objTotalLen[1]         = objTotalLen[1] + objLen;
                }
            }
        }

        if (rspoint[1][1] * rspointE[1] > 0)
        {
            rspointE[0] =
                rte_max(fabs(rspointE[0]), fabs(rspoint[1][0])) * sign(rspointE[0]);
            if ((fabs(sinf(angleDiff)) > sinf(valid_obj_angle * PI_RAD) &&
                 objLen > valid_obj_len) ||
                (i == 0 || i == SF_OBJ_NUM / 2))
            {
                rspointE[0]    = fabs(rspoint[0][0]) * sign(rspointE[0]);
                confidenceE[1] = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
            }

            if (rspoint[1][0] * rspointE[0] > 0)
            {
                rspointE[1] =
                    rte_min(fabs(rspointE[1]), fabs(rspoint[1][1])) * sign(rspointE[1]);
                if ((fabs(sinf(angleDiff)) < sinf(15 * PI_RAD) &&
                     objLen > valid_obj_len) ||
                    (i == 0 || i == SF_OBJ_NUM / 2))
                {
                    rspointE[1]            = fabs(rspoint[0][1]) * sign(rspointE[1]);
                    confidenceE[0]         = (i != 0 && i != SF_OBJ_NUM / 2) ? 0x11 : 0x1;
                    objIdx[1][objCount[1]] = i;
                    objCount[1]            = objCount[1] + 1;
                    objTotalLen[1]         = objTotalLen[1] + objLen;
                }
            }
        }
    }

    if (sensorType != 0)
    {
        float rspoint[2];
        // oldrsB 当前的  rspointB 更新后的 rspoint 视觉提供的
        RevConvert(reffinpoint, (float *)&avmSlot.near_rear, rspoint);

        // 车位坐标 X向
        if (confidenceB[1] != 1 && fabs(rspointB[0] - oldrsB[0]) > 0.01)
        {
            float rx    = rte_max(2.0, fabs(rspointB[0]));
            rspointB[0] = rx * sign(rspointB[0]);
        }

        // 车位坐标 Y向
        // 超声波非常确定, 则使用超声波的;
        // 超声波不确定, 并且有更新, 则由超声波和视觉共同决定
        // 超声波不确定, 并且没有更新, 则使用原来的
        if (confidenceB[0] != 0x11 && fabs(rspointB[1] - oldrsB[1]) > 0.01)
        {
            float ry    = 0.6 * fabs(oldrsB[1]) + 0.4 * fabs(rspointB[1]);
            rspointB[1] = ry * sign(rspointB[1]);
        }

        RevConvert(reffinpoint, (float *)&avmSlot.near_front, rspoint);
        if (confidenceE[1] != 1 && fabs(rspointE[0] - oldrsE[0]) > 0.01)
        {
            float rx    = rte_max(2.0, fabs(rspointE[0]));
            rspointE[0] = rx * sign(rspointE[0]);
        }

        if (confidenceE[0] != 0x11 && fabs(rspointE[1] - oldrsE[1]) > 0.01)
        {
            float ry    = 0.6 * fabs(oldrsE[1]) + 0.4 * fabs(rspointE[1]);
            rspointE[1] = ry * sign(rspointE[1]);
        }

        // B变化, E未变化
        if (fabs(rspointE[1] - oldrsE[1]) < 0.01 && fabs(rspointB[1] - oldrsB[1]) > 0.01)
        {
            if (fabs(rspointB[1]) < fabs(oldptB[1]))
            {
                rspointE[1] = (bewide - fabs(rspointB[1])) * sign(rspointE[1]);
            }
        }

        // E变化, B未变化
        else if (fabs(rspointB[1] - oldrsB[1]) < 0.01 &&
                 fabs(rspointE[1] - oldrsE[1]) > 0.01)
        {
            if (fabs(rspointE[1]) < fabs(oldptE[1]))
            {
                rspointB[1] = (bewide - fabs(rspointE[1])) * sign(rspointB[1]);
            }
        }
        // E变化, B变化
        else if (fabs(rspointB[1] - oldrsB[1]) < 0.01 &&
                 fabs(rspointE[1] - oldrsE[1]) > 0.01)
        {
            if (fabs(rspointB[1]) < fabs(oldptB[1]) &&
                fabs(rspointE[1]) < fabs(oldptE[1]))
            {
                ;
            }
            else if (fabs(rspointB[1]) < fabs(oldptB[1]))
            {
                rspointE[1] = (bewide - fabs(rspointB[1])) * sign(rspointE[1]);
            }
            else if (fabs(rspointE[1]) < fabs(oldptE[1]))
            {
                rspointB[1] = (bewide - fabs(rspointE[1])) * sign(rspointB[1]);
            }
            else
            {
                rspointB[1] = oldptB[1];
                rspointE[1] = oldptE[1];
            }
        }
        // E未变化, B未变化
        else if (fabs(rspointB[1] - oldrsB[1]) < 0.01 &&
                 fabs(rspointE[1] - oldrsE[1]) > 0.01)
        {
            rspointB[1] = oldptB[1];
            rspointE[1] = oldptE[1];
        }
    }
    else
    {
        float refY[2];
        for (int j = 0; j < 2; j++)
        {
            refY[j] = 0.0;
            for (int i = 0; i < objCount[j]; i++)
            {
                int idx = objIdx[j][i];
                RevConvert(reffinpoint, (float *)&nearObj.obj[idx].pt1, rspoint[0]);
                RevConvert(reffinpoint, (float *)&nearObj.obj[idx].pt2, rspoint[1]);

                float angle =
                    atan2(rspoint[1][1] - rspoint[0][1], rspoint[1][0] - rspoint[0][0]);
                if (rspoint[0][0] > rspoint[1][0])
                {
                    angle = PI + angle;
                }
                float objLen = Cal_Dis_Pt2Pt(rspoint[0], rspoint[1]);
                float factor = objLen / objTotalLen[j];
                refY[j]      = refY[j] + factor * (rspoint[0][1] + rspoint[1][1]) * 0.5;
            }
        }

        if (objCount[0] > 0)
        {
            rspointB[1] = refY[0];
        }

        if (objCount[1] > 0)
        {
            rspointE[1] = refY[1];
        }
    }

    auto UpdateBE = [](float v1[2], const float v2[2]) {
        if (v1[1] * v2[1] > 0 && fabs(v1[1] - v2[1]) > 0.01)
        {
            v1[0] = rte_max(fabs(v1[0]), fabs(v2[0])) * sign(v1[0]);
            v1[1] = rte_min(fabs(v1[1]), fabs(v2[1])) * sign(v1[1]);
        }
    };

    float rspointAlarm[2];
    if (nearObj.attr[0] != 0)
    {
        RevConvert(reffinpoint, (float *)&nearObj.obj[0].pt1, rspoint[0]);
        RevConvert(reffinpoint, (float *)&nearObj.obj[0].pt2, rspoint[1]);
        rspointAlarm[0] = (rspoint[0][0] + rspoint[1][0]) * 0.5;
        rspointAlarm[1] = (rspoint[0][1] + rspoint[1][1]) * 0.5;
        nearObj.attr[0] = 0;
        UpdateBE(rspointB, rspointAlarm);
        UpdateBE(rspointE, rspointAlarm);
    }

    if (nearObj.attr[SF_OBJ_NUM / 2] != 0)
    {
        RevConvert(reffinpoint, (float *)&nearObj.obj[SF_OBJ_NUM / 2].pt1, rspoint[0]);
        RevConvert(reffinpoint, (float *)&nearObj.obj[SF_OBJ_NUM / 2].pt2, rspoint[1]);
        rspointAlarm[0]              = (rspoint[0][0] + rspoint[1][0]) * 0.5;
        rspointAlarm[1]              = (rspoint[0][1] + rspoint[1][1]) * 0.5;
        nearObj.attr[SF_OBJ_NUM / 2] = 0;
        UpdateBE(rspointB, rspointAlarm);
        UpdateBE(rspointE, rspointAlarm);
    }

    float temp[2];
    RevConvert(reffinpoint, (float *)&slotObj.ptA, temp);
    temp[0] = rspointB[0];
    Convert(reffinpoint, temp, (float *)&slotObj.ptA);

    Convert(reffinpoint, rspointB, (float *)&slotObj.ptB);

    RevConvert(reffinpoint, (float *)&slotObj.ptC, temp);
    temp[1] = rspointB[1];
    Convert(reffinpoint, temp, (float *)&slotObj.ptC);

    RevConvert(reffinpoint, (float *)&slotObj.ptD, temp);
    temp[1] = rspointE[1];
    Convert(reffinpoint, temp, (float *)&slotObj.ptD);

    Convert(reffinpoint, rspointE, (float *)&slotObj.ptE);

    RevConvert(reffinpoint, (float *)&slotObj.ptF, temp);
    temp[0] = rspointE[0];
    Convert(reffinpoint, temp, (float *)&slotObj.ptF);

    uint8_t result = 0;
    SetSlotFlag(result, ((confidenceB[0] >> 4) & 0x01), 6);
    SetSlotFlag(result, ((confidenceB[1] >> 4) & 0x01), 4);
    SetSlotFlag(result, ((confidenceE[0] >> 4) & 0x01), 2);
    SetSlotFlag(result, ((confidenceE[1] >> 4) & 0x01), 0);
    return result;
}

/**
 * @brief 更新目标车位信息。
 *
 * 该函数通过传感器数据更新目标车位信息，计算车位的各个顶点位置。
 * 通过遍历障碍物对象，计算各个障碍物的角度和长度，根据这些信息更新车位的B点和E点位置，并调整车位的角度和位置。
 *
 * @param curpos 当前车辆位置
 * @param finpoint 目标点位置信息
 * @param avmSlot AVM（环视系统）提供的车位信息
 * @param slotObj 需要更新的车位对象
 */
uint8_t PathExec_UpdateSlotByObjNearSlot(const float curpos[4], const float finpoint[3],
                                         const Avm_Pot_T &avmSlot, SlotObj_T &slotObj)
{
    // 清除靠近车位的障碍物对象
    FusionObj_T &fusion = PathExec_Init_NearSlotObjs();

    // 构建雷达报警对象
    float objs[4][4];
    int count = PathExec_BuildSideRadarAlarmObj(curpos, finpoint, objs);
    for (int i = 0; i < count; i++)
    {
        PathExec_AddObsObjNearSlot(objs[i], finpoint);
    }

    // 更新靠近车位的障碍物对象
    UpdateBObjNearSlot(finpoint, fusion);

    // 如果没有靠近车位的障碍物对象，则直接返回
    if (fusion.num == 0 && fusion.num_accu == 0)
    {
        return 0;
    }

    auto result = UpdateSlotByObjNearSlot(finpoint, avmSlot, fusion, slotObj);

    return result;
}

const float update_target_wide_limit = 0.45;
static void Calc_Target1(const int slotshap, const SlotObj_T &slotobj, VehPos_T &targpos)
{
    int slotdir =
        (slotshap == 1 || slotshap == 3 || slotshap == 5 || slotshap == 6) ? 1 : 2;

    float line[4];
    float targp[2] = {targpos.x, targpos.y};
    line[0]        = slotobj.ptB.x;
    line[1]        = slotobj.ptB.y;
    line[2]        = slotobj.ptC.x;
    line[3]        = slotobj.ptC.y;
    auto distBC    = PK_PointToLineDist(line, targp);

    line[0]     = slotobj.ptD.x;
    line[1]     = slotobj.ptD.y;
    line[2]     = slotobj.ptE.x;
    line[3]     = slotobj.ptE.y;
    auto distDE = PK_PointToLineDist(line, targp);

    float minSide = rte_min(distBC, distDE);
    if (minSide - VEHICLE_WID / 2 > update_target_wide_limit)
    {
        return;
    }

    float offset = fabs(distDE - distBC) / 2;
    if (offset + minSide > VEHICLE_WID / 2 + update_target_wide_limit)
    {
        offset = VEHICLE_WID / 2 + update_target_wide_limit - minSide;
    }

    float tempTarget[3] = {0, offset, targpos.theta};
    if ((distBC > distDE && slotdir == 1) || (distBC < distDE && slotdir == 2))
    {
        tempTarget[1] = -offset;
    }

    float newTarget[2];
    float finpoint[3] = {targpos.x, targpos.y, targpos.theta};
    Convert(finpoint, tempTarget, newTarget);
    memcpy(&targpos, newTarget, sizeof(newTarget));
}

/**
 * @brief 计算车辆停车的目标位置。
 *
 * 该函数根据车位类型和车位对象计算车辆停车的目标位置。
 *
 * @param slottype 车位来源类型（0表示超声波车位，其他值表示不同类型）
 * @param slotobj 包含车位信息的结构体
 * @param avmPot 包含来自AVM（环视系统）系统的车辆位置信息的结构体
 * @param targpos 引用一个结构体，用于存储计算出的目标位置
 */
void Calc_Target(const int slottype, const uint8_t confirence, const SlotObj_T &slotobj,
                 const Avm_Pot_T &avmPot, VehPos_T &targpos)
{
    // 目标位置的安全宽度和狭窄宽度限制
    const float wide_safe   = update_target_wide_limit + VEHICLE_WID / 2;
    const float wide_narrow = 0.35 + VEHICLE_WID / 2;
    const float v_wide_min  = VEHICLE_WID / 2 - 0.50;
    const float calc_step   = 0.015;

    // 计算车位C点和D点之间的距离和方向
    float vl = Cal_Dis_Pt2Pt(slotobj.ptC, slotobj.ptD);
    float vx = (slotobj.ptD.x - slotobj.ptC.x) / vl;
    float vy = (slotobj.ptD.y - slotobj.ptC.y) / vl;

    // 初始化目标位置数组和线数组
    float targp[2] = {targpos.x, targpos.y};
    float line[4];

    if (slottype == 0)
    {
        // 设置线段BC
        line[0]     = slotobj.ptB.x;
        line[1]     = slotobj.ptB.y;
        line[2]     = slotobj.ptC.x;
        line[3]     = slotobj.ptC.y;
        auto distBC = PK_PointToLineDist(line, targp);

        // 设置线段DE
        line[0]     = slotobj.ptD.x;
        line[1]     = slotobj.ptD.y;
        line[2]     = slotobj.ptE.x;
        line[3]     = slotobj.ptE.y;
        auto distDE = PK_PointToLineDist(line, targp);

        // 计算角度
        float angleBC =
            atan2(slotobj.ptB.y - slotobj.ptC.y, slotobj.ptB.x - slotobj.ptC.x);
        float angleED =
            atan2(slotobj.ptE.y - slotobj.ptD.y, slotobj.ptE.x - slotobj.ptD.x);
        targpos.theta = Round_PI((angleBC + angleED) * 0.5);

        // 如果BC和DE距离都大于安全宽度，返回
        if (rte_min(distBC, distDE) > wide_safe)
        {
            return;
        }

        if ((IsSetSlotFlag(confirence, 6) || IsSetSlotFlag(confirence, 2)) &&
            distBC + distDE < VEHICLE_WID + 0.8)
        {
            float offset = 0.0f;
            if (IsSetSlotFlag(confirence, 6) && IsSetSlotFlag(confirence, 2))
            {
                offset = (distBC + distDE) * 0.5 - distBC;
            }
            else if (IsSetSlotFlag(confirence, 6))
            {
                offset = (VEHICLE_WID * 0.5 + 0.35) - distBC;
            }
            else
            {
                offset = distDE - (VEHICLE_WID * 0.5 + 0.35);
            }
            targpos.x = targpos.x + vx * offset;
            targpos.y = targpos.y + vy * offset;
            return;
        }

        float d   = 0.0;
        float dir = (distBC > distDE) ? -1.0 : 1.0;
        while (1)
        {
            float step = rte_max(calc_step, 0.03);
            if (distBC + d > wide_safe && distDE - d > wide_safe)
            {
                break;
            }
            if (fabs((distBC + d) - (distDE - d)) < step)
            {
                break;
            }
            d = d + step * dir;
        }

        targpos.x = targpos.x + vx * d;
        targpos.y = targpos.y + vy * d;
    }
    else if (slottype != 2) // 指定车位
    {
        // 视觉车位角点BC线段
        line[0]      = avmPot.near_rear.x;
        line[1]      = avmPot.near_rear.y;
        line[2]      = avmPot.far_rear.x;
        line[3]      = avmPot.far_rear.y;
        auto distVBC = PK_PointToLineDist(line, targp);

        // 视觉车位角点DE线段
        line[0]      = avmPot.near_front.x;
        line[1]      = avmPot.near_front.y;
        line[2]      = avmPot.far_front.x;
        line[3]      = avmPot.far_front.y;
        auto distVDE = PK_PointToLineDist(line, targp);

        auto distV = (distVBC + distVDE) * 0.5;
        float targv[2];
        targv[0] = targpos.x + vx * (distV - distVBC);
        targv[1] = targpos.y + vy * (distV - distVBC);

        // 设置线段BC
        line[0]     = slotobj.ptB.x;
        line[1]     = slotobj.ptB.y;
        line[2]     = slotobj.ptC.x;
        line[3]     = slotobj.ptC.y;
        auto distBC = PK_PointToLineDist(line, targv);

        // 设置线段DE
        line[0]     = slotobj.ptD.x;
        line[1]     = slotobj.ptD.y;
        line[2]     = slotobj.ptE.x;
        line[3]     = slotobj.ptE.y;
        auto distDE = PK_PointToLineDist(line, targv);

        if ((IsSetSlotFlag(confirence, 6) || IsSetSlotFlag(confirence, 2)) &&
            distBC + distDE < VEHICLE_WID + 0.8)
        {
            float offset = 0.0f;
            if (IsSetSlotFlag(confirence, 6) && IsSetSlotFlag(confirence, 2))
            {
                offset = (distBC + distDE) * 0.5 - distBC;
            }
            else if (IsSetSlotFlag(confirence, 6))
            {
                offset = (VEHICLE_WID * 0.5 + 0.35) - distBC;
            }
            else
            {
                offset = distDE - (VEHICLE_WID * 0.5 + 0.35);
            }

            // float oldx = targpos.x;
            // float oldy = targpos.y;
            targpos.x = targv[0] + vx * offset;
            targpos.y = targv[1] + vy * offset;
            // printf("----------- update target:conference %d %f %f %f old: %f %f , %f %f
            // new: %f %f\n",
            //     confirence, distBC, distDE, offset, oldx, oldy, targv[0], targv[1],
            //     targpos.x, targpos.y);

            return;
        }

        // 设置线段
        line[0] = avmPot.near_rear.x;
        line[1] = avmPot.near_rear.y;
        line[2] = avmPot.far_rear.x;
        line[3] = avmPot.far_rear.y;
        distVBC = PK_PointToLineDist(line, targv);

        // 设置线段near_front和far_front
        line[0] = avmPot.near_front.x;
        line[1] = avmPot.near_front.y;
        line[2] = avmPot.far_front.x;
        line[3] = avmPot.far_front.y;
        distVDE = PK_PointToLineDist(line, targv);

        // 计算车位向量并归一化
        Vec2_T slotVec;
        slotVec.vx = {avmPot.near_front.x + avmPot.near_rear.x - avmPot.far_front.x -
                      avmPot.far_rear.x};
        slotVec.vy = {avmPot.near_front.y + avmPot.near_rear.y - avmPot.far_front.y -
                      avmPot.far_rear.y};
        slotVec.vx = 0.5 * slotVec.vx;
        slotVec.vy = 0.5 * slotVec.vy;
        Normalize_Vec2(&slotVec);
        float newtheta = atan2(slotVec.vy, slotVec.vx);
        targpos.theta  = 0.6 * targpos.theta + 0.4 * newtheta;

        float d = fabs(distVBC - distVDE) * 0.5;
        if (d < 0.015)
        {
            if (distBC > wide_narrow && distDE > wide_narrow)
            {
                targpos.x = targv[0];
                targpos.y = targv[1];
                return;
            }
        }

        d         = 0.0;
        float dir = (distBC > distDE) ? -1.0 : 1.0;
        while (1)
        {
            // minOffset = 0.10;
            if (distBC + d > wide_narrow && distDE - d > wide_narrow &&
                distVBC + d > v_wide_min && distVDE - d > v_wide_min)
            {
                break;
            }

            // minOffset = 0.03;
            if ((distVBC + d < v_wide_min && dir < 0) ||
                (distVDE - d < v_wide_min && dir > 0) ||
                fabs(distBC + 2 * d - distDE) < calc_step)
            {
                break;
            }
            d = d + calc_step * dir;
        }

        targpos.x = targv[0] + vx * d;
        targpos.y = targv[1] + vy * d;
    }
}

/**
 * @brief 更新车位附近的超声波障碍物
 *
 * 根据车位类型选择更新相应长距超声波扫描到的障碍物（只更新一边）
 *
 * @param
 * @return
 */
void PathExec_UpdateObsObj_AroundTargSlot()
{
    FusionObj_T fusionSlot; //  for replan
    ObsObj_AnotherSide.num = ObsObj_AnotherSide.num_accu = 0;
    auto FillResult = [](const int dir, const FusionObj_T &nearSlot,
                         const float target[3]) {
        printf("PathExecute_GetObsObj_AroundTargSlot dir %d number = %d\n", dir,
               nearSlot.num);
        for (int index = 0; index < SF_OBJ_NUM; index++)
        {
            if (Get_Segment_Len(nearSlot.obj[index]) < 0.10)
            {
                continue;
            }

            if (nearSlot.attr[index] / 10000 != dir)
            {
                continue;
            }

            float dist1 = Cal_Dis_Pt2Pt((float *)&nearSlot.obj[index].pt1, target);
            float dist2 = Cal_Dis_Pt2Pt((float *)&nearSlot.obj[index].pt2, target);
            if (dist1 > 15.0f && dist2 > 15.0f)
            {
                continue;
            }

            int objIdx = ObsObj_AnotherSide.num_accu % SF_OBJ_NUM;
            memcpy((float *)&ObsObj_AnotherSide.obj[objIdx], &nearSlot.obj[index],
                   4 * sizeof(float));
            ObsObj_AnotherSide.num_accu += 1;
            ObsObj_AnotherSide.num = rte_min(ObsObj_AnotherSide.num_accu, SF_OBJ_NUM);
        }
    };

    float target[3];
    RTE_PK_SlotDetect_Get_TargPos(target);

    int slotshap = RTE_PK_SlotDetect_Get_SlotShape();
    if (Slot_Dir(slotshap) == 1)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(
            &fusionSlot); // get the opposite obs
        FillResult(2, fusionSlot, target);

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(
            &fusionSlot); // get the opposite obs
        FillResult(2, fusionSlot, target);
    }
    else
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(
            &fusionSlot); // get the opposite obs
        FillResult(1, fusionSlot, target);

        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(
            &fusionSlot); // get the opposite
        FillResult(1, fusionSlot, target);
    }

    // RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(&FS_A_NearSlot);
    // FillResult(All_ObsNum, 2);

    // RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(&FS_A_NearSlot);
    // FillResult(All_ObsNum, 1);
}

/**
 * @brief 判断障碍物是否在垂直车位内
 * @param curSt 当前泊车模块参数结构体
 * @param objs 障碍物线段
 * @param count 障碍物数量
 * @return bool
 */
static bool PathExec_IsObjInVertSlot(const PK_Cur_AutodrPara *curSt,
                                     const LineSeg_T *objs, const int count)
{
    for (int i = 0; i < count; i++)
    {
        float obj[2], rspoint[3];
        obj[0] = (objs[i].pt1.x + objs[i].pt2.x) * 0.5;
        obj[1] = (objs[i].pt1.y + objs[i].pt2.y) * 0.5;
        RevConvert(curSt[0].finpos, obj, rspoint);

        if (fabs(rspoint[1]) < VEHICLE_WID * 0.4 && rspoint[0] < VEHICLE_LEN * 1.0)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 是否有障碍物迫停车辆处理
 *
 * @param curSt 当前泊车模块参数结构体，更新其中的障碍物迫停相关数据
 * @param enviObj 目标车位边界
 * @return
 */
void PathExec_DangerObj(PK_Cur_AutodrPara curSt[1], float enviObj[5][4])
{
    if (RTE_PK_ObjAvoid_Get_DangerSt() == 0)
    {
        PathExec_ClearObjDanger(curSt);
        curSt[0].dangerCounter = 0;
    }
    else
    {
        curSt[0].dangerCounter = curSt[0].dangerCounter + 1;
    }

    if (curSt[0].dangerCounter > B_DangeReplanWaitCnt_ccp) // should last for 2.2s
    {
        uint8_t danger = PK_ObjAvoid_Dir();
        PathExec_SetObjDanger(curSt, danger);
        curSt[0].dangerCounter = 2 * B_DangeReplanWaitCnt_ccp;
    }

    int suspend = 0;
    if (curSt[0].slotshape == PK_SLOT_LEFT_VERT ||
        curSt[0].slotshape == PK_SLOT_RIGHT_VERT) // 垂直车位
    {
        if (PathExec_IsRearDanger(curSt)) // 后方障碍物
        {
            if (fabs(curSt[0].ry_cur_to_fin) > VEHICLE_WID &&
                fabs(sinf(curSt[0].rtheta_cur_to_fin)) > fabs(sinf(70 * PI_RAD)) &&
                curSt[0].rx_cur_to_fin > VEHICLE_LEN)
            {
                suspend = 1;
            }

            for (int i = 1; i <= 4; i++)
            {
                LineSeg_T objLine;
                int count = BuildRearUradarObj(curSt[0].curpos, curSt[0].finpos, i,
                                               objLine); // RSL ROL  Left
                if (count > 0 &&
                    PathExec_IsObjInVertSlot(curSt, &objLine,
                                             1)) // 有障碍物且障碍物在车位内部
                {
                    suspend = 1;
                    break;
                }
            }
        }
    }
    else // 平行车位
    {
        if (PathExec_IsRearDanger(curSt))
        {
            if (curSt[0].rx_cur_to_fin > REAR_SUSPENSION &&
                fabs(curSt[0].ry_cur_to_fin) > VEHICLE_WID * 0.5)
            {
                suspend = 1;
            }
        }

        if (PathExec_IsFrontDanger(curSt))
        {
            if (curSt[0].rx_cur_to_fin < -REAR_SUSPENSION &&
                fabs(curSt[0].ry_cur_to_fin) > VEHICLE_WID * 0.5)
            {
                suspend = 1;
            }
        }
    }

    if (suspend != 0 && !PathExec_IsSuspendByObjs(curSt)) // 有障碍物且障碍物停止标志为0
    {
        PathExec_SetSuspendByObjs(curSt); // 设置障碍物停止标志
    }
    else if (suspend == 0 &&
             PathExec_IsSuspendByObjs(curSt)) // 无障碍物且障碍物停止标志为1
    {
        PathExec_ClearSuspendByObjs(curSt); // 初始化障碍物停止标志
        curSt[0].stillCount = 0;            // 初始化车辆停止计数
    }
}
