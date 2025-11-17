#include "PK_Calibration.h"
#include "PK_ObjAvoid.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PathExec_Debug.h"
#include "PathExec_Config.h"
#include "PK_PathPlan.h"
#include "PK_Utility.h"
#include "PK_B.h"

const float para_ds_tramax   = 1.0f; // Safe_dist
const float after_tune_dist  = 2.0f;
const float after_tune_eps   = 0.6f;
const float post_min_trajlen = 4.0f * post_swell;

static void SlotToObjs(const float points[12], float objs[5][4])
{
    memcpy(objs[0], &points[0], sizeof(float) * 4);
    memcpy(objs[1], &points[2], sizeof(float) * 4);
    memcpy(objs[2], &points[4], sizeof(float) * 4);
    memcpy(objs[3], &points[6], sizeof(float) * 4);
    memcpy(objs[4], &points[8], sizeof(float) * 4);
}

/**
 * @brief 根据当前车辆位置和目标点生成正前和正后后方雷达的报警对象。
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
static int BuildCenterUradarObj(const float curpos[4], const float finpoint[3],
                                const int left, LineSeg_T &objs)
{
    // 定义危险状态掩码
    const uint16_t fleft_mask = 0x04;
    const uint16_t frigh_mask = 0x08;
    const uint16_t rleft_mask = 0x40;
    const uint16_t rrigh_mask = 0x80;
    // const uint16_t fleft_stopmask = 0x0400;
    // const uint16_t frigh_stopmask = 0x0800;
    // const uint16_t rleft_stopmask = 0x4000;
    // const uint16_t rrigh_stopmask = 0x8000;

    // 计算当前车辆方向角与目标方向角的差值
    // float angle = Round_PI(curpos[2] - finpoint[2]);
    // 获取危险状态
    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();

    float temp[4];
    if (left == 1 && (dangerSt & fleft_mask) != 0)
    {
        temp[0] = VEHICLE_LEN - REAR_SUSPENSION + 0.35f;
        temp[2] = VEHICLE_LEN - REAR_SUSPENSION + 0.35f;
        temp[1] = 0.30;
        temp[3] = 0.65;

        if ((dangerSt >> 8) & fleft_mask)
        {
            temp[0] = temp[0] - sign(temp[0]) * 0.06;
            temp[2] = temp[2] - sign(temp[2]) * 0.06;
        }

        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 2 && (dangerSt & frigh_mask) != 0)
    {
        temp[0] = VEHICLE_LEN - REAR_SUSPENSION + 0.35f;
        temp[2] = VEHICLE_LEN - REAR_SUSPENSION + 0.35f;
        temp[1] = -0.30;
        temp[3] = -0.65;

        if ((dangerSt >> 8) & frigh_mask)
        {
            temp[0] = VEHICLE_LEN - REAR_SUSPENSION + 0.30f;
            temp[2] = VEHICLE_LEN - REAR_SUSPENSION + 0.30f;
        }

        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 3 && (dangerSt & rleft_mask) != 0)
    {
        float temp[4] = {-(REAR_SUSPENSION + 0.35f), 0.65f, -(REAR_SUSPENSION + 0.35f),
                         0.30f};
        if ((dangerSt >> 8) & rleft_mask)
        {
            temp[0] = temp[0] - sign(temp[0]) * 0.06;
            temp[2] = temp[2] - sign(temp[2]) * 0.06;
        }
        Convert(curpos, &temp[0], (float *)&objs.pt1);
        Convert(curpos, &temp[2], (float *)&objs.pt2);
    }
    else if (left == 4 && (dangerSt & rrigh_mask) != 0)
    {
        float temp[4] = {-(REAR_SUSPENSION + 0.45f), -0.65, -(REAR_SUSPENSION + 0.50f),
                         -0.30};
        if ((dangerSt >> 8) & rrigh_mask)
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

int PathExec_BuildCenterRadarAlarmObj(const float curpos[4], const float finpoint[3],
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
    if (BuildCenterUradarObj(curpos, finpoint, 1, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    // 生成右侧雷达对象
    if (BuildCenterUradarObj(curpos, finpoint, 2, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    // 生成左偏移雷达对象
    if (BuildCenterUradarObj(curpos, finpoint, 3, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    // 生成右偏移雷达对象
    if (BuildCenterUradarObj(curpos, finpoint, 4, line) > 0)
    {
        memcpy(objs[count], &line, sizeof(LineSeg_T));
        count++;
    }

    return count;
}

static void UpdateParaSlotByObj(const PK_Cur_AutodrPara curSt[1], float enviObj[5][4])
{
    PRINT_SWITCH_INFO("PathExec_PathPostCal Update Slot!");

    // 调整BE距离
    if (fabs(cos(curSt[0].rtheta_cur_to_fin)) < cos(20 * PI_RAD) ||
        fabs(curSt[0].ry_cur_to_fin) > VEHICLE_WID * 0.4F || !PathExec_IsVehStill())
    {
        return;
    }

    // 生成左侧雷达对象
    float sideObjs[4][4];
    int sideObjNum =
        PathExec_BuildSideRadarAlarmObj(curSt[0].curpos, curSt[0].finpos, sideObjs);

    float centerObjs[4][4];
    int centerObjNum =
        PathExec_BuildCenterRadarAlarmObj(curSt[0].curpos, curSt[0].finpos, centerObjs);

    if (centerObjNum == 0)
    {
        float temp[4];
        temp[0] = VEHICLE_LEN - REAR_SUSPENSION + 0.8f;
        temp[2] = VEHICLE_LEN - REAR_SUSPENSION + 0.8f;
        temp[1] = -0.30;
        temp[3] = 0.30;

        Convert(curSt[0].curpos, &temp[0], &centerObjs[0][0]);
        Convert(curSt[0].curpos, &temp[2], &centerObjs[0][2]);

        temp[0] = -REAR_SUSPENSION - 0.8f;
        temp[2] = -REAR_SUSPENSION - 0.8f;
        temp[1] = -0.30;
        temp[3] = 0.30;

        Convert(curSt[0].curpos, &temp[0], &centerObjs[1][0]);
        Convert(curSt[0].curpos, &temp[2], &centerObjs[1][2]);
        centerObjNum = 2;
    }

    SlotObj_T slotObj;
    SlotObj_Convert_struct(&slotObj, enviObj);
    float bcDistOld = Project_PosTo1st_rx(curSt[0].finpos, (float *)&slotObj.ptB);
    float deDistOld = Project_PosTo1st_rx(curSt[0].finpos, (float *)&slotObj.ptE);

    float bcDistNew = bcDistOld;
    float deDistNew = deDistOld;
    for (int i = 0; i < centerObjNum; i++)
    {
        float dist1 = Project_PosTo1st_rx(curSt[0].finpos, (float *)&centerObjs[i][0]);
        float dist2 = Project_PosTo1st_rx(curSt[0].finpos, (float *)&centerObjs[i][2]);
        if (dist1 > VEHICLE_LEN - REAR_SUSPENSION &&
            dist2 > VEHICLE_LEN - REAR_SUSPENSION)
        {
            bcDistNew = rte_min(dist1, bcDistNew);
            bcDistNew = rte_min(dist2, bcDistNew);
        }

        if (dist1 < -REAR_SUSPENSION && dist2 < -REAR_SUSPENSION)
        {
            deDistNew = rte_min(dist1, deDistNew);
            deDistNew = rte_min(dist2, deDistNew);
        }
    }

    float cdDistOld = Project_PosTo1st_ry(curSt[0].finpos, (float *)&slotObj.ptB);
    float cdDistNew = fabs(cdDistOld);
    for (int i = 0; i < sideObjNum; i++)
    {
        float dist1 = Project_PosTo1st_ry(curSt[0].finpos, (float *)&sideObjs[i][0]);
        float dist2 = Project_PosTo1st_ry(curSt[0].finpos, (float *)&sideObjs[i][2]);
        if (dist1 * dist2 > 0 && fabs(dist1) > VEHICLE_WID * 0.4 &&
            fabs(dist2) > VEHICLE_WID * 0.4)
        {
            cdDistNew = rte_min(cdDistNew, fabs(dist1));
            cdDistNew = rte_min(cdDistNew, fabs(dist2));
        }
    }
    cdDistNew = cdDistNew * sign(cdDistOld);

    float rsPoint[2];
    if (fabs(bcDistNew - bcDistOld) > 0.03f)
    {
        RevConvert(curSt[0].finpos, (float *)&slotObj.ptA, rsPoint);
        rsPoint[0] = bcDistNew - 1.0 * sign(bcDistNew);
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptA);

        RevConvert(curSt[0].finpos, (float *)&slotObj.ptB, rsPoint);
        rsPoint[0] = bcDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptB);

        RevConvert(curSt[0].finpos, (float *)&slotObj.ptC, rsPoint);
        rsPoint[0] = bcDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptA);
    }

    if (fabs(deDistNew - deDistOld) > 0.03f)
    {
        RevConvert(curSt[0].finpos, (float *)&slotObj.ptD, rsPoint);
        rsPoint[0] = deDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptA);

        RevConvert(curSt[0].finpos, (float *)&slotObj.ptE, rsPoint);
        rsPoint[0] = deDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptB);

        RevConvert(curSt[0].finpos, (float *)&slotObj.ptF, rsPoint);
        rsPoint[0] = deDistNew - 1.0 * sign(deDistNew);
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptF);
    }

    if (fabs(cdDistNew - cdDistOld) > 0.03f)
    {
        RevConvert(curSt[0].finpos, (float *)&slotObj.ptC, rsPoint);
        rsPoint[1] = cdDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptC);

        RevConvert(curSt[0].finpos, (float *)&slotObj.ptD, rsPoint);
        rsPoint[1] = cdDistNew;
        Convert(curSt[0].finpos, rsPoint, (float *)&slotObj.ptD);
    }

    SlotObj_Convert_float(slotObj, enviObj);
}

static void AdjustTraject(float *traject, int obj_num, float objslots[5][4],
                          float post_swell)
{
    while (fabs(traject[3]) > post_min_trajlen - 0.01f)
    {
        if (PK_PathPlan_PathVerify(traject, obj_num, objslots, post_swell) == 0)
        {
            break;
        }
        traject[3] = (fabs(traject[3]) - 0.95f * post_swell) * sign(traject[3]);
    }
}

int PathExec_ParaPostTrajPre(const PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                             float traject[2][TRAJITEM_LEN])
{
    // 1. 重新确定目标位置
    float updatePath[2][TRAJITEM_LEN]; // 更新后的路径
    float newEnviObj[5][4];

    PRINT_SWITCH_INFO("PathExec_ParaPostTrajPre 0")
    if (curSt[0].ry_cur_to_fin * curSt[0].rtheta_cur_to_fin < 0)
    {
        return 0;
    }

    PRINT_SWITCH_INFO("PathExec_ParaPostTrajPre 1")
    memcpy(newEnviObj, enviObj, sizeof(newEnviObj));
    UpdateParaSlotByObj(curSt, newEnviObj);

    float distBE = PK_PointToLineDist(newEnviObj[3], &newEnviObj[1][0]) - VEHICLE_LEN;
    PK_CopyPos(updatePath[0], curSt[0].curpos);
    updatePath[0][3] = rte_min(after_tune_dist, distBE * 2.0f);

    const float rr_array[] = {6.5f, 3.0f, 1.2f, 0.6f};
    for (uint8_t i = 0; i < sizeof(rr_array) / sizeof(rr_array[0]); i++)
    {
        // 不碰撞并且路径长度满足要求
        updatePath[0][4] = (Rrmin + rr_array[i]) * sign(curSt[0].ry_cur_to_fin);
        AdjustTraject(updatePath[0], 1, &newEnviObj[3], post_swell);

        // 路径太短
        if (fabs(updatePath[0][3]) < post_min_trajlen)
        {
            return 0;
        }

        PK_Get_Path_EndPos(updatePath[0], updatePath[1]);
        auto trajlen = rte_max(after_tune_dist, 1.5f * fabs(updatePath[0][3]));

        for (uint8_t j = 0; i < 2; i++)
        {
            updatePath[1][3] = -1.0f * trajlen;
            updatePath[1][4] = (1 - j) * 1.5f * updatePath[0][4];

            // 不碰撞并且路径长度满足要求
            AdjustTraject(updatePath[1], 2, &newEnviObj[2], post_swell);
            if (fabs(updatePath[1][3]) < post_min_trajlen)
            {
                continue;
            }

            float endPos[3];
            PK_Get_Path_EndPos(updatePath[1], endPos);

            // 不碰撞限位杆
            while (curSt[0].hasStop && fabs(updatePath[1][3]) > post_min_trajlen)
            {
                float rx = Project_PosTo1st_rx(curSt[0].finpos, endPos);
                if (rx > 0)
                {
                    break;
                }
                updatePath[1][3] =
                    (fabs(updatePath[1][3]) - post_swell) * sign(updatePath[1][3]);
                PK_Get_Path_EndPos(updatePath[1], endPos);
            }

            if (fabs(updatePath[1][3]) < post_min_trajlen)
            {
                continue;
            }

            // 车尾过中线
            float ry     = Project_PosTo1st_ry(curSt[0].finpos, endPos);
            float rtheta = Project_PosTo1st_rtheta(curSt[0].finpos, endPos);
            if (ry * rtheta < 0)
            {
                memcpy(traject, updatePath, 2 * PATH_ITEM_LEN);
                memcpy(enviObj, newEnviObj, sizeof(newEnviObj));
                PRINT_SWITCH_INFO("PathExec_ParaPostTrajPre 2")
                return 2;
            }
        }
    }

    return 0;
}

static int PathExec_ParaPostTrajStep2(PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                                      float traject[2][TRAJITEM_LEN])
{
    float radius = Rrmin + after_tune_eps;
    float distBE = PK_PointToLineDist(enviObj[3], &enviObj[1][0]) - VEHICLE_LEN;

    PK_CopyPos(traject[0], curSt[0].curpos);
    for (int i = 2; i > 0; i--)
    {
        auto theta   = curSt[0].rtheta_cur_to_fin * i * 0.5f;
        float circle = radius * fabs(theta);
        if (circle < post_min_trajlen)
        {
            radius = Rrmin + 3 * after_tune_eps;
            circle = radius * fabs(theta);
        }

        circle        = rte_min(circle, distBE * 2.0f);
        traject[0][3] = rte_min(after_tune_dist, circle);

        // 不碰撞并且路径长度满足要求
        traject[0][4] = circle * sign(curSt[0].ry_cur_to_fin);
        AdjustTraject(traject[0], 1, &enviObj[3], post_swell);
        if (fabs(traject[0][3]) < post_min_trajlen)
        {
            return 0;
        }

        while (fabs(traject[0][3]) > 3.0 * post_swell)
        {
            // 前向终点
            float rsPos[3];
            PK_Get_Path_EndPos(traject[0], traject[1]);
            CoordinadteTransfer(curSt[0].finpos, traject[1], rsPos);

            circle        = radius * fabs(rsPos[2]);
            traject[1][3] = -circle;
            traject[1][4] = -traject[0][4];

            // 反向终点
            float endPos[3];
            PK_Get_Path_EndPos(traject[1], endPos);
            CoordinadteTransfer(curSt[0].finpos, endPos, rsPos);
            if (rsPos[0] * curSt[0].ry_cur_to_fin > 0)
            {
                return 2;
            }

            AdjustTraject(traject[1], 2, &enviObj[2], post_swell);
            if (fabs(traject[1][3]) < 4.0 * post_swell)
            {
                return 0;
            }

            PK_Get_Path_EndPos(traject[1], endPos);
            CoordinadteTransfer(curSt[0].finpos, endPos, rsPos);
            if (rsPos[0] * curSt[0].ry_cur_to_fin > 0 ||
                fabs(rsPos[1]) < 0.4 * dfradioGap_allow)
            {
                return 2;
            }

            traject[0][3] =
                (fabs(traject[0][3]) - 0.95f * post_swell) * sign(traject[0][3]);
            if (fabs(traject[0][3]) < post_min_trajlen)
            {
                return 0;
            }
        }
    }

    return 0;
}

static int PathExec_ParaPostTrajStep3(PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                                      float traject[2][TRAJITEM_LEN])
{
    float radius = Rrmin + after_tune_eps;
    float circle = radius * fabs(curSt[0].rtheta_cur_to_fin);
    float distBE = PK_PointToLineDist(enviObj[3], &enviObj[1][0]) - VEHICLE_LEN;
    circle       = rte_min(circle, 2.0 * distBE);
    if (circle < 0.3f * post_min_trajlen)
    {
        radius = Rrmin + 2 * after_tune_eps;
        circle = radius * fabs(curSt[0].rtheta_cur_to_fin);
    }

    PK_CopyPos(traject[0], curSt[0].curpos);
    traject[0][3] = -1.0 * rte_min(after_tune_dist, circle);
    traject[0][4] = circle * sign(curSt[0].rtheta_cur_to_fin);
    return 1;
}

int PathExec_ParaPostTraj1(PK_Cur_AutodrPara curSt[1], float enviObj[5][4])
{
    float traject[2][TRAJITEM_LEN];

    // 通过BE距离判断是否需要重新规划路径
    float distBE = PK_PointToLineDist(enviObj[3], &enviObj[1][0]) - VEHICLE_LEN;
    if (distBE < post_min_trajlen)
    {
        return 1;
    }

    // 2.根据当前位置确定向前的路径
    if (curSt[0].AfterTra_nth > 0 && fabs(curSt[0].rx_brake) > post_swell * 0.5f &&
        ((!PathExec_IsVehStill() && !PathExec_IsObjDanger(curSt)) ||
         (PathExec_IsVehStill() && !PathExec_ObjDelayTimeOut(curSt))))
    {
        PRINT_SWITCH_INFO("PathExec_PathPostCal Keep Backup!");
        curSt[0].rx_brake =
            (curSt[0].leftDist - curSt[0].ds_gone_dir) * sign(curSt[0].rx_brake);
        return 0;
    }

    if (PathExec_IsVehStill())
    {
        if (curSt[0].AfterTra_nth > 0 && fabsf(curSt[0].rthetaf) < thetaGap_allow * 0.8 &&
            fabs(curSt[0].ry_cur_to_fin) < dfradioGap_allow * 0.8 &&
            curSt[0].rx_cur_to_fin > 1.2 * post_swell)
        {
            PRINT_SWITCH_INFO("PathExec_PathPostCal Continue Backup!");
            curSt[0].rx_brake = -curSt[0].rx_cur_to_fin;
            return 0;
        }

        // 1. 重新确定目标位置
        UpdateParaSlotByObj(curSt, enviObj);

        int pathNum = 0;
        if (sign(curSt[0].cvel) < 0.01f || curSt[0].AfterTra_nth == 0)
        {
            curSt[0].rtheta_cur_to_fin = Round_PI(curSt[0].rtheta_cur_to_fin);
            if (curSt[0].ry_cur_to_fin * curSt[0].rtheta_cur_to_fin > 0)
            {
                pathNum = PathExec_ParaPostTrajPre(curSt, enviObj, traject);
                PRINT_SWITCH_INFO("PathExec_PathPostCal Front 1!");
            }
            else
            {
                pathNum = PathExec_ParaPostTrajStep2(curSt, enviObj, traject);
                PRINT_SWITCH_INFO("PathExec_PathPostCal Front 2!");
            }
        }

        // 3.规划向后的路径
        else
        {
            pathNum = PathExec_ParaPostTrajStep3(curSt, enviObj, traject);
            PRINT_SWITCH_INFO("PathExec_PathPostCal Rear!");
        }

        if (pathNum > 0)
        {
            curSt[0].AfterTra_nth++;
            curSt[0].cvel     = sign(traject[0][3]) * 0.3;
            curSt[0].ceps     = PK_Rr_to_EPS(traject[0][4], curSt[0].cvel);
            curSt[0].rx_brake = traject[0][3];
            return 0;
        }
        else
        {
            return 2;
        }
    }

    return 0;
}

int PathExec_ParaPostTraj(PK_Cur_AutodrPara curSt[1], float cvel_direct_fac,
                          float enviObj[5][4])
{
    // const float para_tune_ds_max = 0.20f;
    const float after_tune_dist = 1.5f;
    const float after_tune_spd  = 0.32f; // m/s

    float after_tune_eps = rte_min(420.0f, PK_EPS_ANGLE_MAX - 60.0f);
    float cvelDirect     = cvel_direct_fac;
    if ((curSt[0].AfterTra_nth % 2) == 1 || curSt[0].AfterTra_nth == 0)
    {
        cvelDirect = -cvel_direct_fac;
    }

    bool newtraj = false;
    if (curSt[0].AfterTra_nth == 0)
    {
        // 1 先进入停止状态
        if (PathExec_IsVehStill())
        {
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj start!");
            newtraj = true;
        }
        else
        {
            curSt[0].cvel     = 0;
            curSt[0].rx_brake = 0;
            curSt[0].ceps     = 0;
            return 0;
        }

        PathExec_SetPostBrakeState(curSt);
    }
    else if (curSt[0].AfterTra_nth > 0 &&
             curSt[0].ds_gone_dir > 0.5 * curSt[0].leftDist && PathExec_IsVehStill() &&
             PathExec_SwitchDelayTimeOut(curSt))
    {
        // 调整中的停止状态
        PRINT_SWITCH_INFO("PathExec_ParaPostTraj temp stop!");
        newtraj = true;
    }
    else
    {
        // 调整中的运行状态
        curSt[0].cvel     = cvelDirect * after_tune_spd;
        curSt[0].rx_brake = curSt[0].leftDist - curSt[0].ds_gone_dir;

        // 如果调整到位, 则回正方向
        if (fabs(curSt[0].rthetaf) < 0.6 * thetaGap_allow)
        {
            curSt[0].ceps = 0.0f;
            if (PathExec_IsPostBrakeState(curSt))
            {
                if (curSt[0].brakeSt == 0)
                {
                    curSt[0].brakeSt = 1;
                }
            }
        }
    }

    if (newtraj && ((curSt[0].ds_gone_dir > 0.20f && curSt[0].AfterTra_nth > 0) ||
                    curSt[0].AfterTra_nth == 0))
    {
        curSt[0].ds_start_dir = curSt[0].curpos[3]; // update

        // 如果姿态调整到位, 则调整X方向
        if (!PathExec_IsObjDanger(curSt) &&
            fabs(curSt[0].rthetaf) < 0.8f * thetaGap_allow)
        {
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj last line!");
            curSt[0].cvel = -sign(curSt[0].rxf) * 0.2f;
            if (fabsf(curSt[0].rxf) > 3.0f * dfradioGap_allow)
            {
                // 防止下一次进入速度方向错误
                if (cvelDirect * curSt[0].cvel < ZERO_FLOAT)
                {
                    curSt[0].AfterTra_nth++;
                }
                curSt[0].ceps     = 0;
                curSt[0].leftDist = fabsf(curSt[0].rxf);
                curSt[0].rx_brake = curSt[0].leftDist;
                PathExec_ClrPostBrakeState(curSt);
                return 0;
            }
            else
            {
                return 1;
            }
        }

        curSt[0].AfterTra_nth++;
        if ((curSt[0].AfterTra_nth % 2) == 1)
        {
            cvelDirect = -cvel_direct_fac;
        }
        else
        {
            cvelDirect = cvel_direct_fac;
        }

        int flag = (cvelDirect > 0) ? 16 : 0;
        flag     = flag + 16;

        PathExec_PrintPathSwitch("PathExec_ParaPostTraj run", flag);
        curSt[0].cvel = cvelDirect * after_tune_spd;

        UpdateParaSlotByObj(curSt, enviObj);
        float traject[TRAJITEM_LEN];
        while (1)
        {
            curSt[0].ceps =
                -sign(cvelDirect * curSt[0].rthetaf) *
                after_tune_eps; // use a fixed eps angle but not the max eps angle
            if (cvelDirect > 0.001f)
            {
                curSt[0].leftDist =
                    0.55f * curSt[0].rthetaf /
                    PK_EPS_to_Rou(after_tune_eps, cvelDirect * after_tune_spd);
            }
            else
            {
                curSt[0].leftDist =
                    curSt[0].rthetaf /
                    PK_EPS_to_Rou(after_tune_eps, cvelDirect * after_tune_spd);
            }

            curSt[0].leftDist = rte_min(fabs(curSt[0].leftDist), after_tune_dist);

            // thetaGap_allow * r = 0.03 * 6 = 0.18;
            if (curSt[0].leftDist < 0.5f && after_tune_eps > 150)
            {
                after_tune_eps = after_tune_eps - 50;
                continue;
            }

            PK_CopyPos(traject, curSt[0].curpos);
            traject[3] = curSt[0].leftDist * cvelDirect;
            traject[4] = 1.0 / PK_EPS_to_Rou(curSt[0].ceps, curSt[0].cvel);

            if (cvelDirect > 0.001f)
            {
                PK_PathVerify(traject, 1, &enviObj[3], post_swell);
            }
            else
            {
                PK_PathVerify(traject, 1, &enviObj[1], post_swell);
            }

            PathExec_PrintPathInfo(&traject, 1);
            curSt[0].ceps = PK_Rr_to_EPS(
                traject[4],
                curSt[0].cvel); // use a fixed eps angle but not the max eps angle
            curSt[0].leftDist = fabs(traject[3]);
            break;
        }

        curSt[0].rx_brake = curSt[0].leftDist;
        if (fabs(curSt[0].leftDist) < 0.20)
        {
            return 1;
        }
    }
    return 0;
}

int PathExec_ParaPostParkOutTraj(PK_Cur_AutodrPara curSt[1], float cvel_direct_fac,
                                 float enviObj[5][4])
{
    const float after_tune_dist = 1.5f;
    const float after_tune_spd  = 0.32f; // m/s

    float after_tune_eps = rte_min(450.0f, PK_EPS_ANGLE_MAX - 20.0f);
    bool newtraj         = false;

    // 1 初始化先进入停止状态
    if (curSt[0].AfterTra_nth == 0)
    {
        if (PathExec_IsVehStill())
        {
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj start!");
            newtraj = true;
        }
        else
        {
            curSt[0].cvel     = 0;
            curSt[0].rx_brake = 0;
            curSt[0].ceps     = 0;
            return 0;
        }

        curSt[0].cvel_direct_fac = -1.0f;
        PathExec_SetPostBrakeState(curSt);
    }
    // 2 当前路径已经走完
    else if (curSt[0].AfterTra_nth > 0 &&
             (curSt[0].ds_gone_dir > 0.5 * curSt[0].leftDist ||
              curSt[0].ds_gone_dir > 0.10f) &&
             PathExec_IsVehStill() && PathExec_SwitchDelayTimeOut(curSt))
    {
        PRINT_SWITCH_INFO("PathExec_ParaPostTraj temp pause!");
        newtraj                  = true;
        curSt[0].cvel_direct_fac = -1.0f * curSt[0].cvel_direct_fac;
    }
    // 3 当前路径运行中
    else
    {
        curSt[0].cvel     = curSt[0].cvel_direct_fac * after_tune_spd;
        curSt[0].rx_brake = curSt[0].leftDist - curSt[0].ds_gone_dir;

        if (fabs(curSt[0].rtheta_cur_to_fin) < PI * 0.1f &&
            fabs(curSt[0].rx_cur_to_fin) < 0.5f * VEHICLE_LEN)
        {
            curSt[0].ceps = 0.0f;
            if (PathExec_IsPostBrakeState(curSt))
            {
                if (curSt[0].brakeSt == 0)
                {
                    curSt[0].brakeSt = 1;
                }
            }
        }
        return 0;
    }

    // 需要重新规划路径
    if (newtraj)
    {
        curSt[0].ds_start_dir = curSt[0].curpos[3];

        float traject[TRAJITEM_LEN];
        PK_CopyPos(traject, curSt[0].curpos);
        curSt[0].AfterTra_nth++;

        // 当前已经没有障碍物, 且离开原地有一段距离了
        if (!PathExec_IsObjDanger(curSt) &&
            fabs(curSt[0].curpos[1]) > 0.35 * VEHICLE_WID &&
            fabs(curSt[0].curpos[2]) > PI / 9.0F)
        {
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj last line!");
            curSt[0].cvel_direct_fac = 1.0;
            curSt[0].cvel            = 0.2f * curSt[0].cvel_direct_fac;
            curSt[0].leftDist =
                curSt[0].rtheta_cur_to_fin /
                PK_EPS_to_Rou(after_tune_eps, curSt[0].cvel_direct_fac * after_tune_spd);
            curSt[0].ceps = -sign(curSt[0].rtheta_cur_to_fin) * after_tune_eps;
            traject[3]    = fabs(curSt[0].leftDist) + 0.05f;
            traject[4]    = 1.0 / PK_EPS_to_Rou(curSt[0].ceps, curSt[0].cvel);

            // 校验通过
            PK_PathVerify(traject, 1, &enviObj[3], post_swell);
            if (fabs(traject[3]) > curSt[0].leftDist)
            {
                PathExec_ClrPostBrakeState(curSt);
                PathExec_PrintPathInfo(&traject, 1);
                curSt[0].AfterTra_nth = CONFIG_POSREPLAN_NUM;
                return 0;
            }
        }

        float sideObjs[4][4];
        int sideObjNum =
            PathExec_BuildSideRadarAlarmObj(curSt[0].curpos, curSt[0].finpos, sideObjs);

        // 规划向后的路径
        if ((curSt[0].cvel_direct_fac < ZERO_FLOAT && !PathExec_IsRearDanger(curSt)) ||
            PathExec_IsFrontDanger(curSt))
        {
            // 向后且后面没有障碍物
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj  backward!");
            curSt[0].cvel_direct_fac = -1.0;
            curSt[0].cvel            = 0.2f * curSt[0].cvel_direct_fac;
            // 小于30DEG, 转420DEG
            if (fabs(curSt[0].rtheta_cur_to_fin) < PI / 6.0F) // 30DEG
            {
                curSt[0].ceps = curSt[0].cvel_direct_fac *
                                sign(curSt[0].rtheta_cur_to_fin) * after_tune_eps;
            }
            else
            {
                curSt[0].ceps = curSt[0].cvel_direct_fac *
                                sign(curSt[0].rtheta_cur_to_fin) * after_tune_eps * 0.5f;
            }

            curSt[0].leftDist = 1.0f;
            traject[3]        = curSt[0].leftDist * curSt[0].cvel_direct_fac;

            while (1)
            {
                traject[4] = 1.0 / PK_EPS_to_Rou(curSt[0].ceps, curSt[0].cvel);
                PK_PathVerify(traject, 2, &enviObj[1], post_swell);
                float trajEnd[3];
                PK_Get_Path_EndPos(traject, trajEnd);

                // 终点角度大于55, 减少距离或者角度
                if (fabs(trajEnd[2]) > PI / 3.2f)
                {
                    if (fabs(trajEnd[3]) > 0.5f)
                    {
                        traject[3] = (fabs(traject[3]) - 0.10f) * sign(traject[3]);
                    }
                    else if (fabs(curSt[0].ceps) > 120.0f)
                    {
                        curSt[0].ceps = (fabs(curSt[0].ceps) - 20) * sign(curSt[0].ceps);
                    }
                }
                break;
            }

            PathExec_PrintPathInfo(&traject, 1);

            curSt[0].leftDist = fabs(traject[3]);
            curSt[0].leftDist = rte_max(curSt[0].leftDist, 0.30f);
            curSt[0].rx_brake = curSt[0].leftDist;
            return 0;
        }

        // 规划向前的路径
        if ((curSt[0].cvel_direct_fac > ZERO_FLOAT && !PathExec_IsFrontDanger(curSt)) ||
            PathExec_IsRearDanger(curSt))
        {
            PRINT_SWITCH_INFO("PathExec_ParaPostTraj forward!");
            curSt[0].cvel_direct_fac = 1.0;
            curSt[0].cvel            = 0.2f * curSt[0].cvel_direct_fac;

            if (fabs(curSt[0].rtheta_cur_to_fin) < PI / 12.0F) // 15DEG
            {
                curSt[0].ceps = curSt[0].cvel_direct_fac *
                                sign(curSt[0].rtheta_cur_to_fin) * after_tune_eps;
                traject[4] = 1.0 / PK_EPS_to_Rou(curSt[0].ceps, curSt[0].cvel);
            }
            else if (fabs(curSt[0].rtheta_cur_to_fin) < PI / 8.0F) // 3DEG
            {
                curSt[0].ceps = curSt[0].cvel_direct_fac *
                                sign(curSt[0].rtheta_cur_to_fin) * after_tune_eps * 0.5f;
                traject[4] = 1.0 / PK_EPS_to_Rou(curSt[0].ceps, curSt[0].cvel);
            }
            else
            {
                curSt[0].ceps = 0.0;
                traject[4]    = 0.0f;
            }

            traject[3] = after_tune_dist;

            float disty2 = fabs(enviObj[4][1]) + VEHICLE_WID * 0.5 + 0.8;
            while (1)
            {
                float lastLen = fabs(traject[3]);
                PK_PathVerify(traject, 1, &enviObj[3], post_swell);
                PK_PathVerify(traject, sideObjNum, sideObjs, post_swell);

                // 仍然会碰到障碍物, 则使用当前长度
                if (fabs(traject[3]) < lastLen - 0.01f)
                {
                    break;
                }

                // 不会碰到障碍物, 防止路径过长
                float trajEnd[3];
                PK_Get_Path_EndPos(traject, trajEnd);

                float disty = fabs(trajEnd[1]) +
                              (1.0 - cosf(fabs(curSt[0].rtheta_cur_to_fin))) * 7.5f;
                if ((disty > disty2 && fabs(trajEnd[1]) > 0.5 * VEHICLE_WID) ||
                    fabs(traject[3]) > 3.0f * after_tune_dist)
                {
                    break;
                }
                traject[3] = traject[3] + 0.3f;
            }

            PathExec_PrintPathInfo(&traject, 1);

            curSt[0].leftDist = fabs(traject[3]);
            curSt[0].leftDist = rte_max(curSt[0].leftDist, 0.30f);
            curSt[0].rx_brake = curSt[0].leftDist;
            return 0;
        }
    }

    return 1;
}
