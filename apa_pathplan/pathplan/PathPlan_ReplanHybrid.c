#include "PK_PathPlanP.h"
#include "PathPlan_Tools.h"
#include "json/json.h"
#include "Pathplan_Hybrid.h"
#include <fstream>

static bool IsSlotContained(const float slot[5][4], const float line[4])
{
    Quad_T quad;
    memcpy(&quad.pt_fl, &slot[3][2], sizeof(Point_T));
    memcpy(&quad.pt_fr, &slot[3][0], sizeof(Point_T));
    memcpy(&quad.pt_rr, &slot[1][2], sizeof(Point_T));
    memcpy(&quad.pt_rl, &slot[1][0], sizeof(Point_T));

    Point_T points[2];
    memcpy((float *)&points[0], &line[0], sizeof(Point_T));
    memcpy((float *)&points[1], &line[2], sizeof(Point_T));

    if (Is_Point_In_Quad(quad, points[0]) == PK_OUTSIDE ||
        Is_Point_In_Quad(quad, points[1]) == PK_OUTSIDE)
    {
        return false;
    }
    return true;
}

static void CheckObjsValid(const PlanDataCase planData[1])
{
    for (int i = 0; i < planData[0].obj_danger_num; i++)
    {
        int j = 0;
        while (j < 5)
        {
            if (Calc_Dist_Line2Line((float *)&planData[0].obj_danger[i][0],
                                    (float *)&planData[0].obj_danger[i][2],
                                    (float *)&planData[0].obj_slot[j][0],
                                    (float *)&planData[0].obj_slot[j][2]) < 0.10f)
            {
                break;
            }
            j++;
        }

        while (IsSlotContained(planData[0].obj_slot, planData[0].obj_danger[i]))
        {
            printf("line :%f %f %f %f attr: %d\n", planData[0].obj_danger[i][0],
                   planData[0].obj_danger[i][1], planData[0].obj_danger[i][2],
                   planData[0].obj_danger[i][3], planData[0].obj_danger_dir[i]);
        }
    }
}

static float FindFrontObjs(const PlanDataCase planData[1], const float endPos[3])
{
    float result = -0.1;
    float rsPoint[2][3];
    for (int i = 0; i < planData[0].obj_danger_num; i++)
    {
        RevConvert(planData[0].stpoint, &planData[0].obj_danger[i][0], rsPoint[0]);
        RevConvert(planData[0].stpoint, &planData[0].obj_danger[i][2], rsPoint[1]);

        if (rsPoint[0][0] < VEHICLE_LEN - REAR_SUSPENSION ||
            rsPoint[1][0] < VEHICLE_LEN - REAR_SUSPENSION)
        {
            continue;
        }

        if (rsPoint[0][1] * rsPoint[1][1] > ZERO_FLOAT)
        {
            continue;
        }

        if (fabs(rsPoint[0][1]) < 0.4f || fabs(rsPoint[1][1]) < 0.4f)
        {
            continue;
        }

        float rx = Project_PosTo1st_rx(endPos, &planData[0].obj_danger[i][0]);
        result   = rte_max(rx, result);

        rx     = Project_PosTo1st_rx(endPos, &planData[0].obj_danger[i][2]);
        result = rte_max(rx, result);
    }

    return result;
}

static int BuildHybridObjs(const PlanDataCase planData[1], const float endPos[3],
                           Json::Value &root)
{
    const float hybrid_swell = 0.15f;
    int objidx               = 0;

    //  B/C X值
    float brx = Project_PosTo1st_rx(endPos, &planData[0].obj_slot[2][0]);

    // 车辆自身位置的最小值
    float vrx = Project_PosTo1st_rx(endPos, planData[0].stpoint);

    // 确定X方向最小值minrx
    float minstripx = brx - hybrid_swell - 0.05f;
    float temp      = vrx - REAR_SUSPENSION - 0.5f;
    minstripx       = rte_min(temp, minstripx);

    // 确定X方向最大值maxrx
    //  B/C X值
    temp            = vrx + VEHICLE_LEN - REAR_SUSPENSION + 0.5f;
    float maxstripx = rte_max(temp, 2.2 * VEHICLE_LEN);
    float frontrx   = FindFrontObjs(planData, endPos);
    if (frontrx > 0.0f)
    {
        maxstripx = rte_min(frontrx, maxstripx);
    }

    float erx = Project_PosTo1st_rx(endPos, &planData[0].obj_slot[4][0]);
    erx       = erx + hybrid_swell + 0.20f;
    maxstripx = rte_max(erx, maxstripx);

    if (fabs(maxstripx - minstripx) < 2.5 * VEHICLE_LEN)
    {
        minstripx = maxstripx - 2.5 * VEHICLE_LEN;
    }

    // Y方向最大值
    float maxstripy = Project_PosTo1st_ry(endPos, planData[0].stpoint);
    maxstripy       = (fabs(maxstripy) + VEHICLE_WID * 0.5 + 0.6f) * sign(maxstripy);
    if (planData[0].slotshape < PK_SLOT_LEFT_VERT)
    {
        maxstripy = 1.8 * VEHICLE_LEN * sign(maxstripy);
    }
    else
    {
        maxstripy = 2.5 * VEHICLE_LEN * sign(maxstripy);
    }

    float rsPoint[2][3];
    for (int i = 0; i < planData[0].obj_danger_num; i++)
    {
        RevConvert(endPos, &planData[0].obj_danger[i][0], rsPoint[0]);
        RevConvert(endPos, &planData[0].obj_danger[i][2], rsPoint[1]);

        // 平行车位忽略掉B点以后的
        float lineLen = Cal_Dis_Pt2Pt(rsPoint[0], rsPoint[1]);
        if (lineLen < ZERO_FLOAT)
        {
            continue;
        }

        PathPlan_ClipLineSegmentx(rsPoint[0], rsPoint[1], minstripx, -1.0);
        lineLen = Cal_Dis_Pt2Pt(rsPoint[0], rsPoint[1]);
        if (lineLen < ZERO_FLOAT)
        {
            continue;
        }

        PathPlan_ClipLineSegmentx(rsPoint[0], rsPoint[1], maxstripx, 1.0f);
        lineLen = Cal_Dis_Pt2Pt(rsPoint[0], rsPoint[1]);
        if (lineLen < ZERO_FLOAT)
        {
            continue;
        }

        PathPlan_ClipLineSegmenty(rsPoint[0], rsPoint[1], maxstripy,
                                  1.0f * sign(maxstripy));
        lineLen = Cal_Dis_Pt2Pt(rsPoint[0], rsPoint[1]);
        if (lineLen < ZERO_FLOAT)
        {
            continue;
        }

        if (fpclassify(rsPoint[0][0]) == FP_NAN ||
            fpclassify(rsPoint[0][0]) == FP_INFINITE ||
            fpclassify(rsPoint[0][1]) == FP_NAN ||
            fpclassify(rsPoint[0][1]) == FP_INFINITE ||
            fpclassify(rsPoint[1][0]) == FP_NAN ||
            fpclassify(rsPoint[1][0]) == FP_INFINITE ||
            fpclassify(rsPoint[1][1]) == FP_NAN ||
            fpclassify(rsPoint[1][1]) == FP_INFINITE)
        {
            printf("-----------------------");
            continue;
        }

        root["objs"][objidx].append((float)rsPoint[0][0]);
        root["objs"][objidx].append((float)rsPoint[0][1]);
        root["objs"][objidx].append((float)rsPoint[1][0]);
        root["objs"][objidx].append((float)rsPoint[1][1]);
        objidx++;
    }

    float objPoint[6][2];
    RevConvert(endPos, &planData[0].obj_slot[0][0], objPoint[0]);
    RevConvert(endPos, &planData[0].obj_slot[0][2], objPoint[1]);
    RevConvert(endPos, &planData[0].obj_slot[2][0], objPoint[2]);
    RevConvert(endPos, &planData[0].obj_slot[2][2], objPoint[3]);
    RevConvert(endPos, &planData[0].obj_slot[4][0], objPoint[4]);
    RevConvert(endPos, &planData[0].obj_slot[4][2], objPoint[5]);

    int objstart   = 0;
    objPoint[0][0] = (fabs(objPoint[0][0]) + hybrid_swell) * sign(objPoint[0][0]); // A
    objPoint[0][1] = (fabs(objPoint[0][1]) - hybrid_swell) * sign(objPoint[0][1]);
    objPoint[1][0] = (fabs(objPoint[1][0]) + hybrid_swell) * sign(objPoint[1][0]); // B
    objPoint[1][1] = (fabs(objPoint[1][1]) - hybrid_swell) * sign(objPoint[1][1]);

    objPoint[2][0] = (fabs(objPoint[2][0]) + hybrid_swell) * sign(objPoint[2][0]); // C
    objPoint[3][0] = (fabs(objPoint[3][0]) + hybrid_swell) * sign(objPoint[3][0]); // D

    objPoint[4][0] = (fabs(objPoint[4][0]) + hybrid_swell) * sign(objPoint[4][0]); // E
    objPoint[4][1] = (fabs(objPoint[4][1]) - hybrid_swell) * sign(objPoint[4][1]);
    objPoint[5][0] = (fabs(objPoint[5][0]) + hybrid_swell) * sign(objPoint[5][0]); // F
    objPoint[5][1] = (fabs(objPoint[5][1]) - hybrid_swell) * sign(objPoint[5][1]);

    while (objstart < 5)
    {
        memcpy(rsPoint[0], objPoint[objstart + 0], sizeof(float) * 2);
        memcpy(rsPoint[1], objPoint[objstart + 1], sizeof(float) * 2);
        objstart = objstart + 1;

        PathPlan_ClipLineSegmentx(rsPoint[0], rsPoint[1], minstripx, -1.0);
        PathPlan_ClipLineSegmentx(rsPoint[0], rsPoint[1], maxstripx, 1.0f);

        float lineLen = Cal_Dis_Pt2Pt(rsPoint[0], rsPoint[1]);
        if (lineLen < ZERO_FLOAT)
        {
            continue;
        }

        root["objs"][objidx].append((float)rsPoint[0][0]);
        root["objs"][objidx].append((float)rsPoint[0][1]);
        root["objs"][objidx].append((float)rsPoint[1][0]);
        root["objs"][objidx].append((float)rsPoint[1][1]);
        objidx++;
    }

    root["objs"][objidx].append((float)minstripx);
    root["objs"][objidx].append((float)0.0f);
    root["objs"][objidx].append((float)minstripx);
    root["objs"][objidx].append(0.4f);
    objidx++;

    root["objs"][objidx].append((float)(maxstripx - 0.4f));
    root["objs"][objidx].append((float)maxstripy);
    root["objs"][objidx].append((float)maxstripx);
    root["objs"][objidx].append((float)maxstripy);
    objidx++;

    return objidx;
}

int PathPlan_HybridReplan(const PlanDataCase planData[1],
                          float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], int pathNum,
                          int slotindex)
{
    if (RTE_PK_StateManage_Get_ModuleCom_PathPlan() != MCOM_ON)
    {
        return 0;
    }

    Json::Value root;

    float endPos[3];
    PK_CopyPos(endPos, planData[0].finpoint);
    if (planData[0].slotshape >= PK_SLOT_LEFT_VERT)
    {
        float dir = (Project_PosTo1st_ry(planData[0].stpoint, endPos) > ZERO_FLOAT)
                        ? 1.0f
                        : -1.0f;
        endPos[2] = Round_PI(endPos[2] + PI * 0.5f * dir);
    }

    float rsPoint[3];
    std::vector<Traject_Node> leftTrajs;
    for (int i = 0; i < pathNum; i++)
    {
        Traject_Node trajItem;
        PK_CopyTra((float *)&trajItem, trajs[i]);
        leftTrajs.push_back(trajItem);

        CoordinadteTransfer(endPos, trajs[i], rsPoint);
        root["trajs"][i].append((float)rsPoint[0]);
        root["trajs"][i].append((float)rsPoint[1]);
        root["trajs"][i].append((float)rsPoint[2]);
        root["trajs"][i].append((float)trajs[i][3]);
        root["trajs"][i].append((float)trajs[i][4]);
    }

    // 生成障碍物
    BuildHybridObjs(planData, endPos, root);

    CoordinadteTransfer(endPos, planData[0].stpoint, rsPoint);
    root["curpos"].append((float)rsPoint[0]);
    root["curpos"].append((float)rsPoint[1]);
    root["curpos"].append((float)rsPoint[2]);
    root["curpos"].append((float)planData[0].stpoint[3]);

    float pathEnd[3];
    PK_Get_Path_EndPos(trajs[pathNum - 1], pathEnd);
    CoordinadteTransfer(endPos, pathEnd, rsPoint);
    root["slottarg"].append((float)rsPoint[0]);
    root["slottarg"].append((float)rsPoint[1]);
    root["slottarg"].append((float)rsPoint[2]);
    root["slotId"] = slotindex;

    std::string strtemp = root.toStyledString();
    if (HybridAStarRun(strtemp.c_str(), leftTrajs, slotindex) != 0)
    {
        return 0x101;
    }

    return 0;
}

static int MergeTrajs1(const PlanDataCase planData[0], float trajs[][TRAJITEM_LEN],
                       int trajNum, int &start)
{
    start       = 0;
    int pathNum = 0;
    int end     = trajNum - 1;
    float lastTrajs[10][TRAJITEM_LEN];
    while (start < end - 2)
    {
        pathNum = PK_PathPlan_BetterTra(trajs[start], trajs[end], 5, planData[0].obj_slot,
                                        lastTrajs, 0.0, 0.3);
        if (pathNum > 0 && pathNum < end - start)
        {
            auto check =
                PK_Check_Path(pathNum, lastTrajs, trajs[start], trajs[end],
                              planData[0].obj_danger_num, planData[0].obj_danger, 0.0);
            if (check == PATHPLAN_SUCCESS)
            {
                break;
            }
        }

        pathNum = PathPlan_TwoCircleCon(trajs[start], trajs[end], 5, planData[0].obj_slot,
                                        lastTrajs, 0.0);
        if (pathNum > 0 && pathNum < end - start)
        {
            auto check =
                PK_Check_Path(pathNum, lastTrajs, trajs[start], trajs[end],
                              planData[0].obj_danger_num, planData[0].obj_danger, 0.0);
            if (check == PATHPLAN_SUCCESS)
            {
                break;
            }
        }

        start++;
    }

    if (pathNum > 0 && pathNum < trajNum - start)
    {
        memcpy(trajs[start], lastTrajs, PATH_ITEM_LEN * pathNum);
        trajNum = start + pathNum;
        start   = start;
    }

    return trajNum;
}

static int MergeTrajs2(const PlanDataCase planData[0], float trajs[][TRAJITEM_LEN],
                       int trajNum, int &start)
{
    start       = 0;
    int pathNum = 0;
    int end     = trajNum - 1;
    float lastTrajs[10][TRAJITEM_LEN];
    while (start < end - 3)
    {
        float startTraj[5];
        PK_CopyTra(startTraj, trajs[start]);
        startTraj[3] = (fabs(startTraj[3]) + 0.8f) * sign(startTraj[3]);
        PathPlan_LandMark(startTraj, planData[0].obj_danger_num, planData[0].obj_danger,
                          0);
        PathPlan_LandMark(startTraj, 5, planData[0].obj_slot, 0);

        pathNum = PK_PathPlan_TraExplorer_Con(
            startTraj, trajs[end], 5, planData[0].obj_slot, lastTrajs, 0, 1, 0, 0.0);
        if (pathNum > 0 && pathNum < end - start)
        {
            auto check =
                PK_Check_Path(pathNum, lastTrajs, startTraj, trajs[end],
                              planData[0].obj_danger_num, planData[0].obj_danger, 0.0);
            if (check == PATHPLAN_SUCCESS)
            {
                break;
            }
        }

        start++;
    }

    if (pathNum > 0 && pathNum < trajNum - start)
    {
        memcpy(trajs[start], lastTrajs, PATH_ITEM_LEN * pathNum);
        trajNum = start + pathNum;
        start   = start;
    }

    return trajNum;
}

static int MergeTrajs3(const PlanDataCase planData[0], float trajs[][TRAJITEM_LEN],
                       int trajNum, int &start)
{
    start       = 0;
    int pathNum = 0;
    int end     = trajNum - 1;
    float lastTrajs[10][TRAJITEM_LEN];
    while (end - start > 3)
    {
        float startPos[3];
        PK_Get_Path_EndPos(trajs[start], startPos);
        pathNum = PK_PathPlan_BetterTra(startPos, trajs[end], 5, planData[0].obj_slot,
                                        lastTrajs, 0.0, 0.3);
        if (pathNum > 0)
        {
            auto check =
                PK_Check_Path(pathNum, lastTrajs, trajs[start], trajs[end],
                              planData[0].obj_danger_num, planData[0].obj_danger, 0.0);
            if (check == PATHPLAN_SUCCESS)
            {
                break;
            }
        }

        pathNum = PathPlan_TwoCircleCon(trajs[start], trajs[end], 5, planData[0].obj_slot,
                                        lastTrajs, 0.0);
        if (pathNum > 0 && pathNum < end - start - 1)
        {
            auto check =
                PK_Check_Path(pathNum, lastTrajs, trajs[start], trajs[end],
                              planData[0].obj_danger_num, planData[0].obj_danger, 0.0);
            if (check == PATHPLAN_SUCCESS)
            {
                break;
            }
        }

        start++;
    }

    if (pathNum > 0 && pathNum < trajNum - start - 1)
    {
        memcpy(trajs[start + 1], lastTrajs, PATH_ITEM_LEN * pathNum);
        trajNum = start + pathNum + 1;
        start   = start;
    }

    return trajNum;
}

int MergeHybridTrajs(const PlanDataCase planData[0], float trajs[][TRAJITEM_LEN],
                     int trajNum)
{
    int end       = trajNum;
    int resultNum = trajNum;
    while (end > 2 && resultNum > 15)
    {
        int start   = 0;
        int pathNum = MergeTrajs1(
            planData, &trajs[0], end,
            start); // start 表示起点即第一条，end 表示条数 连接start和最后一条的起点
        if (pathNum < end) // 新条数小于老条数
        {
            memcpy(trajs[pathNum], trajs[end - 1],
                   PATH_ITEM_LEN *
                       (resultNum - (end - 1))); // 因此最后一条后面的还需要保留
            resultNum = pathNum + resultNum - (end - 1);
            end       = start + 1; // 新的条数
            printf("MergeTrajs1 merge\n");
        }
        else
        {
            end = end - 1;
        }
    }

    end = resultNum;
    while (end > 3 && resultNum > 15)
    {
        int start   = 0;
        int pathNum = MergeTrajs2(
            planData, &trajs[0], end,
            start); // start 表示起点即第一条，end 表示条数 连接start和最后一条的起点
        if (pathNum < end) // 新条数小于老条数
        {
            memcpy(trajs[pathNum], trajs[end - 1],
                   PATH_ITEM_LEN *
                       (resultNum - (end - 1))); // 因此最后一条后面的还需要保留
            resultNum = pathNum + resultNum - (end - 1);
            end       = start + 1; // 新的条数
            printf("MergeTrajs2 merge\n");
        }
        else
        {
            end = end - 1;
        }
    }

    end = resultNum;
    while (end > 3 && resultNum > 15)
    {
        int start   = 0;
        int pathNum = MergeTrajs3(
            planData, &trajs[0], end,
            start); // start 表示起点即第一条末端，end 表示条数 连接start和最后一条的起点
        if (pathNum < end) // 新条数小于老条数
        {
            memcpy(trajs[pathNum], trajs[end - 1],
                   PATH_ITEM_LEN *
                       (resultNum - (end - 1))); // 因此最后一条后面的还需要保留
            resultNum = pathNum + resultNum - (end - 1);
            end       = start + 1; // 新的条数
            printf("MergeTrajs3 merge\n");
        }
        else
        {
            end = end - 1;
        }
    }

    return resultNum;
}

int ConnectEndTrajs(PlanDataCase &planData, float trajs[][TRAJITEM_LEN], int trajNum,
                    int maxtrajNum)
{
    float Exp_traj[20][TRAJITEM_LEN];
    int IsDirConnect = 0;

    for (int i = 0; i < trajNum - 1 && trajNum > 10; i++)
    {
        memcpy(planData.stpoint, trajs[i], sizeof(planData.stpoint));
        int pathNum = PathPlan_SpaceExplorer_Vert(&planData, Exp_traj, &IsDirConnect, 1);
        if (pathNum > 0)
        {
            printf("connect success %d old:%d newNum:%d\n", i, trajNum, pathNum);
            memcpy(trajs[i], Exp_traj[0], pathNum * sizeof(Exp_traj[0]));
            return i + pathNum;
        }
    }
    return trajNum;
}

int ConnectTrajs(PlanDataCase &planData, float trajs[][TRAJITEM_LEN], int trajNum,
                 int maxTrajNum)
{
    // 连接断头轨迹
    float Exp_traj[20][TRAJITEM_LEN];
    int IsDirConnect = 0;
    PathPlan_UpdateTrajsLen(trajs, trajNum);

    for (int i = 0; i < trajNum - 1; i++)
    {
        // 根据障碍物裁剪
        float tempTraj[TRAJITEM_LEN];
        PK_CopyTra(tempTraj, trajs[i]);
        PathPlan_LandMark(tempTraj, planData.obj_danger_num, planData.obj_danger, 0);
        PathPlan_LandMark(tempTraj, 5, planData.obj_slot, 0);

        // 反向裁剪
        if (fabs(tempTraj[3]) < 0.01f)
        {
            PK_CopyTra(tempTraj, trajs[i]);
        }

        PathPlan_ReverseTraject(tempTraj);
        PathPlan_LandMark(tempTraj, planData.obj_danger_num, planData.obj_danger, 0);
        PathPlan_LandMark(tempTraj, 5, planData.obj_slot, 0);
        PathPlan_ReverseTraject(tempTraj);
        PK_CopyTra(trajs[i], tempTraj);
    }

    for (int i = trajNum - 1; i > 0; i--)
    {
        float endPos[3];
        PK_Get_Path_EndPos(trajs[i - 1], endPos);
        auto check3 = PathPlan_CheckPosDiv(endPos, trajs[i]);
        while (check3 != 0)
        {
            memcpy(planData.stpoint, endPos, sizeof(endPos));
            memcpy(planData.finpoint, trajs[i], sizeof(PlanDataCase::finpoint));
            int pathNum = PathPlan_TwoCircleCon(planData.stpoint, planData.finpoint, 0,
                                                planData.obj_slot, Exp_traj, 0.0);
            if (pathNum <= 0)
            {
                pathNum =
                    PathPlan_SpaceExplorer_Vert(&planData, Exp_traj, &IsDirConnect, 1);
            }

            if (pathNum > 0)
            {
                if (PK_Check_Path(pathNum, Exp_traj, planData.stpoint, planData.finpoint,
                                  planData.obj_danger_num, planData.obj_danger, 0.0) == 0)
                {
                    printf("breaked traj: %d, id: %d 1 success number: %d\n", check3, i,
                           pathNum);
                    if (trajNum + pathNum < maxTrajNum)
                    {
                        memmove(trajs[i + pathNum], trajs[i],
                                PATH_ITEM_LEN * (trajNum - i));
                        memcpy(trajs[i], Exp_traj[0], pathNum * PATH_ITEM_LEN);
                        trajNum = trajNum + pathNum;
                        break;
                    }
                    else
                    {
                        printf("traj too much: %d, %d\n", trajNum, trajNum + pathNum);
                    }
                }
            }

            memcpy(planData.stpoint, trajs[i - 1], sizeof(PlanDataCase::stpoint));
            pathNum = PathPlan_SpaceExplorer_Vert(&planData, Exp_traj, &IsDirConnect, 1);
            if (pathNum > 0)
            {
                if (PK_Check_Path(pathNum, Exp_traj, planData.stpoint, planData.finpoint,
                                  planData.obj_danger_num, planData.obj_danger, 0.0) == 0)
                {
                    printf("breaked traj: %d, id: %d 3 success number: %d\n", check3, i,
                           pathNum);
                    if (trajNum + pathNum - 1 < maxTrajNum)
                    {
                        memmove(trajs[i - 1 + pathNum], trajs[i],
                                PATH_ITEM_LEN * (trajNum - i));
                        memcpy(trajs[i - 1], Exp_traj[0], pathNum * PATH_ITEM_LEN);
                        trajNum = trajNum + pathNum - 1;
                        break;
                    }
                    else
                    {
                        printf("traj too much: %d, %d\n", trajNum, trajNum + pathNum);
                    }
                }
            }

            if (check3)
            {
                printf("breaked traj: %d, id: %d failed\n", check3, i);
            }
        }
    }

    return trajNum;
}

int PathPlan_HybridReplanResult(const int slot, const PlanDataCase planData[0],
                                float PlannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN])
{
    const int max_traj_num = 200;
    std::vector<Traject_Node> leftTrajs;
    std::vector<Traject_Node> newTrajs;

    HybridAStarResult(slot, planData[0].finpoint, newTrajs, leftTrajs);
    if (newTrajs.size() > max_traj_num || newTrajs.size() <= 0)
    {
        return 0;
    }

    // 预留出原始路径
    float tempTrajs[max_traj_num][TRAJITEM_LEN];
    int trajNum = newTrajs.size();
    for (int i = 0; i < trajNum; i++)
    {
        PK_CopyTra(tempTrajs[i], newTrajs[i].traj);
    }

    trajNum = PathPlan_PathMerge(tempTrajs, trajNum);

    // 写入文件
    std::ofstream outfile3("out3.txt");
    for (int i = 0; i < trajNum; i++)
    {
        outfile3 << tempTrajs[i][0] << " ";
        outfile3 << tempTrajs[i][1] << " ";
        outfile3 << tempTrajs[i][2] << " ";
        outfile3 << tempTrajs[i][3] << " ";
        outfile3 << tempTrajs[i][4] << '\n';
    }
    outfile3.close();

    // 转换为原始坐标系
    float endPos[3];
    PK_CopyPos(endPos, planData[0].finpoint);
    if (planData[0].slotshape >= PK_SLOT_LEFT_VERT)
    {
        float dir = (Project_PosTo1st_ry(planData[0].stpoint, endPos) > ZERO_FLOAT)
                        ? 1.0f
                        : -1.0f;
        endPos[2] = Round_PI(endPos[2] + PI * 0.5f * dir);
    }

    // 根据障碍物重新更新路径长度
    std::ofstream outfile6("out6.txt");
    PathPlan_UpdateTrajsLen(tempTrajs, trajNum);
    for (int i = 0; i < trajNum; i++)
    {
        float rsPoint[3], traject[TRAJITEM_LEN];
        Convert(endPos, tempTrajs[i], rsPoint);
        rsPoint[2] = Round_PI(endPos[2] + tempTrajs[i][2]);
        PK_CopyPos(tempTrajs[i], rsPoint);
        float dst1 = tempTrajs[i][3];

        PK_CopyTra(traject, tempTrajs[i]);
        {
            PathPlan_LandMark(traject, planData[0].obj_danger_num, planData[0].obj_danger,
                              0);
            PathPlan_LandMark(traject, 5, planData[0].obj_slot, 0);
        }

        float dst2 = tempTrajs[i][3];
        {
            if (fabs(traject[3]) < 0.01f)
            {
                PK_CopyTra(traject, tempTrajs[i]);
            }
            PathPlan_ReverseTraject(traject);
            PathPlan_LandMark(traject, planData[0].obj_danger_num, planData[0].obj_danger,
                              0);
            PathPlan_LandMark(traject, 5, planData[0].obj_slot, 0);
            PathPlan_ReverseTraject(traject);
        }

        PK_CopyTra(tempTrajs[i], traject);
        outfile6 << tempTrajs[i][0] << ",";
        outfile6 << tempTrajs[i][1] << ",";
        outfile6 << tempTrajs[i][2] << ",";
        outfile6 << tempTrajs[i][3] << ",";
        outfile6 << dst2 << ",";
        outfile6 << dst1 << ",";
        outfile6 << tempTrajs[i][4] << '\n';
    }
    outfile6.close();

    // 最后添加一条长度为0的数据
    int count = 0;
    for (int i = 0; i < trajNum; i++)
    {
        if (fabs(tempTrajs[i][3]) < 0.01f)
        {
            continue;
        }
        if (count != i)
        {
            PK_CopyTra(tempTrajs[count], tempTrajs[i]);
        }
        count++;
    }

    trajNum = count;
    PK_CopyPos(tempTrajs[trajNum], planData[0].finpoint);
    tempTrajs[trajNum][3] = 0.0f;
    tempTrajs[trajNum][4] = 0.0f;
    trajNum               = trajNum + 1;

    // 合并轨迹
    if (trajNum > 10)
    {
        trajNum = MergeHybridTrajs(planData, tempTrajs, trajNum);
    }

    trajNum = PathPlan_PathMerge(tempTrajs, trajNum);
    PK_CopyPos(tempTrajs[trajNum], planData[0].finpoint);
    tempTrajs[trajNum][3] = 0.0f;
    tempTrajs[trajNum][4] = 0.0f;
    trajNum               = trajNum + 1;

    // 记录数据
    std::ofstream outfile4("out4.txt");
    for (int i = 0; i < trajNum; i++)
    {
        outfile4 << tempTrajs[i][0] << ",";
        outfile4 << tempTrajs[i][1] << ",";
        outfile4 << tempTrajs[i][2] << ",";
        outfile4 << tempTrajs[i][3] << ",";
        outfile4 << tempTrajs[i][4] << '\n';
    }
    outfile4.close();

    PlanDataCase tempPlan;
    memcpy(&tempPlan, planData, sizeof(tempPlan));
    trajNum = ConnectEndTrajs(tempPlan, tempTrajs, trajNum, max_traj_num);

    // 记录数据
    std::ofstream outfile5("out5.txt");
    for (int i = 0; i < trajNum - 1; i++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(tempTrajs[i], endPos);
        outfile5 << tempTrajs[i][0] << ",";
        outfile5 << tempTrajs[i][1] << ",";
        outfile5 << tempTrajs[i][2] << ",";
        outfile5 << tempTrajs[i][3] << ",";
        outfile5 << tempTrajs[i][4] << ' ';
        outfile5 << endPos[0] << ",";
        outfile5 << endPos[1] << ",";
        outfile5 << endPos[2] << "\n";
    }
    outfile5.close();

    // 连接断头轨迹
    trajNum = ConnectTrajs(tempPlan, tempTrajs, trajNum, max_traj_num);

    // 去掉前后的多余路径
    trajNum = PK_PathMerge(tempTrajs, trajNum, slot);
    memcpy(PlannedPath, tempTrajs, PATH_ITEM_LEN * trajNum);
    if (trajNum > MAX_SINGLE_TRAJ_NUM)
    {
        printf("PathPlan_HybridReplanResult trajs too much %d\n", trajNum);
        return 0;
    }

    return trajNum;
}

void PathPlan_HybridClearResult(const Multi_Slot_Array_T &slotArray)
{
    int slotIds[MAX_PARKED_SLOTS];
    int slotNum = slotArray.multiNum;
    for (int i = 0; i < slotNum; i++)
    {
        slotIds[i] = slotArray.multiArray[i].slot_index;
    }

    HybridAClearResult(slotIds, slotNum);
}

bool PathPlan_IsHybridSlot(const SlotInfo_T &slotinf)
{
    return HybridASlot(slotinf.slot_index);
}
