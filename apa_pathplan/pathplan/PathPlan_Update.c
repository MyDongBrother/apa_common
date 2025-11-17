#include "PathPlan_Update.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_PathExecute.h"
#include "PathPlan_BasePlan.h"

inline PlanedPath &PathPlan_GetPlannedPathImpl(const int pathIndex)
{
    static PlanedPath s_plannedpaths[MAX_PARKED_SLOTS] = {{0}};
    while (pathIndex >= MAX_PARKED_SLOTS || pathIndex < 0)
    {
        printf("%s %d buff index error!\n", __FUNCTION__, __LINE__);
    }
    return s_plannedpaths[pathIndex];
}

inline void PathPlan_ClearPathImpl(PlanedPath &path)
{
    path.slotIndex = -1;
    path.counter   = 0;
    path.pathNum   = 0;
    path.record    = 0;
}

/**
 * 更新可规划的车位信息信息。
 *
 * @param Multi_Slot_Array 多槽位数组，包含当前所有的槽位信息。
 */
void PathPlan_UpdatePlannedSlots(const Multi_Slot_Array_T &Multi_Slot_Array)
{
    for (auto pathIndex = 0; pathIndex < MAX_PARKED_SLOTS; pathIndex++)
    {
        PlanedPath &path = PathPlan_GetPlannedPathImpl(pathIndex);
        if (path.pathNum == 0 && path.slotIndex == -1)
        {
            continue;
        }

        auto slot_index = path.slotIndex;
        auto slot       = 0;
        while (slot < Multi_Slot_Array.multiNum &&
               slot_index != Multi_Slot_Array.multiArray[slot].slot_index)
        {
            slot++;
        }

        if (slot >= Multi_Slot_Array.multiNum)
        {
            PathPlan_ClearPathImpl(path);
        }
    }
}

PlanedPath &PathPlan_GetPlannedPath(const int slotIndex)
{
    int freeindex = -1;
    for (auto index = 0; index < MAX_PARKED_SLOTS; index++)
    {
        PlanedPath &path = PathPlan_GetPlannedPathImpl(index);
        if (slotIndex == path.slotIndex)
        {
            return path;
        }
        if (path.slotIndex == -1 && path.pathNum == 0)
        {
            freeindex = index;
        }
    }

    PlanedPath &path = PathPlan_GetPlannedPathImpl(freeindex);
    PathPlan_ClearPathImpl(path);
    path.slotIndex = slotIndex;
    return path;
}

void PathPlan_CleanPlannedPath()
{
    for (auto index = 0; index < MAX_PARKED_SLOTS; index++)
    {
        PlanedPath &path = PathPlan_GetPlannedPathImpl(index);
        PathPlan_ClearPathImpl(path);
    }
}

void PathPlan_AddPlanPath(PlanedPath &path, const int costTime,
                          const float trajs[][TRAJITEM_LEN], const float pathNum)
{
    path.counter = 0;
    path.pathNum = pathNum;
    memcpy(path.trajs, trajs, pathNum * PATH_ITEM_LEN);
}

static void MoveFinPoint(float trajs[][TRAJITEM_LEN], int pathNum, float newTarget[3],
                         uint8_t hasStopBar)
{
    // 获取当前路径的终点位置
    float Pathfinpoint[3] = {0};
    PK_Get_Path_EndPos(trajs[pathNum - 1], Pathfinpoint);

    float rsTarget[3];
    CoordinadteTransfer(Pathfinpoint, newTarget, rsTarget);

    // last traj!
    if (hasStopBar != 0 && trajs[pathNum - 1][4] < ZERO_FLOAT)
    {
        if (trajs[pathNum - 1][3] + rsTarget[0] < -0.35f)
        {
            trajs[pathNum - 1][3] = trajs[pathNum - 1][3] + rsTarget[0];
        }
    }

    // 根据新的目标位置调整路径
    for (int i = 0; i < pathNum; i++)
    {
        float rsStart[3];
        CoordinadteTransfer(Pathfinpoint, trajs[i], rsStart);
        Convert(newTarget, rsStart,
                trajs[i]); // 将路径点从新的目标位置坐标系转换回原坐标系
        trajs[i][2] = Round_PI(newTarget[2] + rsStart[2]);
    }
}

static float CalcMinDistToTraj(const float traj[TRAJITEM_LEN], const float Point[2],
                               float step)
{
    float temp[TRAJITEM_LEN];
    PK_CopyTra(temp, traj);
    float length  = 0.0f;
    float trajlen = 0.0f;
    float mindist = Cal_Dis_Pt2Pt(Point, temp);
    while (length < fabs(traj[3]))
    {
        float trajEnd[3];
        temp[3] = length * sign(traj[3]);
        PK_Get_Path_EndPos(temp, trajEnd);
        float dist = Cal_Dis_Pt2Pt(Point, trajEnd);
        if (dist < mindist)
        {
            mindist = dist;
            trajlen = length;
        }
        length = length + step;
    }
    return trajlen;
}

static int CalcMinDistToTrajs(const float trajs[][TRAJITEM_LEN], const int trajNum,
                              const float stpoint[2])
{
    const float max_dist = 6.0f;
    int trajIndex        = 0;
    float mindist        = max_dist;
    float trajEnd[3];

    for (int i = 0; i < trajNum; i++)
    {
        float temp[TRAJITEM_LEN];
        PK_CopyTra(temp, trajs[i]);
        float trajlen = CalcMinDistToTraj(temp, stpoint, 0.15f);
        temp[3]       = trajlen * sign(temp[3]);

        PK_Get_Path_EndPos(temp, trajEnd);
        float dist = Cal_Dis_Pt2Pt(stpoint, trajEnd);
        if (dist < mindist)
        {
            mindist   = dist;
            trajIndex = i;
        }

        if (mindist < 0.10f)
        {
            break;
        }
    }

    return trajIndex;
}

static int UpdatePlanPath1(const float trajs[][TRAJITEM_LEN], const int trajStart,
                           const float stpoint[3], const int trajNum,
                           float temps[][TRAJITEM_LEN])
{
    // 新路径的起点
    float temp1[TRAJITEM_LEN], trajEnd[3];
    if (trajNum - trajStart <= 0)
    {
        return 0;
    }

    PK_CopyTra(temp1, trajs[trajStart]);
    float trajlen = CalcMinDistToTraj(temp1, stpoint, 0.03f);
    temp1[3]      = trajlen * sign(temp1[3]);
    PK_Get_Path_EndPos(temp1, trajEnd);

    // 起点和当前位置非常接近
    float dist = Cal_Dis_Pt2Pt(stpoint, trajEnd);
    if (dist < 0.06f && fabs(trajEnd[2] - stpoint[2]) < 0.01f)
    {
        // 生成新路径
        PK_CopyPos(temp1, trajEnd);
        temp1[3] = (fabs(trajs[trajStart][3]) - trajlen) * sign(trajs[trajStart][3]);
        temp1[4] = trajs[trajStart][4];

        memcpy(&temps[0][0], temp1, PATH_ITEM_LEN);
        memcpy(&temps[1][0], &trajs[trajStart + 1],
               (trajNum - (trajStart + 1)) * PATH_ITEM_LEN);
        return trajNum - trajStart;
    }

    return 0;
}

static int UpdatePlanPath2(const float trajs[][TRAJITEM_LEN], const int trajStart,
                           const int trajNum, const PlanDataCase &planData,
                           float temps[][TRAJITEM_LEN])
{
    extern int PathPlan_Rs(const float start[3], const float end[3], float trajs[][5]);
    float traject[5][TRAJITEM_LEN], trajEnd[3];
    if (trajNum - trajStart <= 1)
    {
        return 0;
    }

    int trajIndex = trajStart;
    if (Cal_Dis_Pt2Pt(planData.stpoint, trajs[trajIndex]) < 0.8f &&
        trajIndex + 1 < trajNum - 1)
    {
        trajIndex = trajIndex + 1;
    }

    while (trajIndex < trajNum - 1)
    {
        int pathNum0 = PathPlan_Rs(planData.stpoint, trajs[trajIndex], traject);
        if (pathNum0 > 0 && pathNum0 + (trajNum - trajIndex) < MAX_SINGLE_TRAJ_NUM)
        {
            PK_Get_Path_EndPos(traject[pathNum0 - 1], trajEnd);
            auto check = PK_Check_Path(pathNum0, traject, planData.stpoint, trajEnd,
                                       planData.obj_danger_num, planData.obj_danger, 0);
            if (check == PATHPLAN_SUCCESS)
            {
                memcpy(&temps[0][0], traject, pathNum0 * PATH_ITEM_LEN);
                memcpy(&temps[pathNum0][0], &trajs[trajIndex],
                       (trajNum - trajIndex) * PATH_ITEM_LEN);
                return pathNum0 + (trajNum - trajIndex);
            }
        }
        trajIndex = trajIndex + 1;
    }

    return 0;
}

int PathPlan_UpdatePlanPath(PlanedPath &path, PlanDataCase &planData,
                            float trajs[][TRAJITEM_LEN], bool updateTarget)
{
    if (path.pathNum <= 0)
    {
        return -1;
    }
    int type = 0;
    float trajEnd[3], rsTarget[3];
    PK_Get_Path_EndPos(path.trajs[path.pathNum - 1], trajEnd);
    CoordinadteTransfer(planData.finpoint, trajEnd, rsTarget);
    float distOffset = Cal_Dis_Pt2Pt(planData.finpoint, trajEnd);
    if (fabs(rsTarget[1]) > VEHICLE_WID * 0.3f || fabs(rsTarget[2]) > 0.2f ||
        distOffset > 1.0f)
    {
        PathPlan_ClearPathImpl(path);
        return -2;
    }

    float temps[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    memcpy(temps, path.trajs, sizeof(temps));
    if (updateTarget)
    {
        MoveFinPoint(temps, path.pathNum, planData.finpoint, planData.has_stopper);
    }

    // 没有移动则只校验
    CoordinadteTransfer(planData.stpoint, temps[0], rsTarget);
    if (fabs(rsTarget[0]) < 0.5f * PathPlanConstCfg::div_err &&
        fabs(rsTarget[1]) < 0.5f * PathPlanConstCfg::div_err_ry_fin &&
        fabs(rsTarget[2]) < 0.5f * PathPlanConstCfg::div_err_theta_fin)
    {
        int pathNum       = path.pathNum;
        auto checkResult1 = PK_Check_Path(pathNum, temps, planData.stpoint,
                                          planData.finpoint, 5, planData.obj_slot, 0);
        auto checkResult2 =
            PK_Check_Path(pathNum, temps, planData.stpoint, planData.finpoint,
                          planData.obj_danger_num, planData.obj_danger, 0);
        if (checkResult1 == PATHPLAN_SUCCESS && checkResult2 == PATHPLAN_SUCCESS)
        {
            memcpy(trajs, temps, pathNum * PATH_ITEM_LEN);
            pathNum = PK_PathMerge(trajs, pathNum, 0);
            if (PathPlan_CheckPlannedPathValid(trajs, pathNum) == PATHPLAN_SUCCESS)
            {
                return pathNum;
            }
        }
    }

    int trajIndex = CalcMinDistToTrajs(temps, path.pathNum, planData.stpoint);
    if (trajIndex >= path.pathNum)
    {
        PathPlan_ClearPathImpl(path);
        return -5;
    }

    // 过滤掉校验不通过的
    int validPath = path.pathNum - 1;
    while (validPath >= trajIndex)
    {
        PK_Get_Path_EndPos(temps[validPath], trajEnd);
        auto check1 = PK_Check_Path(1, &temps[validPath], temps[validPath], trajEnd,
                                    planData.obj_danger_num, planData.obj_danger, 0);
        auto check2 = PK_Check_Path(1, &temps[validPath], temps[validPath], trajEnd, 5,
                                    planData.obj_slot, 0);
        if (check1 != PATHPLAN_SUCCESS || check2 != PATHPLAN_SUCCESS)
        {
            break;
        }
        validPath--;
    }

    trajIndex    = validPath + 1;
    int nextTraj = trajIndex;
    PK_Get_Path_EndPos(temps[nextTraj], trajEnd);
    float rx1 = Project_PosTo1st_rx(planData.stpoint, temps[nextTraj]);
    float rx2 = Project_PosTo1st_rx(planData.stpoint, trajEnd);
    if (rx1 * rx2 < 0)
    {
        nextTraj = nextTraj + 1;
    }
    if (nextTraj >= path.pathNum)
    {
        PathPlan_ClearPathImpl(path);
        return -6;
    }

    float traject0[3][TRAJITEM_LEN], traject1[3][TRAJITEM_LEN];
    int pathNum0 =
        PK_PathPlan_BetterTra(planData.stpoint, temps[nextTraj], planData.obj_danger_num,
                              planData.obj_danger, traject0, 0);
    int pathNum1 = 0;
    if (nextTraj + 1 < path.pathNum)
    {
        pathNum1 = PK_PathPlan_BetterTra(planData.stpoint, temps[nextTraj + 1],
                                         planData.obj_danger_num, planData.obj_danger,
                                         traject1, 0);
    }

    int pathNum = 0;
    if (pathNum1 > 0 && (pathNum0 >= pathNum1 || pathNum0 <= 0))
    {
        memcpy(&trajs[0][0], traject1, pathNum1 * PATH_ITEM_LEN);
        memcpy(&trajs[pathNum1][0], &temps[nextTraj + 1],
               (path.pathNum - (nextTraj + 1)) * PATH_ITEM_LEN);
        pathNum = pathNum1 + path.pathNum - (nextTraj + 1);
        type    = 1;
    }
    else if (pathNum0 > 0 && pathNum1 <= 0)
    {
        memcpy(&trajs[0][0], traject0, pathNum0 * PATH_ITEM_LEN);
        memcpy(&trajs[pathNum0][0], &temps[nextTraj],
               (path.pathNum - nextTraj) * PATH_ITEM_LEN);
        pathNum = pathNum0 + (path.pathNum - nextTraj);
        type    = 2;
    }
    else if (updateTarget)
    {
        float startOffset = Cal_Dis_Pt2Pt(planData.stpoint, path.trajs[0]);
        if (startOffset < 0.01f && fabs(rsTarget[1]) < 0.05f &&
            fabs(rsTarget[2]) < 0.02f && rsTarget[0] < 0.10f)
        {
            path.counter++;
            memcpy(&trajs[0][0], temps, path.pathNum * PATH_ITEM_LEN);
            return path.pathNum;
        }
    }

    if (pathNum <= 0)
    {
        if (rx1 * rx2 < 0)
        {
            pathNum =
                UpdatePlanPath1(temps, trajIndex, planData.stpoint, path.pathNum, trajs);
            type = 0x100 + pathNum;
        }

        if (pathNum <= 0)
        {
            pathNum = UpdatePlanPath2(temps, nextTraj, path.pathNum, planData, trajs);
            type    = 0x200 + pathNum;
        }

        if (pathNum <= 0)
        {
            PathPlan_ClearPathImpl(path);
            return -7;
        }
    }

    PK_Get_Path_EndPos(trajs[pathNum - 1], trajEnd);
    while (Cal_Dis_Pt2Pt(planData.finpoint, trajEnd) > 0.40f && !updateTarget)
    {
        printf("---------------PathPlan_UpdatePlanPath %d----------------------------\n",
               type);
        usleep(10000);
    }

    pathNum = PK_PathMerge(trajs, pathNum, 0);
    if (PathPlan_CheckPlannedPathValid(trajs, pathNum) != PATHPLAN_SUCCESS)
    {
        printf("target plan path overflow ! index %d, target: %f %f %f slotinfo: %d\n",
               path.slotIndex, planData.finpoint[0], planData.finpoint[1],
               planData.finpoint[2], planData.slotshape);
        PathPlan_ClearPathImpl(path);
        return -8;
    }

    auto checkResult1 = PK_Check_Path(pathNum, trajs, planData.stpoint, planData.finpoint,
                                      5, planData.obj_slot, 0);
    auto checkResult2 = PK_Check_Path(pathNum, trajs, planData.stpoint, planData.finpoint,
                                      planData.obj_danger_num, planData.obj_danger, 0);
    if (checkResult1 != PATHPLAN_SUCCESS || checkResult2 != PATHPLAN_SUCCESS)
    {
        PathPlan_ClearPathImpl(path);
        return -9;
    }

    path.counter++;
    return pathNum;
}

/***********************************************
add2023.2.29 Tangsj
如果优化后节点差异过大，尝试重新连接
Connect_BreakPoint
INTPUT:     需要连接的两个路径节点
OUTPUT:     0-不能连接 1-能连接，并输出更改后的traj
***********************************************/
int Connect_BreakPoint(float *traj, float *trajnext)
{
    float r, rx, ry, rtheta, fy, dmin;
    float temtraj[5];

    PK_CopyTra(temtraj, traj);
    rx     = Project_PosTo1st_rx(traj, trajnext);
    ry     = Project_PosTo1st_ry(traj, trajnext); // relative pos
    fy     = Project_PosTo1st_ry(trajnext, traj);
    rtheta = Project_PosTo1st_rtheta(traj, trajnext);
    dmin   = rte_min(fabsf(ry), fabsf(fy));
    r      = dmin / (1 - cosf(rtheta));

    if (fabsf(r) - Rrmin >= ZERO_FLOAT && fabsf(r) < 60)
    {
        temtraj[4] = sign(ry) * r; // r
        temtraj[3] = sign(rx) * fabsf(rtheta * r);

        if (PathPlan_CheckPosDiv(temtraj, trajnext) == 0) //检测
        {
            traj[3] = temtraj[3];
            traj[4] = temtraj[4];

            return 1;
        }
    }

    return 0;
}
