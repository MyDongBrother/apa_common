#include "PathPlan_Update.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_PathExecute.h"
#include "PathPlan_Func.h"
#include "PathPlan_Tools.h"

static bool MinTwoTrajsDiv(float traj1[TRAJITEM_LEN], float traj2[TRAJITEM_LEN]);

const float merge_traj_minlen = 0.20f;

static inline void TrajMove(float trajs[][TRAJITEM_LEN], const int trajNum)
{
    for (int i = 0; i < trajNum; i++)
    {
        PK_CopyTra(trajs[i], trajs[i + 1]);
    }
}

static bool CheckTrajConnect(const float trajs[][TRAJITEM_LEN], const int trajNum)
{
    int testflag = 0;
    for (int i = 0; i < trajNum - 1; i++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(trajs[i], endPos);
        if (PathPlan_CheckPosDiv(endPos, trajs[i + 1]) != 0 && testflag == 0)
        {
            return false;
        }
    }
    return true;
}

static inline bool CheckMergeResult(const float oldEndPos[3], const float newEndPos[3])
{
    if (PathPlan_CheckPosDiv(oldEndPos, newEndPos) != 0)
    {
        return false;
    }

    float thetaDiv = Round_PI(oldEndPos[2] - newEndPos[2]);
    if (fabs(thetaDiv) > PathPlanConstCfg::div_err_theta_fin)
    {
        return false;
    }

    return true;
}

// 直接删除非常短的路径
static int MergeShortLine1(float trajs[][TRAJITEM_LEN], const int lineNum)
{
    int count = lineNum;
    for (int i = 0; i < count - 1; i++)
    {
        if (fabs(trajs[i][3]) < 0.01f)
        {
            TrajMove(&trajs[i], count - (i + 1));
            count = count - 1;
        }
    }
    return count;
}

// 次短路径尝试合并
static int MergeShortLine2(float trajs[][TRAJITEM_LEN], const int lineNum)
{
    int count = lineNum;
    for (int i = 1; i < count - 1; i++) // 首尾不能够合并
    {
        if (fabs(trajs[i][3]) < merge_traj_minlen &&
            fabs(trajs[i - 1][3]) > merge_traj_minlen &&
            fabs(trajs[i + 1][3]) > merge_traj_minlen)
        {
            float tempTrajs[2][TRAJITEM_LEN];
            PK_CopyTra(tempTrajs[0], trajs[i - 1]);
            PK_CopyTra(tempTrajs[1], trajs[i + 1]);

            PathPlan_ReverseTraject(tempTrajs[1]);
            if (MinTwoTrajsDiv(tempTrajs[0], tempTrajs[1]))
            {
                PathPlan_ReverseTraject(tempTrajs[1]);

                // 删除当前的短路径
                PK_CopyTra(trajs[i - 1], tempTrajs[0]);
                PK_CopyTra(trajs[i], tempTrajs[1]);
                TrajMove(&trajs[i + 1], count - (i + 2));
                count = count - 1;
            }
        }
    }

    return count;
}

static bool MinTwoTrajsDiv(float traj1[TRAJITEM_LEN], float traj2[TRAJITEM_LEN])
{
    float EndPos[2][3], rsStart[2][3];

    PK_Get_Path_EndPos(traj1, EndPos[0]);
    PK_Get_Path_EndPos(traj2, EndPos[1]);

    CoordinadteTransfer(EndPos[0], EndPos[1], rsStart[0]);
    CoordinadteTransfer(EndPos[1], EndPos[0], rsStart[1]);

    // 初始误差
    float divSum = fabs(rsStart[0][0]) + 2 * fabs(rsStart[0][1]);
    float temp[2][TRAJITEM_LEN], temp0[2][TRAJITEM_LEN];

    PK_CopyTra(temp[0], traj1);
    PK_CopyTra(temp[1], traj2);

    float dir0 = (rsStart[0][0] * temp[0][3] > 0.0001f) ? 1.0f : -1.0f;
    float dir1 = (rsStart[1][0] * temp[1][3] > 0.0001f) ? 1.0f : -1.0f;

    while (fabs(rsStart[0][0]) > 0.3f * PathPlanConstCfg::div_err)
    {
        float dist = sqrt(rsStart[0][0] * rsStart[0][0] + rsStart[0][1] * rsStart[0][1]);
        if (fabs(rsStart[0][0]) < 0.5f * PathPlanConstCfg::div_err &&
            fabs(rsStart[0][1]) < 0.5f * PathPlanConstCfg::div_err_ry_fin &&
            dist < PathPlanConstCfg::div_err)
        {
            break;
        }

        PK_CopyTra(temp0[0], temp[0]);
        PK_CopyTra(temp0[1], temp[1]);

        // 1 延长
        temp0[0][3] = (fabs(temp[0][3]) + PathPlanConstCfg::div_err * 0.3 * dir0) *
                      sign(temp[0][3]);
        PK_Get_Path_EndPos(temp0[0], EndPos[0]);
        PK_Get_Path_EndPos(temp[1], EndPos[1]);
        CoordinadteTransfer(EndPos[0], EndPos[1], rsStart[0]);
        float divSum1 = fabs(rsStart[0][0]) + 2 * fabs(rsStart[0][1]);

        // 2 延长
        temp0[1][3] = (fabs(temp0[1][3]) + PathPlanConstCfg::div_err * 0.3 * dir1) *
                      sign(temp[1][3]);
        PK_Get_Path_EndPos(temp0[1], EndPos[1]);
        PK_Get_Path_EndPos(temp[0], EndPos[0]);
        CoordinadteTransfer(EndPos[1], EndPos[0], rsStart[1]);
        float divSum2 = fabs(rsStart[1][0]) + 2 * fabs(rsStart[1][1]);

        // 3 取效果最好的
        if (divSum1 < divSum2 && divSum1 < divSum)
        {
            PK_CopyTra(temp[0], temp0[0]);
            divSum = divSum1;
        }
        else if (divSum2 < divSum1 && divSum2 < divSum)
        {
            PK_CopyTra(temp[1], temp0[1]);
            divSum = divSum2;
        }
        else
        {
            return false;
        }
    }

    PK_CopyTra(traj1, temp[0]);
    PK_CopyTra(traj2, temp[1]);
    return true;
}

// 通过调整长度减少相邻路径起点和终点距离
void PathPlan_UpdateTrajsLen(float trajs[][TRAJITEM_LEN], const int count)
{
    for (int i = 0; i < count - 1; i++)
    {
        float EndPos[3], rsStart[3];
        PK_Get_Path_EndPos(trajs[i], EndPos);
        CoordinadteTransfer(EndPos, trajs[i + 1], rsStart);

        if ((fabs(rsStart[0]) > 3.0f * PathPlanConstCfg::div_err ||
             fabs(rsStart[1]) > 3.0f * PathPlanConstCfg::div_err_ry_fin))
        {
            // printf("UpdateTrajLen error! %04f,%04f,%04f,%04f,%04f
            // %04f,%04f,%04f,%04f,%04f\n",
            //     trajs[i][0], trajs[i][1], trajs[i][2], trajs[i][3], trajs[i][4],
            //     trajs[i + 1][0], trajs[i + 1][1], trajs[i + 1][2], trajs[i + 1][3],
            //     trajs[i + 1][4]);
            continue;
        }

        if ((fabs(rsStart[0]) > PathPlanConstCfg::div_err ||
             fabs(rsStart[1]) > PathPlanConstCfg::div_err_ry_fin) &&
            (fabs(rsStart[0]) > 0.5 * PathPlanConstCfg::div_err))
        {
            // 方向相同, 需要反向
            PathPlan_ReverseTraject(trajs[i + 1]);

            // 缩小相邻路径距离
            MinTwoTrajsDiv(trajs[i], trajs[i + 1]);

            // 恢复方向
            PathPlan_ReverseTraject(trajs[i + 1]);
        }
    }
}

// 合并直线
static int MergeLine(float trajs[][TRAJITEM_LEN],
                     const int lineNum) // HAVECHANGE 2023.2.29 Tangsj
{
    int count                    = lineNum;
    int count1                   = 0;
    float tempTraj[TRAJITEM_LEN] = {0};

    while (count != count1)
    {
        count1 = count;
        for (int i = 0; i < count - 1; i++)
        {
            if (fabsf(trajs[i][4]) < ZERO_FLOAT && fabsf(trajs[i + 1][4]) < ZERO_FLOAT &&
                fabsf(trajs[i + 1][2] - trajs[i][2]) < 0.0001f)
            {
                float rx = Project_PosTo1st_rx(trajs[i], trajs[i + 1]);
                if (rx * trajs[i][3] < ZERO_FLOAT)
                {
                    printf(
                        "MergeLine error! %04f,%04f,%04f,%04f,%04f "
                        "%04f,%04f,%04f,%04f,%04f\n",
                        trajs[i][0], trajs[i][1], trajs[i][2], trajs[i][3], trajs[i][4],
                        trajs[i + 1][0], trajs[i + 1][1], trajs[i + 1][2],
                        trajs[i + 1][3], trajs[i + 1][4]);
                    continue;
                }

                float oldEndPos[3];

                // 计算长度
                PK_Get_Path_EndPos(trajs[i + 1], oldEndPos);
                float newTrajLen = Cal_Dis_Pt2Pt(trajs[i], oldEndPos);

                // 起点
                PK_CopyTra(tempTraj, trajs[i]);

                // 行驶方向
                rx          = Project_PosTo1st_rx(trajs[i], oldEndPos);
                tempTraj[3] = sign(rx) * newTrajLen;

                float newEndPos[3];
                PK_Get_Path_EndPos(tempTraj, newEndPos);

                bool checkDiv = false;
                if (i + 2 < count)
                {
                    checkDiv = CheckMergeResult(newEndPos, trajs[i + 2]);
                }
                else
                {
                    checkDiv = CheckMergeResult(newEndPos, oldEndPos);
                }

                if (checkDiv)
                {
                    PK_CopyTra(trajs[i], tempTraj);
                    TrajMove(&trajs[i + 1], count - (i + 2));
                    count = count - 1;
                }
            }
        }
    }

    return count;
}

// 合并曲线
static int MergeCircle(float trajs[][TRAJITEM_LEN],
                       const int lineNum) // HAVECHANGE 2023.2.29 Tangsj
{
    int count                     = lineNum;
    int count1                    = 0;
    float tempTraj1[TRAJITEM_LEN] = {0};
    float tempTraj2[TRAJITEM_LEN] = {0};

    while (count != count1)
    {
        count1 = count;
        for (int i = 0; i < count - 1; i++)
        {
            if (fabsf(trajs[i][4] - trajs[i + 1][4]) < 0.001f)
            {
                float rx = Project_PosTo1st_rx(trajs[i], trajs[i + 1]);
                if (sign(rx) * trajs[i][3] < -0.001f)
                {
                    // printf("MergeCircle error 1! %04f,%04f,%04f,%04f,%04f
                    // %04f,%04f,%04f,%04f,%04f\n",
                    //     trajs[i][0], trajs[i][1], trajs[i][2], trajs[i][3],
                    //     trajs[i][4], trajs[i + 1][0], trajs[i + 1][1], trajs[i + 1][2],
                    //     trajs[i + 1][3], trajs[i + 1][4]);
                    continue;
                }

                PK_CopyTra(tempTraj1, trajs[i]);
                PK_CopyTra(tempTraj2, trajs[i + 1]);

                // 先对齐前后路径的终点和起点
                PathPlan_ReverseTraject(tempTraj2);
                MinTwoTrajsDiv(tempTraj1, tempTraj2);
                PathPlan_ReverseTraject(tempTraj2);

                // 计算长度和方向
                tempTraj1[3] = tempTraj2[3] + tempTraj1[3];

                // 通过比较终点差异校验合并差异
                float newEndPos[3], oldEndPos[3];
                PK_Get_Path_EndPos(tempTraj1, newEndPos);
                PK_Get_Path_EndPos(trajs[i + 1], oldEndPos);

                bool checkDiv = false;
                if (i + 2 < count)
                {
                    checkDiv = CheckMergeResult(newEndPos, trajs[i + 2]);
                }
                else
                {
                    checkDiv = CheckMergeResult(newEndPos, oldEndPos);
                }

                float theta = fabs(Round_PI(tempTraj1[2] - oldEndPos[2]));
                if (!checkDiv && fabs(tempTraj1[4]) > Rrmin &&
                    theta < PI * 0.25f) // 45deg
                {
                    float dist   = Cal_Dis_Pt2Pt(tempTraj1, oldEndPos);
                    float radius = dist * 0.5 / sin(theta * 0.5);
                    if (radius > Rrmin)
                    {
                        tempTraj1[3] = theta * radius * sign(tempTraj1[3]);
                        tempTraj1[4] = radius * sign(tempTraj1[4]);
                        PK_Get_Path_EndPos(tempTraj1, newEndPos);
                        if (i + 2 < count)
                        {
                            checkDiv = CheckMergeResult(newEndPos, trajs[i + 2]);
                        }
                        else
                        {
                            checkDiv = CheckMergeResult(newEndPos, oldEndPos);
                        }
                    }
                }

                if (checkDiv)
                {
                    PK_CopyTra(trajs[i], tempTraj1);
                    TrajMove(&trajs[i + 1], count - (i + 2));
                    count = count - 1;
                }
            }
        }
    }

    return count;
}

static int PathPlan_MergeLine(float trajs[][TRAJITEM_LEN],
                              const int lineNum) // HAVECHANGE 2023.2.29 Tangsj
{
    int count                    = lineNum;
    int count1                   = 0;
    float temtraj[TRAJITEM_LEN]  = {0};
    float temtraj2[TRAJITEM_LEN] = {0};

    while (count != count1)
    {
        count1 = count;
        for (int i = 0; i < count - 1; i++)
        {
            if ((fabsf(trajs[i][4]) < ZERO_FLOAT &&
                 fabsf(trajs[i + 1][4]) <
                     ZERO_FLOAT /*&& sign(trajs[i][3]) == sign(trajs[i + 1][3]) */
                 && fabsf(trajs[i + 1][2] - trajs[i][2]) < ZERO_FLOAT) ||
                (fabsf(trajs[i][4] - trajs[i + 1][4]) < ZERO_FLOAT &&
                 sign(trajs[i][3]) != sign(trajs[i + 1][3])) ||
                (i < count - 2 && trajs[i + 1][3] < 0.03 &&
                 fabsf(trajs[i + 1][2] - trajs[i + 2][2]) < 0.01))
            {
                PK_CopyTra(temtraj, trajs[i]); // i节点的缓存temtraj
                temtraj[3] = trajs[i][3] + trajs[i + 1][3];

                if (fabsf(trajs[i][4] - trajs[i + 1][4]) < ZERO_FLOAT &&
                    sign(trajs[i][3]) != sign(trajs[i + 1][3]) && trajs[i][3] >= 0.5 &&
                    temtraj[3] > 0 && temtraj[3] < 0.5)
                {
                    trajs[i][3] = 0.5;

                    PK_Get_Path_EndPos(trajs[i], trajs[i + 1]);
                    trajs[i + 1][3] = trajs[i][3] - temtraj[3];

                    continue;
                }

                if (i < count - 2)
                {
                    if (PathPlan_CheckPosDiv(temtraj, trajs[i + 2]) == 0) //通过检测
                    {
                        PK_CopyTra(trajs[i], temtraj);
                        memcpy(trajs[i + 1], trajs[i + 2],
                               (count - i - 2) * PATH_ITEM_LEN); //去除i+1节点
                        count = count - 1;
                    }
                    else
                    {
                        if (Connect_BreakPoint(temtraj, trajs[i + 2])) //连接成功
                        {
                            PK_CopyTra(trajs[i], temtraj);
                            memcpy(trajs[i + 1], trajs[i + 2],
                                   (count - i - 2) * PATH_ITEM_LEN);
                            count = count - 1;
                        }
                        else //连接失败，尝试更改i+2节点，并连接i+3节点
                        {
                            PK_Get_Path_EndPos(
                                trajs[i], temtraj2); // temtraj变为更改后的i+2节点缓存
                            if (i < count - 3 &&
                                Connect_BreakPoint(temtraj2, trajs[i + 3])) //连接成功
                            {
                                PK_CopyTra(trajs[i + 2], temtraj2); //更改i+2节点
                                memcpy(trajs[i + 1], trajs[i + 2],
                                       (count - i - 2) * PATH_ITEM_LEN);
                                count = count - 1;
                            }
                        }
                    }
                }
                else if (i < count - 1)
                {
                    PK_CopyTra(trajs[i], temtraj);
                    memcpy(trajs[i + 1], trajs[i + 2],
                           (count - i - 2) * PATH_ITEM_LEN); //去除i+1节点
                    count = count - 1;
                }
            }
        }
    }

    return count;
}

static int MergeEnd(float trajs[][TRAJITEM_LEN], const int lineNum)
{
    int count = lineNum;
    if (count <= 1)
    {
        return count;
    }
    if (fabs(trajs[0][3]) < 0.03 && sign(trajs[0][3]) == sign(trajs[1][3]))
    {
        if (PathPlan_CheckPosDiv(trajs[0], trajs[1]) == 0)
        {
            TrajMove(&trajs[0], count - 1);
            count = count - 1;
        }
    }

    if (count <= 1)
    {
        return count;
    }

    if (fabs(trajs[count - 1][3]) < 0.03 && fabs(trajs[count - 2][3]) > 0.20f &&
        sign(trajs[count - 2][3]) == sign(trajs[count - 1][3]))
    {
        float endPos1[3], endPos2[3];
        PK_Get_Path_EndPos(trajs[count - 2], endPos2);
        PK_Get_Path_EndPos(trajs[count - 1], endPos1);
        float rx = Project_PosTo1st_rx(endPos2, endPos1);

        float temp[TRAJITEM_LEN];
        PK_CopyTra(temp, trajs[count - 2]);
        temp[3] = temp[3] + rx;
        PK_Get_Path_EndPos(temp, endPos2);
        if (PathPlan_CheckPosDiv(endPos1, endPos2) == 0)
        {
            PK_CopyTra(trajs[count - 2], temp);
            count = count - 1;
        }
    }

    return count;
}

static int TrajsExtend(const float start[TRAJITEM_LEN], const float end[TRAJITEM_LEN],
                       const int objNum, const float objs[][4],
                       float trajs[10][TRAJITEM_LEN])
{
#if 0
    float temp1[TRAJITEM_LEN], stpoint[3];
    PK_CopyTra(temp1, start);
    PK_Get_Path_EndPos(temp1, stpoint);
    temp1[3] = (fabs(temp1[3]) + 1.2) * sign(temp1[3]);
    float extendStart = PathPlan_LandMark(temp1, objNum, objs, 0.3);
    float exendTemp1 = fabs(extendStart) - fabs(start[3]);

    float temp2[TRAJITEM_LEN];
    PK_CopyTra(temp2, end);
    PathExec_PathInverse(1, &temp2);
    temp2[3] = (fabs(temp2[3]) + 1.2) * sign(temp2[3]);
    float extendEnd = PathPlan_LandMark(temp2, objNum, objs, 0.3);
    float exendTemp2 = fabs(extendEnd) - fabs(end[3]);

    int pathNum = 0;
    if (exendTemp1 > exendTemp2 && exendTemp1 > 0.6)
    {
        exendTemp1 = rte_min(0.8, exendTemp1);
        temp1[3] = (fabs(temp1[3]) + exendTemp1) * sign(temp1[3]);
        PK_Get_Path_EndPos(temp1, stpoint);

        PK_CopyTra(temp2, end);
        pathNum = PathPlan_BetterTra(stpoint, temp2, objNum, objs, &trajs[1], 0.3, 0.5);
    }
    else if (exendTemp2 > exendTemp1 && exendTemp2 > 0.6)
    {
        PK_CopyTra(temp1, start);
        PK_Get_Path_EndPos(temp1, stpoint);

        exendTemp2 = rte_min(0.8, exendTemp2);
        temp2[3] = (fabs(temp2[3]) + exendTemp2) * sign(temp2[3]);
        PathExec_PathInverse(1, &temp2);
        pathNum = PathPlan_BetterTra(stpoint, temp2, objNum, objs, &trajs[1], 0.3, 0.5);
    }
    else if (exendTemp1 + exendTemp2 > 0.6)
    {
        exendTemp1 = rte_min(0.6, exendTemp1 * 0.8);
        temp1[3] = (fabs(temp1[3]) + exendTemp1) * sign(temp1[3]);
        PK_Get_Path_EndPos(temp1, stpoint);

        exendTemp2 = rte_min(0.6, exendTemp2 * 0.8);
        temp2[3] = (fabs(temp2[3]) + exendTemp2) * sign(temp2[3]);
        PathExec_PathInverse(1, &temp2);
        pathNum = PathPlan_BetterTra(stpoint, temp2, objNum, objs, &trajs[1], 0.3, 0.5);
    }

    if (pathNum > 0)
    {
        PK_CopyTra(trajs[0], temp1);
        PK_CopyTra(trajs[pathNum], temp2);
        pathNum = pathNum + 2;
    }
    return pathNum;

#endif
    return 0;
}

float PathPlan_ShortDist(float trajs[][TRAJITEM_LEN], const int lineNum, int *revNum)
{
    if (lineNum <= 1)
    {
        return 2.0;
    }
    float totaldist = fabs(trajs[0][3]);
    float mindist   = rte_max(totaldist, 2.0);
    int count       = 0;
    for (int i = 1; i < lineNum; i++)
    {
        if (trajs[i - 1][3] * trajs[i][3] < 0)
        {
            if (mindist > totaldist)
            {
                mindist = totaldist;
            }
            totaldist = fabs(trajs[i][3]);
            count     = count + 1;
        }
        else
        {
            totaldist = totaldist + fabs(trajs[i][3]);
        }
    }
    if (mindist > totaldist)
    {
        mindist = totaldist;
    }
    if (revNum != NULL)
    {
        revNum[0] = count;
    }
    return mindist;
}

static int PathPlan_Extend(float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                           const int lineNum, const int objNum, const float objs[][4])
{
    int count = lineNum;
    if (count <= 2 || lineNum >= MAX_SINGLE_TRAJ_NUM)
    {
        return count;
    }

    int revertCount = 0;
    float revertlens[MAX_SINGLE_TRAJ_NUM];
    uint8_t revert[MAX_SINGLE_TRAJ_NUM];
    memset(revert, 0, sizeof(revert));

    revertlens[revertCount] = 0;
    for (int i = 0; i < lineNum - 1; i++)
    {
        if (trajs[i][3] * trajs[i + 1][3] < 0)
        {
            revertlens[revertCount] = revertlens[revertCount] + fabs(trajs[i][3]);
            revert[revertCount]     = i;
            revertCount             = revertCount + 1;
            revertlens[revertCount] = 0;
        }
    }

    // 仅仅换挡一次
    if (revertCount <= 1)
    {
        return 0;
    }

    // 换挡两次，但是第二次超过1m, 不需要延长
    // if (revertCount <= 3 && revertlens[1] > 1.0) {  return 0;  }

    for (int i = 1; i < revertCount; i++)
    {
        // 延长连接处理
        float extendTraj[10][TRAJITEM_LEN];
        float temp[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
        if (revertlens[i] < 0.8)
        {
            int newPathNum = TrajsExtend(trajs[revert[i - 1]], trajs[revert[i] + 1],
                                         objNum, objs, extendTraj);
            if (newPathNum > 0)
            {
                if (lineNum - revert[i] - 2 > 0)
                {
                    memcpy(temp[0], trajs[revert[i] + 1], lineNum - revert[i] - 2);
                }
                memcpy(trajs[revert[i - 1]], extendTraj, newPathNum * PATH_ITEM_LEN);

                if (lineNum - revert[i] - 2 > 0)
                {
                    memcpy(trajs[revert[i - 1] + newPathNum], temp[0],
                           lineNum - revert[i] - 2);
                }

                count = newPathNum + lineNum - revert[i] - 2;
                return count;
            }
        }
    }
    return 0;
}

int PathPlan_PathMerge(float trajs[][TRAJITEM_LEN], const int lineNum)
{
    int merged = 1;       // 合并标志，如果合并发生则为1
    int count  = lineNum; // 当前路径点的数量

    while (merged == 1)
    {
        int count1 = 0;
        merged     = 0;
        // 删除短路径
        count1 = MergeShortLine1(trajs, count);
        if (count1 != count)
        {
            merged = 1;
        }
        count = count1;

        // 合并重复的直线
        count1 = MergeLine(trajs, count);
        if (count1 != count)
        {
            merged = 1;
        }
        count = count1;

        // 合并重复的曲线
        count1 = MergeCircle(trajs, count);
        if (count1 != count)
        {
            merged = 1;
        }
        count = count1;

        // 删除短路径
        count1 = MergeShortLine2(trajs, count);
        if (count1 != count)
        {
            merged = 1;
        }
        count = count1;

        count1 = MergeEnd(trajs, count);
        if (count1 != count)
        {
            merged = 1;
        }
        count = count1;
    }

    return count;
}

/**
 * @brief 优化和合并路径点的函数
 *
 * 该函数通过多个步骤优化路径，包括移除过短路径、不必要的路径段和末端多余路径点。优化完成后输出最终路径点。
 *
 * @param trajs 二维数组，存储路径点及其相关信息
 * @param lineNum 输入的路径点总数
 *
 * @return 优化后路径点的总数
 */
int PK_PathMerge(float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const int lineNum,
                 const int slotid)
{
    static FILE *mergelog = NULL;
    // int merged = 1; // 合并标志，如果合并发生则为1
    int count = lineNum; // 当前路径点的数量
    if (lineNum >= MAX_SINGLE_TRAJ_NUM)
    {
        return count;
    }

    if (mergelog == NULL && is_save_log)
    {
#if defined(ANDROID_PLATFORM) || defined(SIMULATE)
        mergelog = fopen("./log/record_path.log", "w+");
#else
        mergelog = fopen("/tmp/apalog/record_path.log", "w+");
#endif
    }

    auto result1 = CheckTrajConnect(trajs, lineNum);
    float tempTrajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    int tempTrajsNum = count;
    memcpy(tempTrajs, trajs, lineNum * PATH_ITEM_LEN);

    count = PathPlan_PathMerge(trajs, count);

    auto result2 = CheckTrajConnect(trajs, count);

    PathPlan_UpdateTrajsLen(trajs, count);

    auto result3 = CheckTrajConnect(trajs, count);

    int record = 0;
    if (!result1)
    {
        printf("PK_PathMerge warning: %d, %d %d\n", slotid, result2, result3);
        record = 1;
    }

    while (result1 && (!result2 || !result3))
    {
        usleep(10000);
        printf("PK_PathMerge error: %d result2: %d result3:%d \n", slotid, result2,
               result3);
        record = 2;
    }

    if (mergelog != NULL && record != 0)
    {
        fprintf(mergelog, "pathnum start: %d %d\n", count, slotid);
        for (int i = 0; i < tempTrajsNum; i++)
        {
            fprintf(mergelog, "%02d:%06lf,%06lf,%06lf,%06lf,%06lf\n", i, tempTrajs[i][0],
                    tempTrajs[i][1], tempTrajs[i][2], tempTrajs[i][3], tempTrajs[i][4]);
        }

        fprintf(mergelog, "pathnum merge: %d %d %d %d\n", result1, result2, result3,
                count);
        for (int i = 0; i < count; i++)
        {
            fprintf(mergelog, "%02d:%06lf,%06lf,%06lf,%06lf,%06lf\n", i, trajs[i][0],
                    trajs[i][1], trajs[i][2], trajs[i][3], trajs[i][4]);
        }
    }

    if (record == 2)
    {
        count = tempTrajsNum;
        memcpy(trajs, tempTrajs, lineNum * PATH_ITEM_LEN);
    }

    return count;
}

/**
 * @brief 优化和合并路径点的函数
 *
 * 该函数通过多个步骤优化路径，包括移除过短路径、不必要的路径段和末端多余路径点。优化完成后输出最终路径点。
 *
 * @param trajs 二维数组，存储路径点及其相关信息
 * @param lineNum 输入的路径点总数
 *
 * @return 优化后路径点的总数
 */
static int PathOptimize(float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const int lineNum,
                        const uint32_t slotid, const int objNum, const float objs[][4])
{
    static FILE *mergelog = NULL;
    int count             = lineNum; // 当前路径点的数量

    if (mergelog == NULL && is_save_log)
    {
#if defined(ANDROID_PLATFORM) || defined(SIMULATE)
        mergelog = fopen("./log/record_path.log", "w+");
#else
        mergelog = fopen("/tmp/apalog/record_path.log", "w+");
#endif
    }

    if (mergelog != NULL)
    {
        fprintf(mergelog, "pathnum start: %d %d\n", count, slotid);
        for (int i = 0; i < count; i++)
        {
            fprintf(mergelog, "%02d:%06lf,%06lf,%06lf,%06lf,%06lf\n", i, trajs[i][0],
                    trajs[i][1], trajs[i][2], trajs[i][3], trajs[i][4]);
        }
    }

    count = PathPlan_Extend(trajs, count, objNum, objs);
    if (mergelog != NULL)
    {
        fprintf(mergelog, "pathnum optimize: %d\n", count);
        for (int i = 0; i < count; i++)
        {
            fprintf(mergelog, "%02d:%06lf,%06lf,%06lf,%06lf,%06lf\n", i, trajs[i][0],
                    trajs[i][1], trajs[i][2], trajs[i][3], trajs[i][4]);
        }
    }

    return count;
}
