#include "PathPlan_Func.h"
#include "PathPlan_Tools.h"
#include "PK_PathPlanP.h"
#include "SystemPara.h"
#include "Rte.h"

// 检查规划路径的方向, 防止产生不合理路径
static int CheckTrajTheta(const float traject[TRAJITEM_LEN], const float targpos[3])
{
    // 路径不可能和目标方向相反
    float thetaDiv = Round_PI(targpos[2] - traject[2]);
    if (fabs(thetaDiv) > 0.80f * PI)
    {
        return PATHPLAN_SUCCESS + 1;
    }

    // 直线或者短路径只需要校验起点
    if (fabs(traject[3]) < 3.0f || fabs(traject[4]) < 1.0f)
    {
        return PATHPLAN_SUCCESS;
    }

    // 校验中间点
    float temp[TRAJITEM_LEN], endPos[3];
    PK_CopyTra(temp, traject);
    float trajLen = 0.5f;
    while (trajLen < fabs(traject[3]))
    {
        temp[3] = trajLen * sign(traject[3]);
        PK_Get_Path_EndPos(temp, endPos);
        float thetaDiv = Round_PI(targpos[2] - temp[2]);
        if (fabs(thetaDiv) > 0.80f * PI)
        {
            return PATHPLAN_SUCCESS + 1;
        }
        trajLen = trajLen + 0.5f;
    }

    return 0;
}

static int CheckMultiShotPath(const float traject[][TRAJITEM_LEN], int pathNum)
{
    // 在直行本检查之前已经消除了非常小的短路径
    float lastLen = traject[0][3];
    uint8_t count = 0;
    for (int i = 1; i < pathNum; i++)
    {
        // 方向相反, 且路径很短
        if (lastLen * traject[i][3] > ZERO_FLOAT ||
            fabs(traject[i][3]) > PathPlanConstCfg::short_path_len ||
            fabs(lastLen) > PathPlanConstCfg::short_path_len)
        {
            count = 0;
        }
        else
        {
            count = count + 1;
        }

        lastLen = traject[i][3];
        if (count > 3)
        {
            return i;
        }
    }

    return 0;
}

// 对规划算法输出路径做全面检查, 消除不合理规划
int PathPlan_CheckPlannedPathValid(const float traject[][TRAJITEM_LEN], int pathNum)
{
#define CheckAction(errCode)                                                       \
    {                                                                              \
        while (errCode != PATHPLAN_SUCCESS)                                        \
        {                                                                          \
            printf("CheckPathValid line:%s %d %d\n", __FILE__, __LINE__, errCode); \
            usleep(1000);                                                          \
        }                                                                          \
    }
#define ExitAction(errCode)              \
    {                                    \
        if (errCode != PATHPLAN_SUCCESS) \
        {                                \
            return errCode;              \
        }                                \
    }

    float trajEndPos[3] = {0};
    PK_Get_Path_EndPos(traject[pathNum - 1], trajEndPos);

    for (int i = 0; i < pathNum; i++)
    {
        // 检查起点终点和长度
        auto checkResult1 = PathPlan_CheckTrajValid(traject[i]);
        CheckAction(checkResult1);
        ExitAction(checkResult1);

        // 检查方向
        auto checkResult2 = CheckTrajTheta(traject[i], trajEndPos);
        ExitAction(checkResult2);
    }

    // 检查连接性
    auto check1 = PK_Check_Path(pathNum, traject, traject[0], trajEndPos, 0, NULL, 0);
    // CheckAction(check1);
    // ExitAction(check1);

    // 检查原地反复往复
    // auto check2 = CheckMultiShotPath(traject, pathNum);
    // ExitAction(check2);
    return PATHPLAN_SUCCESS;
}

int PathPlan_CheckPathOverFlow(const float *curPlannedPath)
{
    uint32_t *flag = (uint32_t *)(curPlannedPath +
                                  MAX_SINGLE_TRAJ_NUM * PATH_ITEM_LEN / sizeof(float));
    return (flag[0] + flag[1] + flag[2] + flag[3] + flag[4] + flag[5] != 0) ? 1 : 0;
}

// 检查路径的起点终点和长度姿态
int PathPlan_CheckTrajValid(const float traject[TRAJITEM_LEN])
{
    if (fpclassify(traject[0]) == FP_NAN || fpclassify(traject[0]) == FP_INFINITE)
    {
        return PATHPLAN_SUCCESS + 1;
    }
    if (fpclassify(traject[1]) == FP_NAN || fpclassify(traject[1]) == FP_INFINITE)
    {
        return PATHPLAN_SUCCESS + 2;
    }
    if (fpclassify(traject[2]) == FP_NAN || fpclassify(traject[2]) == FP_INFINITE)
    {
        return PATHPLAN_SUCCESS + 3;
    }
    if (fpclassify(traject[3]) == FP_NAN || fpclassify(traject[3]) == FP_INFINITE)
    {
        return PATHPLAN_SUCCESS + 4;
    }
    if (fpclassify(traject[4]) == FP_NAN || fpclassify(traject[4]) == FP_INFINITE)
    {
        return PATHPLAN_SUCCESS + 5;
    }
    if (fabs(traject[0]) > 60000.0f)
    {
        return PATHPLAN_SUCCESS + 6;
    }
    if (fabs(traject[1]) > 60000.0f)
    {
        return PATHPLAN_SUCCESS + 7;
    }
    if (fabs(traject[2]) > 60000.0f)
    {
        return PATHPLAN_SUCCESS + 8;
    }
    if (fabs(traject[3]) > 60000.0f)
    {
        return PATHPLAN_SUCCESS + 9;
    }
    if (fabs(traject[4]) > 60000.0f)
    {
        return PATHPLAN_SUCCESS + 10;
    }

    if (fabs(traject[3]) > PATH_LENGTH_MAX + 0.001f && fabs(traject[4]) > 0 &&
        fabs(traject[4]) < 2 * Rrmin)
    {
        PathPlan_CheckTrajLen("too long line1!", traject[3]);
    }

    if (fabs(traject[3]) > 2 * PATH_LENGTH_MAX + 0.001f && fabs(traject[4]) < 0.001f)
    {
        PathPlan_CheckTrajLen("too long line2!", traject[3]);
    }

    return PATHPLAN_SUCCESS;
}

/***********************************************
add2023.2.29 Tangsj
检查路径节点是否一致 (前一节点计算结果与后一节点是否一致)
Check_Traject
INTPUT:     检测的两个路径节点
OUTPUT:     0-不通过 1-通过
***********************************************/
int PathPlan_CheckPosDiv(const float oldPos[3], const float newPos[3])
{
    float ydiv = Project_PosTo1st_ry(oldPos, newPos);
    if (fabsf(ydiv) > PathPlanConstCfg::div_err_ry_fin)
    {
        return 1;
    }

    float xdiv = Project_PosTo1st_rx(oldPos, newPos);
    if (xdiv > PathPlanConstCfg::div_err)
    {
        return 2;
    }

    float dist = Cal_Dis_Pt2Pt(oldPos, newPos);
    if (dist >= 1.2f * PathPlanConstCfg::div_err)
    {
        return 4;
    }

    return 0;
}

void PathPlan_CheckTrajLen(const char *info, const float length)
{
    // printf("---------------- check traj length: %s length(%f): %f
    // --------------------\n", info, PATH_LENGTH_MAX, length);
}

/*********************************************************************
Function description : PathPlan_ReverseTraject-->> reverse the input path

Calibration state    : NO -->>
Edition              : 2016/3/6  1.0 ：

Input list:
traject：the trajectory to be reversed
Output list:
**********************************************************************/
void PathPlan_ReverseTraject(float traject[TRAJITEM_LEN])
{
    float x0, y0, theta0, r, ds;
    float Xr, Yr, ftheta;
    x0     = traject[0];
    y0     = traject[1];
    theta0 = traject[2];
    r      = traject[4];
    ds     = traject[3];
    if (fabsf(r) < 1)
    {
        traject[0] = cosf(theta0) * ds + x0;
        traject[1] = sinf(theta0) * ds + y0;
        traject[2] = theta0;
        traject[3] = -ds;
    }
    else
    {
        ftheta     = -sign(r) * PI / 2 + theta0;
        Xr         = x0 - fabsf(r) * cosf(ftheta);
        Yr         = y0 - fabsf(r) * sinf(ftheta);
        traject[0] = cosf(ds / r + ftheta) * fabsf(r) + Xr;
        traject[1] = sinf(ds / r + ftheta) * fabsf(r) + Yr;
        traject[2] = theta0 + ds / r;
        traject[3] = -ds;
    }
}

/***************************************************************************************************************************优化添加*/
/*
功能描述：计算揉库次数
函数名称：Get_Kneading_Times
INTPUT:     路径TempPath,路径节点数line_num
OUTPUT:     返回揉库次数
*/
int Get_Kneading_Times(float TempPath[][TRAJITEM_LEN], float line_num)
{
    if (line_num == 0)
    {
        return 0;
    }

    int kneading_times = 0;
    for (int i = 0; i < line_num - 1; i++)
    {
        if (sign(TempPath[i + 1][3]) != 0 &&
            sign(TempPath[i + 1][3]) != sign(TempPath[i][3]))
        {
            kneading_times++;
        }
    }

    return kneading_times;
}

/*********************************************************************
Function description : PathPlan_PathVerify-->>check the path from its stpos and update the
ds for the path when it cross the objects

Calibration state    : NO -->>
Edition              : 2016/5/6  1.0

Input list:
traject：the entire path
obj_num：the number of objects
obj：objects near slot

Output list:
return: B_cross 1：collision, 0： no collision, -1: fail;
**********************************************************************/
int PK_PathVerify(float traject[TRAJITEM_LEN], const int obj_num, const float obj[][4],
                  const float swell)
{
    float oldds = traject[3];
    float newds = PathPlan_LandMark2(traject, obj_num, obj, swell);

    if (newds < EXPLORE_DIST_MIN)
    {
        return -1;
    }
    else if (fabs(oldds - newds) < ZERO_FLOAT)
    {
        return 0;
    }

    return 1;
}

/*
add 2023/8/19 Tangsj
功能描述：检查路径连接
函数名称：Path_Check_Connect
INTPUT:     路径终点finpoint,路径TempPath,路径节点数line_num
OUTPUT:     成功-1，失败-0
*/
int Path_Check_Connect(const float stpoint[3], const float finpoint[3],
                       float TempPath[][TRAJITEM_LEN], int line_num)
{
    float EndPos[3], dth, dPowPos;
    const float dPowPos_max = 0.10f; // 2cm
    const float dth_max     = 0.02f; // 0.02 rad

    if (line_num == 0)
    {
        return 0;
    }

    if (fabs(stpoint[0] - TempPath[0][0]) > 0.001 ||
        fabs(stpoint[1] - TempPath[0][1]) > 0.001f ||
        fabs(stpoint[2] - TempPath[0][2]) > 0.001f)
    {
        return 0;
    }

    for (int i = 0; i < line_num - 1; i++)
    {
        PK_Get_Path_EndPos(TempPath[i], EndPos);
        dPowPos =
            pow2(EndPos[0] - TempPath[i + 1][0]) + pow2(EndPos[1] - TempPath[i + 1][1]);
        dth = Round_PI(EndPos[2] - TempPath[i + 1][2]);

        if (dPowPos > dPowPos_max || fabsf(dth) > dth_max)
        {
            return 0;
        }

        float tempTraj[TRAJITEM_LEN];
        PK_CopyTra(tempTraj, TempPath[i]);
        // PathPlan_CheckTrajLen(tempTraj, finpoint);
        // int testprint = 1;
        // if (fabs(tempTraj[3]) < fabs(TempPath[i][3]) + 0.01f && testprint > 0)
        {
            // printf("------------------- check traj failed!
            // -------------------------\n"); return 0;
        }
    }

    PK_Get_Path_EndPos(TempPath[line_num - 1], EndPos);
    dPowPos = pow2(EndPos[0] - finpoint[0]) + pow2(EndPos[1] - finpoint[1]);
    dth     = Round_PI(EndPos[2] - finpoint[2]);

    if (dPowPos > dPowPos_max || fabsf(dth) > dth_max)
    {
        return 0;
    }

    return 1;
}

int PathPlan_SlotDir(const int shapeType)
{
    if (shapeType == PK_SLOT_LEFT_PARA || shapeType == PK_SLOT_LEFT_VERT ||
        shapeType == PK_SLOT_LEFT_ANG_FORWARD || shapeType == PK_SLOT_LEFT_ANG_REVERSE)
    {
        return 1;
    }

    if (shapeType == PK_SLOT_RIGHT_PARA || shapeType == PK_SLOT_RIGHT_VERT ||
        shapeType == PK_SLOT_RIGHT_ANG_FORWARD || shapeType == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        return 2;
    }

    return 0;
}

// 截取线段函数
void PathPlan_ClipLineSegmentx(float pt1[2], float pt2[2], float limit, float dir)
{
    // 检查两个端点是否都在xlimit右侧
    if (pt1[0] <= limit && pt2[0] <= limit && dir < 0.1f)
    {
        pt1[0] = pt2[0];
        pt1[1] = pt2[1];
        return;
    }

    if (pt1[0] > limit && pt2[0] > limit && dir > 0.1f)
    {
        pt1[0] = pt2[0];
        pt1[1] = pt2[1];
        return;
    }

    // 检查两个端点是否都在xlimit左侧
    if (pt1[0] >= limit && pt2[0] >= limit && dir < 0.1f)
    {
        return; // 无需截取
    }

    // 检查两个端点是否都在xlimit左侧
    if (pt1[0] <= limit && pt2[0] <= limit && dir > 0.1f)
    {
        return; // 无需截取
    }

    // 计算斜率
    double slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0]);

    // 计算交点y坐标
    float crossPt[2];
    crossPt[0] = limit;
    crossPt[1] = pt1[1] + slope * (limit - pt1[0]);

    // 替换x坐标大于x_max的端点
    if (dir < 0.1f)
    {
        if (pt1[0] < limit + 0.001f)
        {
            pt1[0] = crossPt[0];
            pt1[1] = crossPt[1];
        }
        else if (pt2[0] < limit + 0.001f)
        {
            pt2[0] = crossPt[0];
            pt2[1] = crossPt[1];
        }
    }
    else
    {
        if (pt1[0] > limit - 0.001f)
        {
            pt1[0] = crossPt[0];
            pt1[1] = crossPt[1];
        }
        else if (pt2[0] > limit - 0.001f)
        {
            pt2[0] = crossPt[0];
            pt2[1] = crossPt[1];
        }
    }
}

// 截取线段函数
void PathPlan_ClipLineSegmenty(float pt1[2], float pt2[2], float limit, float dir)
{
    // 检查两个端点是否都在xlimit右侧
    if (pt1[1] <= limit && pt2[1] <= limit && dir < 0.1f)
    {
        pt1[0] = pt2[0];
        pt1[1] = pt2[1];
        return;
    }

    if (pt1[1] > limit && pt2[1] > limit && dir > 0.1f)
    {
        pt1[0] = pt2[0];
        pt1[1] = pt2[1];
        return;
    }

    // 检查两个端点是否都在xlimit左侧
    if (pt1[1] >= limit && pt2[1] >= limit && dir < 0.1f)
    {
        return; // 无需截取
    }

    // 检查两个端点是否都在xlimit左侧
    if (pt1[1] <= limit && pt2[1] <= limit && dir > 0.1f)
    {
        return; // 无需截取
    }

    // 计算斜率
    double slope = (pt2[0] - pt1[0]) / (pt2[1] - pt1[1]);

    // 计算交点x坐标
    float crossPt[2];
    crossPt[0] = pt1[0] + slope * (limit - pt1[1]);
    crossPt[1] = limit;

    // 替换x坐标大于x_max的端点
    if (dir < 0.1f)
    {
        if (pt1[1] < limit + 0.001f)
        {
            pt1[0] = crossPt[0];
            pt1[1] = crossPt[1];
        }
        else if (pt2[1] < limit + 0.001f)
        {
            pt2[0] = crossPt[0];
            pt2[1] = crossPt[1];
        }
    }
    else
    {
        if (pt1[0] > limit - 0.001f)
        {
            pt1[0] = crossPt[0];
            pt1[1] = crossPt[1];
        }
        else if (pt2[0] > limit - 0.001f)
        {
            pt2[0] = crossPt[0];
            pt2[1] = crossPt[1];
        }
    }
}
