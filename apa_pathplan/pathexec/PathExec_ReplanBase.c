#include "PK_Calibration.h"
#include "PK_ObjAvoid.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Config.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PathExec_Replan.h"
#include "PathExec_Debug.h"
#include "PK_PathPlan.h"
#include "PK_B.h"

const float para_ds_tramax = 1.0f; // Safe_dist

/**
 * @brief 反转轨迹方向
 *
 * 该函数用于反转给定轨迹的方向，使其从终点返回起点。
 *
 * @param traject 轨迹数组，包含起点位置、角度、曲率半径、路径长度等信息
 */
void PK_ReverseTraject(float traject[TRAJITEM_LEN])
{
    float x0, y0, theta0, r, ds; // 起点位置、角度、曲率半径、路径长度
    float Xr, Yr, ftheta;        // 临时变量

    x0     = traject[0];
    y0     = traject[1];
    theta0 = traject[2];
    r      = traject[4];
    ds     = traject[3];

    if (fabsf(r) < 1) // 如果曲率半径接近于0，即车辆直线行驶
    {
        traject[0] = cosf(theta0) * ds + x0; // 更新x坐标
        traject[1] = sinf(theta0) * ds + y0; // 更新y坐标
        traject[2] = theta0;                 // 角度保持不变
        traject[3] = -ds;                    // 路径长度取反，实现反向行驶
    }
    else // 如果曲率半径不接近于0，即车辆曲线行驶
    {
        ftheta     = -sign(r) * PI / 2 + theta0;            // 计算中心点角度
        Xr         = x0 - fabsf(r) * cosf(ftheta);          // 计算中心点x坐标
        Yr         = y0 - fabsf(r) * sinf(ftheta);          // 计算中心点y坐标
        traject[0] = cosf(ds / r + ftheta) * fabsf(r) + Xr; // 更新x坐标
        traject[1] = sinf(ds / r + ftheta) * fabsf(r) + Yr; // 更新y坐标
        traject[2] = theta0 + ds / r;                       // 更新角度
        traject[3] = -ds;                                   // 路径长度取反，实现反向行驶
        traject[4] = r;                                     // 曲率半径保持不变
    }
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
void PathExec_PathInverse(int tra_num, float Act_traj[][TRAJITEM_LEN])
{
    float temp_tra_end[5], temp_tra_start[5];

    for (int i = 0; i < tra_num; i++)
    {
        PK_ReverseTraject(Act_traj[i]);
    }

    int copy_counter = (tra_num % 2 == 0) ? tra_num / 2 : tra_num / 2 + 1;

    for (int i = 0; i < copy_counter; i++)
    {
        PK_CopyTra(temp_tra_start, Act_traj[i]);
        PK_CopyTra(temp_tra_end, Act_traj[tra_num - i - 1]);
        PK_CopyTra(Act_traj[i], temp_tra_end);
        PK_CopyTra(Act_traj[tra_num - i - 1], temp_tra_start);
        // PK_CopyTra(Reversed_traj[tra_num-i-1],Act_traj[i]);
    }
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
static int Check_IsFirstPathCloseToSlot(const float path[][TRAJITEM_LEN],
                                        const float Envi_obj_left[4],
                                        const float Envi_obj_right[4],
                                        const float min_arround_dist_1)
{
    float EndPos[3];
    float Envi_Obj_LR[2][4];
    memcpy(Envi_Obj_LR[0], Envi_obj_left, 4 * sizeof(float));
    memcpy(Envi_Obj_LR[1], Envi_obj_right, 4 * sizeof(float));
    PK_Get_Path_EndPos(path[0], EndPos);
    float min_arround_dist_2 = PK_PosObjCrossMinDist(EndPos, 2, Envi_Obj_LR);
    return (min_arround_dist_2 < min_arround_dist_1) ? 1 : 0;
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
static bool CircleInscribed(const float theta, const float yoffset, float r[2])
{
    float radius[]  = {0.5, 1.0, 2.0, 3.5, 6.0, 9.0, 13.0, 18.0};
    int try_num     = sizeof(radius) / sizeof(radius[0]);
    float costheta1 = cosf(theta);
    float costheta2 = cosf(theta * 0.3);
    for (int i = try_num; i > 0; i--)
    {
        r[0]         = radius[i - 1] + Rrmin;
        r[1]         = (yoffset - r[0] * (1 - costheta1)) / (1 - costheta2) + r[0];
        float cirlen = r[0] * 1.3 * fabs(theta);
        if (cirlen < 0.5 || cirlen > 2.5)
        {
            continue;
        }
        return true;
    }

    return false;
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
static bool CheckCircleTrajColliion(const float traj[TRAJITEM_LEN], float ckpoint[2])
{
    float temp[2] = {0.0, traj[4]};
    float center[2];
    Convert(traj, temp, center);
    return Cal_Dis_Pt2Pt(center, ckpoint);
}

/**
 * @brief 检查车辆在停车位内是否发生碰撞。
 *
 * 该函数通过计算车辆角点与环境障碍物之间的距离，检查车辆在停车位内是否发生碰撞。
 *
 * @param finpoint 最终点的坐标 [x, y, theta]。
 * @param enviObj 环境障碍物的线段表示 [x1, y1, x2, y2]。
 * @param ckpoint 检查点的车辆位置信息 [x, y, theta, 其他参数]。
 * @param dangerDist 碰撞的危险距离阈值。
 * @return bool 如果检测到碰撞，返回 true；否则返回 false。
 */
bool PathExec_CheckVehColliionInSlot(const float finpoint[3], const float enviObj[4],
                                     float ckpoint[4], float dangerDist)
{
    Rect_T cornerPt;                           // 车辆角点
    VehPos_T point;                            // 车辆位置
    memcpy(&point, ckpoint, sizeof(VehPos_T)); // 复制车辆位置数据到point
    Get_Veh_CornerPt(point, &cornerPt, 0.00);  // 获取车辆的四个角点

    float line[4];                       // 环境障碍物的线段表示
    memcpy(line, enviObj, sizeof(line)); // 复制环境障碍物的线段数据到line
    auto dist1 = PK_PointToLineDist(
        line, (float *)&cornerPt.pt_fl); // 计算前左角点到障碍物线段的距离
    auto dist2 = PK_PointToLineDist(
        line, (float *)&cornerPt.pt_rl);   // 计算后左角点到障碍物线段的距离
    auto leftdist = rte_min(dist1, dist2); // 取左侧最小距离

    dist1 = PK_PointToLineDist(
        line, (float *)&cornerPt.pt_fr); // 计算前右角点到障碍物线段的距离
    dist2 = PK_PointToLineDist(
        line, (float *)&cornerPt.pt_rr);    // 计算后右角点到障碍物线段的距离
    auto rightdist = rte_min(dist1, dist2); // 取右侧最小距离

    // 如果最小距离小于危险距离，则返回true表示碰撞
    if (rte_min(leftdist, rightdist) < dangerDist)
    {
        return true;
    }

    // 更新线段为车辆左侧边线段
    memcpy(&line[0], (float *)&cornerPt.pt_fl, sizeof(cornerPt.pt_fl));
    memcpy(&line[2], (float *)&cornerPt.pt_rl, sizeof(cornerPt.pt_rl));
    dist1 =
        PK_PointToLineDist(line, (float *)&enviObj[0]); // 计算障碍物点到左侧边线段的距离
    dist2 =
        PK_PointToLineDist(line, (float *)&enviObj[2]); // 计算障碍物点到左侧边线段的距离
    leftdist = rte_min(dist1, dist2);                   // 取左侧最小距离

    // 更新线段为车辆右侧边线段
    memcpy(&line[0], (float *)&cornerPt.pt_fr, sizeof(cornerPt.pt_fr));
    memcpy(&line[2], (float *)&cornerPt.pt_rr, sizeof(cornerPt.pt_rr));
    dist1 =
        PK_PointToLineDist(line, (float *)&enviObj[0]); // 计算障碍物点到右侧边线段的距离
    dist2 =
        PK_PointToLineDist(line, (float *)&enviObj[2]); // 计算障碍物点到右侧边线段的距离
    rightdist = rte_min(dist1, dist2);                  // 取右侧最小距离

    // 如果最小距离小于危险距离，则返回true表示碰撞
    if (rte_min(leftdist, rightdist) < dangerDist)
    {
        return true;
    }

    return false; // 无碰撞
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
static int DrawTrajectory(const float dsmin, const float traject[5],
                          float Position_arr[30][4], const float start_ds)
{
    int i;
    float step_th, start_th;
    float Xr, Yr, ftheta;
    float x0, y0, theta0, r, ds;
    int points;
    float floor_ds, remian_ds;
    float cos_theta0, sin_theta0, cos_ftheta, sin_ftheta; //  avoid repeat calculation

    x0     = traject[0];
    y0     = traject[1];
    theta0 = traject[2];
    r      = traject[4];
    ds     = traject[3] - start_ds;

    float dsminv = dsmin;
    floor_ds     = (float)(int)(fabsf(ds / dsminv));
    remian_ds    = fabsf(ds) - fabsf(floor_ds * dsminv);

    if (fabsf(remian_ds) < 0.005f)
    {
        points = (int)floor_ds;
    }
    else
    {
        points = 1 + (int)floor_ds;
    }

    if (fabsf(ds) <= fabsf(dsminv))
    {
        points = 1;
    }

    if (points > 30)
    {
        points = 30;
        dsminv = fabsf(ds / 30.0f);
    }

    ftheta     = -sign(r) * PI / 2 + theta0;
    cos_theta0 = cosf(theta0);
    sin_theta0 = sinf(theta0);
    cos_ftheta = cosf(ftheta);
    sin_ftheta = sinf(ftheta);
    if (fabsf(r) < 1) // line state
    {
        for (i = 0; i < points - 1; i++)
        {
            Position_arr[i][0] =
                (i + 1) * cos_theta0 * (dsminv + start_ds) * sign(ds) + x0;
            Position_arr[i][1] =
                (i + 1) * sin_theta0 * (dsminv + start_ds) * sign(ds) + y0;
            Position_arr[i][2] = theta0;
            // Position_arr[i][3]=(i+1)*dsmin*sign(ds)+start_ds;//big_bug fix 2018.4
            // (i+1)*(dsmin+ds)*sign(ds)
            Position_arr[i][3] =
                (i + 1) * (dsminv + start_ds) * sign(ds); // CSJ:  bug fix 2018.8.9
        }

        Position_arr[points - 1][0] = cos_theta0 * ds + x0;
        Position_arr[points - 1][1] = sin_theta0 * ds + y0;
        Position_arr[points - 1][2] = theta0;
        Position_arr[points - 1][3] = ds;
    }
    else
    {
        // dth>0 left
        Xr       = x0 - fabsf(r) * cos_ftheta;
        Yr       = y0 - fabsf(r) * sin_ftheta;
        step_th  = fabsf(dsminv / r) * sign(ds * r);
        start_th = fabsf(start_ds / r) * sign(ds * r);
        for (i = 0; i < points - 1; i++)
        {
            Position_arr[i][0] =
                cosf((i + 1) * step_th + start_th + ftheta) * fabsf(r) + Xr;
            Position_arr[i][1] =
                sinf((i + 1) * step_th + start_th + ftheta) * fabsf(r) + Yr;
            Position_arr[i][2] = (i + 1) * step_th + start_th + theta0;
            Position_arr[i][3] = (i + 1) * dsminv * sign(ds) + start_ds;
        }

        Position_arr[points - 1][0] = cosf((ds + start_ds) / r + ftheta) * fabsf(r) + Xr;
        Position_arr[points - 1][1] = sinf((ds + start_ds) / r + ftheta) * fabsf(r) + Yr;
        Position_arr[points - 1][2] = theta0 + (ds + start_ds) / r;
        Position_arr[points - 1][3] = ds + start_ds;
    }
    return points;
}

/**
 * @brief 连接探索路径
 *
 * 该函数尝试通过生成多个临时轨迹点，将起点与目标点连接，并生成可行的路径数组。
 *
 * @param temp_tra 起始轨迹数组
 * @param tarpos 目标位置数组
 * @param obj_num 环境对象数量
 * @param obj_slot 环境对象数组
 * @param stp_trajs 生成的步进轨迹数组
 * @param swell 路径膨胀系数
 * @param startDs 起始步长
 * @return int 返回生成的路径数量
 */
int ExplorerConnect(float temp_tra[TRAJITEM_LEN], const float tarpos[3],
                    const int obj_num, const float obj_slot[][4],
                    float stp_trajs[][TRAJITEM_LEN], float swell, float startDs)
{
    const float dsmin = 0.3f; // 生成步点的最小距离

    float tempTraj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]; // 临时轨迹数组
    static float explorposArr[30][4];                  // 存储生成的探索位置数组

    int lineNum = 0; // 路径数量

    // 确保起始步长方向与轨迹方向一致
    float signStartDs = sign(temp_tra[3]) * fabsf(startDs);
    float dsminv      = dsmin;                                       // 最小步长距离
    int number        = (fabs(temp_tra[3]) - fabs(startDs)) / dsmin; // 计算步数

    // 调整步长距离
    if (number < 7)
    {
        dsminv = 0.15;
    }
    else if (number < 4)
    {
        dsminv = 0.1;
    }

    // 绘制轨迹，生成探索位置数组
    int pointsNum = DrawTrajectory(dsminv, temp_tra, explorposArr, signStartDs);

    for (int j = 0; j < pointsNum; j++)
    {
        // 尝试通过探索位置与目标点生成路径
        lineNum = PathExec_BetterTra(explorposArr[j], tarpos, obj_num, obj_slot, tempTraj,
                                     swell);
        if (lineNum > 0)
        {
            // 如果找到可行路径，复制路径到步进轨迹数组
            memcpy(stp_trajs[1], tempTraj[0], lineNum * PATH_ITEM_LEN);
            stp_trajs[0][0] = temp_tra[0];
            stp_trajs[0][1] = temp_tra[1];
            stp_trajs[0][2] = temp_tra[2];
            stp_trajs[0][3] = explorposArr[j][3];
            stp_trajs[0][4] = temp_tra[4];
            lineNum         = lineNum + 1;
            temp_tra[3]     = explorposArr[j][3];
            break;
        }
    }

    return lineNum; // 返回生成的路径数量
}

/**
 * @brief 探索点到点的路径
 *
 * 该函数从起点位置到终点位置探索路径，并生成可行的路径数组，筛选最佳的路径。
 *
 * @param left_fac 方向因子，决定探索路径的方向
 * @param stpoint 起点位置数组
 * @param finpoint 终点位置数组
 * @param obj_num 环境对象数量
 * @param obj_slot 环境对象数组
 * @param swell 路径膨胀系数
 * @param planPath 生成的计划路径数组
 * @return int 返回生成的路径数量
 */
int ExplorePointToPoint(const float left_fac, const float stpoint[3],
                        const float finpoint[3], const int obj_num,
                        const float obj_slot[][4], float &swell,
                        float planPath[6][TRAJITEM_LEN])
{
    const int trycount = 5;                           // 尝试次数
    static float tempPath[trycount][6][TRAJITEM_LEN]; // 临时路径数组
    float Rr_array[trycount]     = {Rrmin + 0.3f, Rrmin + 0.8f, 0, -(Rrmin + 0.8f),
                                    -(Rrmin + 0.3f)}; // 曲率半径数组
    float tra_dist_sum[trycount] = {0};               // 路径距离总和数组
    int tra_num_temp[trycount]   = {0};               // 临时路径数量数组
    float temp_ds_sum            = 0;                 // 临时路径距离总和
    float tempTra[trycount]      = {0};               // 临时路径数组
    int SearchPath_num = 0, BestPath_index = -1,
        tra_num = 0;         // 搜索路径数量、最佳路径索引、路径数量
    float sideSlotObj[2][4]; // 环境对象数组
    memcpy(sideSlotObj[0], obj_slot[1], 4 * sizeof(float)); // 复制环境对象
    memcpy(sideSlotObj[1], obj_slot[3], 4 * sizeof(float)); // 复制环境对象
    PK_CopyPos(tempTra, stpoint);                           // 复制起点位置到临时路径数组

    // 根据方向因子调整曲率半径数组
    if (left_fac < 0)
    {
        for (int i = 0; i < 5; i++)
        {
            Rr_array[i] = -Rr_array[i];
        }
    }

    float checkSwell = 0.6; // 初始化检查膨胀系数
    do
    {
        SearchPath_num = 0;
        for (int i = 0; i < 5; i++) // 遍历曲率半径数组进行探索
        {
            tempTra[4] = Rr_array[i]; // 设置曲率半径
            tempTra[3] =
                2.5f; // 设置路径长度，this value is restricted by r and obstacles
            // 连接起点和终点生成路径
            tra_num_temp[i] = ExplorerConnect(tempTra, finpoint, obj_num, obj_slot,
                                              tempPath[i], checkSwell, 0.5);
            // 检查路径的有效性
            if (tra_num_temp[i] > 0 && tra_num_temp[i] < 7 &&
                PK_Check_Path(tra_num_temp[i], tempPath[i], stpoint, finpoint, 2,
                              sideSlotObj, checkSwell) == PATHPLAN_SUCCESS)
            {
                SearchPath_num++; // 增加搜索路径数量
                for (int j = 0; j < tra_num_temp[i]; j++)
                {
                    tra_dist_sum[i] =
                        tra_dist_sum[i] + fabsf(tempPath[i][j][3]); // 计算路径距离总和
                }
            }
            else
            {
                tra_num_temp[i] = 0; // 设置无效路径数量为0
            }
        }

        if (SearchPath_num > 0)
        {
            break;
        }                               // 如果找到有效路径则退出循环
        checkSwell = checkSwell - 0.05; // 减少膨胀系数
    } while (checkSwell > swell);

    if (SearchPath_num > 0)
    {
        printf("ExplorePointToPoint traj num %d, check swell %06f, swell %06f\n",
               SearchPath_num, checkSwell, swell);
        swell = checkSwell; // 更新膨胀系数
        for (int i = 0; i < 5; i++)
        {
            if (tra_num_temp[i] > 0)
            {
                if (temp_ds_sum < 0.01)
                {
                    temp_ds_sum    = tra_dist_sum[i]; // 更新最佳路径距离总和
                    BestPath_index = i;               // 更新最佳路径索引
                }
                else
                {
                    if (temp_ds_sum > tra_dist_sum[i])
                    {
                        temp_ds_sum    = tra_dist_sum[i]; // 更新最佳路径距离总和
                        BestPath_index = i;               // 更新最佳路径索引
                    }
                }
            }
        }
        memcpy(planPath, tempPath[BestPath_index],
               30 * sizeof(float));             // 复制最佳路径到计划路径数组
        tra_num = tra_num_temp[BestPath_index]; // 更新路径数量
    }
    else
    {
        tra_num = 0; // 如果未找到有效路径，返回0
    }

    return tra_num; // 返回生成的路径数量
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int ExplorePointToTra(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                      float dstTra[TRAJITEM_LEN], float enviObj[5][4],
                      const float curpos[4], const int slotshap)
{
    const float dsmin       = 0.3f; //   the minimun dist to generate a step point
    const float expectSwell = 0.6;
    float tempTraj[6][TRAJITEM_LEN];
    static float explorposArr[30][4]; // X,Y,THETA,ds

    int lineNum = 0;
    int number  = fabs(dstTra[3]) / dsmin;

    float dsminv = dsmin;
    if (number < 7)
    {
        dsminv = 0.15;
    }
    else if (number < 4)
    {
        dsminv = 0.1;
    }

    float checkSwell = 0.3;
    int pointsNum =
        DrawTrajectory(dsminv, dstTra, explorposArr,
                       0); // 20180619: maybe it's no need to calculate these points first
    for (int j = 0; j < pointsNum; j++)
    {
        float tempPaths[10][TRAJITEM_LEN];
        float swell = expectSwell;

        int trajNum = ExplorePointToPoint(1, curpos, explorposArr[j], 3, &enviObj[1],
                                          swell, tempPaths);
        if (trajNum > 0 && swell > checkSwell)
        {
            memcpy(tempTraj, tempPaths, PATH_ITEM_LEN * trajNum);
            lineNum    = trajNum;
            checkSwell = swell;
            if (checkSwell > expectSwell * 0.8)
            {
                break;
            }
        }
    }

    if (lineNum > 0)
    {
        memcpy(plannedPath, tempTraj, PATH_ITEM_LEN * lineNum);
    }
    return lineNum;
}

/**
 * @brief 探索路径至目标路径
 *
 * 该函数尝试从起点位置向目标位置探索路径，并生成路径到目标路径的过渡。
 *
 * @param plannedPath 计划路径数组
 * @param dstPos 目标位置数组
 * @param enviObj 环境对象数组
 * @param stPos 起点位置数组
 * @return int 返回生成的路径数量
 */
int ExploreTraToTra(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                    const float dstPos[3], const float enviObj[5][4],
                    const float stPos[3])
{
    const float rr_array[] = {0.6f,  1.2f,  2.5f, 5.0f,
                              10.0f, 20.0f, 0.0f};                   // 不同曲率半径的数组
    const int trycount     = sizeof(rr_array) / sizeof(rr_array[0]); // 尝试次数
    float ry = Project_PosTo1st_ry(dstPos, stPos); // 计算目标点到起点的y方向距离
    int dir  = sign(ry);                           // 确定方向

    float srcTra[TRAJITEM_LEN], srcEnd[3];             // 起点轨迹和临时终点数组
    float tempPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]; // 临时路径数组

    // 遍历不同曲率半径和距离的组合进行探索
    for (int r1idx = 0; r1idx < trycount; r1idx++)
    {
        float l1 = fabs(ry); // 距离初始化为目标点到起点的y方向距离
        while (l1 < 7.5)     // 控制探索距离不超过7.5
        {
            l1 = l1 + 0.5;             // 增加探索距离
            PK_CopyPos(srcTra, stPos); // 复制起点位置到轨迹数组
            srcTra[3] = -l1;           // 设置轨迹长度为负值，表示反向移动
            srcTra[4] = (rr_array[r1idx] < 0.01)
                            ? 0
                            : (rr_array[r1idx] + Rrmin) * dir; // 设置曲率半径，考虑方向
            PK_Get_Path_EndPos(srcTra, srcEnd);                // 计算轨迹的终点位置
            float ry1 =
                Project_PosTo1st_ry(dstPos, srcEnd); // 计算终点到目标点的y方向距离

            // 如果终点与目标点的y方向距离同号，则继续增加探索距离
            if (ry1 * ry > 0)
            {
                continue;
            }

            // 如果终点与目标点的y方向距离超出车辆宽度范围，或者距离过小，则跳过当前探索方案
            if (fabs(ry1) > VEHICLE_WID || fabs(ry1) < VEHICLE_WID * 0.3)
            {
                continue;
            }

            // 探索安全行驶距离
            PK_PathPlan_PathVerify(srcTra, 1, &enviObj[0], side_swell_1);
            PK_PathPlan_PathVerify(srcTra, 1, &enviObj[4], side_swell_1);

            PK_Get_Path_EndPos(srcTra, srcEnd);        // 重新计算轨迹的终点位置
            ry1 = Project_PosTo1st_ry(dstPos, srcEnd); // 重新计算终点到目标点的y方向距离
            if (ry1 * ry > 0)
            {
                continue;
            }
            if (fabs(ry1) > VEHICLE_WID || fabs(ry1) < VEHICLE_WID * 0.3)
            {
                continue;
            }
            break;
        }

        // 如果探索距离达到上限，继续下一组合的探索
        if (l1 >= 7.4)
        {
            continue;
        }

        // 获取新的终点位置
        PK_Get_Path_EndPos(srcTra, srcEnd);
        float checkSwell = side_swell_0; // 初始化检查膨胀系数
        // 进行探索并生成过渡路径
        int trajNum =
            ExplorePointToPoint(dir, srcEnd, dstPos, 5, enviObj, checkSwell, tempPath);
        if (trajNum <= 0)
        {
            continue;
        }

        // 如果生成的过渡路径安全距离大于阈值，则更新计划路径
        if (checkSwell > side_swell_1)
        {
            PK_CopyTra(plannedPath[0], srcTra);
            memcpy(&plannedPath[1], tempPath, PATH_ITEM_LEN * trajNum);
            return trajNum + 1;
        }
        else
        {
            int idx = 0;
            while (idx < trajNum)
            {
                float rtheta = Project_PosTo1st_rtheta(
                    dstPos, tempPath[idx]); // 计算路径与目标点的角度差
                // 如果角度差绝对值超过30度，则更新路径
                if (fabs(Round_PI(rtheta)) > 30 * PI_RAD) // 有疑问？ 30度是不是有点小
                {
                    PK_Get_Path_EndPos(tempPath[idx], srcEnd); // 获取路径的终点位置
                    // 检查路径在前方和后方环境中的有效性
                    if (PK_Check_Path(1, &tempPath[idx], tempPath[idx], srcEnd, 1,
                                      &enviObj[0], side_swell_1) != PATHPLAN_SUCCESS)
                    {
                        break;
                    }
                    if (PK_Check_Path(1, &tempPath[idx], tempPath[idx], srcEnd, 1,
                                      &enviObj[4], side_swell_1) != PATHPLAN_SUCCESS)
                    {
                        break;
                    }
                }
                idx++;
            }

            // 如果路径有效性检查通过，则更新计划路径
            if (idx >= trajNum)
            {
                PK_CopyTra(plannedPath[0], srcTra);
                memcpy(&plannedPath[1], tempPath, PATH_ITEM_LEN * trajNum);
                return trajNum + 1;
            }
        }
    }

    return 0; // 如果未能生成路径，返回0
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int PathExec_BetterTra(const float stpoint[3], const float finpoint[3], const int objNum,
                       const float obj_slot[][4], float traject[][TRAJITEM_LEN],
                       const float swell, const float radiusSwell)
{
    return PK_PathPlan_BetterTra(stpoint, finpoint, objNum, obj_slot, traject, swell,
                                 radiusSwell);
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int PathExec_BetterTraFront(const float stpoint[3], const float finpoint[3],
                            const int objNum, const float objSlot[][4],
                            float traject[][TRAJITEM_LEN])
{
    int trajNum = PK_PathPlan_BetterTra(finpoint, stpoint, objNum, objSlot, traject,
                                        CONFIG_SIDE_SWELL_2, 0.1);
    if (trajNum > 0)
    {
        PathExec_PathInverse(trajNum, traject);
    }
    return trajNum;
}

// 20181105: the first path is to move out of the slot
/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int PathExec_B_MoveOutFront(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            const float finpoint[3], const float enviObj[5][4],
                            const float curpos[4], const int slotshap)
{
    printf("----------PathExec_B_MoveOutFront -------------------------\r\n");
    const float min_line_tra      = 0.5f;
    const float Rr                = Rrmin + 0.15f; // G3 4.15f;
    const uint8 ExpMoveOutCnt_Max = 2;
    const float Narrow_Side_dist  = 0.35f;
    const float vert_swell_min    = 0.15f;
    const float MoveOut_Dist_base = 3.5f; //  first move out the slot along a line
    const float Circle_Dist_max   = 2.5f; //  second move out along a circle path
    const int max_traj_num        = 10;

    float cur_swell = 0.0f;
    float Envi_obj_left[4], Envi_obj_right[4];
    float left_fac = 0;
    float temploc[3];
    float Envi_obj_side[2][4];
    float Planned_Path_Out[max_traj_num][TRAJITEM_LEN] = {0};
    float Targ_SlotOut_rx;

    temploc[0] = finpoint[0];
    temploc[1] = finpoint[1];
    temploc[2] = finpoint[2] + PI_2;

    SlotObj_T slotobj;
    float oldfinpos[3];
    PK_CopyPos(oldfinpos, finpoint);
    SlotObj_Convert_struct(&slotobj, enviObj);

    // PathExec_UpdateSlotTarget(finpoint, enviObj, objs, 2, slotshap);
    PathExec_UpdateObsObj_AroundTargSlot(); //  Calculate once

    if (PK_PosObjCross(temploc, 1, &enviObj[1], 10.0f, 0) > 0)
    {
        memcpy(Envi_obj_left, &enviObj[1], sizeof(Envi_obj_left));
        memcpy(Envi_obj_right, &enviObj[3], sizeof(Envi_obj_right));
    }
    else
    {
        memcpy(Envi_obj_left, &enviObj[3], sizeof(Envi_obj_left));
        memcpy(Envi_obj_right, &enviObj[1], sizeof(Envi_obj_right));
    }

    memcpy(Envi_obj_side[0], enviObj[1], 4 * sizeof(float));
    memcpy(Envi_obj_side[1], enviObj[3], 4 * sizeof(float));
    float min_arround_dist_left  = PK_PosObjCrossMinDist(curpos, 1, &Envi_obj_left);
    float min_arround_dist_right = PK_PosObjCrossMinDist(curpos, 1, &Envi_obj_right);

    if (min_arround_dist_left < min_arround_dist_right)
    {
        left_fac        = 1.0f;
        Targ_SlotOut_rx = rte_max(Project_PosTo1st_rx(curpos, Envi_obj_left),
                                  Project_PosTo1st_rx(curpos, &Envi_obj_left[2]));
    }
    else
    {
        left_fac        = -1.0f;
        Targ_SlotOut_rx = rte_max(Project_PosTo1st_rx(curpos, Envi_obj_right),
                                  Project_PosTo1st_rx(curpos, &Envi_obj_right[2]));
    }
    float min_arround_dist = rte_min(min_arround_dist_left, min_arround_dist_right);

    if (min_arround_dist < Narrow_Side_dist)
    {
        cur_swell = rte_min(rte_max(min_arround_dist - 0.1f, vert_swell_min),
                            g_local_swell); //  20180930:single side swell
    }
    else
    {
        cur_swell = rte_min(0.65f * min_arround_dist, g_local_swell);
    }

    cur_swell = rte_min(cur_swell, 0.3);
    // first try left and right forward turning exploring method
    float checkSwell = 0.3;
    float tempPaths[10][TRAJITEM_LEN];
    int tra_num_temp = ExplorePointToPoint(left_fac, curpos, finpoint, 5, enviObj,
                                           checkSwell, tempPaths);
    if (tra_num_temp > 0)
    {
        int result = PathExec_Replan_PathCheck(tra_num_temp, tempPaths, min_arround_dist,
                                               checkSwell, finpoint, Envi_obj_left,
                                               Envi_obj_right, Envi_obj_side);
        if (result > 0)
        {
            cur_swell = checkSwell;
            memcpy(Planned_Path_Out, tempPaths, sizeof(tempPaths));
            printf(
                "ExplorePointToPoint success! traj num %d swell %06f start: %06f %06f "
                "%06f end: %06f %06f %06f\n",
                tra_num_temp, cur_swell, curpos[0], curpos[1], curpos[2], finpoint[0],
                finpoint[1], finpoint[2]);
        }
        else
        {
            tra_num_temp = 0;
        }
    }

    float rx_cur_to_fin = Project_PosTo1st_rx(finpoint, curpos);
    for (int i = 0; i < ExpMoveOutCnt_Max; i++)
    {
        float lineTraLen = 0.0f;
        if (min_arround_dist < Narrow_Side_dist)
        {
            if (i == 0)
            {
                lineTraLen =
                    rte_max(2.0f * min_line_tra,
                            MoveOut_Dist_base -
                                rx_cur_to_fin); // 0831: if the vehicle move deep to slot
                                                // then it will need to move forward more
                                                // dist to do a replan
            }
            else
            {
                lineTraLen =
                    rte_min(MoveOut_Dist_base,
                            rte_max(min_line_tra * 4.0f,
                                    Targ_SlotOut_rx)); // 20181026: for slotobj stand out
            }
        }
        else
        {
            lineTraLen = (i == 0) ? min_line_tra : 2.0f * min_line_tra;
        }

        float tempLineTraj[TRAJITEM_LEN];
        float tempcirTra[TRAJITEM_LEN];

        PK_CopyPos(tempLineTraj, curpos); // 0831: should consider surrounding obstacles
        tempLineTraj[3] = lineTraLen;
        tempLineTraj[4] = 0;
        PK_Get_Path_EndPos(tempLineTraj, tempcirTra); // 2.line forward to  temp_pos
        tempcirTra[3] = Circle_Dist_max;
        tempcirTra[4] = left_fac * Rr;

        checkSwell = 0.6;
        while (checkSwell > cur_swell)
        {
            int traNum = ExplorerConnect(tempcirTra, finpoint, 0, enviObj, &tempPaths[1],
                                         checkSwell, min_line_tra);
            if (traNum > 0) // plan success
            {
                memcpy(tempPaths[0], tempLineTraj, sizeof(tempLineTraj));
                traNum     = traNum + 1;
                int result = PathExec_Replan_PathCheck(
                    traNum, tempPaths, min_arround_dist, checkSwell, finpoint,
                    Envi_obj_left, Envi_obj_right, Envi_obj_side);
                if (result > 0)
                {
                    printf(
                        "line circle success success! traj num %d swell %06f start: %06f "
                        "%06f %06f end: %06f %06f %06f\n",
                        tra_num_temp, checkSwell, curpos[0], curpos[1], curpos[2],
                        finpoint[0], finpoint[1], finpoint[2]);
                    cur_swell = checkSwell;
                    memcpy(Planned_Path_Out, tempPaths, traNum * PATH_ITEM_LEN);
                    tra_num_temp = traNum;
                    break;
                }
            }
            checkSwell = checkSwell - 0.05;
        }
    }

    if (tra_num_temp > 0)
    {
        tra_num_temp = PK_PathMerge(Planned_Path_Out, tra_num_temp, 0);
        memcpy(plannedPath, Planned_Path_Out, sizeof(Planned_Path_Out));
    }

    return tra_num_temp;
}

// path check for replan: 1-valid; 0-collision;
/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int PathExec_Replan_PathCheck(
    const int tra_num_temp,
    const float Planned_Path_Out[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
    const float min_arround_dist, const float cur_swell, const float finpoint[3],
    const float Envi_obj_left[4], const float Envi_obj_right[4],
    const float Envi_obj_side[2][4])
{
    int IsFirstPathCloseToSlot;
    float stpos[3];
    int SlotObjCheckRst = 0;
    int CheckRst        = 1;

    IsFirstPathCloseToSlot = Check_IsFirstPathCloseToSlot(
        Planned_Path_Out, Envi_obj_left, Envi_obj_right, min_arround_dist);
    if (IsFirstPathCloseToSlot == 0)
    {
        memcpy(stpos, Planned_Path_Out[1], sizeof(stpos));
        SlotObjCheckRst = PK_Check_Path(tra_num_temp - 1, &Planned_Path_Out[1], stpos,
                                        finpoint, 2, Envi_obj_side, cur_swell);
    }
    else
    {
        memcpy(stpos, Planned_Path_Out[0], sizeof(stpos));
        SlotObjCheckRst = PK_Check_Path(tra_num_temp, Planned_Path_Out, stpos, finpoint,
                                        2, Envi_obj_side, cur_swell);
    }

    if (SlotObjCheckRst == PATHPLAN_SUCCESS)
    {
        memcpy(stpos, Planned_Path_Out[0], sizeof(stpos)); //  the entire path
        FusionObj_T anotherSideObj;
        PathExec_GetObsObj_AnotherSide(anotherSideObj);
        for (int i = 0; i < anotherSideObj.num; i++)
        {
            float enviObj[4];
            memcpy(enviObj, (float *)&anotherSideObj.obj[i], sizeof(enviObj));
            if (PK_Check_Path(tra_num_temp, Planned_Path_Out, stpos, finpoint, 1,
                              &enviObj, cur_swell) !=
                PATHPLAN_SUCCESS) //  0912:add environmnet obs check
            {
                CheckRst = 0;
                break;
            }
        }
    }
    else
    {
        CheckRst = 0; // csj 0907: if collison should set to zero
    }

    return CheckRst;
}
