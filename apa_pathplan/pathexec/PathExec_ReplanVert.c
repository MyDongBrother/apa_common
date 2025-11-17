#include "PK_ObjAvoid.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PathExec_Debug.h"
#include "PathExec_Replan.h"
#include "PK_B.h"

/**
 * @brief 在停车位内重新规划垂直方向的轨迹。
 *
 * 该函数尝试为车辆在给定的停车位内重新定位规划路径。它通过迭代尝试不同的半径来生成可行的轨迹，
 * 避免碰撞并满足位置约束。
 *
 * @param plannedPath 2D数组，用于存储规划的轨迹。
 * @param finpoint 最终点的坐标 [x, y, theta]。
 * @param enviObj 环境物体表示为障碍物的数组，每个障碍物由4个浮点数组成。
 * @param stpoint 起始点的坐标 [x, y, theta, 其他参数]。
 * @return int 返回值为轨迹点的数量，如果未找到有效轨迹则返回0。
 */
static int VertRearReplanInSlot(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                const float finpoint[3], const float enviObj[5][4],
                                const float stpoint[4])
{
    float radius[]  = {0.3, 0.5, 0.8, 1.2, 1.8,
                       2.5, 3.5, 5.0, 8.0, 13.0};         // 尝试的半径值数组
    int try_num     = sizeof(radius) / sizeof(radius[0]); // 半径值的数量
    int trajNum     = -1;                                 // 轨迹点数量
    float curradius = 20.0;                               // 当前半径

    float tempTra[TRAJITEM_LEN];                               // 临时轨迹点数组
    float newTraj[10][TRAJITEM_LEN];                           // 新轨迹数组
    float rtheta = Project_PosTo1st_rtheta(finpoint, stpoint); // 计算相对角度
    float ry     = Project_PosTo1st_ry(finpoint, stpoint);     // 计算相对y坐标
    float rx     = Project_PosTo1st_rx(finpoint, stpoint);     // 计算相对x坐标

    float cosrtheta = cosf(rtheta);       // 相对角度的余弦值
    float sinrtheta = fabs(sinf(rtheta)); // 相对角度的正弦绝对值

    // 如果角度和y坐标的乘积大于0且y坐标的正弦绝对值小于3.0
    if (rtheta * ry > ZERO_FLOAT && fabs(ry / sinrtheta) < 3.0)
    {
        float l = 0.0;  // 初始距离
        while (l < 3.0) // 循环尝试不同的l值
        {
            float ryl    = ry - l * sinrtheta;             // 计算当前yl值
            float len    = ryl * rtheta / (1 - cosrtheta); // 计算路径长度
            float radius = ryl / (1 - cosrtheta);          // 计算路径半径
            if (fabs(radius) < Rrmin + 0.5)
            {
                break;
            }                                               // 如果半径过小则退出
            if (fabs(ryl / sinrtheta) + l * cosrtheta < rx) // 检查路径是否可行
            {
                PK_CopyTra(newTraj[0], stpoint); // 复制起点轨迹
                newTraj[0][3] = -l;              // 设置路径距离
                newTraj[0][4] = 0;
                PK_Get_Path_EndPos(newTraj[0], newTraj[1]); // 获取路径终点
                newTraj[1][3] = -fabs(len);                 // 设置路径距离
                newTraj[1][4] = sign(ry) * fabs(radius);
                PK_Get_Path_EndPos(newTraj[1], newTraj[2]); // 获取路径终点
                float rx1 =
                    Project_PosTo1st_rx(finpoint, newTraj[2]); // 计算新的相对x坐标
#if 0
                if (rx1 > 0.1) // 检查路径有效性
#else
                float rtheta1 = Project_PosTo1st_rtheta(finpoint, newTraj[2]);
                if (rx1 > 0.3 && fabs(rtheta1) < 2 * PI_RAD) // 检查路径有效性
#endif
                {
                    newTraj[2][3] = -rx1; // 设置最终调整参数
                    newTraj[2][4] = 0;
                    memcpy(plannedPath[0], newTraj[0],
                           sizeof(tempTra) * 3); // 复制轨迹到输出数组
                    PRINT_SWITCH_INFO("VertRearReplanInSlot Success 1");
                    return 3; // 返回轨迹点数量
                }
            }

            l = (l <= ZERO_FLOAT) ? 0.6 : (l + 0.2); // 更新l值
        }
    }

    // 迭代尝试不同的半径值来生成轨迹
    for (int i = try_num; i > 0 && trajNum < 0; i--)
    {
        PK_CopyTra(tempTra, stpoint); // 复制起点轨迹到临时轨迹

        curradius  = radius[i - 1] + Rrmin; // 计算当前半径
        auto dist  = curradius * rtheta;    // 计算路径距离
        auto disty = ry + curradius * (1 - cosrtheta) *
                              sign(rtheta); // 计算目标点与此路径终点y方向距离
        tempTra[3] = -fabs(dist);           // 设置临时轨迹参数
        tempTra[4] = -sign(ry) * curradius;

        PK_Get_Path_EndPos(tempTra, newTraj[0]); // 获取路径终点
        if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[1], newTraj[0],
                                            side_swell_0))
        {
            continue;
        } // 碰撞检测
        if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[3], newTraj[0],
                                            side_swell_0))
        {
            continue;
        } // 碰撞检测

        float rx1 = Project_PosTo1st_rx(finpoint, newTraj[0]); // 计算新的相对x坐标
        if (rx1 < 0.8)
        {
            continue;
        } // 检查路径有效性

        // 如果路径有效且满足位置约束
        if (rx1 < rx && fabs(disty) < 0.02)
        {
            PK_CopyTra(newTraj[0], tempTra);                       // 复制轨迹
            PK_Get_Path_EndPos(tempTra, newTraj[1]);               // 获取路径终点
            float rxe = Project_PosTo1st_rx(finpoint, newTraj[1]); // 计算新的相对x坐标

#if 0
            if (rxe > 0.1) // 检查路径有效性
#else
            float rthetae = Project_PosTo1st_rtheta(finpoint, newTraj[1]);
            if (rxe > 0.3 && fabs(rthetae) < 2 * PI_RAD) // 检查路径有效性
#endif
            {
                newTraj[1][3] = -rxe; // 设置最终调整参数
                newTraj[1][4] = 0;
                memcpy(plannedPath[0], newTraj[0],
                       sizeof(tempTra) * 2); // 复制轨迹到输出数组
                PRINT_SWITCH_INFO("VertRearReplanInSlot Success 2");
                return 2; // 返回轨迹点数量
            }
        }

        // 尝试生成更复杂的轨迹(弧长)
        disty = fabs(disty);
        for (float cl = 0.6; cl < 2.5 && trajNum < 0; cl = cl + 0.1)
        {
            auto theta    = cl / curradius; // 计算角度
            auto thetacos = cosf(theta);    // 计算余弦
            auto thetasin = sinf(theta);    // 计算正弦
            PK_CopyTra(tempTra, stpoint);   // 复制起点轨迹
            auto d1 =
                2 * curradius * (1 - thetacos); // 计算弧两端点之间的直线距离（弦长）

            if (d1 - disty > 0.03)
            {
                break;
            } // 检查路径有效性

            float d2 = disty - d1;    // 计算剩余距离
            float l  = d2 / thetasin; // 计算路径长度
            if (l < 0.4)
            {
                continue;
            } // 检查路径有效性
            if (l * thetacos + 2 * curradius * thetasin > rx1)
            {
                continue;
            } // 检查路径有效性

            int count  = 0;                                         // 轨迹点计数
            tempTra[3] = -fabs(curradius * (fabs(rtheta) + theta)); // 设置临时轨迹参数
            tempTra[4] = -sign(ry) * curradius;
            PK_CopyTra(newTraj[count], tempTra);                // 复制轨迹
            PK_Get_Path_EndPos(newTraj[count], tempTra);        // 获取路径终点
            float rxe = Project_PosTo1st_rx(finpoint, tempTra); // 计算新的相对x坐标
            float rye = Project_PosTo1st_ry(finpoint, tempTra); // 计算新的相对y坐标
            if (rxe < 0.3 || fabs(rye) > VEHICLE_WID)
            {
                continue;
            } // 检查路径有效性
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[1], tempTra,
                                                side_swell_0))
            {
                continue;
            } // 碰撞检测
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[3], tempTra,
                                                side_swell_0))
            {
                continue;
            } // 碰撞检测
            count++;

            tempTra[3] = -l; // 设置临时轨迹参数
            tempTra[4] = 0;
            PK_CopyTra(newTraj[count], tempTra);          // 复制轨迹
            PK_Get_Path_EndPos(newTraj[count], tempTra);  // 获取路径终点
            rye = Project_PosTo1st_ry(finpoint, tempTra); // 计算新的相对y坐标
            rxe = Project_PosTo1st_rx(finpoint, tempTra); // 计算新的相对x坐标
            if (rxe < 0.3 || fabs(rye) > VEHICLE_WID)
            {
                continue;
            } // 检查路径有效性
            count++;

            tempTra[3] = -curradius * fabs(theta); // 设置临时轨迹参数
            tempTra[4] = -newTraj[0][4];
            PK_CopyTra(newTraj[count], tempTra);          // 复制轨迹
            PK_Get_Path_EndPos(newTraj[count], tempTra);  // 获取路径终点
            rye = Project_PosTo1st_ry(finpoint, tempTra); // 计算新的相对y坐标
            rxe = Project_PosTo1st_rx(finpoint, tempTra); // 计算新的相对x坐标
#if 0
            if (rxe < 0.1 || fabs(rye) > 2 * VEHICLE_WID) {  continue;  } // 检查路径有效性
#else
            float rthetae = Project_PosTo1st_rtheta(finpoint, newTraj[2]);
            if (rxe < 0.3 || fabs(rye) > 0.1 || rthetae > 2 * PI_RAD)
            {
                continue;
            } // x预留0.30，防止限位杆更新导致车不能回正，y限制在0.1m内，防止目标点偏离太多
#endif
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[1], tempTra,
                                                side_swell_0))
            {
                continue;
            } // 碰撞检测
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[3], tempTra,
                                                side_swell_0))
            {
                continue;
            } // 碰撞检测
            count++;

            tempTra[3] = -rxe; // 设置最终调整参数
            tempTra[4] = 0;
            PK_CopyTra(newTraj[count], tempTra); // 复制轨迹
            count++;

            memcpy(plannedPath[0], newTraj[0],
                   sizeof(tempTra) * count); // 复制轨迹到输出数组
            PRINT_SWITCH_INFO("VertRearReplanInSlot Success 3");
            return count; // 返回轨迹点数量
        }
    }

    return 0; // 未找到有效轨迹，返回0
}

/**
 * @brief 规划后退路径
 *
 * 该函数用于规划车辆的后退路径，根据当前的起点和目标点，以及环境中的障碍物，生成最优的后退路径。
 *
 * @param plannedPath 计划路径数组
 * @param finpoint 目标位置数组
 * @param enviObj 环境对象数组
 * @param stpoint 起点位置数组
 * @return int 返回生成的路径数量
 */
int PathExec_B_MoveOutRear(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                           const float finpoint[3], const float enviObj[5][4],
                           const float stpoint[4])
{
    float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];            // 存储生成的轨迹
    float rtheta = Project_PosTo1st_rtheta(finpoint, stpoint); // 计算起点到目标点的角度差
    float rx     = Project_PosTo1st_rx(finpoint, stpoint); // 计算起点到目标点的x方向距离

    // 尝试生成最优轨迹
    int trajNum =
        PathExec_BetterTra(stpoint, finpoint, 3, &enviObj[1], trajs, side_swell_1);
    if (trajNum > 0)
    {
        PRINT_SWITCH_INFO("PK_PathPlan_BetterTra update path!");
        memcpy(plannedPath, trajs, trajNum * PATH_ITEM_LEN);
        return trajNum;
    }

    // 如果x方向距离大于1.5且角度差小于30度，则在停车位内重新规划后退路径
    if (rx > 1.5 && fabs(rtheta) < 30 * PI_RAD)
    {
        trajNum = VertRearReplanInSlot(trajs, finpoint, enviObj, stpoint);
        if (trajNum > 0)
        {
            PRINT_SWITCH_INFO("VertRearReplanInSlot update path!");
            memcpy(plannedPath, trajs, trajNum * PATH_ITEM_LEN);
        }
        return trajNum;
    }

    float lastTra[TRAJITEM_LEN] = {0};
    float rxb                   = Project_PosTo1st_rx(finpoint, enviObj[1]);
    float rxe                   = Project_PosTo1st_rx(finpoint, enviObj[4]);
    float xoffset = rte_max(rxb, rxe); // 计算目标点到车位入口BE点的最大x方向距离
    xoffset =
        rte_max(xoffset, 2.5); // 确保距离至少为2.5， 疑问？ 是否可以更小一些，例如宽车位

    PK_CopyPos(lastTra, finpoint); // 复制目标位置到最后轨迹
    lastTra[3] = xoffset;          // 设置路径长度
    lastTra[4] = 0;
    PK_ReverseTraject(lastTra); // 反转轨迹

    float ryb     = Project_PosTo1st_ry(finpoint, enviObj[1]);
    float rye     = Project_PosTo1st_ry(finpoint, enviObj[4]);
    float yoffset = fabs(ryb) - fabs(rye); // 计算y方向偏移量

    // 尝试通过探索轨迹生成路径
    for (int i = 1; i < 4; i++)
    {
        trajNum = ExploreTraToTra(trajs, lastTra, enviObj, stpoint);
        if (trajNum > 0)
        {
            PRINT_SWITCH_INFO("ExploreTraToTra");
            float trajs1[6][TRAJITEM_LEN];
            int trajNum1 = PathExec_BetterTra(lastTra, finpoint, 3, &enviObj[1], trajs1,
                                              side_swell_0);
            if (trajNum1 <= 0)
            {
                PRINT_SWITCH_INFO("VertRearReplanInSlot");
                trajNum1 = VertRearReplanInSlot(trajs1, finpoint, enviObj, lastTra);
            }
            else
            {
                PRINT_SWITCH_INFO("PK_PathPlan_BetterTra");
            }

            if (trajNum1 > 0)
            {
                PRINT_SWITCH_INFO("Replan Successs");
                memcpy(plannedPath, trajs, trajNum * PATH_ITEM_LEN);
                memcpy(&plannedPath[trajNum], trajs1, trajNum1 * PATH_ITEM_LEN);
                trajNum = PK_PathMerge(plannedPath, trajNum + trajNum1, 0);
                return trajNum;
            }
        }

        // 如果y方向偏移量大于0，调整轨迹偏移量
        if (yoffset > 0)
        {
            float finoff[2];
            finoff[0] = rte_max(xoffset - i * 0.50, 2.5);
            finoff[1] = i * rte_min(0.05, yoffset * 0.5);
            yoffset   = yoffset - finoff[1];
            finoff[1] = finoff[1] * sign(ryb);  // 确定偏移方向
            Convert(finpoint, finoff, lastTra); // 转换轨迹
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
static int PathExec_SpaceExplorer_ParaRePlan0(const float finpoint[3],
                                              const float enviObj[5][4],
                                              const float stpoint[4],
                                              float traj[][TRAJITEM_LEN], int leftFac)
{
    const float step = 0.02;

    float radius = Rrmin * 3;
    float swell  = 0.3f;

    float cirTraj[5][TRAJITEM_LEN], tempTra[TRAJITEM_LEN];
    memcpy(tempTra, finpoint, sizeof(float) * 3);

    // 起点到EF距离
    float front_dis =
        PK_PointToLineDist(enviObj[3], stpoint) - (VEHICLE_LEN - REAR_SUSPENSION);
    float ydist = 0.5 * (radius - (Rrmin + 0.5)) * pow2(1.3 * front_dis / radius);

    float rspoint[3];
    RevConvert(finpoint, stpoint, rspoint);
    rspoint[1] = sign(rspoint[1]) * rte_min(fabs(rspoint[1]), ydist);

    PathExec_UpdateObsObj_AroundTargSlot(); //  Calculate once
    int trajNum = 0;

    while (fabs(rspoint[1]) > step)
    {
        tempTra[4] = -(float)leftFac * radius;
        tempTra[3] = front_dis * 2.0;
        PK_PathVerify(tempTra, 2, &enviObj[2], swell);
        int result = PathExec_Replan_PathCheck(1, &tempTra, swell, swell, finpoint,
                                               enviObj[3], enviObj[3], &enviObj[2]);
        if (result == PATHPLAN_SUCCESS)
        {
            float target[3];
            Convert(finpoint, rspoint, target);
            target[2] = stpoint[2];
            trajNum = PK_PathPlan_TraExplorer_Con(tempTra, target, 2, enviObj, cirTraj, 0,
                                                  1, swell, 0.0);
            result  = PathExec_Replan_PathCheck(trajNum, cirTraj, swell, swell, finpoint,
                                                enviObj[0], enviObj[0], &enviObj[1]);
            if (result != PATHPLAN_SUCCESS)
            {
                trajNum = 0;
            }
        }

        if (trajNum > 0)
        {
            break;
        }
        rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) - step);
    }

    for (int j = 0; j < trajNum; j++)
    {
        PK_ReverseTraject(cirTraj[trajNum - 1 - j]);
        PK_CopyTra(traj[j], cirTraj[trajNum - 1 - j]);
    }

    float temp[TRAJITEM_LEN];
    PK_CopyPos(temp, stpoint);
    for (int i = 0; i < trajNum; i++)
    {
        temp[3] = traj[i][3];
        temp[4] = traj[i][4];
        PK_CopyTra(traj[i], temp);
        PK_Get_Path_EndPos(traj[i], temp);
    }

    printf("PathExec_B_MoveOut_ParaRePlan ----------PathExec_SpaceExplorer_ParaRePlan0");

    return trajNum;
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
static int PathExec_SpaceExplorer_ParaRePlan1(const float finpoint[3],
                                              const float enviObj[5][4],
                                              const float stpoint[4],
                                              float traj[][TRAJITEM_LEN], int leftFac)
{
    float radius[] = {0.3, 1.0, 2.0, 3.0, 5.0};

    float tempTra[TRAJITEM_LEN];
    float dTheta = Project_PosTo1st_rtheta(finpoint, stpoint);

    int trajNum = 0;
    size_t idx  = 0;
    while (idx < sizeof(radius) / sizeof(float))
    {
        float curradius = radius[idx] + Rrmin;
        float dist      = curradius * 0.5 * dTheta;
        PK_CopyPos(tempTra, stpoint);
        tempTra[3] = fabs(dist);
        tempTra[4] = leftFac * curradius;

        PK_PathVerify(tempTra, 2, &enviObj[2], 0.3);
        if (fabs(tempTra[3]) > 0.5)
        {
            PK_CopyTra(traj[0], tempTra);
            PK_Get_Path_EndPos(tempTra, traj[1]);
            traj[1][3] = -traj[0][3];
            traj[1][4] = -traj[0][4];
            trajNum    = 2;
            break;
        }
        idx++;
    }

    return trajNum;
}

/**
 * @brief 计算路径参数
 *
 * 详细介绍：函数尝试通过遍历不同的路径参数来寻找符合条件的路径，并返回最短路径的相关参数。
 *
 * @param disty 目标点的y轴距离
 * @param curradius 当前使用的曲率半径
 * @param p 存储路径参数的数组，p[0]存储路径长度，p[1]存储路径角度，p[2]存储路径总长度
 * @return int 返回值，0表示找到符合条件的路径，-1表示未找到符合条件的路径
 */
static int SloveRLR(const float disty, const float curradius, float p[3])
{
    const float max_len = 6.0; // 最大路径长度

    p[2] = max_len + 1.0;                        // 初始化路径总长度为最大路径长度加1
    for (float l = 0.8; l >= 0.01; l = l - 0.05) // 遍历不同的路径长度
    {
        auto theta1 = 0.5 / curradius;          // 计算初始角度
        auto theta2 = 1.2 / curradius;          // 计算结束角度
        auto step   = (theta2 - theta1) * 0.01; // 计算步长
        auto theta  = theta1 - step;            // 初始化角度
        while (theta < theta2)                  // 遍历不同的路径角度
        {
            theta          = theta + step;                      // 更新角度
            float totallen = l + 2.0 * curradius * fabs(theta); // 计算路径总长度
            if (totallen > max_len)
            {
                continue;
            } // 如果总长度超过最大长度，跳过该路径
            auto d1 = 2.0 * curradius * (1 - cosf(theta)); // 计算d1距离
            auto d2 = l * sinf(theta);                     // 计算d2距离
            if (fabs(d1 + d2 - disty) > 0.01)
            {
                continue;
            } // 如果d1和d2之和与目标y轴距离差距超过0.01，跳过该路径

            // 如果当前路径总长度小于已记录的路径总长度
            if (p[2] > totallen)
            {
                p[0] = l;        // 更新路径长度
                p[1] = theta;    // 更新路径角度
                p[2] = totallen; // 更新路径总长度
            }

            // 如果当前路径总长度小于最大长度的0.7倍，返回0表示找到符合条件的路径
            if (p[2] < max_len * 0.7)
            {
                return 0;
            }
        }
    }

    // 如果路径总长度小于最大长度，返回0表示找到符合条件的路径
    if (p[2] < max_len)
    {
        return 0;
    }

    return -1; // 返回-1表示未找到符合条件的路径
}

/**
 * @brief 计算合适的直线和曲率路径
 *
 * 根据给定的偏移距离和转角，使用预设的半径值数组，计算出最佳的直线行驶长度和曲率路径。
 * 函数选择最小的路径总长度并返回。
 *
 * @param disty 垂直方向的位移距离，单位米
 * @param theta 当前的转角，单位弧度
 * @param p [out] 指针数组，存储计算出的直线距离、曲率半径和总路径长度
 * @return int 返回 0 表示成功找到路径，返回 -1 表示无法找到合适路径
 */
static int SloveLR(const float disty, const float theta, float p[3])
{
    const float max_len  = 6.0; // 允许的最大总路径长度
    const float radius[] = {0.5, 1.0, 2.0, 3.5, 6.0, 9.0, 13.0, 18.0}; // 半径数组
    const int try_num    = sizeof(radius) / sizeof(radius[0]);         // 半径数组的大小

    p[2] = max_len + 1.0; // 初始化总路径长度为超过最大值的数值

    // 计算 cos(theta) 和 |sin(theta)|
    float cosrtheta = cosf(theta);
    float sinrtheta = fabs(sinf(theta));

    // 遍历不同的半径值，从大到小尝试不同路径
    for (int i = try_num; i > 0; i--)
    {
        // 根据当前半径计算横向偏移 dr
        float dr = (radius[i - 1] + Rrmin) * (1 - cosrtheta);
        if (dr > disty)
        {
            continue;
        } // 如果偏移值超过总偏移距离，跳过

        // 计算曲率路径长度 r 和直线长度 l
        float r        = (radius[i - 1] + Rrmin) * fabs(theta);
        float l        = (disty - dr) / sinrtheta;
        float totallen = l + r; // 计算总路径长度

        // 如果总路径长度超过最大值，跳过当前半径
        if (totallen > max_len)
        {
            continue;
        }

        // 如果找到更短的路径，则更新直线长度和半径
        if (p[2] > totallen)
        {
            p[0] = l;                       // 直线长度
            p[1] = (radius[i - 1] + Rrmin); // 曲率半径
            p[2] = totallen;                // 总路径长度
        }

        if (p[2] < max_len * 0.7)
        {
            return 0;
        }
    }

    if (p[2] < max_len)
    {
        return 0;
    }
    return -1;
}

/**
 * @brief 在停车位内重新规划路径
 *
 * 详细介绍：函数尝试根据不同半径计算可行的路径，避免与环境中的障碍物碰撞，并最终返回生成的新路径段数。
 *
 * @param plannedPath 存储生成的新路径
 * @param finpoint 最终目标点
 * @param enviObj 环境中的障碍物信息
 * @param stpoint 起始点
 * @return int 返回生成的新路径段数
 */
int PathExec_SpaceExplorer_RePlanInSlot(
    float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const float finpoint[3],
    const float enviObj[5][4], const float stpoint[4])
{
    float radius[]  = {0.5, 1.0, 2.0, 3.5, 6.0, 9.0, 13.0, 18.0}; // 预定义的半径数组
    int try_num     = sizeof(radius) / sizeof(radius[0]);         // 尝试次数
    float curradius = 20.0;                                       // 当前使用的半径

    float tempTra[TRAJITEM_LEN];                               // 临时存储路径点
    float newTraj[10][TRAJITEM_LEN];                           // 存储生成的新路径
    float rtheta = Project_PosTo1st_rtheta(finpoint, stpoint); // 计算起点到终点的角度
    float ry     = Project_PosTo1st_ry(finpoint, stpoint);     // 计算起点到终点的y轴距离

    float cosrtheta = cosf(rtheta);
    float sinrtheta = fabs(sinf(rtheta));
    PK_CopyPos(tempTra, stpoint); // 复制起始点位置到临时路径点

    // 如果角度与y轴距离的乘积大于0或y轴距离的比例超过5
    if (rtheta * ry > 0 || fabs(ry * cosrtheta / sinrtheta) > 5.0)
    {
        PRINT_SWITCH_INFO("PathExec_SpaceExplorer_RePlanInSlot 1");
        float result[3] = {0.0, 0.0, 10.0}; // 存储计算结果，初始值设为较大
        for (int i = try_num; i > 0; i--)
        {
            curradius  = radius[i - 1] + Rrmin; // 更新当前使用的半径
            auto dist  = curradius * rtheta;    // 计算距离
            auto disty = ry + curradius * (1 - cosrtheta) * sign(rtheta);

            // 如果角度与y轴距离的乘积大于0
            if (rtheta * ry > 0)
            {
                tempTra[3] = fabs(dist);                 // 更新临时路径点的路径距离
                tempTra[4] = -sign(ry) * curradius;      // 更新临时路径点的路径曲率
                PK_Get_Path_EndPos(tempTra, newTraj[0]); // 获取路径终点位置
                if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[1], newTraj[0],
                                                    0.33))
                {
                    continue;
                } // 检查碰撞
                if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[3], newTraj[0],
                                                    0.33))
                {
                    continue;
                } // 检查碰撞
            }

            float p[3]; // 临时存储计算结果
            if (SloveRLR(fabs(disty), curradius, p) != 0)
            {
                continue;
            } // 计算路径并检查是否成功

            if (result[2] > p[2])
            {
                result[0] = p[0];
                result[1] = p[1];
                result[2] = p[2];
            } // 更新结果

            if (result[2] < 4.0)
            {
                break;
            } // 如果结果小于4则跳出循环
        }

        // 如果结果小于10
        if (result[2] < 10.0)
        {
            tempTra[3] =
                fabs(curradius * (fabs(rtheta) +
                                  fabs(result[1]) * sign(rtheta * ry))); // 更新路径距离
            tempTra[4] = -sign(ry) * curradius;                          // 更新路径曲率
            PK_CopyTra(newTraj[0], tempTra);                             // 复制路径

            PK_Get_Path_EndPos(newTraj[0], tempTra); // 获取路径终点位置
            tempTra[3] = result[0];                  // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[1], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[1], tempTra);  // 获取路径终点位置
            tempTra[3] = curradius * fabs(result[1]); // 更新路径点
            tempTra[4] = -newTraj[0][4];
            PK_CopyTra(newTraj[2], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[2], tempTra);           // 获取路径终点位置
            float rx = Project_PosTo1st_rx(finpoint, tempTra); // 计算x轴距离
            if (fabs(rx) > 6.0f)
            {
                return 0;
            }

            tempTra[3] = -rx; // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[3], tempTra); // 复制路径

            memcpy(plannedPath[0], newTraj[0],
                   sizeof(tempTra) * 4); // 复制生成的新路径到计划路径
            return 4;                    // 返回生成的新路径段数
        }
    }
    // 如果角度与y轴距离的乘积不大于0且路径曲率满足条件
    else if (Rrmin * (1 - cosrtheta) > fabs(ry))
    {
        PRINT_SWITCH_INFO("PathExec_SpaceExplorer_RePlanInSlot 2");
        float result[3] = {0.0, 0.0, 10.0}; // 存储计算结果，初始值设为较大
        for (int i = try_num; i > 0; i--)
        {
            curradius = radius[i - 1] + Rrmin; // 更新当前使用的半径

            auto disty = ry + curradius * (1 - cosrtheta) * sign(rtheta); // 计算距离

            tempTra[3] = fabs(curradius * rtheta);   // 更新路径距离
            tempTra[4] = -sign(ry) * curradius;      // 更新路径曲率
            PK_Get_Path_EndPos(tempTra, newTraj[0]); // 获取路径终点位置
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[1], newTraj[0], 0.33))
            {
                continue;
            } // 检查碰撞
            if (PathExec_CheckVehColliionInSlot(finpoint, enviObj[3], newTraj[0], 0.33))
            {
                continue;
            } // 检查碰撞

            float p[3]; // 临时存储计算结果
            if (SloveRLR(fabs(disty), curradius, p) != 0)
            {
                continue;
            } // 计算路径并检查是否成功

            if (result[2] > p[2])
            {
                result[0] = p[0];
                result[1] = p[1];
                result[2] = p[2];
            } // 更新结果

            if (result[2] < 3.5)
            {
                break;
            } // 如果结果小于3.5则跳出循环
        }

        // 如果结果小于10
        if (result[2] < 10.0)
        {
            tempTra[3] =
                fabs(curradius * (fabs(rtheta) + fabs(result[1]))); // 更新路径距离
            tempTra[4] = -sign(rtheta) * curradius;                 // 更新路径曲率
            PK_CopyTra(newTraj[0], tempTra);                        // 复制路径

            PK_Get_Path_EndPos(newTraj[0], tempTra); // 获取路径终点位置
            tempTra[3] = result[0];                  // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[1], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[1], tempTra);  // 获取路径终点位置
            tempTra[3] = curradius * fabs(result[1]); // 更新路径点
            tempTra[4] = -newTraj[0][4];
            PK_CopyTra(newTraj[2], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[2], tempTra);           // 获取路径终点位置
            float rx = Project_PosTo1st_rx(finpoint, tempTra); // 计算x轴距离
            if (fabs(rx) > 6.0f)
            {
                return 0;
            }

            tempTra[3] = -rx; // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[3], tempTra); // 复制路径

            memcpy(plannedPath[0], newTraj[0],
                   sizeof(tempTra) * 4); // 复制生成的新路径到计划路径
            return 4;                    // 返回生成的新路径段数
        }
    }
    else
    {
        PRINT_SWITCH_INFO("PathExec_SpaceExplorer_RePlanInSlot 3");
        float p[3];                                  // 临时存储计算结果
        if (SloveLR(fabs(ry), fabs(rtheta), p) == 0) // 计算路径并检查是否成功
        {
            tempTra[3] = p[0]; // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[0], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[0], tempTra); // 获取路径终点位置
            tempTra[3] = p[2] - p[0];                // 更新路径点
            tempTra[4] = -sign(rtheta) * fabs(p[1]);
            PK_CopyTra(newTraj[1], tempTra); // 复制路径

            PK_Get_Path_EndPos(newTraj[1], tempTra);           // 获取路径终点位置
            float rx = Project_PosTo1st_rx(finpoint, tempTra); // 计算x轴距离
            if (fabs(rx) > 6.0f)
            {
                return 0;
            }

            tempTra[3] = -rx; // 更新路径点
            tempTra[4] = 0;
            PK_CopyTra(newTraj[2], tempTra); // 复制路径

            memcpy(plannedPath[0], newTraj[0],
                   sizeof(tempTra) * 3); // 复制生成的新路径到计划路径
            return 3;                    // 返回生成的新路径段数
        }
    }

    return 0; // 如果没有生成路径，返回0
}

/**
 * @brief 函数简介
 *
 * 详细介绍该函数 (可选)
 *
 * @param a 变量简介
 * @return bool 返回值简介
 */
int PathExec_B_MoveOut_ParaRePlan(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                  const float finpoint[3], const float enviObj[5][4],
                                  const float stpoint[4], const int slotShap)
{
    int leftFac   = (Slot_Dir(slotShap) == 1) ? 1 : -1;
    float dTheta  = Project_PosTo1st_rtheta(finpoint, stpoint);
    float startry = Project_PosTo1st_ry(finpoint, stpoint);

    if (fabs(startry) < 0.03 && fabs(Round_PI(dTheta)) < 0.03)
    {
        return 0;
    }
    if (sign(dTheta) * leftFac * fabs(Round_PI(dTheta)) > 0.03)
    {
        return 0;
    }

    float endPos[3];

    PRINT_SWITCH_INFO("PathExec_B_MoveOut_ParaRePlan");
    int trajNum = PathExec_SpaceExplorer_ParaRePlan0(finpoint, enviObj, stpoint,
                                                     plannedPath, leftFac);
    if (trajNum > 0)
    {
        PK_Get_Path_EndPos(plannedPath[trajNum - 1], endPos);
        if (PK_Check_Path(trajNum, plannedPath, stpoint, endPos, 0, NULL, 0) ==
            PATHPLAN_SUCCESS)
        {
            trajNum = PK_PathMerge(plannedPath, trajNum, 0);
        }
    }

    return trajNum;

#if 0
    if (fabs(Round_PI(dTheta)) < 0.03)  {  return 0;  }

    trajNum = PathExec_SpaceExplorer_ParaRePlan1(finpoint, enviObj, stpoint, plannedPath, leftFac);
    PRINT_SWITCH_INFO("PathExec_B_MoveOut_ParaRePlan ----------PathExec_SpaceExplorer_ParaRePlan1");
    if (trajNum <= 0) {  return 0;  }

    PK_Get_Path_EndPos(plannedPath[trajNum - 1], endPos);
    if (PK_Check_Path(trajNum, plannedPath, stpoint, endPos, 0, NULL ,0) == PATHPLAN_SUCCESS)
    {
        trajNum = PK_PathMerge(plannedPath, trajNum, 0);
        return trajNum;
    }
    else
    {
        return 0;
    }
#endif
}
