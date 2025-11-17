#include "PK_PathPlanP.h"
#include "PathPlan_Update.h"
#include "PathPlan_Debug.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_PathExecute.h"
#include "PathPlan_BasePlan.h"
#include "PathPlan_Tools.h"
#include "Record_Log.h"

inline static void PathPlanType(uint8_t level, uint8_t type, int num)
{
    PathPlan_FootPrints(2, level, type, num);
}

/**
 * @brief 路径探索与连接算法
 *        根据目标位置和障碍物信息，规划从当前路径到目标路径的连接路径。
 * @param temp_tra 当前路径的轨迹数据，包括位置、方向和路径长度
 * @param tar_pos 目标位置的坐标（x, y, theta）
 * @param obj_num 障碍物数量
 * @param obj_slot 障碍物的边界信息数组，每个障碍物包含4个浮点数据表示范围
 * @param stp_trajs 输出的连接路径轨迹数组
 * @param mid_straight_line 中间的直线路径长度
 * @param A_tra_to_posint 标志是否优先从当前位置连接到目标位置
 * @param B_tra_to_pos 标志是否从目标位置连接回当前位置
 * @param swell 安全膨胀距离，用于路径边界补偿
 * @param start_ds 初始路径长度，决定探索方向（正值为前进，负值为后退）
 * @return 返回生成的连接路径的段数，如果为0表示未找到可行路径
 */
static int TraExplorer_Con(float temp_tra[TRAJITEM_LEN], const float tar_pos[3],
                           const int obj_num, const float obj_slot[][4],
                           float stp_trajs[][TRAJITEM_LEN], float mid_straight_line,
                           int A_tra_to_posint, int B_tra_to_pos, float swell,
                           float start_ds)
{
    // 临时变量，用于存储中间路径和轨迹点
    float TempTraj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
        TempTraj1[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    static float Explorpos_arr[30][4];            // 探索点数组，存储 (x, y, theta, ds)
    int line_num = 0, line_num1 = 0;              // 当前路径段数和候选路径段数
    int points_num      = 0;                      // 探索点的总数量
    int enough_mid_ds   = 0;                      // 标志是否满足中间直线路径要求
    int connest_success = 0;                      // 标志是否成功连接路径
    const float dsmin   = 0.3f;                   // 最小路径分割步长
    int j, Explorpos_num, Explorpos_num_temp = 0; // 当前探索点和临时探索点索引
    float sign_start_ds;

    // 根据初始路径方向调整起点路径长度的符号（保持前进或后退方向一致）
    sign_start_ds = sign(temp_tra[3]) * fabsf(start_ds);

    // 根据初始路径长度决定分割步长，保证探索点数量适当
    float dsminv = dsmin;

    int number = (fabs(temp_tra[3]) - fabs(start_ds)) / dsmin;
    if (number > 0)
    {
        if (number < 7)
        {
            dsminv = 0.15;
        }
        else if (number < 4)
        {
            dsminv = 0.1;
        }

        // 生成探索点轨迹，存储在 `Explorpos_arr` 中
        points_num =
            PathPlan_DrawTrajectory(dsminv, temp_tra, Explorpos_arr, sign_start_ds);
    }
    else
    {
        points_num = 1;
        PK_Get_Path_EndPos(temp_tra, Explorpos_arr[0]);
        Explorpos_arr[0][3] = temp_tra[3];
    }

    // 遍历探索点，尝试生成连接路径
    for (j = 0; j < points_num; j++)
    {
        // 根据连接方向，选择当前探索点的索引
        if (A_tra_to_posint == 1)
        {
            Explorpos_num = points_num - 1 - j; // 从后向前遍历探索点
        }
        else
        {
            Explorpos_num = j; // 从前向后遍历探索点
        }

        // 判断连接方向，生成对应的候选路径
        if (B_tra_to_pos == 1)
        {
            // 从探索点连接到目标位置
            line_num = PK_PathPlan_BetterTra(Explorpos_arr[Explorpos_num], tar_pos,
                                             obj_num, obj_slot, TempTraj, swell);
            // 检查路径段是否满足中间直线路径和曲率约束
            if (line_num > 0 && fabs(TempTraj[0][3]) > mid_straight_line &&
                fabs(TempTraj[0][4]) < 1)
            {
                enough_mid_ds = 1; // 标记满足中间路径要求
            }
            else
            {
                enough_mid_ds = 0;
            }
        }
        else
        {
            // 从目标位置连接到探索点
            line_num = PK_PathPlan_BetterTra(tar_pos, Explorpos_arr[Explorpos_num],
                                             obj_num, obj_slot, TempTraj, swell);
            if (line_num > 0 && fabs(TempTraj[line_num - 1][3]) > mid_straight_line &&
                fabs(TempTraj[line_num - 1][4]) < 1)
            {
                enough_mid_ds = 1;
            }
            else
            {
                enough_mid_ds = 0;
            }
        }

        // 如果找到满足条件的路径，直接输出并返回
        if (line_num > 0 && A_tra_to_posint && enough_mid_ds)
        {
            if (B_tra_to_pos == 1)
            {
                // 保存生成的路径段
                memcpy(stp_trajs[1], TempTraj[0], line_num * PATH_ITEM_LEN);
                stp_trajs[0][0] = temp_tra[0];
                stp_trajs[0][1] = temp_tra[1];
                stp_trajs[0][2] = temp_tra[2];
                stp_trajs[0][3] = Explorpos_arr[Explorpos_num][3];
                stp_trajs[0][4] = temp_tra[4];
                line_num        = line_num + 1;
            }
            else
            {
                memcpy(stp_trajs[0], TempTraj[0], line_num * PATH_ITEM_LEN);
                stp_trajs[line_num][0] = Explorpos_arr[Explorpos_num][0];
                stp_trajs[line_num][1] = Explorpos_arr[Explorpos_num][1];
                stp_trajs[line_num][2] = Explorpos_arr[Explorpos_num][2];
                stp_trajs[line_num][3] = -Explorpos_arr[Explorpos_num][3];
                stp_trajs[line_num][4] = temp_tra[4];
                line_num = line_num + 1; //  add the last path Explorpos_arr[j]
            }
            temp_tra[3] = Explorpos_arr[Explorpos_num][3]; // 更新路径长度
            return line_num;                               // 返回路径段数
        }
        else if (line_num > 0 && A_tra_to_posint)
        {
            // 保存候选路径，并选择距离最长的路径
            if (B_tra_to_pos == 1)
            {
                if (fabs(TempTraj[0][3]) > fabs(TempTraj1[0][3]) || line_num1 == 0)
                {
                    memcpy(TempTraj1[0], TempTraj[0], line_num * PATH_ITEM_LEN);
                    line_num1          = line_num;
                    Explorpos_num_temp = Explorpos_num;
                    connest_success    = 1; // 标记成功连接路径
                }
            }
            else
            {
                if (fabs(TempTraj[line_num - 1][3]) > fabs(TempTraj1[line_num - 1][3]) ||
                    line_num1 == 0)
                {
                    memcpy(TempTraj1[0], TempTraj[0], line_num * PATH_ITEM_LEN);
                    line_num1          = line_num;
                    Explorpos_num_temp = Explorpos_num;
                    connest_success    = 1;
                }
            }
        }
    }

    // 如果未找到足够中间路径，但存在候选路径，则输出候选路径
    if (!enough_mid_ds && line_num == 0 && connest_success)
    {
        if (B_tra_to_pos == 1)
        {
            memcpy(stp_trajs[1], TempTraj1[0], line_num1 * PATH_ITEM_LEN);
            stp_trajs[0][0] = temp_tra[0];
            stp_trajs[0][1] = temp_tra[1];
            stp_trajs[0][2] = temp_tra[2];
            stp_trajs[0][3] = Explorpos_arr[Explorpos_num_temp][3];
            stp_trajs[0][4] = temp_tra[4];
            line_num        = line_num1 + 1;
        }
        else
        {
            memcpy(stp_trajs[0], TempTraj1[0], line_num1 * PATH_ITEM_LEN);
            stp_trajs[line_num1][0] = Explorpos_arr[Explorpos_num_temp][0];
            stp_trajs[line_num1][1] = Explorpos_arr[Explorpos_num_temp][1];
            stp_trajs[line_num1][2] = Explorpos_arr[Explorpos_num_temp][2];
            stp_trajs[line_num1][3] = -Explorpos_arr[Explorpos_num_temp][3];
            stp_trajs[line_num1][4] = temp_tra[4];
            line_num                = line_num1 + 1;
        }
        temp_tra[3] = Explorpos_arr[Explorpos_num_temp][3];
    }

    PathPlanType(0, 1, line_num);
    return line_num; // 返回最终路径段数
}

// the core function to plan a parallel slot
/**
 * @brief 获取路径规划的最终轨迹，根据车位类型和环境条件动态生成路径。
 *
 * @param PlanData_Ver 包含规划数据的数组，存储起点、终点、车位类型、障碍物等信息。
 * @param Exp_traj 用于存储最终生成的轨迹数组，二维数组格式，包含轨迹点的详细信息。
 *
 * @return 返回生成的轨迹线段数。如果路径规划失败，则返回 0。
 */
static int SpaceExplorer(const float finpoint[3], const float dir, const uint8_t firstDir,
                         const float objs[][4], const int objsNum,
                         float Exp_traj[][TRAJITEM_LEN])
{
    const int Explore_times      = 16; // 探索最大步数
    const float max_explore_dist = 4.0f;
    const float short_path_len   = 0.35f; // 短路径长度阈值

    float exploreRrmin = Rrmin + 1.5f;

    float temp_tra[5]                               = {0.0f};
    float Exp_finpoint[Explore_times][TRAJITEM_LEN] = {{0.0f}};

    PK_CopyPos(temp_tra, finpoint);
    temp_tra[3] = max_explore_dist;
    temp_tra[4] = 0;
    PK_PathVerify(temp_tra, objsNum, objs, 0.0f);
    float front_dis = fabs(temp_tra[3]);

    temp_tra[3] = -1.0f * max_explore_dist;
    temp_tra[4] = 0;
    PK_PathVerify(temp_tra, objsNum, objs, 0.0f);
    float rear_dis = fabs(temp_tra[3]);

    int explor_num = 0;
    PK_CopyPos(temp_tra, finpoint);
    for (int i = 0; i < Explore_times; i++)
    {
        // 无限位杆并且是第一次探索
        if (i == 0 && firstDir <= 0)
        {
            PK_CopyPos(Exp_finpoint[0], finpoint);
            Exp_finpoint[0][4] = 0.0f;
            Exp_finpoint[0][3] = -1.0f * rear_dis;
            PK_PathVerify(Exp_finpoint[0], objsNum, objs, 0);

            PK_CopyPos(Exp_finpoint[1], finpoint);
            Exp_finpoint[1][4] = exploreRrmin * dir;
            Exp_finpoint[1][3] = -1.0f * rear_dis;
            PK_PathVerify(Exp_finpoint[1], objsNum, objs, 0);

            if (fabsf(Exp_finpoint[0][3]) - fabsf(Exp_finpoint[1][3]) > 0.5f)
            {
                PK_CopyTra(temp_tra, Exp_finpoint[0]);
            }
            else
            {
                PK_CopyTra(temp_tra, Exp_finpoint[1]);
            }
        }
        else
        {
            if (temp_tra[3] < ZERO_FLOAT)
            {
                // 上一次是向后
                float traMax = rte_min(front_dis * (1 + i), max_explore_dist);
                temp_tra[3]  = traMax;
                temp_tra[4]  = -exploreRrmin * dir;
            }
            else
            {
                float traMax = rte_min(rear_dis * (1 + i), max_explore_dist);
                temp_tra[3]  = -1.0f * traMax;
                temp_tra[4]  = exploreRrmin * dir;
            }

            PK_PathVerify(temp_tra, objsNum, objs, 0);
        }

        if (fabs(temp_tra[3]) > short_path_len)
        {
            PK_CopyTra(Exp_finpoint[explor_num], temp_tra);
            PK_Get_Path_EndPos(Exp_finpoint[explor_num], temp_tra);
            explor_num++;

            float lastTraj[5];
            PK_CopyTra(lastTraj, temp_tra);
            lastTraj[4] = 0;

            float theta = Project_PosTo1st_rtheta(finpoint, temp_tra);
            if (temp_tra[3] < ZERO_FLOAT && fabs(theta) > PI * 0.15f)
            {
                lastTraj[3]  = rte_max(front_dis * 1.2, 1.5);
                float oldLen = lastTraj[3];
                PK_PathVerify(lastTraj, objsNum, objs, 0.1);
                if (fabs(lastTraj[3] - oldLen) < 0.001f)
                {
                    break;
                }
            }
            else if (temp_tra[3] > ZERO_FLOAT && fabs(theta) > PI * 0.18f)
            {
                lastTraj[3]  = rte_min(front_dis * 2.0, 1.0);
                float oldLen = lastTraj[3];
                PK_PathVerify(lastTraj, objsNum, objs, 0.1);
                if (fabs(lastTraj[3] - oldLen) < 0.001f)
                {
                    break;
                }
            }
        }
    }

    if (explor_num >= Explore_times)
    {
        return 0;
    }

    for (int j = 0; j < explor_num; j++)
    {
        PathPlan_ReverseTraject(Exp_finpoint[explor_num - 1 - j]);
        PK_CopyTra(Exp_traj[j], Exp_finpoint[explor_num - 1 - j]);
    }

    PathPlanType(1, 1, explor_num);
    return explor_num;
}

int PathPlan_SpaceExplorer_Para(const PlanDataCase planData[1],
                                float Exp_traj[][TRAJITEM_LEN])
{
    float objs[SF_OBJ_NUM * 2 + 5][4];
    int objsnum = 0;

    int i = 0;
    while (i < SF_OBJ_NUM * 2 && i < planData[0].obj_danger_num)
    {
        memcpy(objs[i], planData[0].obj_danger[i], sizeof(float) * 4);
        i++;
    }

    int j = 1;
    while (i < SF_OBJ_NUM * 2 && j < 5)
    {
        memcpy(&objs[i], planData[0].obj_slot[j], sizeof(float) * 5 * 4);
        i++;
        j++;
    }
    objsnum = i;

    // uint8_t dir = (planData[0].has_stopper == 0) ? 0 : 1;
    auto pathNum = SpaceExplorer(planData[0].finpoint, planData[0].left_fac, 0, objs,
                                 objsnum, Exp_traj);
    return pathNum;
}

static int SpaceExplorer_Para1(const PlanDataCase PlanData_Ver[1],
                               const float Exp_stpoint[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                               int TrajNum_NS, float Exp_traj[][TRAJITEM_LEN])
{
    const float big_para_side_dist = 1.0f; // 大车位侧向距离
    float min_connect_dis          = 0.3f; // 最小连接距离
    int line_num                   = 0;
    float temp_tra[5]              = {0.0f};

    PK_CopyPos(temp_tra, PlanData_Ver[0].finpoint);

    // 车位很长，即车头到前一辆车距离超过big_para_side_dist
    if ((PlanData_Ver[0].front_dist > big_para_side_dist &&
         PlanData_Ver[0].is_vision_slot != 2) ||
        (PlanData_Ver[0].front_dist > 2.0 * big_para_side_dist &&
         PlanData_Ver[0].is_vision_slot == 2)) //  for a big slot
    {
        temp_tra[3] = 3.0 * (PlanData_Ver[0].front_dist - PlanData_Ver[0].swell / 2.0f);
        temp_tra[3] = rte_min(fabs(temp_tra[3]), 10.0f) * sign(temp_tra[3]);
        PathPlan_UpdateTrajLen(temp_tra, PlanData_Ver[0].finpoint,
                               PlanData_Ver[0].slotshape);

        while (fabs(temp_tra[3]) > 3.0)
        {
            temp_tra[4] =
                Rrmin * (-1) * PlanData_Ver[0].left_fac; // HAVECHANGE Tangsj 2023/5/29
            float trajEnd[3];
            PK_Get_Path_EndPos(temp_tra, trajEnd);
            auto offset = Project_PosTo1st_ry(PlanData_Ver[0].finpoint, trajEnd);
            if (fabs(offset) < 4.0)
            {
                break;
            }

            temp_tra[3] = (fabs(temp_tra[3]) - 0.5) * sign(temp_tra[3]);
        }

        line_num   = TraExplorer_Con(temp_tra, PlanData_Ver[0].stpoint, 3,
                                     &PlanData_Ver[0].obj_slot[1], Exp_traj, 0.80f, 1, 0,
                                     PlanData_Ver[0].swell, min_connect_dis);
        float dist = fabs(Exp_stpoint[TrajNum_NS - 1][3]);
        if (line_num <= 0 && dist > 2.0f * min_connect_dis) //使用备选起点
        {
            memcpy(Exp_traj, Exp_stpoint, PATH_ITEM_LEN * (TrajNum_NS - 1));
            line_num =
                TraExplorer_Con(temp_tra, Exp_stpoint[TrajNum_NS - 1], 3,
                                &PlanData_Ver[0].obj_slot[1], &Exp_traj[TrajNum_NS - 1],
                                0.80f, 1, 0, PlanData_Ver[0].swell, min_connect_dis);
            if (line_num > 0)
            {
                line_num += TrajNum_NS - 1;
                line_num = PK_PathMerge(Exp_traj, line_num, 0);
            }
        }
    }

    if (line_num > 0 &&
        PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                      PlanData_Ver[0].finpoint, 5, PlanData_Ver[0].obj_slot, 0) !=
            PATHPLAN_SUCCESS) // PlanData_Ver[0].swellAny small movement or Path damage
    {
        line_num = 0;
    }

    if (line_num > 0 &&
        PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                      PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                      PlanData_Ver[0].obj_danger,
                      0) == PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny small
                                              // movement or Path damage
    {
        return line_num;
    }
    return 0;
}

// the core function to plan a parallel slot
/**
 * @brief 获取路径规划的最终轨迹，根据车位类型和环境条件动态生成路径。
 *
 * @param PlanData_Ver 包含规划数据的数组，存储起点、终点、车位类型、障碍物等信息。
 * @param Exp_traj 用于存储最终生成的轨迹数组，二维数组格式，包含轨迹点的详细信息。
 *
 * @return 返回生成的轨迹线段数。如果路径规划失败，则返回 0。
 */
int PK_PathPlan_SpaceExplorer_Para(
    PlanDataCase PlanData_Ver[1],
    float Exp_traj[][TRAJITEM_LEN]) // HAVECHANGE Tangsj 2023/6/27
{
    // 初始化常量和局部变量
    const float Rmax           = 0.0f;  // 最大转弯半径 (已设置为 0，可能用于某种约束)
    const int Explore_times    = 16;    // 探索最大步数
    const float short_path_len = 0.10f; // 短路径长度阈值
    const float first_move_back_dist_min = 0.25f; // 初始后退距离的最小值

    float Ds_TraMin  = 0.3f; // 最小安全距离
    float Ds_TraMax  = 4.5f;
    float side_swell = 0;       // 侧向扩展系数
    int i, j, Cir_line_num = 0; // 轨迹计数变量
    float temp_tra[5] = {0.0f}; // front_ry_org;  //ds_1st_min,ds_1st_max,
    float Exp_finpoint[Explore_times][TRAJITEM_LEN] = {{0.0f}};
    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    int obj_all_num      = 0;                     // 障碍物数量和初始探索路径数量
    int explor_num       = 0;                     // 当前探索步数
    int is_explor_twocon = 1, is_explor_last = 1; // 探索标志变量

    float fy = Project_PosTo1st_ry(PlanData_Ver[0].finpoint, PlanData_Ver[0].stpoint);

    // 备选 - 先直线向前探索，得到新起点
    float Exp_stpoint[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN] = {{0.0f}};
    int TrajNum_NS = GetPath_StToMidPos_Na2(PlanData_Ver[0].stpoint, PlanData_Ver,
                                            Exp_stpoint, 0); // 直线向前

    // 统一障碍物
    i = 0;
    memset(obj_all, 0, sizeof(obj_all));
    while (i < SF_OBJ_NUM * 2 && i < PlanData_Ver[0].obj_danger_num)
    {
        memcpy(obj_all[i], PlanData_Ver[0].obj_danger[i], sizeof(float) * 4);
        i++;
    }

    j = 1;
    while (i < SF_OBJ_NUM * 2 && j < 5)
    {
        memcpy(&obj_all[i], PlanData_Ver[0].obj_slot[j], sizeof(float) * 5 * 4);
        i++;
        j++;
    }
    obj_all_num = i;

    // 延长EF
    // PK_PathPlan_Envi_obj_cut(PlanData_Ver[0].obj_slot, 1.0f, Envi_obj_side);   //get
    // BC,DE
    if (obj_all_num < SF_OBJ_NUM * 2)
    {
        float rspoint[3];
        RevConvert(PlanData_Ver[0].finpoint, &PlanData_Ver[0].obj_slot[4][2], rspoint);
        rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 2 * fabs(Exp_stpoint[0][3]));
        Convert(PlanData_Ver[0].finpoint, rspoint, &obj_all[obj_all_num - 1][2]);
    }

    // 车位很长，即车头到前一辆车距离超过big_para_side_dist
    int line_num = SpaceExplorer_Para1(PlanData_Ver, Exp_stpoint, TrajNum_NS, Exp_traj);
    if (line_num > 0)
    {
        PathPlanType(0, 2, line_num);
        return line_num;
    }

    PK_CopyPos(temp_tra, PlanData_Ver[0].finpoint);
    for (i = 0, line_num = 0; i < Explore_times; i++)
    {
        is_explor_twocon = 1; // 是否需要两段连接
        is_explor_last   = 1; // 上一次路径探索是否成功

        float rtheta =
            PI / 2 - fabs(Project_PosTo1st_rtheta(PlanData_Ver[0].finpoint, temp_tra));
        float rear_dis =
            fabsf(Project_PosTo1st_rx(temp_tra, &PlanData_Ver[0].obj_slot[2][0])) -
            PlanData_Ver[0].h; // C点
        float front_dis =
            fabsf(Project_PosTo1st_rx(temp_tra, &PlanData_Ver[0].obj_slot[4][0])) -
            PlanData_Ver[0].len + PlanData_Ver[0].h; // E点

        if (i % 2 == 1) // explore forward
        {
            Ds_TraMax = rte_min(front_dis, 5.5f);
        }
        else // explore backward
        {
            Ds_TraMax = rte_min(rear_dis, 5.5f);
        }

        if (i == 0 &&
            PlanData_Ver[0].has_stopper ==
                0) //  the first explore step is to move back,and it is the first path
        {
            float explore[2][5];
            float ds_norm = -rte_min(
                rte_max((rear_dis - PlanData_Ver[0].swell), Ds_TraMin), Ds_TraMax);

            PK_CopyPos(explore[0], temp_tra);
            explore[0][3] = ds_norm;
            explore[0][4] = Rrmin * PlanData_Ver[0].left_fac; //  use rte_min turn radium
            PK_PathVerify(explore[0], 5, PlanData_Ver[0].obj_slot, side_swell);

            PK_CopyPos(explore[1], temp_tra);
            explore[1][4] = Rmax * PlanData_Ver[0].left_fac; //  use rte_max turn radium
            explore[1][3] = ds_norm;
            PK_PathVerify(explore[1], 5, PlanData_Ver[0].obj_slot, side_swell);

            if (fabsf(explore[1][3]) - fabsf(explore[0][3]) > 0.5f)
            {
                PK_CopyTra(temp_tra, explore[1]);
            }
            else
            {
                PK_CopyTra(temp_tra, explore[0]);
            }
        }
        else
        {
            if (i % 2 == 1) // explore forward
            {
                float temp_tra_endpos[3];
                temp_tra[3] = rte_min(
                    rte_max((front_dis - PlanData_Ver[0].swell / 2.0f), Ds_TraMin),
                    Ds_TraMax); // 20180718:why here use swell/2.0?
                temp_tra[4] = -Rrmin * PlanData_Ver[0].left_fac;
                PK_Get_Path_EndPos(temp_tra, temp_tra_endpos);
                float front_ry =
                    Project_PosTo1st_ry(temp_tra_endpos, PlanData_Ver[0].pointf_danger);

                float min_connect_dis = 0.0f;
                if (fabsf(front_ry) > (PlanData_Ver[0].width / 2 + 0.05f) &&
                    (front_ry * PlanData_Ver[0].left_fac >
                     0)) // 20180718: and here use 1.0*swell ?
                {
                    // HAVECHANGE Tangsj 2023/6/7
                    min_connect_dis = 0.3f;
                    // temp_tra[3] = sign(temp_tra[3]) * fabs(rtheta) * Rrmin;

                    // 修改探索距离长度
                    float trajlen0 = fabs(rtheta) * Rrmin;
                    float trajlen1 = rte_min(fabs(temp_tra[3]), 0.8f);
                    float trajlen  = rte_max(trajlen0, trajlen1);
                    temp_tra[3]    = sign(temp_tra[3]) * trajlen;
                }
                else
                {
                    temp_tra[3] = sign(temp_tra[3]) * rte_max(fabs(rtheta) * Rrmin, 0.8f);
                    min_connect_dis = 0.3f;
                }

                is_explor_last = PathPlan_PathVerify2(
                    temp_tra, 1, &PlanData_Ver[0].obj_slot[3], side_swell);
                PK_PathVerify(temp_tra, obj_all_num, obj_all, side_swell);

                if (is_explor_last == PATHPLAN_SUCCESS)
                {
                    Cir_line_num =
                        TraExplorer_Con(temp_tra, PlanData_Ver[0].stpoint, 3,
                                        &PlanData_Ver[0].obj_slot[1], Exp_traj, 0.8f, 1,
                                        0, PlanData_Ver[0].swell, min_connect_dis);
                }

                if (Cir_line_num > 0)
                {
                    int result =
                        PathPlan_PathVerify2(Exp_traj[Cir_line_num - 1], 3,
                                             &PlanData_Ver[0].obj_slot[1], side_swell);
                    // int result = PK_Check_Path(1, &Exp_traj[Cir_line_num - 1],
                    // &Exp_traj[Cir_line_num - 1][0], pathEnd, 2, Envi_obj_side, 0);
                    if (result == PATHPLAN_SUCCESS)
                    {
                        result =
                            PK_Check_Path(Cir_line_num, Exp_traj, PlanData_Ver[0].stpoint,
                                          temp_tra, PlanData_Ver[0].obj_danger_num,
                                          PlanData_Ver[0].obj_danger, 0);
                        if (result !=
                            PATHPLAN_SUCCESS) // 障碍物校验不通过，可能是对向空间受限
                        {
                            is_explor_twocon = 0;
                        }
                    }

                    if (result != PATHPLAN_SUCCESS)
                    {
                        Cir_line_num = 0;
                    }
                }

                if (Cir_line_num == 0 && is_explor_last == PATHPLAN_SUCCESS &&
                    is_explor_twocon > 0)
                {
                    memcpy(Exp_traj, Exp_stpoint, PATH_ITEM_LEN * (TrajNum_NS - 1));
                    Cir_line_num = TraExplorer_Con(
                        temp_tra, Exp_stpoint[TrajNum_NS - 1], 3,
                        &PlanData_Ver[0].obj_slot[1], &Exp_traj[TrajNum_NS - 1], 0.80f, 1,
                        0, PlanData_Ver[0].swell, min_connect_dis);
                    if (Cir_line_num > 0)
                    {
                        Cir_line_num += TrajNum_NS - 1;
                        Cir_line_num = PK_PathMerge(Exp_traj, Cir_line_num, 0);
                    }

                    if (Cir_line_num > 0)
                    {
                        int result = PathPlan_PathVerify2(Exp_traj[Cir_line_num - 1], 3,
                                                          &PlanData_Ver[0].obj_slot[1],
                                                          side_swell);
                        if (result == PATHPLAN_SUCCESS)
                        {
                            result = PK_Check_Path(Cir_line_num, Exp_traj,
                                                   PlanData_Ver[0].stpoint, temp_tra,
                                                   PlanData_Ver[0].obj_danger_num,
                                                   PlanData_Ver[0].obj_danger, 0);
                            if (result != PATHPLAN_SUCCESS)
                            {
                                is_explor_twocon = 0;
                            }
                        }

                        if (result != PATHPLAN_SUCCESS)
                        {
                            Cir_line_num = 0;
                        }
                    }
                }

                if (Cir_line_num == 0 && is_explor_twocon > 0)
                {
                    int num2         = 0;
                    float rspoint[5] = {0};
                    float fx2        = Project_PosTo1st_rx(PlanData_Ver[0].pointf_danger,
                                                           Exp_stpoint[TrajNum_NS - 1]);
                    float fy2        = Project_PosTo1st_ry(PlanData_Ver[0].pointf_danger,
                                                           Exp_stpoint[TrajNum_NS - 1]);

                    memcpy(Exp_traj, Exp_stpoint, PATH_ITEM_LEN * (TrajNum_NS - 1));

                    for (int k = 0; k < 10; k++)
                    {
                        float midpos[3];
                        rspoint[0] = fx2 > 2.50 ? fx2 : 2.50;
                        rspoint[1] = sign(fy) * (fabs(fy2) - 0.10 * k);
                        Convert(PlanData_Ver->pointf_danger, rspoint, midpos);
                        midpos[2] = PlanData_Ver->pointf_danger[2];

                        Cir_line_num =
                            TraExplorer_Con(temp_tra, midpos, obj_all_num, obj_all,
                                            &Exp_stpoint[TrajNum_NS], 0.8f, 1, 0,
                                            PlanData_Ver[0].swell, min_connect_dis);

                        if (Cir_line_num > 0)
                        {
                            if (fabs(Exp_stpoint[TrajNum_NS][4]) < 1 &&
                                fabs(Exp_stpoint[TrajNum_NS][3]) >
                                    1) // 先走直线,省去一段直线路径
                            {
                                PK_CopyPos(Exp_stpoint[TrajNum_NS],
                                           Exp_stpoint[TrajNum_NS + 1]);
                                Exp_stpoint[TrajNum_NS][3] = 0.5;
                                Exp_stpoint[TrajNum_NS][4] = 0;
                                PathPlan_ReverseTraject(Exp_stpoint[TrajNum_NS]);
                            }

                            num2 = PK_PathTryConnect_TwoCon(0, Exp_stpoint[TrajNum_NS],
                                                            PlanData_Ver, Exp_traj);

                            if (num2 > 0)
                            {
                                memcpy(&Exp_traj[num2], &Exp_stpoint[TrajNum_NS],
                                       sizeof(Exp_stpoint[1]) * Cir_line_num);
                                Cir_line_num += num2;
                                Cir_line_num = PK_PathMerge(Exp_traj, Cir_line_num, 0);
                                break;
                            }
                        }
                    }

                    if (Cir_line_num > 0)
                    {
                        int result = PathPlan_PathVerify2(Exp_traj[Cir_line_num - 1], 3,
                                                          &PlanData_Ver[0].obj_slot[1],
                                                          side_swell);
                        // int result = PK_Check_Path(1, &Exp_traj[Cir_line_num - 1],
                        // &Exp_traj[Cir_line_num - 1][0], pathEnd, 2, Envi_obj_side, 0);
                        if (result == PATHPLAN_SUCCESS)
                        {
                            result = PK_Check_Path(Cir_line_num, Exp_traj,
                                                   PlanData_Ver[0].stpoint, temp_tra,
                                                   obj_all_num, obj_all, 0);
                        }

                        if (result != PATHPLAN_SUCCESS)
                        {
                            Cir_line_num = 0;
                        }
                    }
                }
            }
            else if (i > 0) // explore backward     change 2023/6/27
            {
                temp_tra[3] = -rte_min(
                    rte_max((rear_dis - PlanData_Ver[0].swell), Ds_TraMin), Ds_TraMax);
                temp_tra[3] = -rte_max(fabsf(temp_tra[3]), 1.0);
                temp_tra[4] = Rrmin * PlanData_Ver[0].left_fac;
                PK_PathVerify(temp_tra, 5, PlanData_Ver[0].obj_slot, side_swell);

                if (PlanData_Ver[0].has_stopper == 1)
                {
                    PathPlan_LandMarkByStopper(temp_tra, PlanData_Ver[0].obj_stopper);
                }

                float min_connect_dis = 0.0f;
                float temp_tra_endpos[3];
                PK_Get_Path_EndPos(temp_tra,
                                   temp_tra_endpos); // HAVECHANGE Tangsj 2023/6/7
                float front_ry =
                    Project_PosTo1st_ry(temp_tra_endpos, PlanData_Ver[0].pointf_danger);
                if (fabsf(front_ry) > (PlanData_Ver[0].width / 2 + +0.05f) &&
                    (front_ry * PlanData_Ver[0].left_fac >
                     0)) // 20180718: and here use 1.0*swell ?
                {
                    min_connect_dis = 0.3f;
                    is_explor_last  = 0;
                }

                if (!is_explor_last)
                {
                    Cir_line_num = PK_PathPlan_TraExplorer_Con(
                        temp_tra, PlanData_Ver[0].stpoint, 3,
                        &PlanData_Ver[0].obj_slot[1], Exp_traj, 0, 0,
                        PlanData_Ver[0].swell, min_connect_dis);
                }

                if (Cir_line_num > 0)
                {
                    float pathEnd[3];
                    PK_Get_Path_EndPos(Exp_traj[Cir_line_num - 1], pathEnd);
                    int result = PK_Check_Path(1, &Exp_traj[Cir_line_num - 1],
                                               &Exp_traj[Cir_line_num - 1][0], pathEnd, 3,
                                               &PlanData_Ver[0].obj_slot[1], 0);
                    if (result == PATHPLAN_SUCCESS)
                    {
                        result =
                            PK_Check_Path(Cir_line_num, Exp_traj, PlanData_Ver[0].stpoint,
                                          temp_tra, PlanData_Ver[0].obj_danger_num,
                                          PlanData_Ver[0].obj_danger, 0);
                    }
                    if (result != PATHPLAN_SUCCESS)
                    {
                        Cir_line_num = 0;
                    }
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (Cir_line_num > 0)
        {
            for (j = 0; j < explor_num; j++)
            {
                PathPlan_ReverseTraject(Exp_finpoint[explor_num - 1 - j]);
                PK_CopyTra(Exp_traj[Cir_line_num + j], Exp_finpoint[explor_num - 1 - j]);
            }

            line_num = Cir_line_num +
                       explor_num; //  the explored path is contained by Cir_line_num
            //  2018/07/19:should move the check_path function to above "if
            //  (Cir_line_num>0)" and only if check_path pass can the "break" be used
            if (PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                              PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                              PlanData_Ver[0].obj_danger,
                              0) != PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny
                                                      // small movement or Path damage
            {
                line_num = 0;
            }
            else
            {
                PathPlanType(0, 3, line_num);
                break; //  have found a valid path
            }
        }
        // else if ((explor_num == 0 && ((PlanData_Ver[0].has_stopper != 1 &&
        // fabs(temp_tra[3]) > ))) //  first explor path move back should be enough dist
        // or move forward with no dist limit
        //          || (explor_num > 0 && fabsf(temp_tra[3]) > short_path_len))
        if (fabsf(temp_tra[3]) > short_path_len)
        {
            PK_CopyTra(Exp_finpoint[explor_num], temp_tra); // store all explored path
            PK_Get_Path_EndPos(
                temp_tra,
                temp_tra); //  change temp_tra's start point to last temp_tra's end point
            explor_num++;
        }
    }

    PathPlanType(0, 4, line_num);
    return line_num;
}
