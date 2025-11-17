#include "PK_PathPlanP.h"
#include "PathPlan_Update.h"
#include "PathPlan_Debug.h"
#include "PathPlan_BasePlan.h"
#include "PK_Calibration.h"
#include "MathFunc.h"
#include "PK_PathExecute.h"
#include "Record_Log.h"

inline static void PathPlanType(uint8_t level, uint8_t type, int num)
{
    PathPlan_FootPrints(3, level, type, num);
}

/*
功能描述：输出起点直线移动的一段路径，获取新的规划起点
函数名称：Stpoint_Extend
INTPUT:     PlanDataCase， 直线移动的距离extend_ds
OUTPUT:     路径Exp_stpoint
*/
int Stpoint_Extend(PlanDataCase PlanData_Ver[1], float expTraj[][TRAJITEM_LEN],
                   float extend_ds)
{
    PK_CopyPos(expTraj[0], PlanData_Ver[0].stpoint);
    expTraj[0][3] = extend_ds;
    expTraj[0][4] = 0;
    PathPlan_UpdateTrajLen(expTraj[0], PlanData_Ver[0].finpoint,
                           PlanData_Ver[0].slotshape);
    PathPlan_LandMark2(expTraj[0], 3, &PlanData_Ver[0].obj_slot[1], 0);
    PathPlan_LandMark2(expTraj[0], PlanData_Ver[0].obj_danger_num,
                       PlanData_Ver[0].obj_danger, 0);

    PK_Get_Path_EndPos(expTraj[0], expTraj[1]); // get rear_stpoint
    return 1;
}

void PathPlan_UpdateTrajLen(float traject[TRAJITEM_LEN], const float finpoint[3],
                            uint8_t slotshape)
{
    float endPos[3];
    PK_Get_Path_EndPos(traject, endPos);
    float rsx = Project_PosTo1st_rx(finpoint, traject);
    float rfx = Project_PosTo1st_rx(finpoint, endPos);

    traject[3] = rte_min(fabs(traject[3]), PATH_LENGTH_MAX) * sign(traject[3]);
    if (slotshape != PK_SLOT_RIGHT_PARA && slotshape != PK_SLOT_LEFT_PARA)
    {
        // X 方向
        float vehfront[2], rvehfront[2];
        vehfront[0] = VEHICLE_LEN - REAR_SUSPENSION;
        vehfront[1] = 0;
        Convert(endPos, vehfront, rvehfront);
        float rvehfx = Project_PosTo1st_rx(finpoint, rvehfront);
        while ((fabs(traject[3]) > PATH_LAST_STRAIGHT_MIN) &&
               ((rfx > 2.5f * VEHICLE_LEN || rvehfx > 2.5f * VEHICLE_LEN) && rfx > rsx))
        {
            traject[3] =
                (fabs(traject[3]) - 0.9 * PATH_LAST_STRAIGHT_MIN) * sign(traject[3]);
            PK_Get_Path_EndPos(traject, endPos);
            rfx = Project_PosTo1st_rx(finpoint, endPos);
            Convert(endPos, vehfront, rvehfront);
            rvehfx = Project_PosTo1st_rx(finpoint, rvehfront);
        }

        // Y方向
        float rsy = Project_PosTo1st_ry(finpoint, traject);
        float rfy = Project_PosTo1st_ry(finpoint, endPos);
        while ((fabs(traject[3]) > PATH_LAST_STRAIGHT_MIN) &&
               (fabs(rfy) > 2.5f * VEHICLE_LEN && fabs(rfy) > fabs(rsy)))
        {
            traject[3] =
                (fabs(traject[3]) - 0.9 * PATH_LAST_STRAIGHT_MIN) * sign(traject[3]);
            PK_Get_Path_EndPos(traject, endPos);
            rfy = Project_PosTo1st_ry(finpoint, endPos);
        }
    }
    else
    {
        // Y方向
        float rsy = Project_PosTo1st_ry(finpoint, traject);
        float rfy = Project_PosTo1st_ry(finpoint, endPos);
        while ((fabs(traject[3]) > PATH_LAST_STRAIGHT_MIN) &&
               (fabs(rfy) > 1.5 * VEHICLE_WID && fabs(rfy) > fabs(rsy)))
        {
            traject[3] =
                (fabs(traject[3]) - 0.9 * PATH_LAST_STRAIGHT_MIN) * sign(traject[3]);
            PK_Get_Path_EndPos(traject, endPos);
            rfy = Project_PosTo1st_ry(finpoint, endPos);
        }
    }
}

/*
add 2023/3/21 Tangsj
功能描述：根据最小转弯半径，初步判断路径段temp_tra中可以作为中间路径节点（新起点）的位置--更改自函数PK_PathPlan_TraExplorer_Con
函数名称：PK_PathPlan_TraExplorer_Con2
INTPUT:     模式B_tra_to_pos：0-从这一段路径起点开始判断，1-从这一段路径终点开始判断
            路径段temp_tra，泊车目标点tar_pos，车位边界入口前方点坐标pointf_danger，路径stp_trajs
OUTPUT:     路径stp_trajs，返回值: 0-没有可以作为中间路径节点的位置， 1-获取成功
*/
static int PK_PathPlan_TraExplorer_Con2(int B_tra_to_pos, float temp_tra[TRAJITEM_LEN],
                                        const float tar_pos[3],
                                        const float point_danger[3],
                                        float stp_trajs[][TRAJITEM_LEN], float start_ds)
{
    int points_num = 0;
    int j          = 0;
    static float Explorpos_arr[30][4]; // X,Y,THETA,ds
    const float dsmin = 0.3f;          //   the minimun dist to generate a step point
    float sign_start_ds;
    sign_start_ds =
        sign(temp_tra[3]) *
        fabsf(start_ds); // keep direction the same, to decide to go forward or backward

    float dsminv = dsmin;
    int number   = (fabs(temp_tra[3]) - fabs(start_ds)) / dsmin;
    if (number < 7)
    {
        dsminv = 0.15;
    }
    else if (number < 4)
    {
        dsminv = 0.1;
    }

    points_num = PathPlan_DrawTrajectory(
        dsminv, temp_tra, Explorpos_arr,
        sign_start_ds); // 20180619: maybe it's no need to calculate these points first
    if (B_tra_to_pos == 0)
    {
        for (j = 0; j < points_num; j++)
        {
            // float rtheta = Round_PI(Project_PosTo1st_rtheta(tar_pos,
            // Explorpos_arr[j]));
            float rtheta2 = Project_PosTo1st_rtheta(point_danger, Explorpos_arr[j]);
            float rx      = Project_PosTo1st_ry(tar_pos, Explorpos_arr[j]);
            float ry      = Project_PosTo1st_ry(Explorpos_arr[j], point_danger);
            if (fabs(ry) > (VEHICLE_WID / 2 + 0.2) && sign(rx) == sign(ry) &&
                fabsf(rtheta2) >
                    PI /
                        2 /*&& fabs(rx) > dx*/) // r =
                                                // dmin/(1-cosf(rtheta));保证车辆可以以最小转弯半径泊入车位的距离
            {
                temp_tra[3] = Explorpos_arr[j][3];
                if (PathPlan_CheckTrajValid(temp_tra) != PATHPLAN_SUCCESS)
                {
                    log_err("PK_PathPlan_TraExplorer_Con2 Check:trj:%f,%f,%f,%f,%f\n",
                            temp_tra[0], temp_tra[1], temp_tra[2], temp_tra[3],
                            temp_tra[4]);
                    return 0;
                }

                PathPlanType(0, 1, 1);
                return 1;
            }
        }
    }
    else
    {
        for (j = points_num - 1; j >= 0; j--)
        {
            float rtheta = Project_PosTo1st_rtheta(tar_pos, Explorpos_arr[j]);
            float rx     = Project_PosTo1st_ry(tar_pos, Explorpos_arr[j]);
            // float rx = Explorpos_arr[j][0] - tar_pos[0];
            float ry = Project_PosTo1st_ry(Explorpos_arr[j], point_danger);
            if (fabs(ry) > (VEHICLE_WID / 2 + 0.2) && sign(rx) == sign(ry))
            {
                if (fabs(rx) > fabs(Rrmin * (1 - cosf(rtheta))) &&
                    (rtheta <= PI_2 ||
                     rtheta >=
                         3 * PI_2)) // r =
                                    // dmin/(1-cosf(rtheta));保证车辆可以以最小转弯半径泊入车位的距离
                {
                    temp_tra[3] = Explorpos_arr[j][3];
                    if (PathPlan_CheckTrajValid(temp_tra) != PATHPLAN_SUCCESS)
                    {
                        log_err("PK_PathPlan_TraExplorer_Con2 Check:trj:%f,%f,%f,%f,%f\n",
                                temp_tra[0], temp_tra[1], temp_tra[2], temp_tra[3],
                                temp_tra[4]);
                        return 0;
                    }

                    PathPlanType(0, 2, 1);
                    return 1;
                }
            }
        }
    }

    return 0;
}

/**
 * @brief 生成从起始位置到中间位置的路径，考虑潜在障碍物。
 *
 * 计算从起始点（`stpoint`）到中间点的轨迹，同时考虑潜在的障碍物、车位形状和转弯半径约束。
 * 路径生成包括前向行驶、调整以适应车位边界，并确保路径避免碰撞。
 *
 * @param stpoint 起始点的坐标，包含 5 个元素的数组：[x, y, theta, distance, curvature]。
 * @param PlanData_Ver
 * 包含当前规划数据的结构体数组，包含危险点、终点、车位形状等相关信息。
 * @param StToMid_Traj 存储生成轨迹的二维数组，包含每个轨迹点的详细信息。
 * @param radius 当前转弯半径，用于计算曲线路径。
 * @param type 路径类型标志，当为 0 时为普通车位；当为 1 时表示断头车位（前方有障碍物）。
 *
 * @return 返回生成的轨迹点数量，如果未能生成有效路径，返回 0。
 */
int GetPath_StToMidPos_Na2(float stpoint[5], PlanDataCase PlanData_Ver[1],
                           float StToMid_Traj[][TRAJITEM_LEN], float radius, int type)
{
    Point_T SlotE, newCurPt, newCurPt_1;
    float R_inner, crossAngle, E_Cir_theta, ry, rx1, fx, rtheta, dist, rx_cir, CirPt[2];
    const float ds_err        = 0.01f; //  to avoid can't pass the collision check
    float verCurPosLeftVec[2] = {-sinf(stpoint[2]), cosf(stpoint[2])};
    float EndPos[3];
    int TrajNum_NS = 0, result = 0;
    float run_dist_max     = 0;
    float Rr               = Rrmin + 0.4;
    float explor_front_min = 0.3;
    memcpy(StToMid_Traj[0], stpoint, 3 * sizeof(float));

    fx                 = Project_PosTo1st_rx(stpoint, PlanData_Ver[0].pointf_danger);
    float FinMidPos[3] = {
        (PlanData_Ver[0].pointb_danger[0] + PlanData_Ver[0].pointf_danger[0]) / 2,
        (PlanData_Ver[0].pointb_danger[1] + PlanData_Ver[0].pointb_danger[1]) / 2,
        PlanData_Ver[0].finpoint[2]};
    rx1    = Project_PosTo1st_rx(stpoint, FinMidPos);
    rtheta = Round_PI(Project_PosTo1st_rtheta(PlanData_Ver[0].finpoint, StToMid_Traj[0]));
    if (type) //前方有障碍物（断头车位）
    {
        PathPlanType(1, 1, 1);
        Stpoint_Extend(PlanData_Ver, StToMid_Traj, explor_front_min);
        TrajNum_NS   = 1;
        run_dist_max = 3.5;
    }
    else if (radius)
    {
        if (rx1 >= explor_front_min)
        {
            PathPlanType(1, 2, 1);
            Stpoint_Extend(PlanData_Ver, StToMid_Traj, rx1);
            TrajNum_NS = 1;
        }
        else if (rx1 < explor_front_min)
        {
            PathPlanType(1, 3, 1);
            Stpoint_Extend(PlanData_Ver, StToMid_Traj, explor_front_min);
            TrajNum_NS = 1;
        }

        run_dist_max = rte_min(fabs(0.3f * radius), 3.5);
    }
    else if (PlanData_Ver[0].slotshape <= 2)
    {
        run_dist_max = fx + 5;
    }
    else if (fabs(rtheta) < PI * 0.4 || fabs(rtheta) > PI * 0.6)
    {
        if (PlanData_Ver[0].slotshape > 2 && fabs(rtheta) > PI * 0.6)
        {
            run_dist_max = fx + 10;
        }
        else
        {
            run_dist_max = fx + 10;
        }
    }
    else //直线
    {
        SlotE.x      = PlanData_Ver[0].obj_slot[4][0];
        SlotE.y      = PlanData_Ver[0].obj_slot[4][1];
        R_inner      = Rr - VEHICLE_WID / 2;
        ry           = Project_PosTo1st_ry(stpoint, PlanData_Ver[0].finpoint);
        newCurPt.x   = stpoint[0] + sign(ry) * verCurPosLeftVec[0] * Rr;
        newCurPt.y   = stpoint[1] + sign(ry) * verCurPosLeftVec[1] * Rr;
        newCurPt_1.x = newCurPt.x + cosf(stpoint[2]);
        newCurPt_1.y = newCurPt.y + sinf(stpoint[2]);

        Get_Dist_Dir_Pt2PointLine(newCurPt, newCurPt_1, SlotE, &dist, NULL);
        if (dist / R_inner < 1) // asin()中的參數不能大於1
        {
            crossAngle  = asin(dist / R_inner);
            E_Cir_theta = stpoint[2] + sign(ry) * crossAngle;
            CirPt[0]    = SlotE.x + R_inner * cosf(E_Cir_theta);
            CirPt[1]    = SlotE.y + R_inner * sinf(E_Cir_theta);
            rx_cir      = Project_PosTo1st_rx(stpoint, CirPt);

            run_dist_max = rx_cir + sign(rx_cir) * ds_err;
        }
        else
        {
            run_dist_max = fx + 5.0;
        }
    }
    run_dist_max = rte_max(run_dist_max, PATH_LAST_STRAIGHT_MIN);
    run_dist_max = rte_min(run_dist_max, 2.0 * VEHICLE_LEN);

    StToMid_Traj[TrajNum_NS][3] = run_dist_max;
    StToMid_Traj[TrajNum_NS][4] = radius;
    PathPlan_UpdateTrajLen(StToMid_Traj[TrajNum_NS], PlanData_Ver[0].finpoint,
                           PlanData_Ver[0].slotshape);
    PathPlan_LandMark2(StToMid_Traj[TrajNum_NS], 5, PlanData_Ver[0].obj_slot, 0);
    PathPlan_LandMark2(StToMid_Traj[TrajNum_NS], PlanData_Ver[0].obj_danger_num,
                       PlanData_Ver[0].obj_danger, 0);

    if (radius)
    {
        result = PK_PathPlan_TraExplorer_Con2(
            1, StToMid_Traj[TrajNum_NS], PlanData_Ver[0].finpoint,
            PlanData_Ver[0].pointf_danger, &StToMid_Traj[TrajNum_NS], 0.0f);
        PathPlanType(1, 4, result);
    }

    TrajNum_NS += 1;
    if (result || radius < 1 || type)
    {
        PK_Get_Path_EndPos(StToMid_Traj[TrajNum_NS - 1], EndPos);
        memcpy(StToMid_Traj[TrajNum_NS], EndPos, 3 * sizeof(float));
        TrajNum_NS += 1;
        PathPlanType(0, 3, TrajNum_NS);
        return TrajNum_NS;
    }

    return 0;
}

/**
 * @brief
 * 规划从起始点到终点的最佳路径。路径可以是直线、圆弧或它们的组合，并进行障碍物避让和路径验证。
 * @param   stpoint [3]       起始点的坐标和角度 {x, y, theta}。
 * @param   finpoint [3]      终点的坐标和角度 {x, y, theta}。
 * @param   obj_num           障碍物的数量。
 * @param   obj_slot[][4]     障碍物的信息数组，每个障碍物由四个参数描述。
 * @param   traject[][TRAJITEM_LEN] 存储生成的轨迹信息，每行描述一段路径。
 * @param   swell             障碍物膨胀量，用于避障。
 * @param   radiusSwell       最小转弯半径的增量，用于调整路径规划的半径。
 * @return  返回生成的路径段数，如果无法生成路径则返回 0。
 */
int PK_PathPlan_BetterTra(const float stpoint[3], const float finpoint[3],
                          const int obj_num, const float obj_slot[][4],
                          float traject[][TRAJITEM_LEN], const float swell,
                          const float radiusSwell)
{
    // 初始化变量
    float r, rx, ry, rtheta, fy, fx, dmin, dmax;        // 用于计算相对位置、角度和距离
    float x0, y0, theta0, x, y, theta, sl;              // 起点坐标、当前点坐标和路径长度
    float dyfin, ds;                                    // 中间变量
    float Rfin                   = Rrmin + radiusSwell; // 调整后的终点转弯半径
    int line_num                 = 0;                   // 路径段数
    float dmin_3line             = 0.0f;                // 用于 3 段路径判断的最小距离
    const float vert_d_theta_max = PI * 2 / 3;          // 最大垂直偏差角
    const float SameLine_ry_max  = 0.03f;               // 判断是否为直线路径的最大偏移量
    const float SameLine_rth_min = 0.01f;               // 判断是否为直线路径的最小偏差角

    // 获取起始点到终点的相对位置和角度
    x0     = stpoint[0];
    y0     = stpoint[1];
    theta0 = stpoint[2];
    rx     = Project_PosTo1st_rx(stpoint, finpoint);
    ry     = Project_PosTo1st_ry(stpoint, finpoint);
    fy     = Project_PosTo1st_ry(finpoint, stpoint);
    fx     = Project_PosTo1st_rx(finpoint, stpoint);
    rtheta = Project_PosTo1st_rtheta(stpoint, finpoint);
    dmin   = rte_min(fabsf(ry), fabsf(fy));
    dmax   = rte_max(fabsf(ry), fabsf(fy));

    // 判断路径类型
    if ((ry * fy < 0 && dmax >= SameLine_ry_max) ||
        fabsf(rtheta) > vert_d_theta_max) // 无法生成路径
    {
        line_num = 0;
    }
    else if (dmax < SameLine_ry_max && fabsf(rtheta) < SameLine_rth_min) // 直线路径
    {
        ds            = rx; // 路径长度
        traject[0][0] = x0;
        traject[0][1] = y0;
        traject[0][2] = theta0;
        traject[0][3] = ds; // 距离
        traject[0][4] = 0;  // 曲率为 0
        line_num      = 1;

        // 检查路径是否与障碍物相交
        if (PK_PosObjCross(traject[0], obj_num, obj_slot, ds, swell) > 0)
        {
            line_num = 0;
        }
    }
    else if (rx * ry * rtheta > 0) // 圆弧路径
    {
        if (fabsf(rtheta) < SameLine_rth_min)
        {
            return 0;
        }

        // 计算曲率半径
        r = dmin / (1 - cosf(rtheta));
        if (r < Rrmin) // 无法满足最小转弯半径
        {
            line_num = 0;
        }
        else
        {
            // 计算终点到路径末端的偏移和路径长度
            dyfin = (1 - cosf(rtheta)) * Rfin;
            ds    = (fabsf(ry) - fabsf(dyfin)) / fabsf(sinf(rtheta));
            ds    = sign(rx) * ds;

            // 设置路径终点
            traject[2][0] = finpoint[0] - cosf(finpoint[2]) * ds;
            traject[2][1] = finpoint[1] - sinf(finpoint[2]) * ds;
            traject[2][2] = finpoint[2];
            traject[2][3] = ds;
            traject[2][4] = 0;

            dmin_3line = rte_min(fabsf(Project_PosTo1st_ry(stpoint, traject[2])),
                                 fabsf(Project_PosTo1st_ry(traject[2], stpoint)));

            // 判断路径类型
            if (fabsf(ds) > 0.45f && fabsf(dmin_3line) > 0.1f)
            {
                line_num = 3;
                rx       = Project_PosTo1st_rx(stpoint, traject[2]);
                ry       = Project_PosTo1st_ry(stpoint, traject[2]); // relative pos
                fy       = Project_PosTo1st_ry(traject[2], stpoint);
                fx       = Project_PosTo1st_rx(traject[2], stpoint);
                rtheta   = Project_PosTo1st_rtheta(stpoint, traject[2]);
                dmin     = rte_min(fabsf(ry), fabsf(fy));
                dmax     = rte_max(fabsf(ry), fabsf(fy));
                r        = dmin / (1 - cosf(rtheta));
                x        = traject[2][0];
                y        = traject[2][1];
                theta    = traject[2][2];
            }
            else
            {
                line_num = 2;
                x        = finpoint[0];
                y        = finpoint[1];
                theta    = finpoint[2];
            }

            // 计算路径细节
            sl = fabsf(fabsf(dmax / sinf(rtheta)) - fabsf(r * tanf(rtheta / 2)));
            if (fabsf(fy) >= fabsf(ry)) // 直线 + 圆弧
            {
                traject[0][0] = x0;
                traject[0][1] = y0;
                traject[0][2] = theta0;
                sl            = sl * sign(rx);
                traject[0][3] = sl;
                traject[0][4] = 0;
                traject[1][0] = x0 + sl * cosf(theta0);
                traject[1][1] = y0 + sl * sinf(theta0);
                traject[1][2] = theta0;
                traject[1][4] = sign(ry) * r;
                traject[1][3] = sign(rx) * fabsf(rtheta * r);
            }
            else // 圆弧 + 直线
            {
                traject[0][0] = x0;
                traject[0][1] = y0;
                traject[0][2] = theta0;
                traject[0][3] = -sign(fx) * fabsf(rtheta * r);
                traject[0][4] = sign(ry) * r;
                sl            = -sl * sign(fx);
                traject[1][3] = sl;
                traject[1][4] = 0;
                traject[1][0] = x - sl * cosf(theta);
                traject[1][1] = y - sl * sinf(theta);
                traject[1][2] = theta;
            }

            // 检查路径是否有效
            auto check = PK_Check_Path(line_num, traject, stpoint, finpoint, obj_num,
                                       obj_slot, swell);
            if (check != PATHPLAN_SUCCESS)
            {
                line_num = 0;
            }

            for (int i = 0; i < line_num; i++)
            {
                // printf("check %d %f,%f,%f,%f,%f\n", check, traject[i][0],
                // traject[i][2], traject[i][3], traject[i][3], traject[i][4]);
            }
        }
    }

    // 最后验证路径是否连通
    if (line_num > 0)
    {
        if (Path_Check_Connect(stpoint, finpoint, traject, line_num) == 0)
        {
            line_num = 0;
        }
    }

    bool trajValid = true;
    for (int i = 0; i < line_num; i++)
    {
        int result = PathPlan_CheckTrajValid(traject[i]);
        if (result != PATHPLAN_SUCCESS)
        {
            trajValid = false;
            log_err(
                "PK_PathPlan_BetterTra Check:%d,%d,%d start:%f,%f,%f end:%f,%f,%f "
                "trj:%f,%f,%f,%f,%f\n",
                line_num, i, result, stpoint[0], stpoint[1], stpoint[2], finpoint[0],
                finpoint[1], finpoint[2], traject[i][0], traject[i][1], traject[i][2],
                traject[i][3], traject[i][4]);
        }
    }

    if (!trajValid)
    {
        line_num = 0;
    }

    if (line_num > 0)
    {
        PathPlanType(1, 6, line_num);
    }
    return line_num;
}

int PathPlan_TwoCircleCon(float stpos[3], float finpos[3], int objnum,
                          const float slotobj[FS_OBJ_ARR_NUM][4],
                          float expTraj[][TRAJITEM_LEN], float swell)
{
    float r, rx, fx, fy, rth, mid_th_max, dtheta, ds_max, ds_step;
    int ds_Num, TrajNum = 0;
    float ds, ids, Path[5], new_stpos[3], fx_new, tempTraj[4][5];
    const float plan_rx_min  = 0.8f; // m, stpos and finpos can't be too close
    const float plan_rth_min = 0.2f; // rad

    memcpy(Path, stpos, 3 * sizeof(float));
    rx         = Project_PosTo1st_rx(stpos, finpos);
    fy         = Project_PosTo1st_ry(finpos, stpos);
    fx         = Project_PosTo1st_rx(finpos, stpos);
    rth        = Round_PI(finpos[2] - stpos[2]);
    mid_th_max = Round_PI(Round_PI(finpos[2]) + sign(fx * fy) * PI / 2);
    dtheta     = fabsf(mid_th_max - Round_PI(stpos[2]));
    dtheta     = rte_min(dtheta, PI / 2);
    ds_max     = Rrmin * dtheta;
    ds_step    = 0.1f;
    ds_Num     = rte_min((int)(ds_max / ds_step), 20);
    ds_step    = ds_max / ds_Num;
    if (fabsf(rx) > plan_rx_min) // too close
    {
        if (fabsf(rth) > plan_rth_min)
        {
            r = Rrmin * sign(rx) * sign(rth); // turn close to the angle of finpos
        }
        else
        {
            r = Rrmin * sign(fx) * sign(fy) *
                sign(rx); // turn close to the axis x of finpos
        }

        Path[4] = r;
        for (ids = 0; ids <= ds_max; ids += ds_step)
        {
            ds      = sign(rx) * ids;
            Path[3] = ds;
            PK_Get_Path_EndPos(Path, new_stpos);
            fx_new = Project_PosTo1st_rx(finpos, new_stpos);
            if (fx_new * fx > 0)
            {
                TrajNum = PK_PathPlan_BetterTra(new_stpos, finpos, objnum, slotobj,
                                                tempTraj, swell);
                if (TrajNum > 0)
                {
                    if (ids > 0)
                    {
                        memcpy(tempTraj[1], tempTraj, TrajNum * 5 * sizeof(float));
                        memcpy(tempTraj[0], Path, sizeof(Path));
                        TrajNum = TrajNum + 1;
                    }

                    if (PK_Check_Path(TrajNum, tempTraj, stpos, finpos, objnum, slotobj,
                                      swell) == PATHPLAN_SUCCESS)
                    {
                        memcpy(expTraj, tempTraj, TrajNum * 5 * sizeof(float));
                    }
                    else
                    {
                        TrajNum = 0;
                    }
                    break;
                }
            }
        }
    }

    bool trajValid = true;
    for (int i = 0; i < TrajNum; i++)
    {
        int result = PathPlan_CheckTrajValid(expTraj[i]);
        if (result != PATHPLAN_SUCCESS)
        {
            trajValid = false;
            log_err(
                "PathPlan_TwoCircleCon Check:%d,%d,%d start:%f,%f,%f end:%f,%f,%f "
                "trj:%f,%f,%f,%f,%f\n",
                TrajNum, i, result, stpos[0], stpos[1], stpos[2], finpos[0], finpos[1],
                finpos[2], expTraj[i][0], expTraj[i][1], expTraj[i][2], expTraj[i][3],
                expTraj[i][4]);
        }
    }

    if (!trajValid)
    {
        TrajNum = 0;
    }
    if (TrajNum > 0)
    {
        PathPlanType(0, 4, TrajNum);
    }

    return TrajNum;
}

// traj_pre == 1: 固定向后探索
int PK_PathTryConnect_TwoCon(int traj_pre, float finpoint[TRAJITEM_LEN],
                             PlanDataCase PlanData_Ver[1],
                             float Act_traj_dir[][TRAJITEM_LEN])
{
    int num2                         = 0;
    const float mid_ds               = 0.4;
    const float explor_ds_min        = 0.2;
    float explor_ds                  = 0;
    float Exp_midpoint[TRAJITEM_LEN] = {0}, Exp_finpoint[TRAJITEM_LEN] = {0};

    PK_CopyTra(Exp_finpoint, finpoint);

    float dx     = Project_PosTo1st_rx(Exp_finpoint, PlanData_Ver[0].stpoint);
    float dy     = Project_PosTo1st_ry(Exp_finpoint, PlanData_Ver[0].stpoint) / 2;
    float ftheta = acosf(1 - fabs(dy) / Rrmin);
    float ds     = fabs(ftheta) * Rrmin;
    ftheta += -sign(Rrmin) * PI / 2;
    float dmin =
        2 * (cosf(ds / Rrmin + ftheta) * fabsf(Rrmin) - fabsf(Rrmin) * cosf(ftheta));

    int obj_all_num = 0;
    float obj_all[SF_OBJ_NUM * 2 + 5][4];

    memcpy(obj_all, PlanData_Ver[0].obj_danger,
           sizeof(float) * PlanData_Ver[0].obj_danger_num * 4);
    obj_all_num = PlanData_Ver[0].obj_danger_num;

    memcpy(&obj_all[obj_all_num], &PlanData_Ver[0].obj_slot[0], 4 * sizeof(float));
    obj_all_num += 1;
    memcpy(&obj_all[obj_all_num], &PlanData_Ver[0].obj_slot[4], 4 * sizeof(float));
    obj_all_num += 1;

    if (fabs(dy) < 0.06 && dy != 0)
    {
        memcpy(Act_traj_dir[0], PlanData_Ver[0].stpoint, sizeof(PlanData_Ver[0].stpoint));
        if (dx < 0)
        {
            Act_traj_dir[0][3] = fabs(dx);
            Act_traj_dir[0][4] = 0;
            PK_Get_Path_EndPos(Act_traj_dir[0], Act_traj_dir[1]);

            Act_traj_dir[1][3] = 2.5;
            Act_traj_dir[1][4] = sign(dy) * Rrmin * 3;
            num2               = PK_PathPlan_TraExplorer_Con(
                              Act_traj_dir[1], Exp_finpoint, PlanData_Ver[0].obj_danger_num,
                              PlanData_Ver[0].obj_danger, &Act_traj_dir[1], 0, 1, PlanData_Ver[0].swell,
                              0.3);
            if (num2 > 0 && !Connect_BreakPoint(Act_traj_dir[num2 - 1], Exp_finpoint))
            {
                num2 = 0;
            }
            else if (num2 > 0)
            {
                num2 += 1;
            }
        }
        else
        {
            Act_traj_dir[0][3] = 2.5;
            Act_traj_dir[0][4] = sign(dy) * Rrmin * 3;
            num2 = PK_PathPlan_TraExplorer_Con(Act_traj_dir[0], Exp_finpoint,
                                               PlanData_Ver[0].obj_danger_num,
                                               PlanData_Ver[0].obj_danger, Act_traj_dir,
                                               0, 1, PlanData_Ver[0].swell, 0.3);
            if (num2 > 0 && !Connect_BreakPoint(Act_traj_dir[num2 - 1], Exp_finpoint))
            {
                num2 = 0;
            }
        }
    }

    if (num2 <= 0 && fabs(dx) >= (dmin + mid_ds))
    {
        num2 = PathPlan_TwoCircleCon(PlanData_Ver[0].stpoint, Exp_finpoint, obj_all_num,
                                     obj_all, Act_traj_dir, PlanData_Ver[0].swell);
        if (num2 > 0 &&
            Path_Check_Connect(PlanData_Ver[0].stpoint, Exp_finpoint, Act_traj_dir, num2))
        {
            num2 = 0;
        }
    }

    if (num2 <= 0 && dx > mid_ds && dx < 2 * (dmin + mid_ds))
    {
        if (dx >= (dmin + mid_ds))
        {
            explor_ds = (dx - (dmin + mid_ds)) > explor_ds_min ? (dx - (dmin + mid_ds))
                                                               : explor_ds_min;
        }
        else
        {
            explor_ds = ((dmin + mid_ds) - dx) > explor_ds_min ? ((dmin + mid_ds) - dx)
                                                               : explor_ds_min;
        }

        memcpy(Act_traj_dir[0], PlanData_Ver[0].stpoint, sizeof(PlanData_Ver[0].stpoint));
        Act_traj_dir[0][4] = 0;

        for (int i = 0; i < 10; i++) //先x方向后y方向
        {
            Act_traj_dir[0][3] = explor_ds + i * 0.40;
            PK_Get_Path_EndPos(Act_traj_dir[0], Act_traj_dir[1]);
            num2 = PathPlan_TwoCircleCon(
                Act_traj_dir[1], Exp_finpoint, obj_all_num, obj_all,
                &Act_traj_dir[1] /*&Exp_finpoint_rear[num_i+1]*/, PlanData_Ver[0].swell);
            if (num2 > 0 && Connect_BreakPoint(Act_traj_dir[num2], Exp_finpoint))
            {
                num2 += 1;
                break;
            }
            else
            {
                num2 = 0;
            }
        }
    }

    if (num2 <= 0)
    {
        int explor_direction = 1;
        if ((fabs(dx) > dmin + 2 * mid_ds) || traj_pre)
        {
            explor_ds        = sign(dx) * (fabs(dx) - (dmin + mid_ds));
            explor_direction = -sign(dx);
        }
        else
        {
            explor_ds        = (dmin + dx) > 0 ? dmin + dx + mid_ds : mid_ds;
            explor_direction = 1;
        }

        Exp_finpoint[4] = 0;

        for (int i = 0; i < 10; i++) //先y方向后x方向
        {
            Exp_finpoint[3] = explor_ds + explor_direction * i * 0.40;
            PK_Get_Path_EndPos(Exp_finpoint, Exp_midpoint);
            num2 =
                PathPlan_TwoCircleCon(PlanData_Ver[0].stpoint, Exp_midpoint, obj_all_num,
                                      obj_all, Act_traj_dir, PlanData_Ver[0].swell);
            if (num2 > 0 && Connect_BreakPoint(Act_traj_dir[num2 - 1], Exp_midpoint))
            {
                PathPlan_ReverseTraject(Exp_finpoint);
                PK_CopyTra(Act_traj_dir[num2], Exp_finpoint);
                num2 += 1;
                break;
            }
            else if (num2 > 0)
            {
                num2 = 0;
            }
        }
    }

    bool trajValid = true;
    for (int i = 0; i < num2; i++)
    {
        int result = PathPlan_CheckTrajValid(Act_traj_dir[i]);
        if (result != PATHPLAN_SUCCESS)
        {
            trajValid = false;
            log_err(
                "PK_PathPlan_BetterTra Check:%d,%d,%d end:%f,%f,%f,%f,%f "
                "trj:%f,%f,%f,%f,%f\n",
                num2, i, result, finpoint[0], finpoint[1], finpoint[2], finpoint[3],
                finpoint[4], Act_traj_dir[i][0], Act_traj_dir[i][1], Act_traj_dir[i][2],
                Act_traj_dir[i][3], Act_traj_dir[i][4]);
        }
    }

    if (!trajValid)
    {
        num2 = 0;
    }
    if (num2 > 0)
    {
        PathPlanType(0, 5, num2);
    }

    return num2;
}

// B_tra_to_pos==0:connect from tar_pos to temp_tra; B_tra_to_pos==1:connect from temp_tra
// to tar_pos; A_tra_to_pos==0:connect from "Explorpos_arr" head; A_tra_to_pos==1:connect
// from "Explorpos_arr" last;
int PK_PathPlan_TraExplorer_Con(float temp_tra[TRAJITEM_LEN], const float tar_pos[3],
                                const int obj_num, const float obj_slot[][4],
                                float stp_trajs[][TRAJITEM_LEN], int A_tra_to_posint,
                                int B_tra_to_pos, float swell, float start_ds)
{
    float TempTraj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
    static float Explorpos_arr[30][4]; // X,Y,THETA,ds
    int line_num = 0, points_num = 0;
    const float dsmin = 0.3f; //   the minimun dist to generate a step point
    int j, Explorpos_num;
    float sign_start_ds;

    sign_start_ds =
        sign(temp_tra[3]) *
        fabsf(start_ds); // keep direction the same, to decide to go forward or backward
    int number = (fabs(temp_tra[3]) - fabs(start_ds)) / dsmin;

    float dsminv = dsmin;
    if (number < 4)
    {
        dsminv = 0.1;
    }
    else if (number < 7)
    {
        dsminv = 0.15;
    }

    points_num = PathPlan_DrawTrajectory(
        dsminv, temp_tra, Explorpos_arr,
        sign_start_ds); // 20180619: maybe it's no need to calculate these points first
    for (j = 0; j < points_num; j++)
    {
        if (A_tra_to_posint == 1)
        {
            Explorpos_num = points_num - 1 - j;
        }
        else
        {
            Explorpos_num = j;
        }

        if (B_tra_to_pos == 1)
        {
            line_num = PK_PathPlan_BetterTra(Explorpos_arr[Explorpos_num], tar_pos,
                                             obj_num, obj_slot, TempTraj, swell); // vert
        }
        else
        {
            line_num = PK_PathPlan_BetterTra(tar_pos, Explorpos_arr[Explorpos_num],
                                             obj_num, obj_slot, TempTraj, swell); // para
        }

        if (line_num > 0) // HAVECHANGE Tansj 2023/8/25 if (line_num>0 &&
                          // fabs(Explorpos_arr[Explorpos_num][3]) >= short_path_len )
        {
            float tempEnd[3];
            if (B_tra_to_pos == 1)
            {
                memcpy(stp_trajs[1], TempTraj[0], line_num * PATH_ITEM_LEN);
                stp_trajs[0][0] = temp_tra[0];
                stp_trajs[0][1] = temp_tra[1];
                stp_trajs[0][2] = temp_tra[2];
                stp_trajs[0][3] = Explorpos_arr[Explorpos_num][3];
                stp_trajs[0][4] = temp_tra[4];
                PK_Get_Path_EndPos(stp_trajs[0], tempEnd);
                auto check = PK_Check_Path(1, stp_trajs, stp_trajs[0], stp_trajs[1],
                                           obj_num, obj_slot, swell);
                // printf("check 2 %d %f,%f,%f,%f,%f\n", check, stp_trajs[0][0],
                // stp_trajs[0][2], stp_trajs[0][3], stp_trajs[0][3], stp_trajs[0][4]);
                if (check != PATHPLAN_SUCCESS)
                {
                    line_num = 0;
                    continue;
                }
                line_num = line_num + 1;
            }
            else
            {
                memcpy(stp_trajs[0], TempTraj[0], line_num * PATH_ITEM_LEN);
                stp_trajs[line_num][0] = Explorpos_arr[Explorpos_num][0];
                stp_trajs[line_num][1] = Explorpos_arr[Explorpos_num][1];
                stp_trajs[line_num][2] = Explorpos_arr[Explorpos_num][2];
                stp_trajs[line_num][3] = -Explorpos_arr[Explorpos_num][3];
                stp_trajs[line_num][4] = temp_tra[4];
                PK_Get_Path_EndPos(stp_trajs[line_num], tempEnd);
                auto check = PK_Check_Path(1, &stp_trajs[line_num], stp_trajs[line_num],
                                           tempEnd, obj_num, obj_slot, swell);
                // printf("check 2 %d %f,%f,%f,%f,%f\n", check, stp_trajs[line_num][0],
                // stp_trajs[line_num][2], stp_trajs[line_num][3], stp_trajs[line_num][3],
                // stp_trajs[line_num][4]);
                if (check != PATHPLAN_SUCCESS)
                {
                    line_num = 0;
                    continue;
                }

                line_num = line_num + 1; //  add the last path Explorpos_arr[j]
            }
            temp_tra[3] = Explorpos_arr[Explorpos_num][3];
            break;
        }
    }

    bool trajValid = true;
    for (int i = 0; i < line_num; i++)
    {
        int result = PathPlan_CheckTrajValid(Explorpos_arr[i]);
        if (result != PATHPLAN_SUCCESS)
        {
            trajValid = false;
            log_err("PK_PathPlan_TraExplorer_Con Check:%d,%d,%d trj:%f,%f,%f,%f,%f\n",
                    line_num, i, result, Explorpos_arr[i][0], Explorpos_arr[i][1],
                    Explorpos_arr[i][2], Explorpos_arr[i][3], Explorpos_arr[i][4]);
        }
    }

    if (!trajValid)
    {
        line_num = 0;
    }
    if (line_num > 0)
    {
        PathPlanType(0, 6, line_num);
    }

    return line_num;
}
