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
    PathPlan_FootPrints(1, level, type, num);
}

static float Get_VertSlot_MidPoint_Rect_ds_Max(PlanDataCase PlanData_Ver[1], float ntheta,
                                               float fin_ds)
{
    LineSeg_T line_AB;
    VehPos_T CurPos;
    Rect_T CornerPt;
    Point_T VehCornPoint;
    Relation_T dir;
    uint8 Is_CornerPt_Out = 0;
    float neg_slotVec_th;
    Point_T CrossPt;
    Point_T Slot_A, Slot_B, Slot_C, VehCornPt_prime;
    VehPos_T VehCornPt_NegVec;
    memcpy(&Slot_A, PlanData_Ver[0].obj_slot[0], sizeof(Slot_A));
    memcpy(&Slot_B, PlanData_Ver[0].obj_slot[1], sizeof(Slot_B));
    memcpy(&Slot_C, PlanData_Ver[0].obj_slot[2], sizeof(Slot_C));
    memcpy(&line_AB, PlanData_Ver[0].obj_slot[0], sizeof(line_AB));

    // 终点向X轴方向移动fin_ds
    float cos_finpoint = cosf(PlanData_Ver[0].finpoint[2]);
    float sin_finpoint = sinf(PlanData_Ver[0].finpoint[2]);
    CurPos.x           = PlanData_Ver[0].finpoint[0] + cos_finpoint * fin_ds;
    CurPos.y           = PlanData_Ver[0].finpoint[1] + sin_finpoint * fin_ds;
    // stpoint[2] + (0.3 0.43 0.57 0.7)* (finpoint[2] - stpoint[2]) = (1 - t) * stpoint[2]
    // + t * finpoint[2]; t ~ 0.3 0.43 0.57 0.7
    CurPos.theta = PlanData_Ver[0].stpoint[2] + ntheta;

    // 计算目标点是CurPos, 长度为VEHICLE_LEN, 宽度为VEHICLE_WID新的车位信息
    Get_Veh_CornerPt(CurPos, &CornerPt, 0);

    // cur_targ_dth = finpoint[2] - (1 - t) * stpoint[2] - t * finpoint[2] = (1 - t) *
    // (finpoint[2] - stpoint[2])  t ~ 0.3 0.43 0.57 0.7
    float rect_ds_max  = 0.0f;
    float cur_targ_dth = PlanData_Ver[0].finpoint[2] - CurPos.theta;
    if (PlanData_Ver[0].slotshape == PK_SLOT_LEFT_VERT ||
        PlanData_Ver[0].slotshape == PK_SLOT_LEFT_ANG_FORWARD ||
        PlanData_Ver[0].slotshape == PK_SLOT_LEFT_ANG_REVERSE) //  left
    {
        VehCornPoint   = CornerPt.pt_rr;
        neg_slotVec_th = PlanData_Ver[0].finpoint[2] - PI / 2.0f;
        Get_Dist_Dir_Pt2SegLine(line_AB, VehCornPoint, NULL, &dir);
        if (dir == PK_ON_RIGHT)
        {
            Is_CornerPt_Out = 1;
        }
    }
    else
    {
        VehCornPoint   = CornerPt.pt_rl;
        neg_slotVec_th = PlanData_Ver[0].finpoint[2] + PI / 2.0f;
        Get_Dist_Dir_Pt2SegLine(line_AB, VehCornPoint, NULL, &dir);
        if (dir == PK_ON_LEFT)
        {
            Is_CornerPt_Out = 1;
        }
    }

    if (Is_CornerPt_Out)
    {
        float temp_dist = 0.0f;
        if (Is_TwoSegment_Cross(Slot_A, Slot_B, CornerPt.pt_rl, CornerPt.pt_rr) == 1 ||
            Is_TwoSegment_Cross(Slot_B, Slot_C, CornerPt.pt_rl, CornerPt.pt_rr) == 1)
        {
            Get_Dist_Dir_Pt2PointLine(Slot_B, Slot_C, VehCornPoint, &temp_dist,
                                      NULL); // CornerPt.pt_rr
            rect_ds_max = -temp_dist;
        }
        else
        {
            Get_Dist_Dir_Pt2PointLine(CornerPt.pt_rl, CornerPt.pt_rr, Slot_B, &temp_dist,
                                      NULL); //
            rect_ds_max = fabsf(temp_dist / sinf(cur_targ_dth));
        }
    }
    else
    {
        VehCornPt_NegVec.x     = VehCornPoint.x;
        VehCornPt_NegVec.y     = VehCornPoint.y;
        VehCornPt_NegVec.theta = neg_slotVec_th;
        VehCornPt_prime.x      = VehCornPoint.x + cosf(neg_slotVec_th);
        VehCornPt_prime.y      = VehCornPoint.y + sinf(neg_slotVec_th);
        float rx               = Project_PointTo1stPos_rx(VehCornPt_NegVec, Slot_B);
        if (rx <= 0) //  after fin_ds translation the curpos cross with slotobj
        {
            rect_ds_max = rx;
        }
        else
        {
            Get_CrossPt_PointLine(VehCornPoint, VehCornPt_prime, Slot_B, Slot_C,
                                  &CrossPt);
            rect_ds_max = Cal_Dis_Pt2Pt(VehCornPoint, CrossPt);
        }
    }

    return rect_ds_max;
}

// 只连或者只换一次档的垂直车位规划
int PathPlan_SpaceExplorer_Vert(PlanDataCase PlanData_Ver[1],
                                float Exp_traj[][TRAJITEM_LEN], int *IsDirConnect,
                                int type)
{
    const int Explore_times = 3;
    int line_num            = 0;
    float danger_pos[3];
    float danger_ry, fin_ds, rect_ds; // dth_midtra,ftheta;danger_rx
    float midtra[5]; // mid_endpoint[3];//Exp_tra_vert[MAX_PARKED_SLOTS][5];
    float Rr            = Rrmin + 0.1f;             //  20180808: G3
    float fin_ds_arr[]  = {0.35f, 0.65f, 0.90f};    // 0.35f, getting bigger
    float rect_ds_arr[] = {-0.15f, -0.40f, -0.75f}; // //-0.75f getting smaller
    float dfin_ds = 0, drect_ds = 0; // refine parameter,according real slot//slotlen,
    float Envi_obj_side[2][4];
    float cos_finpoint, cos_pointf, sin_finpoint,
        sin_pointf; //  to calculate first avoid repeat calculations
    float forward_ds_ratio         = 1.0f;
    float rect_ds_max              = 0;
    const float big_vert_side_dist = 0.2; // 0.45f;
    const float TargHead_to_BE_max = 2.6f;
    static int IsNarrowSlot        = 0;

    *IsDirConnect = 0; //  init
    PK_PathPlan_Envi_obj_cut(
        PlanData_Ver[0].obj_slot, 1.0f,
        Envi_obj_side); // 20180719: is it necessary? //0816: be deleted
    float rtheta =
        Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    float rx = Project_PosTo1st_rx(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);
    float ry = Project_PosTo1st_ry(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);

    if (fabs(rtheta) < 0.05 && fabs(ry) < 0.1)
    {
        PK_CopyPos(Exp_traj[0], PlanData_Ver[0].stpoint);
        Exp_traj[0][3] = rx;
        Exp_traj[0][4] = 0;
        PathPlan_UpdateTrajLen(Exp_traj[0], PlanData_Ver[0].finpoint,
                               PlanData_Ver[0].slotshape);

        line_num = 1;
        if (line_num > 0 &&
            PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                          PlanData_Ver[0].finpoint, PlanData_Ver[0].obj_danger_num,
                          PlanData_Ver[0].obj_danger,
                          0) != PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny small
                                                  // movement or Path damage
        {
            line_num = 0;
        }
        else if (line_num > 0)
        {
            *IsDirConnect = 1;
            PathPlanType(0, 1, line_num);
            return line_num;
        }
    }

    if (PlanData_Ver->front_dist > big_vert_side_dist ||
        fabs(rtheta) <
            PI * 0.4 /*&& PlanData_Ver->rear_dist > big_vert_side_dist*/) //  0816: try
                                                                          //  direct
                                                                          //  connection
                                                                          //  more easily
                                                                          //  // 1108: 0.3
                                                                          //  -> 0.45 more
                                                                          //  safe
    {
        float Exp_finpoint_rear[2][5] = {{0}};
        PK_CopyPos(Exp_finpoint_rear[0], PlanData_Ver[0].finpoint);
        Exp_finpoint_rear[0][3] = 0.5;
        Exp_finpoint_rear[0][4] = 0.0;
        PK_Get_Path_EndPos(Exp_finpoint_rear[0], Exp_finpoint_rear[1]);

        // 左右边距大于45cm, 直接进车位
        line_num = PK_PathPlan_BetterTra(PlanData_Ver[0].stpoint, Exp_finpoint_rear[1], 2,
                                         Envi_obj_side, Exp_traj,
                                         1.5f * PlanData_Ver[0].swell); // for a big slot
        if (line_num > 0 &&
            PK_Check_Path(line_num, Exp_traj, PlanData_Ver[0].stpoint,
                          Exp_finpoint_rear[1], PlanData_Ver[0].obj_danger_num,
                          PlanData_Ver[0].obj_danger,
                          0) != PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swellAny small
                                                  // movement or Path damage
        {
            line_num = 0;
        }
        else if (line_num > 0 &&
                 Path_Check_Connect(PlanData_Ver[0].stpoint, Exp_finpoint_rear[1],
                                    Exp_traj, line_num))
        {
            *IsDirConnect = 1;
            PathPlan_ReverseTraject(Exp_finpoint_rear[0]);
            PK_CopyTra(Exp_traj[line_num], Exp_finpoint_rear[0]);
            line_num += 1;
            PathPlanType(0, 2, line_num);
            return line_num;
        }
    }

    ///////////////////////////////////////////////////////// calculate  drect_ds
    if (PlanData_Ver->front_dist > 0.4f &&
        PlanData_Ver->rear_dist > 0.4f) // enough side gap, then ignore
    {
        drect_ds = 0.0f;
    }
    else if (fabsf(PlanData_Ver->front_dist - PlanData_Ver->rear_dist) <
             0.1f) // same side gap, then ignore
    {
        drect_ds = 0.0f;
    }
    else if (rte_max(PlanData_Ver->front_dist, PlanData_Ver->rear_dist) <
             0.45f) //  both small side gap, then ignore
    {
        drect_ds = 0.0f;
    }
    else
    {
        if (PlanData_Ver->front_dist > PlanData_Ver->rear_dist)
            drect_ds = 0.4f - PlanData_Ver->rear_dist;
        else
            drect_ds = PlanData_Ver->front_dist - 0.4;
    }

    if (fabsf(drect_ds) > 0.5)
    {
        drect_ds = 0.0f;
    }

    ///////////////////////////////////////////////////////// calculate  dfin_ds
    dfin_ds = rte_max(Project_PosTo1st_rx(PlanData_Ver[0].finpoint,
                                          PlanData_Ver[0].pointf_danger),
                      Project_PosTo1st_rx(PlanData_Ver[0].finpoint,
                                          PlanData_Ver[0].pointb_danger)) -
              (PlanData_Ver[0].len - PlanData_Ver[0].h);

    if (dfin_ds > TargHead_to_BE_max || dfin_ds < 0.0f)
    {
        dfin_ds = 0.0f;
    }

    cos_finpoint = cosf(PlanData_Ver[0].finpoint[2]);
    sin_finpoint = sinf(PlanData_Ver[0].finpoint[2]);
    cos_pointf   = cosf(PlanData_Ver[0].pointf_danger[2]);
    sin_pointf   = sinf(PlanData_Ver[0].pointf_danger[2]);

    for (int i = 0; i <= Explore_times && line_num == 0; i++) // i < 2
    {
        float nfac   = 0.7f - i * 0.4f / Explore_times; //  0.3f
        float ntheta = nfac * rtheta; // 起点和终点之间的夹角，(0.3 0.43 0.57 0.7)*PI / 2

        danger_pos[0] = PlanData_Ver[0].pointf_danger[0];
        danger_pos[1] = PlanData_Ver[0].pointf_danger[1];
        danger_pos[2] = PlanData_Ver[0].stpoint[2] + ntheta;
        danger_ry     = Project_PosTo1st_ry(danger_pos, PlanData_Ver[0].finpoint);

        for (uint32 j = 0; j < PATHPLAN_ARRSIZE(fin_ds_arr) && line_num == 0; j++)
        {
            fin_ds = fin_ds_arr[j] + VEHICLE_LEN - REAR_SUSPENSION + dfin_ds;
            for (uint32 k = 0; k < PATHPLAN_ARRSIZE(rect_ds_arr) && line_num == 0; k++)
            {
                rect_ds = rect_ds_arr[k] + drect_ds;

                rect_ds_max =
                    Get_VertSlot_MidPoint_Rect_ds_Max(PlanData_Ver, ntheta, fin_ds);
                if (rect_ds_max < 0 ||
                    (rect_ds < 0 &&
                     fabsf(rect_ds) >
                         rect_ds_max)) // 20180727: for narrow slot situations
                {
                    rect_ds = -rect_ds_max + 0.05f; // error allowance
                }

                midtra[0] = PlanData_Ver[0].finpoint[0] + cos_finpoint * fin_ds +
                            cos_pointf * rect_ds;
                midtra[1] = PlanData_Ver[0].finpoint[1] + sin_finpoint * fin_ds +
                            sin_pointf * rect_ds;
                midtra[2] = PlanData_Ver[0].stpoint[2] + ntheta;
                midtra[3] = forward_ds_ratio *
                            fabsf((1 - nfac) * rtheta *
                                  Rr); // ds    should move forward with this dist to make
                                       // the theta of vehicle to targpos
                midtra[4] = -sign(danger_ry) * Rr; // 20180720: what's the meaning?

                if (PK_PosObjCross(midtra, 5, PlanData_Ver[0].obj_slot, 0.0f, 0) ==
                    0) // 1.55f*PlanData_Ver[0].swell
                {
                    line_num = PK_PathPlan_BetterTra(
                        PlanData_Ver[0].stpoint, midtra, 5, PlanData_Ver[0].obj_slot,
                        Exp_traj, PlanData_Ver[0].swell); // first stage connection
                    if (line_num == 0)
                    {
                        continue;
                    }

                    if (PK_Check_Path_simple(
                            line_num, Exp_traj, PlanData_Ver[0].obj_danger_num,
                            PlanData_Ver[0].obj_danger, 0, PlanData_Ver[0].stpoint,
                            PlanData_Ver[0].finpoint, &IsNarrowSlot) !=
                        PATHPLAN_SUCCESS) // 1.5f*PlanData_Ver[0].swell//Any small
                                          // movement or Path damage
                    {
                        line_num = 0;
                    }

                    if (line_num == 0)
                    {
                        continue;
                    }

                    int temp_line2num = 0;
                    // 去除直线
                    if (fabs(Exp_traj[line_num - 1][4]) < 1)
                    {
                        Exp_traj[line_num - 1][4] = Exp_traj[line_num - 2][4];

                        PK_Get_Path_EndPos(Exp_traj[line_num - 1], midtra);
                        if (PK_PathVerify(midtra, 5, PlanData_Ver[0].obj_slot, 0) == -1)
                        {
                            line_num = 0;
                            continue;
                        }
                    }

                    if (temp_line2num == 0)
                    {
                        temp_line2num = PK_PathPlan_TraExplorer_Con(
                            midtra, PlanData_Ver[0].finpoint, 5, PlanData_Ver[0].obj_slot,
                            &Exp_traj[line_num], 0, 1, PlanData_Ver[0].swell, 0.0f);
                    }

                    if (temp_line2num < 1)
                    {
                        line_num = 0;
                    }
                    else
                    {
                        line_num = line_num + temp_line2num;
                        // 1.5f*PlanData_Ver[0].swell//Any small movement or Path damage
                        if (PK_Check_Path_simple(
                                line_num, Exp_traj, PlanData_Ver[0].obj_danger_num,
                                PlanData_Ver[0].obj_danger, 0, PlanData_Ver[0].stpoint,
                                PlanData_Ver[0].finpoint,
                                &IsNarrowSlot) != PATHPLAN_SUCCESS)
                        {
                            line_num = 0; // enviroment test
                        }
                        else
                        {
                            *IsDirConnect = 2;
                            PathPlanType(0, 3, line_num);
                            break;
                        }
                    }
                }
            }
        }
    }

    //  encounter a narrow environment slot need more forward-backward adjustment
    //  if (line_num == 0)
    //  {
    // float Nearest_dy = Get_AnotherObs_Nearest_dy_to_CurPos(PlanData_Ver[0].stpoint,
    // PlanData_Ver[0].slotshape, PlanData_Ver[0].obj_danger,
    // PlanData_Ver[0].obj_danger_num);
    //  }

    if (line_num == 0 /*&& !fail_reason_is_firstline*/ &&
        type) // && (IsNarrowSlot == 1 || Nearest_dy < Narrow_AnotherDist) )
    {
        line_num = PathPlan_NarrowSpaceExplorer_Vert(PlanData_Ver, Exp_traj);
        PathPlanType(0, 4, line_num);
    }

    IsNarrowSlot = 0; //  init
    return line_num;
}

//  encounter a narrow environment verticle slot need more forward-backward adjustments
int PathPlan_NarrowSpaceExplorer_Vert(PlanDataCase PlanData_Ver[1],
                                      float Exp_traj[][TRAJITEM_LEN])
{
    const int Explore_times = 10;
    float danger_pos[3];
    float fin_ds, rect_ds;
    float midtra[5];
    float Rr            = Rrmin + 0.1f; //  20180808: G3
    float fin_ds_arr[]  = {0.15f, 0.30f, 0.45f, 0.60f, 0.75f, 0.90f};
    float rect_ds_arr[] = {0.15f, -0.15f, -0.35f, -0.50f, -0.75f, -0.80f}; //,-0.6f
    float dfin_ds = 0, drect_ds = 0; // refine parameter,according real slot//slotlen,
    float cos_finpoint, cos_pointf, sin_finpoint,
        sin_pointf; //  to calculate first avoid repeat calculations
    float forward_ds_ratio = 1.0f;
    float rect_ds_max      = 0;
    int Exp_PathNum        = 0;
    float midPos_rth;
    const float TargHead_to_BE_max = 2.6f;

    float rtheta =
        Project_PosTo1st_rtheta(PlanData_Ver[0].stpoint, PlanData_Ver[0].finpoint);

    float obj_all[SF_OBJ_NUM * 2 + 5][4];
    int obj_all_num = 0;
    // 统一障碍物
    memcpy(obj_all, PlanData_Ver[0].obj_danger,
           sizeof(float) * PlanData_Ver[0].obj_danger_num * 4);
    obj_all_num = PlanData_Ver[0].obj_danger_num;
    memcpy(&obj_all[obj_all_num], PlanData_Ver[0].obj_slot, sizeof(float) * 5 * 4);
    obj_all_num += 5;

    ///////////////////////////////////////////////////////// calculate  dfin_ds
    dfin_ds =
        Project_PosTo1st_rx(PlanData_Ver[0].finpoint, PlanData_Ver[0].pointf_danger) -
        PlanData_Ver[0].len + PlanData_Ver[0].h;
    if (dfin_ds > TargHead_to_BE_max ||
        dfin_ds < 0.0f) //  dfin_ds>0.5f||  0905: for targpos stay inner ,think 2.6m is
                        //  abnormal situation
    {
        dfin_ds = 0.0f;
    }

    // first stage: from stpos to midpos and midpos is set near the center of slot
    cos_finpoint = cosf(PlanData_Ver[0].finpoint[2]);
    sin_finpoint = sinf(PlanData_Ver[0].finpoint[2]);
    cos_pointf   = cosf(PlanData_Ver[0].pointf_danger[2]);
    sin_pointf   = sinf(PlanData_Ver[0].pointf_danger[2]);
    for (int i = 0; i <= Explore_times && Exp_PathNum == 0; i++) // i < 2
    {
        float nfac   = 0.7f - i * 0.4f / Explore_times; //  0.3f
        float ntheta = nfac * rtheta;

        danger_pos[0] = PlanData_Ver[0].pointf_danger[0];
        danger_pos[1] = PlanData_Ver[0].pointf_danger[1];
        danger_pos[2] = PlanData_Ver[0].stpoint[2] + ntheta;
        midPos_rth    = Round_PI(PlanData_Ver[0].finpoint[2] - danger_pos[2]);

        for (uint32_t j = 0; j < PATHPLAN_ARRSIZE(fin_ds_arr) && Exp_PathNum == 0; j++)
        {
            fin_ds = fin_ds_arr[j] + VEHICLE_LEN - REAR_SUSPENSION + dfin_ds;

            for (uint32_t k = 0; k < PATHPLAN_ARRSIZE(rect_ds_arr) && Exp_PathNum == 0;
                 k++)
            {
                rect_ds = rect_ds_arr[k] + drect_ds;
                rect_ds_max =
                    Get_VertSlot_MidPoint_Rect_ds_Max(PlanData_Ver, ntheta, fin_ds);

                if (rect_ds_max < 0 ||
                    (rect_ds < 0 &&
                     fabsf(rect_ds) >
                         rect_ds_max)) // 20180727: for narrow slot situations
                {
                    rect_ds = -rect_ds_max + 0.05f;
                }

                midtra[0] = PlanData_Ver[0].finpoint[0] + cos_finpoint * fin_ds +
                            cos_pointf * rect_ds;
                midtra[1] = PlanData_Ver[0].finpoint[1] + sin_finpoint * fin_ds +
                            sin_pointf * rect_ds;
                midtra[2] = PlanData_Ver[0].stpoint[2] + ntheta;
                midtra[3] = forward_ds_ratio *
                            fabsf((1 - nfac) * rtheta *
                                  Rr); // ds    should move forward with this dist to make
                                       // the theta of vehicle to targpos
                midtra[4] = sign(midPos_rth) * Rr;

                if (PK_PosObjCross(midtra, 5, PlanData_Ver[0].obj_slot, 0.0f, 0) != 0)
                {
                    continue;
                }

                int TrajNum_NS =
                    PK_PathPlan_BetterTra(PlanData_Ver[0].stpoint, midtra, obj_all_num,
                                          obj_all, Exp_traj, PlanData_Ver[0].swell);
                if (TrajNum_NS <= 0)
                {
                    continue;
                }

                // 去除直线
                if (fabs(Exp_traj[TrajNum_NS - 1][4]) < 1)
                {
                    Exp_traj[TrajNum_NS - 1][4] = Exp_traj[TrajNum_NS - 2][4];
                    if (PK_PathVerify(Exp_traj[TrajNum_NS - 1], obj_all_num, obj_all,
                                      0) == -1)
                    {
                        continue;
                    }

                    PK_Get_Path_EndPos(Exp_traj[TrajNum_NS - 1], midtra);
                }

                // int result = PK_Check_Path(TrajNum_NS, Exp_traj,
                // PlanData_Ver[0].stpoint, midtra, PlanData_Ver[0].obj_danger_num,
                // PlanData_Ver[0].obj_danger, 0);
                //                    if (result != PATHPLAN_SUCCESS) {  continue;  }

                Exp_PathNum = PathPlan_SpaceExplorer_Mid2Targ_Vert(
                    Exp_traj[TrajNum_NS - 1], PlanData_Ver, &Exp_traj[TrajNum_NS]);
                if (Exp_PathNum <= 0)
                {
                    continue;
                }

                Exp_PathNum = TrajNum_NS + Exp_PathNum;
                PathPlanType(0, 5, Exp_PathNum);
            }
        }
    }

    //    if (Exp_PathNum <= 0)
    //    {
    //        int TrajNum_NS = GetPath_StToMidPos_Na(PlanData_Ver, Exp_traj);
    //        if (TrajNum_NS > 0)
    //        {
    //            Exp_PathNum = PathPlan_SpaceExplorer_Mid2Targ_Vert(Exp_traj[TrajNum_NS -
    //            1], PlanData_Ver, &Exp_traj[TrajNum_NS]); Exp_PathNum = (Exp_PathNum >
    //            0) ? TrajNum_NS + Exp_PathNum : 0;
    //        }
    //    }

    return Exp_PathNum;
}
