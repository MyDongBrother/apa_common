#include "PK_Calibration.h"
#include "PK_ObjAvoid.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Func.h"
#include "PathExec_Debug.h"
#include "PK_PathPlan.h"
#include "PK_Utility.h"
#include "Rte_ComIF.h"
#include "PK_B.h"

int PathExec_ParaParkOutDataProcess(PlanDataCase PlanDataVer[1],
                                    const U_RadarType &uradar,
                                    PK_Cur_AutodrPara &curState)
{
    float side_dis;
    float temploc[4]     = {0.0f};
    float Envi_obj[5][4] = {0.0f};
    SlotObj_T slotobj;
    float targpos[3];
    const float u_radar_ratio             = 0.6f; //  believe value
    const float front_rear_limit_dist_max = 1.8f;
    const float front_rear_valid_dist_max = 1.5f;
    const float outer_dist_to_new_targpos =
        0.85f; // relative to slotE to create a targpos for parallel parking out
    const float long_dist_to_new_targpos =
        1.1f; // relative to slotE to create a targpos for parallel parking out

    Point_T Ref_B, Ref_C, Ref_D, Ref_E;
    Point_T NewSlot_B, NewSlot_C, NewSlot_D, NewSlot_E;
    Vec2_T TargVec, VertTargVec;

    RTE_PK_SlotDetect_Get_SlotObj(&slotobj);
    SlotObj_Convert_float(slotobj, Envi_obj);
    RTE_PK_Location_Get_CurPos(temploc);
    PK_CopyPos(PlanDataVer[0].finpoint, temploc); //  right!
    RTE_PK_SlotDetect_Get_TargPos(targpos);
    PlanDataVer[0].finpoint[2] = Round_PI(temploc[2]); //  wrong! must be update later
    PlanDataVer[0].len         = VEHICLE_LEN;
    PlanDataVer[0].width       = VEHICLE_WID;
    PlanDataVer[0].h           = REAR_SUSPENSION;
    float front_dy =
        rte_max(Project_PosTo1st_ry(temploc, Envi_obj[4]), PlanDataVer[0].width / 2.0f);
    PlanDataVer[0].obj_danger_num = 0; // init

    // the front_dist and rear_dist calculation is only usefull for parallel slots
    PlanDataVer[0].rear_dist = Project_PosTo1st_rx(PlanDataVer[0].finpoint, Envi_obj[1]) +
                               PlanDataVer[0].h; // has sign
    PlanDataVer[0].front_dist =
        Project_PosTo1st_rx(PlanDataVer[0].finpoint, Envi_obj[4]) - PlanDataVer[0].len +
        PlanDataVer[0].h;
    PlanDataVer[0].slotshape =
        Cal_SlotShap(PlanDataVer[0].finpoint, Envi_obj, &PlanDataVer[0].left_fac);
    if ((PlanDataVer[0].slotshape == 1 || PlanDataVer[0].slotshape == 2) &&
        (PlanDataVer[0].rear_dist >= 0 ||
         PlanDataVer[0].front_dist <= 0)) //  the input slot is wrong
    {
        pre_nout_ccp = 16;
        return 0;
    }

    PlanDataVer[0].rear_dist = fabsf(PlanDataVer[0].rear_dist); //  get the positive value
    float u_front_min_dist =
        rte_min(rte_min(uradar.FCL, uradar.FCR) / 1000.0f, front_rear_limit_dist_max);
    float u_rear_min_dist =
        rte_min(rte_min(uradar.RCL, uradar.RCR) / 1000.0f, front_rear_limit_dist_max);
    if (u_front_min_dist < PlanDataVer[0].front_dist) //  1.nearer dist update immediatly
    {
        PlanDataVer[0].front_dist = u_front_min_dist; //  believe the uradar totally
    }
    else if (PlanDataVer[0].front_dist <
             front_rear_valid_dist_max) //  2. further dist need previous dist in uradar
                                        //  valid range,trust 60% of detect value
    {
        PlanDataVer[0].front_dist = u_radar_ratio * u_front_min_dist +
                                    (1 - u_radar_ratio) * PlanDataVer[0].front_dist;
    }
    if (u_rear_min_dist < PlanDataVer[0].rear_dist) //  1.nearer dist update immediatly
    {
        PlanDataVer[0].rear_dist = u_rear_min_dist; //   believe the uradar totally
    }
    else if (PlanDataVer[0].rear_dist <
             front_rear_valid_dist_max) //  2. further dist need previous dist in uradar
                                        //  valid range,trust 60% of detect value
    {
        PlanDataVer[0].rear_dist = u_radar_ratio * u_rear_min_dist +
                                   (1 - u_radar_ratio) * PlanDataVer[0].rear_dist;
    }

    // RTE_PK_SlotDetect_Get_SlotObj(PlanDataVer[0].obj_slot);
    /////////////////////////////////////////////////////////////////////////////////////////
    if (PlanDataVer[0].slotshape == 1 || PlanDataVer[0].slotshape == 2)
    {
        if (PlanDataVer[0].front_dist < 2.0f)
        {
            side_dis = outer_dist_to_new_targpos + fabsf(front_dy);
        }
        else
        {
            side_dis = outer_dist_to_new_targpos + PlanDataVer[0].width / 2.0f;
        }
        float long_dis = long_dist_to_new_targpos + PlanDataVer[0].h;
        float slotlen =
            PlanDataVer[0].rear_dist + PlanDataVer[0].front_dist + PlanDataVer[0].len;

        TargVec.vx     = cosf(targpos[2]);
        TargVec.vy     = sinf(targpos[2]);
        VertTargVec.vx = -TargVec.vy;
        VertTargVec.vy = TargVec.vx;
        Ref_B.x = targpos[0] - (PlanDataVer[0].h + PlanDataVer[0].rear_dist) * TargVec.vx;
        Ref_B.y = targpos[1] - (PlanDataVer[0].h + PlanDataVer[0].rear_dist) * TargVec.vy;
        Ref_C.x = Ref_B.x + VertTargVec.vx;
        Ref_C.y = Ref_B.y + VertTargVec.vy;
        Ref_D.x = targpos[0] +
                  (PlanDataVer[0].len - PlanDataVer[0].h + PlanDataVer[0].front_dist) *
                      TargVec.vx;
        Ref_D.y = targpos[1] +
                  (PlanDataVer[0].len - PlanDataVer[0].h + PlanDataVer[0].front_dist) *
                      TargVec.vy;
        Ref_E.x = Ref_D.x + VertTargVec.vx;
        Ref_E.y = Ref_D.y + VertTargVec.vy;
        Get_CrossPt_PointLine(slotobj.ptA, slotobj.ptB, Ref_B, Ref_C, &NewSlot_B);
        Get_CrossPt_PointLine(slotobj.ptC, slotobj.ptD, Ref_B, Ref_C, &NewSlot_C);
        Get_CrossPt_PointLine(slotobj.ptE, slotobj.ptF, Ref_D, Ref_E, &NewSlot_E);
        Get_CrossPt_PointLine(slotobj.ptC, slotobj.ptD, Ref_D, Ref_E, &NewSlot_D);

        slotobj.ptB = NewSlot_B;
        slotobj.ptC = NewSlot_C;
        slotobj.ptD = NewSlot_D;
        slotobj.ptE = NewSlot_E;
        SlotObj_Convert_float(slotobj, PlanDataVer[0].obj_slot);
        // SlotObj_Convert_struct(&slotobj,PlanDataVer[0].obj_slot);
        RTE_PK_SlotDetect_Set_SlotObj(&slotobj);
        // RTE_PK_SlotDetect_Set_TargPos(TargPos_RTE);

        //////////////////////////////////////////////////////////////////////////////////
        PlanDataVer[0].pointb_danger[0] = PlanDataVer[0].obj_slot[0][2];
        PlanDataVer[0].pointb_danger[1] = PlanDataVer[0].obj_slot[0][3];
        PlanDataVer[0].pointf_danger[0] = PlanDataVer[0].obj_slot[3][2];
        PlanDataVer[0].pointf_danger[1] = PlanDataVer[0].obj_slot[3][3];
        PlanDataVer[0].pointb_danger[2] = PlanDataVer[0].finpoint[2];
        PlanDataVer[0].pointf_danger[2] = PlanDataVer[0].finpoint[2];

        PlanDataVer[0].stpoint[0] =
            PlanDataVer[0].pointf_danger[0] +
            cosf(PlanDataVer[0].finpoint[2]) * long_dis +
            cosf(PlanDataVer[0].finpoint[2] - PlanDataVer[0].left_fac * PI / 2) *
                side_dis;
        PlanDataVer[0].stpoint[1] =
            PlanDataVer[0].pointf_danger[1] +
            sinf(PlanDataVer[0].finpoint[2]) * long_dis +
            sinf(PlanDataVer[0].finpoint[2] - PlanDataVer[0].left_fac * PI / 2) *
                side_dis;
        PlanDataVer[0].stpoint[2] = PlanDataVer[0].finpoint[2];

        // PlanDataVer[0].swell =rte_min(((slotlen-2.2f)*0.25f+0.8f),1.6f)*g_local_swell;
        PlanDataVer[0].close_side_dist =
            fabsf(Project_PosTo1st_ry(PlanDataVer[0].finpoint,
                                      PlanDataVer[0].obj_slot[2])) -
            PlanDataVer[0].width / 2.0f;
        PlanDataVer[0].swell =
            rte_min(((slotlen - 5.6f) * 0.2f + 0.9f), 1.6f) * g_local_swell;
    }

    float avmDist[3] = {0.0f, 0.0f, 0.0f};
    RTE_PK_Location_Get_CurPos(temploc);
    PK_CopyPos(PlanDataVer[0].finpoint,
               temploc); //  20181011: make sure use the real vehicle's pos
    PK_PathPlan_SlotObj_To_stPos_By_Swell(PlanDataVer,
                                          avmDist); //  20181012: to expand the slot
    PlanDataVer[0].swell = 0; //  have already expended obstacles,so the vehicle's
                              //  expention can be set to zero
    curState.slotshape =
        PlanDataVer[0].slotshape; //  20181012: park out update the slotshap
    return 1;
}

int ParkoutPathplan(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                    const U_RadarType uradar, PK_Cur_AutodrPara &curState)
{
    const float out_dist = 2.8f; //  the head of vehicle out dist of ABEFs
    int pathNum          = 0;
    PlanDataCase PlanDataVer[1];
    float temploc[4] = {0.0f};
    float Envi_obj[5][4];
    SlotObj_T slotobj;

    memset(PlanDataVer, 0, sizeof(PlanDataVer)); //  use the global
    RTE_PK_Location_Get_CurPos(temploc);
    RTE_PK_SlotDetect_Get_SlotObj(
        &slotobj); //  make sure slotobj not be changed by previous dataprocess
    SlotObj_Convert_float(slotobj, Envi_obj);

    if (PathExec_ParaParkOutDataProcess(PlanDataVer, uradar, curState) == 0)
    {
        return 0;
    } //  input error

    float rx_max = rte_max(Project_PosTo1st_rx(temploc, Envi_obj[1]),
                           Project_PosTo1st_rx(temploc, Envi_obj[4]));

    if (PlanDataVer[0].slotshape == 0)
    {
        pathNum = 0;
    }
    else if (PlanDataVer[0].slotshape > 2) // vert parking
    {
        PK_CopyPos(plannedPath[0], temploc);
        float rx_diff =
            rx_max - (VEHICLE_LEN - REAR_SUSPENSION -
                      out_dist); // 2.8f, forward 2.8m , so we can open the door
        plannedPath[0][3] = rte_min(rx_diff, VEHICLE_LEN); //  avoid abnormal condition
        plannedPath[0][4] = 0.0f;
        pathNum           = 1;
    }
    else
    {
        pathNum = PK_PathPlan_SpaceExplorer_Para(PlanDataVer, plannedPath);
        PathExec_PathInverse(pathNum, plannedPath);
    }

    return pathNum;
}
