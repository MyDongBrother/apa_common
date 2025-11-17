#include "PK_Calibration.h"
#include "Debug_Cmd.h"
#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Func.h"
#include "PathExec_Debug.h"
#include "PK_StateManage.h"
#include "PK_PathPlan.h"
#include "PK_Utility.h"
#include "PK_B.h"

void PathExec_UpdateAvpAutodrPara(PK_Cur_AutodrPara curSt[1],
                                  float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN])
{
    if (RTE_PK_SM_GetApaMode() == PARKOUT_MODE && curSt[0].tra_num != 0)
    {
        return;
    } // 泊出模式并且有路径

    if (curSt[0].leftDist > 2.5f)
    {
        return;
    } // 路径距离大于2.5m

    RTE_PK_Location_Get_CurPos(curSt[0].curpos);
    curSt[0].cur_tra_nth = 0;
    curSt[0].tra_num     = 1;
    curSt[0].cvel        = 3.0;
    curSt[0].slotshape   = PK_SLOT_LEFT_VERT;
    memcpy(Planned_Path[0], curSt[0].curpos, sizeof(float) * 3);

    if (RTE_PK_SM_GetApaMode() == PARKOUT_MODE)
    {
        Planned_Path[0][3] = 2.0f;
    }
    else
    {
        // 已经找到车位了, 降低速度
        Multi_Slot_Array_T slotsInfo;
        RTE_PK_SlotDetect_Get_Multi_SlotInfo(&slotsInfo);
        if (slotsInfo.multiNum > 0)
        {
            Planned_Path[0][3] = 3.1f;
        }
        else
        {
            Planned_Path[0][3] = 4.5f;
        }
    }

    Planned_Path[0][4] = 0;

    PK_Get_Path_EndPos(Planned_Path[0], curSt[0].finpos);
    RTE_PK_SlotDetect_Set_TargPos(curSt[0].finpos);

    Rect_T cornerPt;
    VehPos_T velPos;
    memcpy(&velPos, curSt[0].finpos, sizeof(float) * 3);
    Get_Veh_CornerPt(velPos, &cornerPt, 0.3);

    SlotInfo_T manuSlot;

    Point_T avmPoints[4];
    memcpy(&avmPoints[3], &cornerPt.pt_fl, sizeof(Point_T));
    memcpy(&avmPoints[0], &cornerPt.pt_rl, sizeof(Point_T));
    memcpy(&avmPoints[2], &cornerPt.pt_fr, sizeof(Point_T));
    memcpy(&avmPoints[1], &cornerPt.pt_rr, sizeof(Point_T));

    PK_BuildAvmSlotInfo(avmPoints, PK_SLOT_LEFT_VERT, manuSlot);
    memcpy(&manuSlot.targpos, curSt[0].finpos, sizeof(float) * 3);

    PlanInfoType plansInfo;
    plansInfo.Path_num    = 1;
    plansInfo.Slot_index  = 10000;
    plansInfo.slotshape   = PK_SLOT_LEFT_VERT;
    plansInfo.IsDirConect = 1;
    memcpy(&plansInfo.slotObjs, &manuSlot.slotobj, sizeof(SlotObj_T));
    PK_CopyTra(plansInfo.Act_traj[0], Planned_Path[0]);

    RTE_PK_DataConvt_Set_Target_PlanInfo(&plansInfo);

    RTE_PK_SlotDetect_Set_SlotObj(&manuSlot.slotobj);
}

void PathExec_EndAvpAutodrPara(PK_Cur_AutodrPara curSt[1],
                               float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN])
{
    PlanInfoType plansInfo;
    RTE_PK_DataConvt_Get_Target_PlanInfo(&plansInfo);
    if (plansInfo.Path_num <= 0)
    {
        return;
    }

    uint32_t slotId = plansInfo.Slot_index;

    MultiPlanInfo multiPlans;
    RTE_PK_PathPlan_Get_Multi_PlanInfo(&multiPlans);

    if (multiPlans.slotNum == 0 || multiPlans.slotNum == 0)
    {
        return;
    }

    int index = 0;
    for (index = 0; index < multiPlans.slotNum; index++)
    {
        if (static_cast<int>(slotId) == multiPlans.slotPlans[index].Slot_index)
        {
            break;
        }
    }

    if (index >= multiPlans.slotNum)
    {
        return;
    }

    float firstTraj[TRAJITEM_LEN];
    PK_CopyTra(firstTraj, plansInfo.Act_traj[0]);

    float curspd   = RTE_PK_Location_Get_Vel_Spd();
    float slipDist = rte_min(1.5f * fabs(curspd), 0.6f);
    if (firstTraj[3] < slipDist && slipDist > 0.20f) // 向后退或者路径过短
    {
        PRINT_SWITCH_INFO("EndAvpAutodrPara");
        PathExec_PrintPathInfo(plansInfo.Act_traj, plansInfo.Path_num);

        float tempTraj[2][TRAJITEM_LEN];
        RTE_PK_Location_Get_CurPos(tempTraj[0]);
        tempTraj[0][3] = slipDist;
        tempTraj[0][4] = 0.0;

        PK_Get_Path_EndPos(tempTraj[0], tempTraj[1]);
        tempTraj[1][3] = -Cal_Dis_Pt2Pt(tempTraj[1], firstTraj);
        tempTraj[1][4] = 0.0;

        if (plansInfo.Path_num + 2 < MAX_SINGLE_TRAJ_NUM)
        {
            for (int i = 0; i < plansInfo.Path_num; i++)
            {
                PK_CopyTra(plansInfo.Act_traj[plansInfo.Path_num + 1 - i],
                           plansInfo.Act_traj[plansInfo.Path_num - 1 - i]);
            }

            PK_CopyTra(plansInfo.Act_traj[1], tempTraj[1]);
            PK_CopyTra(plansInfo.Act_traj[0], tempTraj[0]);
            plansInfo.Path_num = plansInfo.Path_num + 2;
        }
        RTE_PK_DataConvt_Set_Target_PlanInfo(&plansInfo);
        PathExec_PrintPathInfo(plansInfo.Act_traj, plansInfo.Path_num);
    }
}
