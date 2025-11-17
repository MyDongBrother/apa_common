#ifndef PK_PATHPLANBASEPLAN_H
#define PK_PATHPLANBASEPLAN_H
#include "PathPlan_Func.h"
#include "Rte.h"
#include "Rte_Func.h"


int GetPath_StToMidPos_Na2(float stpoint[5], PlanDataCase PlanData_Ver[1],
                           float StToMid_Traj[][TRAJITEM_LEN], float radius,
                           int type = 0);

int Stpoint_Extend(PlanDataCase PlanData_Ver[1], float Exp_stpoint[][5], float extend_ds);

int Path_Check_Connect(const float stpoint[3], const float finpoint[3],
                       float TempPath[][TRAJITEM_LEN], int line_num);

int PK_PathTryConnect_TwoCon(int traj_pre, float finpoint[TRAJITEM_LEN],
                             PlanDataCase PlanData_Ver[1],
                             float Act_traj_dir[][TRAJITEM_LEN]);

#endif
