#ifndef PK_PATHPLANUPDATEP_H
#define PK_PATHPLANUPDATEP_H

#include "PK_PathPlanP.h"

typedef struct
{
    int counter;
    int slotIndex;
    int pathNum;
    int record;
    float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];
} PlanedPath;

int Connect_BreakPoint(float *traj, float *trajnext); // add2023.3.2 Tangsj

void PathPlan_UpdatePlannedSlots(const Multi_Slot_Array_T &Multi_Slot_Array);

PlanedPath &PathPlan_GetPlannedPath(const int slotIndex);

void PathPlan_CleanPlannedPath();

void PathPlan_AddPlanPath(PlanedPath &paths, const int costTime,
                          const float trajs[][TRAJITEM_LEN], const float pathNum);

int PathPlan_UpdatePlanPath(PlanedPath &path, PlanDataCase &planData,
                            float trajs[][TRAJITEM_LEN], bool updateTarget);

#endif
