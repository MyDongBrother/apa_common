#ifndef PK_PATHPLAN_DEBUG_H
#define PK_PATHPLAN_DEBUG_H
#include "PK_PathPlanP.h"

void PathPlan_ReportSlotsInfo(const SlotInfo_T &slot, const float stpoint[4]);

void PathPlan_ReportObjsInfo(const int FS_ObjDir[64], const float FS_Obj[64][4],
                             const int FS_ObjDir_Rear[64],
                             const float FS_Obj_Rear[64][4]);

void PathPlan_ReportTrajsInfo(const float trajs[][TRAJITEM_LEN], const uint32_t num,
                              const PlanDataCase &PlanDataVer, const int slotIndex);

void PathPlan_PrintPathStart();

void PathPlan_PrintPathEnd();

void PathPlan_PrintPathInfo(const PlanInfoType &slotPla);

void PathPlan_PrintPathUpdateResult(const int check[6]);

void PathPlan_PrintDangerObj(const char *inf, const float traj[][TRAJITEM_LEN],
                             int pathNum, const PlanDataCase &PlanDataVer);

void PathPlan_PrintPlanDebug(const char *inf, const float traj[][TRAJITEM_LEN],
                             int pathNum, const PlanDataCase &PlanDataVer);

void PathPlan_PrintPlanEnd();

void PathPlan_InitFootPrints();

void PathPlan_FootPrints(int type, uint8_t level, uint8_t step, int num);

int PathPlan_PrintFootPrints(uint32_t *debugs);

#endif
