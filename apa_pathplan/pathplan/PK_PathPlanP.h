#ifndef PK_PATHPLANP_H
#define PK_PATHPLANP_H
#include "PathPlan_Func.h"
#include "PathPlan_Tools.h"
#include "Rte.h"

int PathPlan_TwoCircleCon(float stpos[3], float finpos[3], int objnum,
                          const float slotobj[FS_OBJ_ARR_NUM][4],
                          float expTraj[][TRAJITEM_LEN], float swell);

int PathPlan_DrawTrajectory(const float dsmin, const float traject[TRAJITEM_LEN],
                            float Position_arr[30][4], const float start_ds);

int PathPlan_UpdateParaSlotCDWithObs(PlanDataCase &PlanData,
                                     int FS_ObjDir[FS_OBJ_ARR_NUM],
                                     float FS_Obj[FS_OBJ_ARR_NUM][4],
                                     int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                     float FS_Obj_Rear[FS_OBJ_ARR_NUM][4]);

int PathPlan_SpaceExplorer_Vert(PlanDataCase PlanData_Ver[1],
                                float Exp_traj[][TRAJITEM_LEN], int *IsDirConnect,
                                int type = 1);

int PathPlan_SpaceExplorer_Mid2Targ_Vert(float TrajSt_Mid_NS[TRAJITEM_LEN],
                                         PlanDataCase PlanDataVer[1],
                                         float Exp_traj[][TRAJITEM_LEN], int num = 7);

float Get_AnotherObs_Nearest_dy_to_CurPos(float curpos[3], int slotshap,
                                          float obsobj[][4], int obsNum);

int PathPlan_NarrowSpaceExplorer_Vert(PlanDataCase PlanData_Ver[1],
                                      float Exp_traj[][TRAJITEM_LEN]);

int PathPlan_SetParkingSlot(SlotInfo_T &manuSlot);

int PathPlan_InverseSlot(const PK_SlotShapeType slotShape, const float stpoint[4],
                         SlotInfo_T &manuSlot);

int PathPlan_HybridReplan(const PlanDataCase planData[1],
                          float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], int Path_num,
                          int slotindex);

int PathPlan_HybridReplanResult(const int slot, const PlanDataCase planData[1],
                                float PlannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]);

bool PathPlan_IsHybridSlot(const SlotInfo_T &slotinf);

void PathPlan_HybridClearResult(const Multi_Slot_Array_T &slotArray);

int PathPlan_GetTraj(const Multi_Slot_Array_T &slots, const SlotInfo_T &slot,
                     PlanDataCase PlanData_Ver[1], float Act_traj[][TRAJITEM_LEN],
                     bool usedHybrid, int *IsDirConnect);

// void Stretch_AnotherObs_ForVertSlot_Na(PlanDataCase PlanDataVer[1]);
// int PathPlan_TwoEqualCircleCon_AllowStposRyErr(float stpos[3], float finpos[3], float
// Rr, int objnum, float slotobj[5][4], float expTraj[][TRAJITEM_LEN], float swell);

#endif
