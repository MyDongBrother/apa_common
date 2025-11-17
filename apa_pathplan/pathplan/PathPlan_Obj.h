#ifndef _PATHPLAN_OBJ_H_
#define _PATHPLAN_OBJ_H_
#include "PK_PathPlan.h"
#include "PathPlan_Obj.h"
#include "PathPlan_Config.h"

enum
{
    OBJTYPE_NONE   = 0,
    OBJTYPE_LEFT   = 1,
    OBJTYPE_RIGHT  = 2,
    OBJTYPE_BOTTOM = 3,
    OBJTYPE_TOP,
    OBJTYPE_NUM
};

int PathPlan_BuildObjsByStopper(const SlotInfo_T &slot, LineSeg_T parkBars[2]);

int PathPlan_PathVerifyFusion(const float path[TRAJITEM_LEN],
                              const FusionObj_T &FS_Info_A, const float swell);

void PathPlan_DyanObj(const PlanDataCase planData[1], FusionObj_T &extObj,
                      const float curPoint[3]);

int Add_Obj_Temp(const Multi_Slot_Array_T &slots, PlanDataCase planData[1]);

void UpdateObjLenByRaw(const SlotInfo_T &slot, int Obj[FS_OBJ_ARR_NUM],
                       float ObjDir[FS_OBJ_ARR_NUM][4], int ObjRearDir[FS_OBJ_ARR_NUM],
                       float ObjRear[FS_OBJ_ARR_NUM][4]);

void FilterObjByLen(int objDir[FS_OBJ_ARR_NUM], float obj[FS_OBJ_ARR_NUM][4],
                    int objRearDir[FS_OBJ_ARR_NUM], float objRear[FS_OBJ_ARR_NUM][4]);

#endif
