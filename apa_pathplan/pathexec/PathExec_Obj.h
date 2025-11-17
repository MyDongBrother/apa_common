#ifndef _PATHEXECUTE_OBJ_H_
#define _PATHEXECUTE_OBJ_H_
#include "PK_PathExecuteP.h"

void PathExec_UpdateSlot(const float finpoint[3], float envObj[][4], int shapeType);

void PathExec_UpdateObsObj_AroundTargSlot();

void PathExec_GetObsObj_AnotherSide(FusionObj_T &fusion);

int PathExec_BuildSideRadarAlarmObj(const float curpos[4], const float finpoint[3],
                                    float objs[4][4]);

void PathExec_DangerObj(PK_Cur_AutodrPara curSt[1], float enviObj[5][4]);

void PathExec_GetAvmPoint(float points[8]);

void Calc_Target(const int slottype, const uint8_t confirence, const SlotObj_T &slotobj,
                 const Avm_Pot_T &avmPot, VehPos_T &targpos);

void PathExec_UpdateAvmPoint(PK_Cur_AutodrPara curSt[1], Avm_Pot_T &avmSlot);

void PathExec_AddObsObjNearSlot(const float obj[4], const float finpoint[3]);

uint8_t PathExec_UpdateSlotByObjNearSlot(const float curpos[4], const float finpoint[3],
                                         const Avm_Pot_T &avmSlot, SlotObj_T &slotObj);

void PathExec_GetBeConference(float be[2][4]);

void PathExec_SetBeConference(const float be[2][4]);

void PathExec_ClearParkObj();

uint16_t PathExec_SideRadarAlarmObjState();

#endif
