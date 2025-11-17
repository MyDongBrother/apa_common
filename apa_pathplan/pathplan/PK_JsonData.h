#ifndef PK_JSONPARSE_H
#define PK_JSONPARSE_H

#include "Rte.h"
#include "json/json.h"

int RestoreSlotData(Json::Value &jsonRoot, SlotInfo_T &slotInfo);

int RestoreCurPos(Json::Value &jsonRoot, float curPos[4]);

int RestoreObjData(Json::Value &jsonRoot, FusionObj_T fusInfo[4]);

int RestoreTrajsData(Json::Value &jsonRoot, PlanInfoType &planInfo);

#endif