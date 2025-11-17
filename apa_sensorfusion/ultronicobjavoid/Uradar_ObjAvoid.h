#ifndef _URADAR_OBJAVOID_H
#define _URADAR_OBJAVOID_H

#include "Rte.h"
#include "PK_Calibration.h"
#include "Rte_ComIF.h"

enum
{
    SET_BRAKE,
    SET_STOP,
    CANCEL_BRAKE,
    CANCEL_STOP
};

#define LINE_STEER_ANGLE 90

typedef struct Veh_Param
{
    float Eps;   //  Eps angle from path plan module
    float Speed; // Vehicle speed from path execution module
    float brakeDist;
} Veh_Param;

typedef struct Veh_AvoidState
{
    float Eps;   //  Eps angle from path plan module
    float Speed; // Vehicle speed from path execution module
    RTE_BSW_GearType gearSt;
    Vel_MoveMentStType velMoveSt;
    float curPos[4];
    bool isVehInSlot;
    uint8_t isSummonUp;
} Veh_AvoidState;

uint16_t SetUradarStateBit(const int index, const uint16_t dangerType, const int flag);

int AutoDrive_ObjAvoid(const Veh_Param &Com_VehParm, Veh_AvoidState &curSt,
                       float Uradar[Radar_ID_MAX], Veh_Param *targVehParm,
                       uint8 &dangerWatch, void *logId);

void ObjAvoid_InitInternalVar(float Uradar[Radar_ID_MAX]);

int ObjAvoid_RadarCount(const int radarId);

float CalcRadarSlotsDist(int radarIdx, const float curPos[3]);

float CalcSideNextRadarSlotsDist(int radarIdx, Veh_AvoidState &curSt);

int CheckNearExistObj(int radarIdx, const float curPos[3]);

#endif
