
#ifndef _MATH_FUNC_H
#define _MATH_FUNC_H

#include "Std_Types.h"
#include "MathPara.h"
#include <math.h>

#ifndef rte_max
#define rte_max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef rte_min
#define rte_min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef sign
#define sign(a) ((fabs(a) < ZERO_FLOAT) ? 0 : ((a) > 0 ? 1 : -1))
#endif

float Round_PI(float theta);

float pow2(float f);

float pow3(float f);

void CoordinadteTransfer(const float *cd0, const float *cd1, float *cd);

float InterPolationOneDim(const float *x, const float *y, const float xx, int n);

int SearchOptimalRoute(const int *trajnum, const float *trajsum, int n);

float GetAverAngle(float Theta1, float Theta2);

void Convert(const float *pO, const float *pin, float *pout);

void RevConvert(const float *pO, const float *pin, float *pout);

float PK_Rou_to_EPS(const float Rr, const float tar_spd);

float PK_EPS_to_Rou(const float EPS_angle, const float tar_spd);

float PK_Rr_to_EPS(const float Rr, const float tar_spd);

float Project_PosTo1st_rx(const float *cd0, const float *cd1);

float Project_PosTo1st_ry(const float *cd0, const float *cd1);

float Project_PosTo1st_rtheta(const float *cd0, const float *cd1);

void Project_Line_to_Pos(float *line0, float *pos); //

void PK_CopyPos(float *desti_tra, const float *source_tra);

void PK_CopyTra(float *desti_tra, const float *source_tra);

int PK_Check_Path(const int Path_Num, const float Path[][5], const float stpoint[3],
                  const float finpoint[3], const int obj_num, const float Envi_obj[][4],
                  const float swell); // Check Path valid,1ok, 0 not ok

int PK_Check_Path_simple(int Path_Num, float Path[][5], int obj_num, float Envi_obj[][4],
                         float swell, float stpoint[3], float finpoint[3],
                         int *IsNarrowSlot);

void PK_Get_Path_EndPos(const float Path[5], float EndPos[3]);

float PK_GetPathLen(int tra_num, float Planned_Path[][5]);

int PK_RectLineCross(const float rect[4], float line[4]);

int PK_PosObjCross(const float stpoint[3], const int obj_num, const float obj[][4],
                   const float ds, const float swell);

float PK_PosObjCrossMinDist(const float stpoint[3], const int obj_num,
                            const float obj[][4]);

float PK_PointToLineDist(const float line[4], const float pt[2]);

float PK_RectLineCrossDist(
    const float rect[4],
    float line[4]); // return <0 Cross; return min distance from rectangle;

int Cal_SlotShap(float finpos[3], float slotobj[5][4], float *left_fac);

uint8_t Uint16ToBuf(const uint16_t value, uint8_t *buff);

uint8_t Uint32ToBuf(const uint32_t value, uint8_t *buff);

uint8_t FloatToBuf(const float value, uint8_t *buff);

uint8_t BufToUint16(const uint8_t *buff, uint16_t *value);

uint8_t BufToUint32(const uint8_t *buff, uint32_t *value);

uint8_t BufToFloat(const uint8_t *buff, float *value);

uint8_t BufToUint64(const uint8_t *buff, uint64_t *value);

uint8_t Uint64ToBuf(const uint64_t value, uint8_t *buff);

bool CompareSlotIndex(const int index_apa, const int index_hmi);

#endif
