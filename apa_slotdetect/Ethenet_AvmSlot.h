#ifndef ETHENET_AVMSLOT_H
#define ETHENET_AVMSLOT_H
#include "Rte_Types.h"
#include "PK_Utility.h"
#include "MathFunc.h"

struct VisionSlotInfo
{
    int slotIndex;      // 车位编号
    uint8_t status;     // 车位状态
    uint32_t timeStamp; // 时间戳
    uint8_t slotType;   // 车位类型
    uint8_t isBlocked;  // 占用标识
    uint8_t lockFlag;   // 地锁（暂时，车位被占用等同于地锁打开 0 - 无地锁，无占用 1 -
                        // 地锁打开 2 - 地锁关闭且无占用 3 - 地锁关闭或无地锁，且被占用）
    uint8_t stopFlag;   // 限位杆有效性
    uint8_t typeSideCar;
    uint32_t slotAngle;    // bc与be边夹角
    uint8_t isReliable;    // 可靠性
    float weight;          // 权重
    float target[3];       // 目标点
    float pointOrd[4][2];  // 车位角点（B、C、D、E）
    float stopPoint[2][2]; // 限位杆两端点
};

constexpr float AVM_VERT_START_START = 5.0;
constexpr float AVM_VERT_START_END   = 0.2;
constexpr int AVM_SUCCESS            = 0;
constexpr int AVM_FAIL               = 1;

int ReportAvmSlot(VisionSlotInfo &slot, AVM_SlotPointsType &avmSlot, FILE *avmflog);

int CacheAvmSlot(const VisionSlotInfo &slot);

int UpdateAvmSlot(AVM_SlotPointsType &avmSlot);

int CheckSlotEntry(VisionSlotInfo &slot);

Relation_T AvmSlotDir(const float pointOrd[4][2]);

int CheckStopPoint(VisionSlotInfo &slot);

int CheckSearchSlot(VisionSlotInfo &slot);

float PointToLineDist(const float line1[2], const float line2[2], const float point[2]);

bool InRearErea();

bool InSideErea();

float getDistWeight(float data);

FILE *OpenLogFile(const char *name);

#endif
