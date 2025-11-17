#ifndef ETHENET_OBJECT_H
#define ETHENET_OBJECT_H
#include "PK_Utility.h"

#include "Record_Log.h"
#include "Rte.h"
#include "MathFunc.h"
#include "json/json.h"
#include "PK_PathPlan.h"

typedef enum
{
    Obstacle_Type_No,            // 无效类型 0
    Obstacle_Type_Traffic_Cone,  // 锥桶 1
    Obstacle_Type_StopFlag,      // 限位杆 2
    Obstacle_Type_Ground_Lock,   // 地锁(打开状态) 3
    Obstacle_Type_HANDICAP,      // 残疾人车位标志 4
    Obstacle_Type_PVC,           // PVC管 5
    Obstacle_Type_CARTON,        // 纸箱 6
    Obstacle_Type_CHAIR,         // 凳子 7
    Obstacle_Type_PEDSTRAIN,     // 行人 8
    Obstacle_Type_BUCKET,        // 水桶 9
    Obstacle_Type_E_BIKE,        // 电瓶车 10
    Obstacle_Type_STOP_SIGN,     // 禁停牌 11
    Obstacle_Type_CHARGING_STOP, // 充电桩 12
    Obstacle_Type_DECEL_STRIP,   // 减速带 13
    Obstacle_Type_BUMPER,        // 防撞桶 14
    Obstacle_Type_FENCE,         // 护栏 15
    Obstacle_Type_WHEEL_CHAIR,   // 轮椅 16
    Obstacle_Type_FIRE_KIT,      // 消防箱 17
    Obstacle_Type_Ground_UnLock, // 地锁(打开状态) 3
    Obstacle_Type_Warning_Post,  // 警示柱
} PK_ObstacleType;

struct VisionObstacleInfo
{
    int obstacleIndex;
    uint8_t status;
    uint32_t timeStamp;
    uint8_t obstacleType; // 障碍物类型
    uint32_t obstacleWidth;
    int obstacleTheta;
    float pointOrd[4][2];
    float midPoint[2];
    float centerPoint[2];
};

/*
typedef struct _AvmObstacle_T//以车后轴中心为坐标原点，车前方为x正方向{
{
    int obj_valid[SF_OBJ_NUM];//目标障碍物线段有效性（无效-0，有效-1）
    LineSeg_T obj[SF_OBJ_NUM];//障碍物线段

    int central_point_valid[SF_OBJ_NUM];//目标障碍物中心点有效性（无效-0，有效-1）
    Point_T central_point[SF_OBJ_NUM]; //目标障碍物中心点坐标

    int closest_point_valid[SF_OBJ_NUM];//目标障碍物距车最近点有效性（无效-0，有效-1）
    Point_T closest_point[SF_OBJ_NUM]; //目标障碍物距车最近点坐标

    int angle_valid[SF_OBJ_NUM];//目标障碍物方向有效性（无效-0，有效-1）
    float angle[SF_OBJ_NUM]; //目标障碍物方向（以x正半轴为起始,单位：弧度）

    float length[SF_OBJ_NUM];//障碍物长度（x方向上），无效默认为0
    float width[SF_OBJ_NUM];//障碍物宽度（y方向上），无效默认为0
    float height[SF_OBJ_NUM];//障碍物高度（z方向上），无效默认为0
    int type[SF_OBJ_NUM]; //目标障碍物类型（根据视觉信息定义）
    int num; //障碍物数量（不超过 “SF_OBJ_NUM” 定义的数量）
} AvmObstacle_T;
*/

typedef struct ObstacleThreeDimensions
{
    float len;
    float wight;
    float height;
} ObsThreeDimen;

/*
    前后左右:0000
    以BCD码进行运算
*/
enum supplierNumber
{
    Front = 8,
    Back  = 4,
    Left  = 2,
    Right = 1
};

void ParseVisionObject(const char *str);

void PK_Set_Avm_Obstacle(AVM_ObjPointsType &avmObj);

#endif