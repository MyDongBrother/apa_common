#ifndef _PATHPLAN_FUNC_H_
#define _PATHPLAN_FUNC_H_
#include "PK_PathPlan.h"
#include "PathPlan_Config.h"
#include "Rte_ComIF.h"
#include "MathFunc.h"
#include "PK_Utility.h"

typedef struct
{
    float x0;     // 起点坐标（车后轴中心）
    float y0;     // 起点坐标（车后轴中心）
    float theta0; // 起点车身姿态（车后轴中心）
    float ds;     // 路径长度
    float r;      // 圆半径（弧，车后轴中心）
    float ftheta; // 圆心与起点的连线的弧度
    float dtheta; // 圆心角（车后轴中心）
    float rr[2];  // 0-内弧半径 1- 外弧半径
    float x_end;  // 终点
    float y_end;
    float theta_end;   // 终点车的姿态（弧度）
    float x_c;         // 圆心
    float y_c;         // 圆心
    float x_start2[2]; // 内弧0和外弧1的开始坐标
    float y_start2[2];
    float x_end2[2]; // 内弧0和外弧1的结束坐标
    float y_end2[2];
    float x_out[4]; // 前进 0 结束位置左侧前点 1 开始位置左侧后点  2 开始位置右侧后点  3
                    // 结束位置右侧前点， 后退开始结束位置相反
    float y_out[4];
    float xv_start[4]; // 车辆4个角点在起点的坐标
    float yv_start[4];
    float xv_end[3][4]; // 车辆4个角点在终点的坐标(3个矩形拟合车辆轮廓)
    float yv_end[3][4];
    float x_start3; // 弧起点坐标
    float y_start3;
    float x_end3; // 弧终点坐标
    float y_end3;
    float dtheta_e;    // ？
    float angle_total; // 起点车头到终点车尾的圆心角
} path_para;

typedef struct
{
    float x1;
    float y1;
    float x2;
    float y2;
    float A;
    float B;
    float C;
    float theta_obs;
    float dis_obs;
} obj_para;

typedef struct
{
    int is_cross; // 0 - ok通过 1 - no 不通过  2 - 根据二分法中间结果决定 通过 还是 不通过
    obj_para extr_para;
} ObjPara;

#define PATHPLAN_ARRSIZE(arr) (sizeof(arr) / sizeof(arr[0]))

int PathPlan_PathVerify2(const float path[TRAJITEM_LEN], const int obj_num,
                         const float obj[][4], const float swell,
                         int type = 0); // add 20230921

float PathPlan_LandMark(float path[TRAJITEM_LEN], int obj_num, const float obj[][4],
                        float swell);

float PathPlan_LandMark2(float path[TRAJITEM_LEN], int obj_num, const float obj[][4],
                         float swell, int type = 0); // add 20230921

int UpdateSlotWithObsInSlot(SlotInfo_T &slot, int FS_ObjDir[FS_OBJ_ARR_NUM],
                            float FS_Obj[FS_OBJ_ARR_NUM][4],
                            int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                            float FS_Obj_Rear[FS_OBJ_ARR_NUM][4]);

uint8_t Check_TrajStartPos_Is_CurPos(MultiPlanInfo &multiPlans, float curPos[4],
                                     uint8 IsTrajStPos[MAX_PARKED_SLOTS]);

int PathPlan_UpdateSlotSwell(PlanDataCase &PlanData);

int ExpandSlotObjs(SlotInfo_T &slot, const float stpoint[3]);

void PathPlan_BuildSlotObjDanger(PlanDataCase PlanData_Ver[1], const float slotB_Pos[3],
                                 const float rect_pr_possb[4],
                                 const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                 const float FS_Obj[FS_OBJ_ARR_NUM][4],
                                 const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                 const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4]);

void AddSlotObjDanger(const PlanDataCase planData[1], const int FS_ObjDir[FS_OBJ_ARR_NUM],
                      const float FS_Obj[FS_OBJ_ARR_NUM][4],
                      const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                      const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4],
                      int dangeObjDir[FS_OBJ_ARR_NUM],
                      float dangerObj[FS_OBJ_ARR_NUM][4]);

float PathPlan_LandMarkByStopper(float path[5], const float obj[][2]);

// 获取线段斜率（弧度）
float Get_LineSegment_Theta(const float p0[2], const float p1[2]);

// 以p0为起点延长线段，返回斜率（弧度）
float Extend_LineSegment(const float p0[2], float p1[2], float extend_ds);

void PathPlan_ExpandObjBySwell(const float swell, const float avmDist[3],
                               PlanDataCase planData[1]);

int Get_Kneading_Times(float TempPath[][TRAJITEM_LEN], float line_num);

void PathPlan_UpdateSideDist(const PlanDataCase planData[1], const SlotInfo_T &slot,
                             float avmDist[3]);

#endif