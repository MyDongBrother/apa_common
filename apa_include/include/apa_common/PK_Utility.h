/**
 * @file PK_Utility.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

/** Multiple inclusion protection */
#ifndef PK_UTILITY
#define PK_UTILITY

#include "Rte_Types.h"
#include "PK_Config.h"
#include <math.h>

/** Version and module identification */
#define PK_UTILITY_VENDOR_ID (0u)
#define PK_UTILITY_MODULE_ID (0u)

/** Component Version Information */
#define PK_UTILITY_SW_MAJOR_VERSION (0u)
#define PK_UTILITY_SW_MINOR_VERSION (0u)
#define PK_UTILITY_SW_PATCH_VERSION (6u)

/*********************************************************************************************************
** Definition of ultrasound radar parameters of BYD Song Plus
*********************************************************************************************************/
extern float
    FSR_DELTAX; // 3.19062f        //定义探头1，FSR：右侧探头位置,朝向 对应宋pro-6号FSR
extern float FSR_DELTAY; // -0.9188f          //2022.7.31 超声探头位置调整by lyx,update
                         // 探头位置和角度
extern float
    FSR_AERFA; //-1.57080f        //2022.7.31 超声探头位置调整by lyx,update 探头位置和角度

extern float
    FSL_DELTAX; // 3.19062f        //定义探头2，FSL：左侧探头位置,朝向 对应宋pro-1号FSL
extern float FSL_DELTAY; // 0.9188f           //2022.7.31 超声探头位置调整by lyx,update
                         // 探头位置和角度
extern float FSL_AERFA;  // 1.57080f          //2022.7.31 超声探头位置调整by lyx,update
                         // 探头位置和角度

extern float
    RSL_DELTAX; // -0.45174f   //定义探头3，RSL：左侧探头位置，朝向      对应宋pro-12号RLS
extern float RSL_DELTAY; // 0.92545f
extern float RSL_AERFA;  // 1.605703f

extern float
    RSR_DELTAX; // -0.45174f   //定义探头4，RSR：右侧探头位置，朝向     对应宋pro-7号RRS
extern float RSR_DELTAY; // -0.92545f
extern float RSR_AERFA;  // -1.605703f

extern float FOL_DELTAX; // 3.44664f        //定义探头5，FOL：前方探头位置，朝向
extern float FOL_DELTAY; // 0.76067f
extern float FOL_AERFA;  // 0.806866f

extern float FCL_DELTAX; // 3.61424f        //定义探头6，FCL：前方探头位置，朝向
extern float FCL_DELTAY; // 0.30188f
extern float FCL_AERFA;  // 0.10472f

extern float FCR_DELTAX; // 3.61424f        //定义探头7，FCR：前方探头位置，朝向
extern float FCR_DELTAY; // -0.30188f
extern float FCR_AERFA;  // -0.10472f

extern float FOR_DELTAX; // 3.44664f        //定义探头8，FOR：前方探头位置，朝向
extern float FOR_DELTAY; // -0.76067f
extern float FOR_AERFA;  // -0.806866f

extern float ROL_DELTAX; // -0.84185f       //定义探头9，ROL：后方探头位置，朝向
extern float ROL_DELTAY; // 0.75065f
extern float ROL_AERFA;  // 2.626207f

extern float RCL_DELTAX; // -0.9453f        //定义探头10，RCL：前方探头位置，朝向
extern float RCL_DELTAY; // 0.30076f
extern float RCL_AERFA;  // 3.07283f

extern float RCR_DELTAX; // -0.9453f        //定义探头11，RCR：前方探头位置，朝向
extern float RCR_DELTAY; // -0.30076f
extern float RCR_AERFA;  // -3.07283f

extern float ROR_DELTAX; // -0.84185f       //定义探头12，ROR：前方探头位置，朝向
extern float ROR_DELTAY; // -0.75065f
extern float ROR_AERFA;  // -2.626207f
typedef enum
{
    FOL_IDX = 0, ///< Front Outer Left
    FCL_IDX,     ///< Front Center Left
    FCR_IDX,     ///< Front Center Right
    FOR_IDX,     ///< Front Outer Right

    ROL_IDX, ///< Rear Outer Left
    RCL_IDX, ///< Rear Center Left
    RCR_IDX, ///< Rear Center Right
    ROR_IDX, ///< Rear Outer Right

    FSL_IDX, ///< Front Side Left
    FSR_IDX, ///< Front Side Right
    RSL_IDX, ///< Rear Side Left
    RSR_IDX, ///< Rear Side Right

    FCL_FOL_IDX, ///< Front Center Left ↔ Front Outer Left
    FCL_FCR_IDX, ///< Front Center Left ↔ Front Center Right
    FCR_FOR_IDX, ///< Front Center Right ↔ Front Outer Right
    FCR_FCL_IDX, ///< Front Center Right ↔ Front Center Left

    RCL_ROL_IDX, ///< Rear Center Left ↔ Rear Outer Left
    RCL_RCR_IDX, ///< Rear Center Left ↔ Rear Center Right
    RCR_ROR_IDX, ///< Rear Center Right ↔ Rear Outer Right
    RCR_RCL_IDX, ///< Rear Center Right ↔ Rear Center Left

    FSL2_IDX, ///< Front Side Left Second
    FSR2_IDX, ///< Front Side Right Second
    RSL2_IDX, ///< Rear Side Left Second
    RSR2_IDX, ///< Rear Side Right Second

    USS_ECHO_INDEX_MAX = 24 ///< 总数
} UssEchoIndex;

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/
typedef enum _Relation_T
{
    PK_ON_THE_LINE,
    PK_ON_LEFT,
    PK_ON_RIGHT,
    PK_INSIDE,
    PK_OUTSIDE,
    PK_INTERSECT,
} Relation_T;

typedef struct _Point_T
{
    float x;
    float y;
} Point_T;

typedef struct _LineSeg_T
{
    Point_T pt1;
    Point_T pt2;
} LineSeg_T;

typedef struct _Line_T
{
    float a;
    float b;
    float c;
    Point_T pt1;
    Point_T pt2;
} Line_T;

typedef struct _Vec2_T
{
    float vx;
    float vy;
} Vec2_T;

typedef struct _VehPos_T
{
    float x;
    float y;
    float theta;
} VehPos_T;

typedef struct _Rect_T
{
    Point_T pt_fr;
    Point_T pt_fl;
    Point_T pt_rl;
    Point_T pt_rr;
} Rect_T;

typedef struct _Quad_T
{
    Point_T pt_fr;
    Point_T pt_fl;
    Point_T pt_rr;
    Point_T pt_rl;
} Quad_T;

typedef struct _SlotObj_T
{
    Point_T ptA;
    Point_T ptB;
    Point_T ptC;
    Point_T ptD;
    Point_T ptE;
    Point_T ptF;
} SlotObj_T;
typedef struct _FusionObj_T
{
    // 障碍物线段数组
    // 每个障碍物用线段表示（可能包括起点、终点、长度等信息）
    LineSeg_T obj[SF_OBJ_NUM];

    // 障碍物属性数组
    int attr[SF_OBJ_NUM];

    // 障碍物方向角（rad），表示障碍物朝向
    float ang[SF_OBJ_NUM];

    // 障碍物相对车辆里程计信息（距离或位置）
    float odom[SF_OBJ_NUM];

    // 当前有效障碍物数量
    int num;

    // 累积障碍物数量，用于融合或统计
    int num_accu;
} FusionObj_T;

typedef struct _RectPos_T
{
    float mid_x;
    float mid_y;
    float mid_theta;
    float len_y;
    float len_x;
} RectPos_T;

typedef struct
{
    Point_T near_front;  ///< 近前点
    Point_T near_rear;   ///< 近后点
    Point_T far_front;   ///< 远前点
    Point_T far_rear;    ///< 远后点
    int slot_index;      ///< 用于 Process A：AVM 发送车位时的索引
    uint8 AlignIndex[5]; ///< 用于 Process B 的对齐索引
    float side_dist;     ///< 侧视镜到 AVM 点的距离
} Avm_Pot_T;

typedef struct _Avm_Obj_T
{
    LineSeg_T ls;
    int attr;
    int slot_index;
    uint8 AlignIndex[2]; //  for Process B
    uint8 SlotObsNum;
} Avm_Obj_T;

typedef enum
{
    GREEN_SLOT,
    YELLOW_SLOT,
    RED_SLOT
} SLOT_STATUS;

typedef struct
{
    SlotObj_T slotobj;         ///< 车位对象信息
    VehPos_T targpos;          ///< 目标位置 [x, y, theta]
    int slotobj_attr[5];       ///< 车位属性数组
    PK_SlotShapeType slotshap; ///< 车位形状类型
    int is_vision_slot;        ///< 车位来源：0超声波, 1视觉检测, 2自选车位
    float dist_curveh;         ///< 当前车辆到车位的距离
    int slot_index;            ///< 车位索引
    Avm_Pot_T avm_point;       ///< AVM 检测车位点
    SLOT_STATUS slot_status;   ///< 车位状态
    int8_t has_stopper;        ///< 是否有限位杆：0无,1有
    int8_t occupied; ///< 锁占用状态：0-no lock, 1-lock on, 2-lock off, 3-occupied
} SlotInfo_T;

typedef struct
{
    SlotInfo_T multiArray[MAX_PARKED_SLOTS];
    int multiNum;
} Multi_Slot_Array_T;

typedef struct
{
    // 实际规划轨迹数组
    // MAX_SINGLE_TRAJ_NUM：单条轨迹点数量上限
    // TRAJITEM_LEN：每个轨迹点包含的数据长度（x, y, yaw, curvature 等）
    float Act_traj[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];

    // 车位对象信息结构体，存储规划轨迹对应的车位信息
    SlotObj_T slotObjs;

    // AVM（环视）检测的泊车点信息
    Avm_Pot_T avm_point;

    // 当前规划路径编号
    int Path_num;

    // 选中的车位索引
    int Slot_index;

    // 车位形状类型
    // 0 - 无车位/未知
    // 1 - 左侧平行车位
    // 2 - 右侧平行车位
    // 3 - 左侧垂直车位
    // 4 - 右侧垂直车位
    int slotshape;

    // 路径连接方式
    // 1 - 直接连接（可直接生成路径）
    // 0 - 间接连接（需要中间动作）
    int IsDirConect;

    // 偏离量（out-of-distance metric），用于衡量轨迹或车位偏差
    float oodm;

    // 时间戳，用于记录规划生成时间
    uint32_t stamp;

} PlanInfoType;

typedef struct
{
    int slotNum;
    PlanInfoType slotPlans[MAX_PARKED_SLOTS];
} MultiPlanInfo;

typedef struct
{
    PK_SlotShapeType SlotShap[AVM_BUFF_NUM_MAX];
    Point_T NF[AVM_BUFF_NUM_MAX];
    Point_T NR[AVM_BUFF_NUM_MAX];
    Point_T FF[AVM_BUFF_NUM_MAX];
    Point_T FR[AVM_BUFF_NUM_MAX];
    Point_T BAR[AVM_BUFF_NUM_MAX];
    int AVM_slot_index[AVM_BUFF_NUM_MAX];
    float Side_Mirr_Dist[AVM_BUFF_NUM_MAX]; // side mirror to avm point
                                            // distance,1123,zhangxf add
    float Otom[AVM_BUFF_NUM_MAX];           // s extracted from otometry, 1124, JSF
    int8_t has_stopper[AVM_BUFF_NUM_MAX];   // whether there is stopper in slot
    int8_t has_lock[AVM_BUFF_NUM_MAX];      // whether there is stopper in slot
    int Num;
    int AccNum;
} AVM_Buff_Info;

typedef struct _AvmObstacle_T
{                              // 以车后轴中心为坐标原点，车正前方为x正方向
    int obj_valid[SF_OBJ_NUM]; // 目标障碍物线段有效性（无效-0，有效-1）
    LineSeg_T obj[SF_OBJ_NUM]; // 障碍物线段

    int central_point_valid[SF_OBJ_NUM]; // 目标障碍物中心点有效性（无效-0，有效-1）
    Point_T central_point[SF_OBJ_NUM];   // 目标障碍物中心点坐标

    int closest_point_valid[SF_OBJ_NUM]; // 目标障碍物距车最近点有效性（无效-0，有效-1）
    Point_T closest_point[SF_OBJ_NUM];   // 目标障碍物距车最近点坐标

    int angle_valid[SF_OBJ_NUM]; // 目标障碍物方向有效性（无效-0，有效-1）
    float angle[SF_OBJ_NUM];     // 目标障碍物方向（以x正半轴为起始,单位：弧度）

    float length[SF_OBJ_NUM]; // 障碍物长度（x方向上），无效默认为0
    float width[SF_OBJ_NUM];  // 障碍物宽度（y方向上），无效默认为0
    float height[SF_OBJ_NUM]; // 障碍物高度（z方向上），无效默认为0
    int type[SF_OBJ_NUM];     // 目标障碍物类型（根据视觉信息定义）
    int num;                  // 障碍物数量（不超过 “SF_OBJ_NUM” 定义的数量）
} AvmObstacle_T;

typedef struct
{
    uint8_t index;
    int obstacleType;
    float obstacleTheta;
    float beLen;
    float mid[2];
    float pt1[2];
    float pt2[2];
} AVM_ObjPointsType;

typedef enum
{
    GEAR_RESERVED = 0,
    GEAR_P,
    GEAR_R,
    GEAR_N,
    GEAR_D,
    GEAR_INVALID,
} GEAR_T;
/*********************************************************************************************************
** Definition of basic functions for auto parking
*********************************************************************************************************/
/**************************************************************************
Description     :calculate the distance between point to point, point to line
Owner           :JSF
Modefied Date   :2018.05.09
Parameter[In]   :locations of the two points
Parameter[Out]  :None
Return          :distance of the two points
***************************************************************************/
float Cal_Dis_Pt2Pt(const VehPos_T &pt1, const VehPos_T &pt2);
float Cal_Dis_Pt2Pt(const Point_T &pt1, const Point_T &pt2);
float Cal_Dis_Pt2Pt(const float *cd0, const float *cd1);

float Cal_PowDist_Pos2Pos(VehPos_T pos1, VehPos_T pos2);

/**************************************************
Description     :position project position calculate
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :cd0 -  x0 y0 theta0
cd1 -  x1 y1
Parameter[Out]  :
Return          :rx -  cd1 project on cd0
***********************************************/
float Project_PosTo1stPos_rx(VehPos_T cd0, VehPos_T cd1);

/**************************************************
Description     :position project position calculate
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :cd0 -  x0 y0 theta0
cd1 -  x1 y1
Parameter[Out]  :
Return          :ry -  cd1 project on cd0
***********************************************/
float Project_PosTo1stPos_ry(VehPos_T cd0, VehPos_T cd1);

/**************************************************
Description     :point project position calculate
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :cd0 -  x0 y0 theta0
cd1 -  x1 y1
Parameter[Out]  :
Return          :rx -  cd1 project on cd0
***********************************************/
float Project_PointTo1stPos_rx(VehPos_T cd0, Point_T cd1);

/**************************************************
Description     :point project position calculate
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :cd0 -  x0 y0 theta0
cd1 -  x1 y1
Parameter[Out]  :
Return          :ry -  cd1 project on cd0
***********************************************/
float Project_PointTo1stPos_ry(VehPos_T cd0, Point_T cd1);

/**************************************************
Description     :Convert Line to Position
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :line -LineSeg
Parameter[Out]  :pos Position x,y,theta
Return          :Null
***********************************************/
void Convert_Line_to_Pos(LineSeg_T line, VehPos_T *pos); // line (x0,y0,x1,y1)

/**************************************************
Description     :Car Position with LineSeg Cross Check
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :stpoint - Car Position
obj_num - LineSeg number
obj -LineSeg array
Parameter[Out]  :Null
Return          :return=0,free; =1 cross
***********************************************/
int PK_PosLineSegCross_Check(VehPos_T stpoint, int obj_num, LineSeg_T obj[], float swell);

/**************************************************
Description     :Rectangle with LineSeg Cross Check
Owner           :LJM
Modefied Date   :2018.05.14
Parameter[In]   :Rect_input - Rectangle
obj_num - LineSeg number
obj -LineSeg array
Parameter[Out]  :Null
Return          :return=0,free; =1 cross
***********************************************/
int PK_QuadLineSegCross_Check(Rect_T Rect_input, int obj_num, LineSeg_T obj[]);

/**************************************************************************
Description     :calculate the distance of a line segment
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :line segment
Parameter[Out]  :None
Return          :distance of the line segment
***************************************************************************/
float Get_Segment_Len(LineSeg_T seg);

/**************************************************************************
Description     :calculate the length of a 2D vector
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :a 2D vector
Parameter[Out]  :None
Return          :vector length
***************************************************************************/
float Get_Norm_Vec2(Vec2_T vec);

/**************************************************************************
Description     :normalize a 2D vector
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :a 2D vector
Parameter[Out]  :a normalized 2D vector
Return          :None
***************************************************************************/
void Normalize_Vec2(Vec2_T *vec);

/**************************************************************************
Description     :calculate the dot value of two 2D vectors
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :two 2D vector
Parameter[Out]  :None
Return          :the dot value of two 2D vectors
***************************************************************************/
float Get_Dot_Vec2(Vec2_T vec1, Vec2_T vec2);

float Get_Cross_Vec2(Vec2_T vec1, Vec2_T vec2);

/**************************************************************************
Description     :calculate the cross angle of two 2D vectors
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :two 2D vector
Parameter[Out]  :None
Return          :the cross angle of two 2D vectors
***************************************************************************/
float Get_CrossAngle_Vec2(Vec2_T vec1, Vec2_T vec2);

/**************************************************************************
Description     :judge the relationship between a point and a quadrilateral.
NOTICE that the quadrilateral has to be a convex polygon.
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :a point and a quadrilateral
Parameter[Out]  :None
Return          :PK_ON_THE_LINE/PK_INSIDE/PK_OUTSIDE
***************************************************************************/
Relation_T Is_Point_In_Quad(Quad_T quad, Point_T pt);

/**************************************************************************
Description     :calculate the cross point of two lines
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :four points that make up two lines
Parameter[Out]  :the cross point
Return          :1-get the right result; 0-input error two lines are parallel
***************************************************************************/
int Get_CrossPt_PointLine(Point_T Line1_A, Point_T Line1_B, Point_T Line2_A,
                          Point_T Line2_B, Point_T *CrossPt);

/**************************************************************************
Description     :calculate the average value of two angles with weights
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :two angle values,two weights values
Parameter[Out]  :None
Return          :the average angle value
***************************************************************************/
float Get_AverAngle_W(float Theta1, float Theta2, float w1, float w2);

/**************************************************************************
Description     :get the minimum value of a 1D array
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :a 1D array pointer,the element number
Parameter[Out]  :None
Return          :the minimum value
***************************************************************************/
float Get_MinValue(float *dataArr, int dataNum);

/**************************************************************************
Description     :calculate the perpendicular distance between a point and a line,
calculate the direction the point stay relative to the line
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :line segment's two endpoints and a point
Parameter[Out]  :the perpendicular distance,point stay direction
if only need one output,you can set another output param to NULL
Return          :None
***************************************************************************/
void Get_Dist_Dir_Pt2PointLine(Point_T LinePt_A, Point_T LinePt_B, Point_T Pt_C,
                               float *dist, Relation_T *dir);
/* line param is a segment */
void Get_Dist_Dir_Pt2SegLine(LineSeg_T segline, Point_T Pt_C, float *dist,
                             Relation_T *dir);
/* line param is a point and a vector */
void Get_Dist_Dir_Pt2VecLine(Point_T linePt1, Vec2_T lineVec, Point_T Pt_C, float *dist,
                             Relation_T *dir);

/**************************************************************************
Description     :get the foot point of a point to a line
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :line common params, a point
Parameter[Out]  :foot point
Return          :None
***************************************************************************/
void GetFootPt_LineParam(float a, float b, float c, Point_T Pt, Point_T *FootPt);
/*  Function overloading: the line is made up of two points */
void GetFootPt_PointLine(Point_T line_pt1, Point_T line_pt2, Point_T Pt, Point_T *FootPt);
/*  Function overloading: the line is made up of a point and a vector */
void GetFootPt_PointAndVecLine(Point_T line_pt1, Vec2_T line_vec, Point_T Pt,
                               Point_T *FootPt);

/**************************************************************************
Description     :get the vehicle's four corner points
Owner           :CSJ
Modefied Date   :2018.05.14
Parameter[In]   :vehicle's current posture, swell on one side
Parameter[Out]  :vehicle's four corner points
Return          :None
***************************************************************************/
void Get_Veh_CornerPt(VehPos_T CurPos, Rect_T *CornerPt, float swell);

/**************************************************************************
Description     :check if two line segment are cross
Owner           :CSJ
Modefied Date   :2018.05.16
Parameter[In]   :four points p1p2,p3p4
Parameter[Out]  :None
Return          :1 -cross; 0 -not;
***************************************************************************/
int Is_TwoSegment_Cross(Point_T pt1, Point_T pt2, Point_T pt3, Point_T p4);

int Is_Point_OnLine(Point_T pt1, Point_T pt2, Point_T pt3);

/**************************************************************************
Description     :get the vehicle's four surface line segment
Owner           :CSJ
Modefied Date   :2018.05.22
Parameter[In]   :vehicle's current posture, swell on one side
Parameter[Out]  :vehicle's four surface line segment
Return          :None
***************************************************************************/
void Get_Veh_LineSeg(VehPos_T CurPos, LineSeg_T vehseg[4], float swell);

/**************************************************************************
Description     :check if a segment cross a rectangle and get the segment part in the
rectangle Owner           :CSJ Modefied Date   :2018.06.01 Parameter[In]   :a rectangle, a
segment Parameter[Out]  :the insert part of the input segment Return          :0- the
segment is out of rectangle; 1 - the segment cross the rectangle or in the rectangle;
***************************************************************************/
int RectClipSegment(Rect_T Rect, LineSeg_T Segm, LineSeg_T *Seg_Cross);

/**************************************************************************
Description     :check if a segment cross a rectangle and get the segment part out of the
rectangle Owner           :CSJ Modefied Date   :2018.06.20 Parameter[In]   :a rectangle, a
segment Parameter[Out]  :the outside part of the input segment Return          :0- the
segment is out of rectangle or totally in it; 1 - the segment cross the rectangle;
***************************************************************************/
int Rect_Clip_Segment_OutSeg(Rect_T Rect, LineSeg_T Segm, int *OutSegNum,
                             LineSeg_T OutSegm[2]);

/**************************************************************************
Description     : Convert AVM Output to RectPos for CDU Input
Owner           :LJM
Modefied Date   :2018.06.01
Parameter[In]   : a linesegment,slotshape
Parameter[Out]  :RectPos TO CDU
Return          :1 OK,0 ERROR
***************************************************************************/
// NearFrontPt_x,NearFrontPt_y,NearRearPt_x,NearRearPt_y
int NearPtToRectPos(Point_T NearFrontPt, Point_T NearRearPt, int slotshape,
                    Point_T *CenterPt);

int RectPosToRect(RectPos_T RectPos, Rect_T *Rect);

int RectToRectPos(Rect_T Rect, RectPos_T *RectPos);

int RectPosToRect(RectPos_T RectPos, Rect_T *Rect);

/**********************************************************************************
Description     :Check Whether Two X-axis Projected Segments are overlaped
Owner           :HYB
Modefied Date   :2018.07.24
Parameter[In]   :Two Line Segments
Return          :The Length of Two Segments' overlap
***********************************************************************************/
float LenTwoSegmentsOverlap(LineSeg_T Line1, LineSeg_T Line2);

/**********************************************************************************
Description     :Project length from two points to some direction
Owner           :DZQ
Modefied Date   :2018.10.07
Parameter[In]   :
Return          :
***********************************************************************************/
inline float calcProjLen(const Point_T *pa, const Point_T *pb, const Point_T *uv)
{
    float lx = pb->x - pa->x;
    float ly = pb->y - pa->y;
    float dp = lx * uv->x + ly * uv->y;
    return fabsf(dp);
}

/* Calculate the mean value of a float array */
inline float calcArrMean_f(const float arr[], const int num)
{
    float ret = 0;
    int i;
    for (i = 0; i < num; i++)
    {
        ret += arr[i];
    }
    return ret / num;
}

inline float calcArrMeanInv_f(const float arr[], const int num)
{
    float ret = 0;
    int i;
    for (i = 0; i < num; i++)
    {
        ret += arr[-i];
    }
    return ret / num;
}

void SlotObj_Convert_float(const SlotObj_T &IN_Slot, float OUT_Slot[5][4]);

void SlotObj_Convert_struct(SlotObj_T *OUT_Slot, const float IN_Slot[5][4]);

int Slot_Dir(int shapeType);

float Calc_Dist_Line2Line(float a1[2], float a2[2], float b1[2], float b2[2]);
// #ifdef __cplusplus
// }
// #endif

#endif /* ifndef PK_Utility */
/*********************************************************************************************************
**         End Of File: PK_Utility.h
*********************************************************************************************************/
