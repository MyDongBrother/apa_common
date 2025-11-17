#ifndef PATHPLAN_ACTMODULE_H
#define PATHPLAN_ACTMODULE_H

#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "SystemPara.h"
#include "PK_Config.h"

typedef struct
{
    int obj_danger_num;                     ///< 融合障碍物数量（Sensor Fusion 线段数）
    float obj_danger[SF_OBJ_NUM * 2][4];    ///< 融合障碍物线段数组，每条线段 4 个 float
    uint8_t obj_danger_dir[SF_OBJ_NUM * 2]; ///< 障碍物方向：0-invalid, 1-left, 2-right
    float stpoint[4];                       ///< 规划起点 [x, y, theta, s]
    float finpoint[3];                      ///< 规划终点 [x, y, theta]
    float obj_slot[5][4];                   ///< 车位障碍物线段数组
    float pointf_danger[3];                 ///< 前向危险点 [x, y, theta]
    float pointb_danger[3];                 ///< 后向危险点 [x, y, theta]
    float close_side_dist;                  ///< 最近侧向距离（自由距离）
    float len;                              ///< 车辆长度
    float width;                            ///< 车辆宽度
    float h;                                ///< 车辆后悬长度
    float swell;                            ///< 车辆膨胀量（安全边距）
    float left_fac;                         ///< 左右因子，左=1, 右=-1
    int slotshape;           ///< 车位类型：0/未知,1左平,2右平,3左垂直,4右垂直
    float front_dist;        ///< 前向安全距离
    float rear_dist;         ///< 后向安全距离
    float rect_ds_max;       ///< 最大矩形距离（轨迹判断用）
    float rear_safe_dist;    ///< 后向安全距离（停车或倒车）
    int is_vision_slot;      ///< 车位类型来源：0超声波,1视觉检测,2自选车位
    int has_stopper;         ///< 是否有阻挡器：0无,1有
    float obj_stopper[2][2]; ///< 阻挡器坐标
} PlanDataCase;

typedef struct
{
    float LeftNearObj[3];
    float RightNearObj[3];

    float LeftLimit;
    float RightLimit;
    float FrontLimit;
    float RearLimit;
} PlanFrontArea;

typedef struct _Avm_StopBar_T
{
    LineSeg_T stopbar;         // 限位杆坐标
    int slot_index;            // 限位杆所属车位编号
    PK_SlotShapeType slotshap; // 限位杆所属车位类型
    Point_T near_rear;         // 限位杆所属车位B点坐标
} Avm_StopBar_T;

typedef struct
{
    int stopbar_num[2];
    Avm_StopBar_T stopbar_array[2][7]; // 0 -left 1 - right
} Multi_StopBar_Array_T;

// void PK_Set_Avm_StopBar(Avm_StopBar_T* pa);
// void RTE_PK_SensorFusion_Set_Avm_MultiStopBar(Multi_StopBar_Array_T* pa);
// void RTE_PK_SensorFusion_Get_Avm_MultiStopBar(Multi_StopBar_Array_T* pa);

#define PATHPLAN_SUCCESS 0
enum RARK_REPLAN
{
    PARK_UPDATE,
    PARK_NOUPDATE,
    PARK_SWITCH,          // 有新路径，且无碰撞
    PARK_DANGER_SWITCH,   // 有新路径，老路径有碰撞
    PARK_DANGER_NOSWITCH, // 无新路径，老路径有碰撞
    PARK_NOSWITCH,        // 无新路径，老路径无碰撞
};

void PK_PathPlan(void); // Main Entrance Function

int PK_PathVerify(float traject[TRAJITEM_LEN], const int obj_num, const float obj[][4],
                  const float swell);

int PK_PathPlan_SpaceExplorer_Para(PlanDataCase PlanDataVer[1],
                                   float Exp_traj[][TRAJITEM_LEN]);

float PK_PathPlan_SlotObj_To_stPos_By_Swell(PlanDataCase PlanData_Ver[1],
                                            const float avmDist[3]);

int PK_PathPlan_TraExplorer_Con(float temp_tra[TRAJITEM_LEN], const float tar_pos[3],
                                const int obj_num, const float obj_slot[][4],
                                float stp_trajs[][TRAJITEM_LEN], int A_tra_to_posint,
                                int B_tra_to_pos, float swell, float start_ds);

int PK_PathPlan_SearchMinDsTra(const float left_fac, const float stpoint[3],
                               const float finpoint[3], const int obj_num,
                               const float obj_slot[][4], float &swell,
                               float Plan_Path[MAX_PARKED_SLOTS][TRAJITEM_LEN]);

void PK_PathPlan_Envi_obj_cut(const float Envi_obj[5][4], const float Cut_factor,
                              float Envi_obj_side[2][4]);

int PK_PathPlan_B_DirectCon_Replan(float stpoint[3], float finpoint[3], int obj_num,
                                   float obj_slot[][4], float traject[][TRAJITEM_LEN],
                                   float swell);

int PK_PathMerge(float trajs[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const int lineNum,
                 const int slotid);

int PK_PathPlan_BetterTra(const float stpoint[3], const float finpoint[3],
                          const int obj_num, const float obj_slot[][4],
                          float traject[][TRAJITEM_LEN], const float swell,
                          const float radiusSwell = 0.6);

void PK_InitJsonReport();

int PK_SendJsonData(const char *buff, const int len);

void PK_ProcJsonCmd(const char *recv, const int len);

void PK_BuildAvmSlotInfo(const Point_T avmPoints[4], const PK_SlotShapeType slotShap,
                         SlotInfo_T &manuSlot);

int PK_PathPlan_PathVerify(const float path[TRAJITEM_LEN], const int obj_num,
                           const float obj[][4], const float swell);

int PK_PathPlan_RangeDataProcess(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                 const float finpoint[3], const float enviObj[5][4],
                                 const float curpos[4], const int slotshap,
                                 const int is_vison_slot);

void PK_PathPlanRun(void);
#endif
