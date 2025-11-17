#ifndef _PK_PATHEXECUTEP_H_
#define _PK_PATHEXECUTEP_H_
#include "MathFunc.h"
#include "Rte.h"

extern int pre_nout_ccp;

typedef struct
{
    int cur_tra_nth;   // 当前轨迹编号，表示车辆当前所在轨迹的编号
    int tra_num;       // 轨迹数量，表示可供选择的轨迹总数
    int cur_tra_state; //  0- static curvature path; 1- dynamic curvature path;
    int slotshape;     // 0/no_slot left para=1, right para=2, left vert= 3 right para = 4
    int step_gone;     // 行驶步数，表示车辆已经行驶的步数

    int B_BrakeState;      // 1 start brake
    int B_DynaPlanCounter; // the counter of dynamic planning
    int B_FirstPlanFlag;   // the first plan flag
    int B_Danger_filter;   // from obj aviod
    int B_replan;          // 危险重新规划标志
    int B_Stop_Counter;    // 停止计数器
    int B_ReplanResult;    // 重新规划结果，用于记录重新规划的结果
    int AfterTra_nth;      // 重规划次数
    int brakeSt;           // 刹车状态
    int stillCount;        // 静止计数
    float percent;         // 百分比
    float curpos[4];       // Current position;
    float finpos[3];
    float cur_eps;     // Current EPS angle
    float cur_vel;     // Current Veliche
    float ds_gone_dir; // ds gone after turn dir auto drive; the moving dist from the
                       // current path's start point
    float ds_start_dir; // start ds of turn dir auto drive; when init, jump to next path
                        // or after replan this value will be update to record the
                        // vehicle's start path
    float ceps;        // 转弯角度
    float cvel;        // 当前速度，计算
    float cvel_last;   // 上一次速度
    float rx_RePathSt; // 重新规划起点x
    float ry_RePathSt; // 重新规划起点y
    float de;          // position err cal位置误差
    float de_last;     // 上一次position err cal
    float sum_de;      // position err cal之和
    float radius;      // 半径

    float rx_cur_to_fin;     // 当前位置到目标位置的x距离
    float ry_cur_to_fin;     // 当前位置到目标位置的y距离
    float rtheta_cur_to_fin; // 当前位置到目标位置的角度距离
    float deps;              // deps
    float Rou_req;           // Rou需求
    float EPS_req;           // EPS需求
    float drx_brake;         // the predict sliding distance to stop the vehicle
    float drx_crou; // the dist in advance near a curve rate change point to change eps
                    // angle to next path for a continuous path sate
    float rx_brake; // the distance to a path 's endpos that need to stop the vehicle
    float rxf; //=(x-xf)*cosf(thetaf)+(y-yf)*sinf(thetaf);  //curpos relative to targpos
    float rxs; // 相对于目标位置的y方向距离
    float leftDist;                                      // 剩余路径距离
    float rthetaf;                                       //=Round_PI(thetaf-theta);　
    float rthetas;                                       //=Round_PI(thetas-theta);
    float min_arround_dist;                              // 最小环绕距离
    float min_arround_dist_dt;                           // 最小环绕距离dt
    float de_dt;                                         // de的dt
    uint8_t bUpdate;                                     // 更新标志
    uint8_t dangerCounter;                               // 危险计数器
    float objs[12];                                      // 对象
    float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]; // 更新路径
    uint8_t updatePathNum;                               // 更新路径数量
    float pathDist;
    uint8_t gear;
    float lastpercent;
    int replanSt;
    uint8_t hasStop;
    int Slot_index;
    uint8_t targState;
    uint8_t postState;
    float cvel_direct_fac;
} PK_Cur_AutodrPara;

void PathExec_PathInverse(int tra_num, float traject[][TRAJITEM_LEN]);

void PK_ReverseTraject(float traject[TRAJITEM_LEN]);

void PathExec_ParkingLever(PK_Cur_AutodrPara curSt[1]);

int PathExec_ParkingLeverUpdate(PK_Cur_AutodrPara curSt[1], float enviObj[5][4]);

void PathExec_SpeedBump(PK_Cur_AutodrPara curSt[1]);

int PathExec_B_MoveOutFront(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                            const float finpoint[3], const float enviObj[5][4],
                            const float curpos[4], const int slotshap);

int PathExec_B_MoveOut_ParaRePlan(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                                  const float finpoint[3], const float enviObj[5][4],
                                  const float stpoint[4], const int slotShap);

int PathExec_SpaceExplorer_RePlanInSlot(
    float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], const float finpoint[3],
    const float enviObj[5][4], const float stpoint[4]);

void PathExec_EndAvpAutodrPara(PK_Cur_AutodrPara curSt[1],
                               float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]);

void PathExec_UpdateAvpAutodrPara(PK_Cur_AutodrPara curSt[1],
                                  float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN]);

int PathExec_B_MoveOutRear(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                           const float finpoint[3], const float enviObj[5][4],
                           const float stpoint[4]);

int PathExec_B_MoveSideFront(float plannedPath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN],
                             float finpoint[3], float enviObj[5][4],
                             const float curpos[4], const int slotshap);

int PathExec_BetterTra(const float stpoint[3], const float finpoint[3], const int objNum,
                       const float obj_slot[][4], float traject[][TRAJITEM_LEN],
                       const float swell, const float radiusSwell = 0.6);

int PathExec_BetterTraFront(const float stpoint[3], const float finpoint[3],
                            const int objNum, const float objSlot[][4],
                            float traject[][TRAJITEM_LEN]);

void PathExec_Lock();

void PathExec_UnLock();

#endif
