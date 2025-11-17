/**
 * @file PK_SD_Common.h
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
#ifndef PK_SD_COMMON_
#define PK_SD_COMMON_

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "PK_Utility.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/

typedef enum
{
    TRIM_LEFT  = 0,
    TRIM_RIGHT = 1,
} TRIM_OBJ_DIR;

typedef enum
{
    FRONT_OFFSET = 0,
    REAR_OFFSET  = 1,
} ADJUST_DIR;

typedef enum
{
    CAL_NO,
    CAL_LEFT,
    CAL_RIGHT,
    CAL_BOTH
} RECAL_DIR;

#define AVM_MinProjRx_SlotCal                       -25.0f
#define AVM_MinSegLen_BCTrans                       0.15f
#define AVM_MinSegLen_DETrans                       0.15f
#define AVM_CDOffsetLimit                           5.0f
#define AVM_ABOffsetLimit                           -5.0f
#define AVM_EFOffsetLimit                           -5.0f
#define AVM_OccupyThresh_RearFusionObjCheck         0.2f
#define AVM_MinBCFreeDist_RearFusionObjCheck        0.3f
#define AVM_MinDEFreeDist_RearFusionObjCheck        0.3f
#define AVM_MinSlotToVehBodyDist_RearFusionObjCheck 0.4f
#define AVM_ParaSlotDepthOffset                     0.2f
#define AVM_VertSlotDepthOffset                     0.4f
#define AVM_MinOverlapLen                           0.3f
#define AVM_MinFarToNearDis                         2.4f
#define AVM_MaxFarToNearDis                         2.8f

#define URadar_TwoPtDist1                  0.15f
#define URadar_TwoPtDist2                  0.25f
#define URadar_TwoPtDist3                  0.5f
#define URadar_MaxPtToLineDist             0.08f
#define URadar_LengthWeight                0.81f
#define URadar_MaxLineAngle                0.7f
#define URadar_MinSegLen_CheckVehLinePro   0.5f
#define URadar_CurSegLenThresh             0.8f
#define URadar_LastLineToCurLineDistThresh 0.6f
#define URadar_MaxMidObjLen                0.4f
#define URadar_MidObjToCurbThresh          1.2f
#define URadar_MinCurbLenSum               2.0f
#define URadar_MaxAllowVehLineAngleDiff    30.0f
#define URadar_MaxAllowVehLinesInnerDist   1.0f
#define URadar_MaxVehLinesBias             1.6f
#define URadar_MinMidObjLen_SmallObjCheck  0.3f
#define URadar_ABEFMinOffset               -10.0f
#define URadar_MinProjRx_SlotCal           -25.0f
#define URadar_MaxProjRx_SlotCal           -2.5f
#define URadar_CurbCalAngleThresh          0.785f

/*********************************************************************************************************
** Declearation of exported functions
*********************************************************************************************************/
// the following added by xls@2022.5.14

void adjust_veh_1_mainline(LineSeg_T *Veh_1_MainLine);
void adjust_veh_2_mainline(LineSeg_T *Veh_2_MainLine);
float get_adjust_value_FSL(float sensor_value, int direction);
float get_adjust_value_RSL(float sensor_value, int direction);
FusionObj_T get_rid_of_excess_obj(SlotInfo_T *U_Slot_Array, int U_Slot_Num,
                                  FusionObj_T *Side_FS_Obj, TRIM_OBJ_DIR SlotDirection);
int cal_lines_intersection(double line1_pt1_x, double line1_pt1_y, double line1_pt2_x,
                           double line1_pt2_y, double line2_pt1_x, double line2_pt1_y,
                           double line2_pt2_x, double line2_pt2_y, double *x, double *y);
int check_slot_exist(SlotObj_T *slotobj, SlotInfo_T *U_Slot_Array, int U_Slot_Num,
                     PK_SlotShapeType slotshap);

int check_vehicle_space(LineSeg_T current_Vehicle_MainLine, int SlotDir,
                        PK_SlotShapeType *current_slotshape, SlotObj_T *current_slotobj,
                        VehPos_T *current_targpos); // xls@2022.6.08
int check_vehicle_other_space(LineSeg_T current_Vehicle_MainLine, int SlotDir,
                              PK_SlotShapeType *current_slotshape,
                              SlotObj_T *current_slotobj,
                              VehPos_T *current_targpos); // xls@2022.6.08

int IsTargPosVehInSertSlot(VehPos_T TargPos, SlotObj_T SlotObj);
int IsSegmentInsertTargPos(const LineSeg_T &segment, const VehPos_T &targpos,
                           const PK_SlotShapeType SlotShap);
Relation_T IsTargPosInSlot(Point_T Slot_B, Point_T Slot_C, Point_T Slot_D, Point_T Slot_E,
                           VehPos_T TargPos);
int Try_to_Move_Vert_AVM_Targpos_to_process_A(VehPos_T *targpos, SlotObj_T slotobj,
                                              PK_SlotShapeType slotshap);
int Rear_Side_Radar_Check_Slot(VehPos_T targPos, PK_SlotShapeType slotShap);
Point_T Get_Middle_Point(Point_T spt, Point_T ept);
float Get_Segment_Angle(Point_T pt1, Point_T pt2);
float Get_Average_angle_from_4AVMpts(Point_T NF, Point_T FF, Point_T NR, Point_T FR);
int LineFittingLeastSquare(Point_T *ptSet, int ptNum, LineSeg_T *line);

/*********************************************************************************************************
** Declearation of exported variables
*********************************************************************************************************/
extern FusionObj_T g_FS_ObsInfo_A;
extern int Is_BC_Update_Bst;
extern int Is_DE_Update_Bst;
extern FusionObj_T g_FS_RearObsInfo_A;
extern FusionObj_T g_FS_ObsInfo_B;
// extern Point_T g_CenPtCB_A,g_CenPtDE_A;
// extern float g_ThetaCB_A,g_ThetaDE_A;//	slot info from process A

#endif /* ifndef PK_SD_Common */
/*********************************************************************************************************
**         End Of File: PK_SD_Common.h
*********************************************************************************************************/