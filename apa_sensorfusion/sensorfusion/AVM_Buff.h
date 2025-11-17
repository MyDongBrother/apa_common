/**
 * @file AVM_Buff.h
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
#ifndef AVM_SLOT_BUFF_
#define AVM_SLOT_BUFF_

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "Rte_BSW.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>
#include <netinet/in.h>
/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/** Component Version Information */
#define PK_SF_AVM_BUFF_SW_MAJOR_VERSION (3u)
#define PK_SF_AVM_BUFF_SW_MINOR_VERSION (0u)
#define PK_SF_AVM_BUFF_SW_PATCH_VERSION (43u)

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/
#define STAR_VALID_DISTH \
    0.5f // definition of other stalls change to D distance after the distance
         // exceeds 4.5m to determine valid slots.
#define NF_NR_DIS \
    1.0f // definition of  the distance threshold of between nearfront point and nearrear
         // pot
#define OBLIQUE_ANGLE_MIN 1.308997f // definition the angle range of oblique parking slots
#define OBLIQUE_ANGLE_MAX 1.832596f // definition the angle range of oblique parking slots
#define VEHCOOR_DIS_NFNR \
    2.9f // definition vehicle coordinate system distance to the line formed by NF and NR
         // points
#define DIS_TO_LAST_SLOT \
    1.5f // definition the distance between the current parking space and the last parking
         // space
#define SLOT_OUTRANGE_DIS 0.5f  // definition slot out of range every 0.5m
#define NF_NR_DIS_MAX     10.0f // definition maximum length of slot line
#define DIS_CUR_NFNR_MAX  4.0f  // definition maximum distance between NF or NR and CurPos
// AVM Slot Attribute
typedef enum
{
    DIAGONAL_SLOT = 0,
    AVM_EMPTY     = 1,
    VERTICAL_SLOT = 2,
    AVM_NO_EMPTY  = 6,
    AVM_MARK      = 7,
} AVM_ATTRIBUTE;

typedef enum
{
    OBSTACLE_OUT_SLOT = 0,
    OBSTACLE_IN_SLOT  = 1,
} OBSTACLE_RELATION_SLOT;

typedef enum
{
    INVALID_GEAR = 0,
    D_GEAR       = 1,
    N_GEAR       = 2,
    R_GEAR       = 3,
    P_GEAR       = 4,
} GEAR;

typedef struct
{
    PK_SlotShapeType SlotShap;
    Point_T NF;
    Point_T NR;
    Point_T FF;
    Point_T FR;
    int AVM_slot_index;
    float Side_Mirr_Dist;
    float Otom;
} AVM_Single_Buff_Info;
/*********************************************************************************************************
** Declearation of exported functions
*********************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
    /**********************************************************************************
    Description		:Manage The Valid And Invalid Slot_Buff
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:void
    Return			:void
    ***********************************************************************************/
    void AVM_Slot_Buffer_Manager(void);
    void AVM_Slot_Buffer_Manager_Visualize(int visualfd, struct sockaddr_in servaddr,
                                           AVM_SlotPointsType *AVM_Slot);

    /**********************************************************************************
    Description		:Clean Up All Parking Slots And Some Use Variables
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:void
    Return			:void
    ***********************************************************************************/
    void Clean_All_AVM_SlotBuff(void);

    /**********************************************************************************
    Description		:Clean Up The Front Parking Slots And Some Use Variables
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:void
    Return			:void
    ***********************************************************************************/
    void AVM_Slot_OffClear(void);

    /**********************************************************************************
    Description		:Point Change To Global Coordinate System
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:Point and CurPos
    Return			:void
    ***********************************************************************************/
    void Transfer_AVM_Locat_To_Odom(Point_T *AVM_Point, float CurPos[4]);

    /**********************************************************************************
    Description		:Point Add Global Coordinate System Coordinates
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:Point and CurPos
    Return			:void
    ***********************************************************************************/
    void AVM_Add_Odom(Point_T *NearPt, float curpos[4]);

    /**********************************************************************************
    Description		:Check AVM_SLot Shape
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:NF and NR and CurPos and SlotShape
    Return			:PK_SlotShapeType
    ***********************************************************************************/
    PK_SlotShapeType Check_AVM_SlotShap(PK_SlotShapeType SlotShap, VehPos_T curPos,
                                        Point_T NearFrontPt, Point_T NearRearPt);

    /**********************************************************************************
    Description		:Delete Specify Parking Slot
    Owner			:ZB
    Modefied Date	:2018.12.11
    Parameter[In]	:NF and NR and CurPos and SlotShape
    Return			:1
    ***********************************************************************************/
    int Delete_Specify_SlotBuff(int index, AVM_Buff_Info *buff);

    /**********************************************************************************
    Description		:Check Whether Endpoint of Line Segment In Rectangle
    Owner			:ZB
    Modefied Date	:2018.09.24
    Parameter[In]	:Four Points of a Rectangle and Line Segment Information
    Return			:if 1 in rectangle,0 out.
    ***********************************************************************************/
    int PK_LineSegIn_Check(Point_T NR, Point_T NF, Point_T FR, Point_T FF, int obj_num,
                           LineSeg_T obj[]);

    /**********************************************************************************
    Description		:Through NF and NR Point Output Decision Rectangle Box
    Owner			:ZB
    Modefied Date	:2018.11.05
    Parameter[In]	:Four Points of a Rectangle and Line Segment Information
    Return			:void
    ***********************************************************************************/
    void Vector_Output_Rectangle(Point_T NF, Point_T NR, PK_SlotShapeType Result,
                                 float CurPos[4], Point_T *NFSimu, Point_T *NRSimu,
                                 Point_T *FFSimu, Point_T *FRSimu, float *Cur_Dis);

    /*******************************************************************************************
    Description		:Get the max distance from curpos to NF and NR no more than 4.0m
    Owner			:ZB
    Modefied Date	:2018.11.26
    Parameter[In]	:Points NF and NR and CurPos and SlotShape
    Return			:void
    *********************************************************************************************/
    void Distance_Cur_MaxNFNR(Point_T NF, Point_T NR, float CurPos[4],
                              PK_SlotShapeType Result, float *Dis_Mirr_Line);

    /*******************************************************************************************
    Description		:Get the slotshape through judging the angle of NearRear
    Owner			:ZB
    Modefied Date	:2018.11.24
    Parameter[In]	:Points NF and NR and CurPos and SlotShape
    Return			:void
    *********************************************************************************************/
    void Output_SlotShape(PK_SlotShapeType InShape, Point_T NF, Point_T NR, Point_T FF,
                          Point_T FR, float CurPos[4], float *Ave_theta,
                          PK_SlotShapeType *OutShape);

    /**********************************************************************************
    Description		:Add New Slot to Invalid Buff
    Owner			:ZB
    Modefied Date	:2018.11.30
    Parameter[In]	:Slot_Buff
    Return			:void
    ***********************************************************************************/
    int AVM_Invalid_Buff_Add_New_Slot(AVM_Buff_Info *Slot_Buff, Point_T NF, Point_T NR,
                                      Point_T FF, Point_T FR, PK_SlotShapeType Result,
                                      float side_dis_mirr, float Odom);

    /**********************************************************************************
    Description		:Add Single Slot to Buff
    Owner			:ZB
    Modefied Date	:2018.12.01
    Parameter[In]	:Slot_Buff
    Return			:void
    ***********************************************************************************/
    void AVM_Buff_Add_Single_Slot(AVM_Buff_Info *Slot_Buff, int flag, Point_T FF,
                                  Point_T FR, Point_T NF, Point_T NR, float Odom,
                                  float side_dis_mirr, PK_SlotShapeType slotshap);

    /**********************************************************************************
    Description		:Check AVM Slot Out Of FreeSpace Range
    Owner			:ZB
    Modefied Date	:2018.12.05
    Parameter[In]	:Odometry AVM_Valid_Buff and AVM_Invalid_Buff
    Return			:void
    ***********************************************************************************/
    void Check_AVMBuff_Out_Of_FreeSpace_Odometry_Range(float Space_Odom,
                                                       AVM_Buff_Info *AVM_Valid_Buff,
                                                       AVM_Buff_Info *AVM_Invalid_Buff);

#ifdef __cplusplus
};
#endif

/*********************************************************************************************************
** Declearation of exported variables
*********************************************************************************************************/

#endif /* ifndef AVM_SlotBuff */
/*********************************************************************************************************
**         End Of File: AVM_SlotBuff.h
*********************************************************************************************************/