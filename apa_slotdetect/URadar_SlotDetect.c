/**
 * @file URadar_SlotDetect.c
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
/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "Module_API.h"
#include "MathFunc.h"
#include "hobotlog/hobotlog.hpp"
/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "URadar_SlotDetect.h"
#include "PK_Utility.h"
#include "PK_SD_Common.h"
#include "PK_StateManage.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>

#define SLOT_DETECT_OPEN_SIDE \
    2 //	the slot detect switch： -1 -close both sides; 0 -open only left；1 -open only
      // right；2 -open both sides；
#define MAX_VEH_LINE_NUM  10              //	the max segment num for vehicle's body
#define CURB_LINE_NUM_MAX 10              // the max num for curb segments
LineSeg_T g_CurbLines[CURB_LINE_NUM_MAX]; //	the curb line segments
int g_CurbLinesNum;                       // the real num of detected curbs
static int L_Last_FS_Num_A = 0;
static int R_Last_FS_Num_A = 0;
static FusionObj_T g_Side_FS_Obj; //	segments from sensor fusion A on the same side
static float g_last_path = 0;
static float Slotlength_cal_ccp;
static int USlotDetect_RunStep_ccp;
static float Slotlength_before_ccp;

/*********************************************************************************************************
** Declaration for exported variables
*********************************************************************************************************/
SlotInfo_T g_Left_U_Slot_Array[MAX_U_SLOT_SIDE_NUM];
int g_Left_U_Slot_Num = 0;
SlotInfo_T g_Right_U_Slot_Array[MAX_U_SLOT_SIDE_NUM];
int g_Right_U_Slot_Num = 0;
SlotInfo_T g_Left_U_SingleVeh_Slot_Array[MAX_U_SLOT_SIDE_NUM];
int g_Left_U_SingleVeh_Slot_Num = 0;
SlotInfo_T g_Right_U_SingleVeh_Slot_Array[MAX_U_SLOT_SIDE_NUM];
int g_Right_U_SingleVeh_Slot_Num = 0;

int FilterCurbNoiseBasedOnPosition(LineSeg_T firVehLine, LineSeg_T secVehLine,
                                   LineSeg_T *midObj, int *midObjNum,
                                   LineSeg_T *curbLines, int *curbLineNum);

/*	clean all buff for the multi slots	*/
void Init_URadar_SlotDetect(void)
{
    memset(g_Left_U_Slot_Array, 0, sizeof(g_Left_U_Slot_Array));
    g_Left_U_Slot_Num = 0;
    memset(g_Right_U_Slot_Array, 0, sizeof(g_Right_U_Slot_Array));
    g_Right_U_Slot_Num = 0;

    // Due to add SlotNum to struct SlotInfo, adjust following  code. add by xls@2022.5.10
    // RTE_PK_SlotDetect_Set_Multi_SlotNum(0)
    //  Multi_Slot_Array_T initial_Multi_SlotInfo = {0};
    //  RTE_PK_SlotDetect_Set_Multi_SlotInfo(&initial_Multi_SlotInfo);

    // Multi_slot_num_ccp = 0;
    L_Last_FS_Num_A = 0;
    R_Last_FS_Num_A = 0;
    memset(&g_Side_FS_Obj, 0, sizeof(g_Side_FS_Obj));
    g_last_path = 0;
}

/*	Loop the array to add in the new data	*/
void VehLinesArrayShiftByOne(LineSeg_T VehLines[MAX_VEH_LINE_NUM], LineSeg_T *newLine)
{
    int i;
    for (i = 0; i < MAX_VEH_LINE_NUM - 1; i++)
    {
        memcpy(&VehLines[i], &VehLines[i + 1], sizeof(LineSeg_T));
    }
    memcpy(&VehLines[MAX_VEH_LINE_NUM - 1], newLine, sizeof(LineSeg_T));
}
void VehDirArrayShiftByOne(int VehDir[MAX_VEH_LINE_NUM], int newDir)
{
    int i;
    for (i = 0; i < MAX_VEH_LINE_NUM - 1; i++)
    {
        VehDir[i] = VehDir[i + 1];
    }
    VehDir[i - 1] = newDir;
}

int VehLinesArrayShift(LineSeg_T VehLines[MAX_VEH_LINE_NUM], int *VehLineNum,
                       int ShiftNum)
{
    LineSeg_T LinesArray[MAX_VEH_LINE_NUM];
    int i;
    int RealVehLineNum = *VehLineNum - ShiftNum;
    if (ShiftNum <= 0 || RealVehLineNum < 0) // no operation when the input in wrong
    {
        return 0;
    }
    memset(LinesArray, 0, sizeof(LinesArray)); //	clear
    for (i = 0; i < RealVehLineNum; i++)
    {
        memcpy(&LinesArray[i], &VehLines[i + ShiftNum], sizeof(LineSeg_T));
    }
    memcpy(VehLines, LinesArray, sizeof(LinesArray)); // assign value
    *VehLineNum = RealVehLineNum;                     //	assign value
    return 1;
}

/*	calculate the vehicle's body segment that form a slot	*/
void GetVehMainLine(LineSeg_T VehLines[MAX_VEH_LINE_NUM], int VehLineNum,
                    int VehLineDir[MAX_VEH_LINE_NUM], float *VehTheta,
                    LineSeg_T *VehMainLine, SLOT_DIR SlotDir, float SlotObjLen,
                    int SlotVehNumber)
{
    int i, j; //,MainDir;
    Relation_T dir, OuterDir;
    float Out_dist_max, Side_dist, mSign;
    Point_T MainDirStartPt, MainDirEndPt; //	start point and end point of the main line
    Point_T VehStartPt, VehEndPt;         //	start point and end point of the vehicle line
    float tempLen = 0, MaxVehLineLen = 0; //	max length in vehicle line set
    float tempTheta = 0;

    float VehThetaSum = 0; //	weight sum of vehicle line angles

    float SegPowLenSum = 0; //	sum of square of each line length
    float VehPowTheta  = 0; // weight sum of vehicle line angle
    float MainTheta    = 0; // main line angle
    float powTempLen   = 0;
    float tempRatio    = 0;
    Vec2_T Vec_off_Curb, SlotVec; // directions of slot vector and vertical-to-curb
    float VehLineLen   = 0;       //	length of a vehicle line
    int ShiftNum       = 0;
    int RealVehLineNum = 0;
    int TempVehLineNum = VehLineNum;
    LineSeg_T TempVehLines[MAX_VEH_LINE_NUM]; //	temporary buffers for vehicle lines
                                              // and vehicle directions

    if (VehLineNum <= 1) //	if there is only one line in buffer, then use it directly as
                         // the vehicle line
    {
        memcpy(VehMainLine, VehLines, sizeof(LineSeg_T));
        *VehTheta = atan2f(VehMainLine->pt2.y - VehMainLine->pt1.y,
                           VehMainLine->pt2.x - VehMainLine->pt1.x);
        return;
    }

    memcpy(TempVehLines, VehLines, sizeof(TempVehLines));
    // memcpy(TempVehLineDir,VehLineDir,sizeof(TempVehLineDir));

    if (SlotDir == SLOT_DIR_LEFT)
    {
        OuterDir =
            PK_ON_RIGHT; //	regrading to AB/EF, the side away from curb is right side
        mSign = 1;
    }
    else
    {
        OuterDir =
            PK_ON_LEFT; //	regrading to AB/EF, the side away from curb is left side
        mSign = -1;
    }

    if (SlotObjLen < 0.1f)
    {
        SlotObjLen = VEHICLE_WID; //	not sure it is a parallel slot or vertical slot.
                                  // consider it as a searching process
    }

    for (i = 0; i < TempVehLineNum; i++) //	calculate the main angle of the vehicle line
    {
        tempLen   = Get_Segment_Len(TempVehLines[i]);
        tempTheta = atan2f(TempVehLines[i].pt2.y - TempVehLines[i].pt1.y,
                           TempVehLines[i].pt2.x - TempVehLines[i].pt1.x);

        // search a line with max length
        if (tempLen > MaxVehLineLen)
        {
            MaxVehLineLen = tempLen;
            // MaxVehLineIndex = i;
            MainTheta = tempTheta;
        }

        powTempLen = pow2(tempLen);
        SegPowLenSum += powTempLen;
        tempRatio   = powTempLen / SegPowLenSum;
        VehPowTheta = Get_AverAngle_W(VehPowTheta, tempTheta, (1 - tempRatio), tempRatio);
    }

    // 20170614: if a line with long enough length is found, then set its direction as the
    // vehicle line direction, otherwise calculate weight sum of directions of each line
    // segments as the vehicle line direction
    if (MaxVehLineLen > URadar_LengthWeight * SlotObjLen)
    {
        VehThetaSum = MainTheta; //	set its angle as the main angle
    }
    else
    {
#if 0
		if (SlotVehNumber == 1)	//	1st vehicle
		{
			for(i = 0; i < TempVehLineNum; i++)	//	calculate main angle
			{
				MainDir = (TempVehLineDir[i]/100)%10;
				if (MainDir == 1 )
				{
					tempLen = GetSegmentLen(TempVehLines[i]);
					if (tempLen > 0.3f)
					{
						FindMainDirLine = 1;
						MainTheta = atan2f(TempVehLines[i][3]-TempVehLines[i][1],TempVehLines[i][2]-TempVehLines[i][0]);
						break;
					}

				}
			}
		}
		else if (SlotVehNumber == 2)	//	2nd vehicle
		{
			for(i = TempVehLineNum-1; i >= 0; i--)	//	calculate main angle
			{
				MainDir = (TempVehLineDir[i]/100)%10;
				if (MainDir == 1 )
				{
					tempLen = GetSegmentLen(TempVehLines[i]);
					if (tempLen > 0.3f)
					{
						FindMainDirLine = 1;
						MainTheta = atan2f(TempVehLines[i][3]-TempVehLines[i][1],TempVehLines[i][2]-TempVehLines[i][0]);
						break;
					}

				}
			}
		}
		if (FindMainDirLine == 1)
		{
			VehThetaSum = MainTheta;
		}
		else	//	line with main angle not found
		{
			VehThetaSum = GetAverAngle_W(MainTheta,VehPowTheta,MainDirRatio,(1-MainDirRatio));
			//	VehThetaSum = VehPowTheta;
		}
#else
        VehThetaSum = VehPowTheta;
#endif
    }

    SlotVec.vx = cosf(VehThetaSum);
    SlotVec.vy = sinf(VehThetaSum);
    *VehTheta  = VehThetaSum; //	update result

    //	get vehicle lines according to input slot length
    if (SlotVehNumber == 1) //	1st vehicle
    {
        for (i = TempVehLineNum - 1; i >= 0; i--)
        {
            RealVehLineNum++;
            VehLineLen += Get_Segment_Len(TempVehLines[i]);
            if (VehLineLen - SlotObjLen > 0 || SlotObjLen - VehLineLen < 0.3f)
            {
                break;
            }
        }
        ShiftNum = TempVehLineNum - RealVehLineNum;
        //	VehDirArrayShift(TempVehLineDir,TempVehLineNum,ShiftNum);	//	not changing
        // TempVehLineNum
        VehLinesArrayShift(TempVehLines, &TempVehLineNum, ShiftNum); // change VehLineNum
    }
    else if (SlotVehNumber == 2) // 2nd vehicle
    {
        for (i = 0; i < TempVehLineNum; i++)
        {
            RealVehLineNum++;
            VehLineLen += Get_Segment_Len(TempVehLines[i]);
            if (VehLineLen - SlotObjLen > 0 || SlotObjLen - VehLineLen < 0.1f)
            {
                break;
            }
        }
        TempVehLineNum =
            RealVehLineNum; // line set of 2nd vehicle does not need to be translated
    }

    //	initialization
    memcpy(&MainDirStartPt, &TempVehLines->pt1, sizeof(MainDirStartPt));
    MainDirEndPt.x  = MainDirStartPt.x + SlotVec.vx;
    MainDirEndPt.y  = MainDirStartPt.y + SlotVec.vy;
    Vec_off_Curb.vx = mSign * SlotVec.vy;
    Vec_off_Curb.vy = -mSign * SlotVec.vx;

    // Search the furthest distance away from curb
    Out_dist_max = 0;                    // initialization
    for (j = 0; j < TempVehLineNum; j++) // use the 1st line as the base line
    {

        Get_Dist_Dir_Pt2PointLine(MainDirStartPt, MainDirEndPt, TempVehLines[j].pt1,
                                  &Side_dist, &dir); // segment's start point
        if (dir == OuterDir && Side_dist > Out_dist_max)
        {
            Out_dist_max = Side_dist;
        }
        Get_Dist_Dir_Pt2PointLine(MainDirStartPt, MainDirEndPt, TempVehLines[j].pt2,
                                  &Side_dist, &dir); // segment's end point
        if (dir == OuterDir && Side_dist > Out_dist_max)
        {
            Out_dist_max = Side_dist;
        }
    }

    // translate the 1st line so that all other lines are on the other side near the curb
    if (Out_dist_max > 0)
    {
        MainDirStartPt.x += Vec_off_Curb.vx * Out_dist_max;
        MainDirStartPt.y += Vec_off_Curb.vy * Out_dist_max;
        MainDirEndPt.x += Vec_off_Curb.vx * Out_dist_max;
        MainDirEndPt.y += Vec_off_Curb.vy * Out_dist_max;
    }
    //	define the start point and end point of vehicle line
    GetFootPt_PointLine(MainDirStartPt, MainDirEndPt, TempVehLines[0].pt1, &VehStartPt);
    GetFootPt_PointLine(MainDirStartPt, MainDirEndPt,
                        TempVehLines[TempVehLineNum - 1].pt2, &VehEndPt);
    memcpy(&VehMainLine->pt1, &VehStartPt, sizeof(VehStartPt));
    memcpy(&VehMainLine->pt2, &VehEndPt, sizeof(VehEndPt));
}

/*	20171103: compute the property of a current line according to the last line
Input：
RadarDir
-1-left side ultrasonic sensor FSL
1-right side ultrasonic sensor FSR
return：
1: vehicle line
2: curb line
3：undetermined, set it as MidObj
4: current line property is the same as the last one so add it to the line set of last
line 5：current line property is the same as the second vehicle line one so add it to the
line set of second vehicle line
************************************************/
int CheckVehLinePro(LineSeg_T LastVehLine, LineSeg_T CurVehLine, Relation_T RadarDir)
{
    float SideDist1, SideDist2, SideDist3, SideDist4, EndDist;
    Relation_T dir1, dir2, dir3, dir4;
    const float OneObjDist = 0.3f;   //	having the same property if distance of two points
                                     // is less than this threshold
    const float CurbSideDist = 1.5f; //	the distance from a curb line to a vehicle line
    const float SameProSideDist =
        1.2f; //	having the same property if side distance is less than this threshold

    int LinePro         = 0; //	can not determine
    float CurSegmentLen = Get_Segment_Len(CurVehLine);
    Vec2_T vec_1, vec_2;
    float SegmentCrossAngle; //	the angle between the current line and the last line
    VehPos_T lastVehPos;

    // memcpy(&lastVehPos,&LastVehLine.pt2,sizeof(Point_T));
    lastVehPos.x      = LastVehLine.pt2.x;
    lastVehPos.y      = LastVehLine.pt2.y;
    lastVehPos.theta  = atan2f(LastVehLine.pt2.y - LastVehLine.pt1.y,
                               LastVehLine.pt2.x - LastVehLine.pt1.x);
    vec_1.vx          = LastVehLine.pt2.x - LastVehLine.pt1.x;
    vec_1.vy          = LastVehLine.pt2.y - LastVehLine.pt1.y;
    vec_2.vx          = CurVehLine.pt2.x - CurVehLine.pt1.x;
    vec_2.vy          = CurVehLine.pt2.y - CurVehLine.pt1.y;
    SegmentCrossAngle = fabsf(Get_CrossAngle_Vec2(
        vec_1, vec_2)); //	the angle between the current line and the last line
    if (SegmentCrossAngle > URadar_MaxLineAngle &&
        CurSegmentLen <
            URadar_MinSegLen_CheckVehLinePro) //	if the angle is too large, then the
                                              // following computations will be inaccurate
    {
        return 3;
    }

    // EndDist = Cal_Dis_Pt2Pt(LastVehLine.pt2,CurVehLine.pt1);	//	the adjacent end
    // points of last line and current line
    EndDist = fabsf(Project_PointTo1stPos_rx(lastVehPos, CurVehLine.pt1));
    Get_Dist_Dir_Pt2SegLine(
        LastVehLine, CurVehLine.pt1, &SideDist1,
        &dir1); //	distance from the start point of current line to last line
    Get_Dist_Dir_Pt2SegLine(
        LastVehLine, CurVehLine.pt2, &SideDist2,
        &dir2); //	distance from the end point of current line to last line
    if (CurSegmentLen > URadar_CurSegLenThresh)
    {
        Get_Dist_Dir_Pt2SegLine(
            CurVehLine, LastVehLine.pt1, &SideDist3,
            &dir3); //	distance from the start point of last line to current line
        Get_Dist_Dir_Pt2SegLine(
            CurVehLine, LastVehLine.pt2, &SideDist4,
            &dir4); //	distance from the start point of last line to current line
    }

    if (EndDist < OneObjDist && SideDist1 < SameProSideDist)
    {
        LinePro = 4;
    }
    else
    {
        if (CurSegmentLen > URadar_CurSegLenThresh)
        {
            if (dir1 == RadarDir && SideDist1 > CurbSideDist &&
                SideDist2 > CurbSideDist && dir3 != RadarDir &&
                SideDist3 > CurbSideDist && SideDist4 > CurbSideDist)
            {
                LinePro = 2; // curb line, last line property is correct and current line
                             // property is wrong. Keep searching forward.
            }
            else if (dir1 != RadarDir && SideDist1 > CurbSideDist &&
                     SideDist2 > CurbSideDist && dir3 == RadarDir &&
                     SideDist3 > CurbSideDist && SideDist4 > CurbSideDist)
            {
                LinePro = 1; // vehicle line, last line property is wrong and current line
                             // property is correct. Set current line as the first vehicle
                             // line and keep searching forward.
            }
            else if ((SideDist1 < SameProSideDist && SideDist2 < SameProSideDist) ||
                     (SideDist3 < SameProSideDist &&
                      SideDist4 <
                          SameProSideDist)) // property not the same as the last line
            {
                if (EndDist < URadar_LastLineToCurLineDistThresh)
                {
                    LinePro = 4;
                }
                else
                {
                    LinePro = 5;
                }
                /*if (EndDist > MIN_PARK_LEN_VERTICAL-0.5f)
                {
                    LinePro = 5;
                }
                else if (EndDist < SameProSideDist && (SideDist3 < SameProSideDist &&
                SideDist4 < SameProSideDist) )
                {
                    LinePro = 4;
                }
                else
                {
                    LinePro = 5;
                }*/
            }
        }
        else
        {
            if (dir1 == RadarDir && SideDist1 > CurbSideDist && SideDist2 > CurbSideDist)
            {
                LinePro = 2; // curb line, last line property is correct and current line
                             // property is wrong. Keep searching forward.
            }
            else if (dir1 != RadarDir && SideDist1 > CurbSideDist &&
                     SideDist2 > CurbSideDist)
            {
                LinePro = 1; // vehicle line, last line property is wrong and current line
                             // property is correct. Set current line as the first vehicle
                             // line and keep searching forward.
            }
            else if (SideDist1 < SameProSideDist &&
                     SideDist2 < SameProSideDist) // property same as the last line
            {
                if (EndDist < URadar_LastLineToCurLineDistThresh)
                {
                    LinePro = 4;
                }
                else if (EndDist > MIN_PARK_LEN_VERTICAL - 1.0f)
                {
                    LinePro = 5;
                }
                else if (CurSegmentLen < URadar_MaxMidObjLen)
                {
                    LinePro = 3;
                }
                else
                {
                    LinePro = 5;
                }
                /*if (EndDist > MIN_PARK_LEN_VERTICAL-1.0f)
                {
                    LinePro = 5;
                }
                else if (EndDist < SameProSideDist)
                {
                    LinePro = 4;
                }
                else
                {
                    LinePro = 3;
                }*/
            }
        }
    }
    //	no result from the above processing
    if (LinePro == 0)
    {
        if (CurSegmentLen > URadar_CurSegLenThresh)
        {
            LinePro = 5;
        }
        else
        {
            LinePro = 3;
        }
    }

    return LinePro;
}

// check whether mid object is curb line
void CheckMidObjIsCurb(LineSeg_T Veh_1_MainLine, LineSeg_T *MidObj, int *MidObjNum,
                       LineSeg_T *CurbObj, int *CurbNum)
{
    int i;
    int ValidFlag[MAX_VEH_LINE_NUM] = {0};
    float sideDist1, sideDist2;
    LineSeg_T TempMidObj[MAX_VEH_LINE_NUM];
    int RealCurbNum   = *CurbNum; //	initialization
    int RealMidObjNum = *MidObjNum;
    memcpy(TempMidObj, MidObj, sizeof(TempMidObj)); //	initialization
    for (i = 0; i < RealMidObjNum; i++)
    {
        ValidFlag[i] = 1; // initialization
        Get_Dist_Dir_Pt2SegLine(Veh_1_MainLine, MidObj[i].pt1, &sideDist1, NULL);
        Get_Dist_Dir_Pt2SegLine(Veh_1_MainLine, MidObj[i].pt2, &sideDist2, NULL);
        if (sideDist1 > URadar_MidObjToCurbThresh &&
            sideDist2 > URadar_MidObjToCurbThresh)
        {
            if (RealCurbNum < MAX_VEH_LINE_NUM)
            {
                memcpy(&CurbObj[RealCurbNum], &MidObj[i],
                       sizeof(LineSeg_T)); //	add to curb obj
                RealCurbNum++;
            }
            ValidFlag[i] = 0; //	delete from mid obj
        }
    }
    //	清零
    RealMidObjNum = 0;
    memset(MidObj, 0, sizeof(TempMidObj));
    for (i = 0; i < *MidObjNum; i++)
    {
        if (ValidFlag[i] == 1)
        {
            memcpy(&MidObj[RealMidObjNum], &TempMidObj[i], sizeof(LineSeg_T));
            RealMidObjNum++;
        }
    }
    //	assign value
    *CurbNum   = RealCurbNum;
    *MidObjNum = RealMidObjNum;
}

/* 20170325:  curb computation
According to curb line set, calculate curb line
ReturnL: 1-Success, 2-Error, the length of curb line is too short
*/
int CurbPointCal(SLOT_DIR SlotDir, LineSeg_T *CurbLines, int CurbLine_Num,
                 LineSeg_T Veh_1_MainLine, LineSeg_T Veh_2_MainLine,
                 Point_T *CurbStart_Pt, Point_T *CurbEnd_Pt)
{
    // if curb lines consist of multiple line segments, use the average angle as curb line
    // angle
    int j, k;
    Relation_T dir, RearLine_Pro;
    float Curb_Theta = 0;          //  curb line angle
    Vec2_T Vec_Curb, Vec_off_Curb; // vector vertical to curb line
    float Vert_dir_sign = 0;
    float Out_dist_max, Side_dist;
    // int RearLine_Pro = (int)SlotDir + 1; // compute line property according to the slot
    // direction property by plus 1. 1-left close, 2-right close 	float SegmentDist;

    const float CurbLen_min        = 0.2f; //	minimum curb line length
    const float valid_curb_len_min = 0.5f; //	minimum valid curb line length
    float CurbSegLen;                      //	max curb line length
    float AllCurbLen      = 0;             //	total curb line length
    float Theta1          = atan2f(Veh_1_MainLine.pt2.y - Veh_1_MainLine.pt1.y,
                                   Veh_1_MainLine.pt2.x - Veh_1_MainLine.pt1.x);
    float Theta2          = atan2f(Veh_2_MainLine.pt2.y - Veh_2_MainLine.pt1.y,
                                   Veh_2_MainLine.pt2.x - Veh_2_MainLine.pt1.x);
    float pow_curblen_sum = 0;
    float tmpAngle        = 0.0;
    float theta_nu        = 0; //	numerator used to compute curb angle
    float averAngle       = 0.0;
    Point_T curPt;

    if (SlotDir == SLOT_DIR_LEFT)
    {
        RearLine_Pro  = PK_ON_LEFT;
        Vert_dir_sign = -1;
    }
    else
    {
        RearLine_Pro  = PK_ON_RIGHT;
        Vert_dir_sign = 1;
    }
    for (j = 0; j < CurbLine_Num;
         j++) //	20170415：Compute average angle of all curb lines
    {
        CurbSegLen = Get_Segment_Len(CurbLines[j]);
        if (CurbSegLen > CurbLen_min)
        {
            tmpAngle  = atan2f(CurbLines[j].pt2.y - CurbLines[j].pt1.y,
                               CurbLines[j].pt2.x - CurbLines[j].pt1.x);
            averAngle = GetAverAngle(Theta1, Theta2);

            if (fabsf(tmpAngle - averAngle) >
                URadar_CurbCalAngleThresh) // 0.785 radian roughly equals 45 degree
            {
                continue;
            }

            theta_nu += pow2(CurbSegLen) * tmpAngle;
            pow_curblen_sum += pow2(CurbSegLen);
        }

        // if (CurbSegLen > CurbSegLen_Max)	//	too short to consider
        //{
        //	CurbSegLen_Max = CurbSegLen;
        //	Curb_Theta =
        // atan2f(CurbLines[j][3]-CurbLines[j][1],CurbLines[j][2]-CurbLines[j][0]);
        // }
        AllCurbLen += CurbSegLen;
    }
    if (pow_curblen_sum > 0)
    {
        Curb_Theta = theta_nu / pow_curblen_sum;
    }
    if (AllCurbLen < valid_curb_len_min)
    {
        return 0;
    }
    if (AllCurbLen < URadar_MinCurbLenSum) //	Not trust the angle if length is too short
    {
        Curb_Theta = GetAverAngle(Theta1, Theta2);
    }
    Vec_Curb.vx     = cosf(Curb_Theta);
    Vec_Curb.vy     = sinf(Curb_Theta);
    Vec_off_Curb.vx = -Vec_Curb.vy * Vert_dir_sign;
    Vec_off_Curb.vy = Vec_Curb.vx * Vert_dir_sign; // direction that vertical to curb line
                                                   // and pointing away from curb line
    //	initialize start point of the curb line
    for (j = 0; j < CurbLine_Num; j++)
    {
        if (Get_Segment_Len(CurbLines[j]) > CurbLen_min)
        {
            memcpy(CurbStart_Pt, &CurbLines[j], sizeof(Point_T));
            break;
        }
    }

    //  initialize end point of the curb line(moving 1 meters away along side the average
    //  angle)
    CurbEnd_Pt->x = CurbStart_Pt->x + Vec_Curb.vx;
    CurbEnd_Pt->y = CurbStart_Pt->y + Vec_Curb.vy;

    // check whether end point of first line and the start points and end points of the
    // rest lines are on the off-curb-side, if true, find out the largest distance from a
    // point to the curb line
    Out_dist_max = 0;                  // initialization
    for (j = 0; j < CurbLine_Num; j++) // loop through all curb lines
    {
        if (Get_Segment_Len(CurbLines[j]) > CurbLen_min) // 20170515: filter noise segment
        {
            for (k = 0; k < 2; k++)
            {
                if (k == 0)
                {
                    memcpy(&curPt, &(CurbLines[j].pt1), sizeof(Point_T));
                }
                else
                {
                    memcpy(&curPt, &(CurbLines[j].pt2), sizeof(Point_T));
                }

                Get_Dist_Dir_Pt2PointLine(*CurbStart_Pt, *CurbEnd_Pt, curPt, &Side_dist,
                                          &dir); // calculate the outermost distance
                if (dir != RearLine_Pro && Side_dist > Out_dist_max)
                {
                    Out_dist_max = Side_dist;
                }
            }
        }
    }
    // Translate curb start point and curb end point using the offset from above
    // computation
    if (Out_dist_max > 0)
    {
        CurbStart_Pt->x += Vec_off_Curb.vx * Out_dist_max;
        CurbStart_Pt->y += Vec_off_Curb.vy * Out_dist_max;
        CurbEnd_Pt->x += Vec_off_Curb.vx * Out_dist_max;
        CurbEnd_Pt->y += Vec_off_Curb.vy * Out_dist_max;
    }
    return 1;
}

// return the length of curb
float Curb_SegLineCal(SLOT_DIR SlotDir, LineSeg_T *CurbLines, int CurbLine_Num,
                      LineSeg_T *CurbSeg)
{
    // if curb lines consist of multiple line segments, use the average angle as curb line
    // angle
    int j, k;
    Relation_T dir, RearLine_Pro;
    float Curb_Theta = 0;          //  average angle of curb line
    Vec2_T Vec_Curb, Vec_off_Curb; // // vector vertical to curb line
    float Vert_dir_sign = 0;
    float Out_dist_max, Side_dist;
    Point_T CurbStart_Pt, CurbEnd_Pt;

    const float CurbLen_min = 0.2f; //	minimum length of a curb line which will be taken
                                    // into consideration to computation
    const float valid_curb_len_min = 0.5f; //	minimum total length for a valid curb line
    float CurbSegLen;                      //	max curb line length
    float AllCurbLen      = 0;             //	total curb line length
    float pow_curblen_sum = 0;
    float theta_nu        = 0; //	temp variable as numerator
    int find_startPt      = 0;

    if (SlotDir == SLOT_DIR_LEFT)
    {
        RearLine_Pro  = PK_ON_LEFT;
        Vert_dir_sign = -1;
    }
    else
    {
        RearLine_Pro  = PK_ON_RIGHT;
        Vert_dir_sign = 1;
    }
    for (j = 0; j < CurbLine_Num;
         j++) //	20170415：compute average angle of all curb lines
    {
        CurbSegLen = Get_Segment_Len(CurbLines[j]);
        if (CurbSegLen > CurbLen_min)
        {
            theta_nu +=
                pow2(CurbSegLen) * atan2f(CurbLines[j].pt2.y - CurbLines[j].pt1.y,
                                          CurbLines[j].pt2.x - CurbLines[j].pt1.x);
            pow_curblen_sum += pow2(CurbSegLen);
        }
        AllCurbLen += CurbSegLen;
    }

    if (AllCurbLen < valid_curb_len_min)
    {
        return 0;
    }

    if (pow_curblen_sum > 0)
    {
        Curb_Theta = theta_nu / pow_curblen_sum;
    }

    Vec_Curb.vx     = cosf(Curb_Theta);
    Vec_Curb.vy     = sinf(Curb_Theta);
    Vec_off_Curb.vx = -Vec_Curb.vy * Vert_dir_sign;
    Vec_off_Curb.vy = Vec_Curb.vx * Vert_dir_sign; // direction vertical to curb
    //	initialize start point
    for (j = 0; j < CurbLine_Num; j++)
    {
        if (Get_Segment_Len(CurbLines[j]) > CurbLen_min)
        {
            memcpy(&CurbStart_Pt, &CurbLines[j], sizeof(Point_T));
            find_startPt = 1;
            break;
        }
    }

    // Initialize end point
    if (find_startPt)
    {
        CurbEnd_Pt.x = CurbStart_Pt.x + Vec_Curb.vx;
        CurbEnd_Pt.y = CurbStart_Pt.y + Vec_Curb.vy;
    }
    else
    {
        return 0;
    }
    // check whether end point of first line and the start points and end points of the
    // rest lines are on the off-curb-side, if true, find out the largest distance from a
    // point to the curb line
    Out_dist_max = 0;
    for (j = 0; j < CurbLine_Num; j++) // loop through all curb lines
    {
        for (k = 0; k < 2; k++)
        {
            if (Get_Segment_Len(CurbLines[j]) > CurbLen_min) // 20170515: filter noise
            {
                Get_Dist_Dir_Pt2PointLine(CurbStart_Pt, CurbEnd_Pt, CurbLines[j].pt1,
                                          &Side_dist,
                                          &dir); // calculate the outermost distance
                if (dir != RearLine_Pro && Side_dist > Out_dist_max)
                {
                    Out_dist_max = Side_dist;
                }
            }
        }
    }

    // Ensure all curb line segments are on the other side
    if (Out_dist_max > 0)
    {
        CurbStart_Pt.x += Vec_off_Curb.vx * Out_dist_max;
        CurbStart_Pt.y += Vec_off_Curb.vy * Out_dist_max;
        CurbEnd_Pt.x += Vec_off_Curb.vx * Out_dist_max;
        CurbEnd_Pt.y += Vec_off_Curb.vy * Out_dist_max;
    }
    GetFootPt_PointLine(CurbStart_Pt, CurbEnd_Pt, CurbLines[0].pt1, &CurbSeg->pt1);
    GetFootPt_PointLine(CurbStart_Pt, CurbEnd_Pt, CurbLines[CurbLine_Num - 1].pt2,
                        &CurbSeg->pt2);
    return AllCurbLen;
}

//	20170511：calculate parallel slot when there is no curb
int ParalSlotCal_NoCurb_V2(SLOT_DIR SlotDir, LineSeg_T Veh_1_Main_Line,
                           LineSeg_T Veh_2_Main_Line, VehPos_T *TargPos,
                           SlotObj_T *SlotObj)
{
    const float Safe_dist_curb = 0.6f;
    float Veh_Theta_1, Veh_Theta_2,
        TargTheta; //	angles of the first vehicle line and second vehicle line
    Vec2_T SlotVec, Vet_to_Curb_Vec;
    Point_T FootPt, VirtCurbPt;
    float SlotLen, d_len, div_x;
    float Vert_dir_sign;
    VehPos_T m_TargPos;                                     //	target pose local variable
    Point_T Slot_A, Slot_B, Slot_C, Slot_D, Slot_E, Slot_F; //	slot corners
                                                            //	int i;

    Veh_Theta_1 = atan2f(Veh_1_Main_Line.pt2.y - Veh_1_Main_Line.pt1.y,
                         Veh_1_Main_Line.pt2.x - Veh_1_Main_Line.pt1.x);
    Veh_Theta_2 = atan2f(Veh_2_Main_Line.pt2.y - Veh_2_Main_Line.pt1.y,
                         Veh_2_Main_Line.pt2.x - Veh_2_Main_Line.pt1.x);
    if (Round_PI(fabsf(Veh_Theta_2 - Veh_Theta_1)) >
        URadar_MaxAllowVehLineAngleDiff * PI /
            180.0f) // difference of two angles should not be too large
    {
        LOGD << "DROPPED the parallel slot because angles difference of two  vechicle "
                "too large(>30)";
        USlotDetect_RunStep_ccp = 73;
        return -1;
    }
    if (SlotDir == SLOT_DIR_LEFT)
    {
        Vert_dir_sign = -1;
    }
    else
    {
        Vert_dir_sign = 1;
    }
    TargTheta = GetAverAngle(
        Veh_Theta_1, Veh_Theta_2); //	20170420: average angle of two vehicle lines
    SlotVec.vx         = cosf(TargTheta);
    SlotVec.vy         = sinf(TargTheta);            // use average angle as the result
    Vet_to_Curb_Vec.vx = SlotVec.vy * Vert_dir_sign; // vector vertical to slot
    Vet_to_Curb_Vec.vy = -SlotVec.vx * Vert_dir_sign;

    // distance of two adjacent points of vehicle lines should not exceed a threshold!
    GetFootPt_PointAndVecLine(Veh_1_Main_Line.pt2, Vet_to_Curb_Vec, Veh_2_Main_Line.pt1,
                              &FootPt);

    if (Cal_Dis_Pt2Pt(Veh_1_Main_Line.pt2, FootPt) > URadar_MaxAllowVehLinesInnerDist)
    {
        LOGD << "DROPPED the parallel slot because distance of two adjacent points of "
                "vehicle lines is too large(>1.0)";
        USlotDetect_RunStep_ccp = 74;
        return -1;
    }
    memcpy(&Slot_A, &Veh_1_Main_Line.pt1, sizeof(Slot_A));
    memcpy(&Slot_B, &Veh_1_Main_Line.pt2, sizeof(Slot_B));
    memcpy(&Slot_E, &Veh_2_Main_Line.pt1, sizeof(Slot_E));
    memcpy(&Slot_F, &Veh_2_Main_Line.pt2, sizeof(Slot_F));
    GetFootPt_PointAndVecLine(Slot_B, SlotVec, Slot_E, &FootPt);

    // if slot length is less than 3 meters, center the parking pose, otherwise, park the
    // vehicle in front of last vehicle 1 meter ahead
    SlotLen = Cal_Dis_Pt2Pt(
        FootPt, Slot_B); // calculate the slot length along foot point-B direction
    Slotlength_before_ccp = SlotLen;
    if (SlotLen < MIN_PARK_LEN_PARALLEL)
    {
        USlotDetect_RunStep_ccp = 75;
        return 0;
    }
    else if (SlotLen > MAX_PARK_LEN_PARALLEL)
    {
        LOGD << "DROPPED the parallel slot because SlotLen is too large(>10.0)";
        USlotDetect_RunStep_ccp = 76;
        return -1;
    }
    GetFootPt_PointAndVecLine(Slot_B, SlotVec, Slot_E, &FootPt);
    SlotLen =
        Cal_Dis_Pt2Pt(Slot_E, Slot_B); // calculate the slot length along BE direction
    Slotlength_cal_ccp = SlotLen;
    d_len              = SlotLen - VEHICLE_LEN;
    if (d_len < BIG_PARA_SLOT_LEN_DIFF)
    {
        div_x = d_len / 2 + REAR_SUSPENSION; // displacement that center the vehicle
    }
    else
    {
        div_x = (d_len - BIG_PARA_SEC_VEH_DIST) + REAR_SUSPENSION;
    }
    GetFootPt_PointAndVecLine(Slot_E, SlotVec, Slot_B, &FootPt);

    m_TargPos.x = (Slot_B.x + FootPt.x) / 2.0f + Vet_to_Curb_Vec.vx * VEHICLE_WID / 2.0f +
                  div_x * SlotVec.vx;
    m_TargPos.y = (Slot_B.y + FootPt.y) / 2.0f + Vet_to_Curb_Vec.vy * VEHICLE_WID / 2.0f +
                  div_x * SlotVec.vy;
    m_TargPos.theta = TargTheta;

    // Calculate virtual curb
    VirtCurbPt.x =
        m_TargPos.x + Vet_to_Curb_Vec.vx * (VEHICLE_WID / 2.0f + Safe_dist_curb);
    VirtCurbPt.y =
        m_TargPos.y + Vet_to_Curb_Vec.vy * (VEHICLE_WID / 2.0f + Safe_dist_curb);
    GetFootPt_PointAndVecLine(VirtCurbPt, SlotVec, Slot_B, &Slot_C);
    GetFootPt_PointAndVecLine(VirtCurbPt, SlotVec, Slot_E, &Slot_D);

    // Check whether target pose is within slot
    if (IsTargPosInSlot(Slot_B, Slot_C, Slot_D, Slot_E, m_TargPos) == PK_OUTSIDE)
    {
        USlotDetect_RunStep_ccp = 77;
        return 0;
    }
    //	Update target pose
    memcpy(TargPos, &m_TargPos, sizeof(m_TargPos));
    //	Update slot object
    memcpy(&SlotObj->ptA, &Slot_A, sizeof(Slot_A));
    memcpy(&SlotObj->ptB, &Slot_B, sizeof(Slot_B));
    memcpy(&SlotObj->ptC, &Slot_C, sizeof(Slot_C));
    memcpy(&SlotObj->ptD, &Slot_D, sizeof(Slot_D));
    memcpy(&SlotObj->ptE, &Slot_E, sizeof(Slot_E));
    memcpy(&SlotObj->ptF, &Slot_F, sizeof(Slot_F));
    USlotDetect_RunStep_ccp = 78;
    return 1; //	return success flag
}

// calculate parallel slot and target pose using curb
// return: -1: can not produce a slot, keep searching. 0: do not meet the requirements to
// produce a parallel slot. it might be a vertical slot. 1: success
int ParalSlotCal_WithCurb_V2(SLOT_DIR SlotDir, Point_T CurbStart_Pt, Point_T CurbEnd_Pt,
                             LineSeg_T Veh_1_Main_Line, LineSeg_T Veh_2_Main_Line,
                             VehPos_T *TargPos, SlotObj_T *SlotObj)
{
    // const float Para_SafeDist_Curb_ccp =  0.35f;   //  safe distance to both sides when
    // parking finishes
    Vec2_T Curb_Vec, Vec_off_Curb; // curb direction
    float Vert_off_sign;           // vertical-to-curb direction
    Point_T Slot_A, Slot_B, Slot_C, Slot_D, Slot_E, Slot_F, FootPt; // slot corners
    float SlotLen, d_len, div_x;

    float dist1 = 0, dist2 = 0;
    float OffCurbDist = 0; //	offset that moving target pose away from curb line
    VehPos_T m_TargPos;    //	local target pose variable

    //	calculate without curb
    Get_Dist_Dir_Pt2PointLine(CurbStart_Pt, CurbEnd_Pt, Veh_1_Main_Line.pt2, &dist1,
                              NULL); //	distance from point B to curb
    Get_Dist_Dir_Pt2PointLine(CurbStart_Pt, CurbEnd_Pt, Veh_2_Main_Line.pt1, &dist2,
                              NULL); //	distance from point E to curb
    if (dist1 >= VEHICLE_WID + Para_SafeDist_Curb_ccp &&
        dist2 >= VEHICLE_WID + Para_SafeDist_Curb_ccp)
    {
        OffCurbDist = (dist1 + dist2) * 0.5f - VEHICLE_WID; // average
    }
    else
    {
        OffCurbDist = Para_SafeDist_Curb_ccp;
    }

    //	calculate with curb
    if (SlotDir == SLOT_DIR_LEFT)
    {
        Vert_off_sign = -1;
    }
    else
    {
        Vert_off_sign = 1;
    }

    Curb_Vec.vx =
        CurbEnd_Pt.x - CurbStart_Pt.x; // use curb direction as target pose orientation
    Curb_Vec.vy     = CurbEnd_Pt.y - CurbStart_Pt.y;
    Vec_off_Curb.vx = -Curb_Vec.vy * Vert_off_sign;
    Vec_off_Curb.vy =
        Curb_Vec.vx *
        Vert_off_sign; // vertical-to-curb direction that points away from curb

    //  Slot_B,Slot_C,Slot_D,Slot_E describe the parking slot
    memcpy(&Slot_B, &Veh_1_Main_Line.pt2, sizeof(Slot_B));
    GetFootPt_PointLine(CurbStart_Pt, CurbEnd_Pt, Slot_B, &Slot_C);
    memcpy(&Slot_E, &Veh_2_Main_Line.pt1, sizeof(Slot_E));
    GetFootPt_PointLine(CurbStart_Pt, CurbEnd_Pt, Slot_E, &Slot_D);

    // if slot length is less than 3 meters, center the parking pose, otherwise, park the
    // vehicle in front of last vehicle 1 meter ahead
    GetFootPt_PointAndVecLine(Slot_B, Curb_Vec, Slot_E, &FootPt);
    SlotLen = Cal_Dis_Pt2Pt(
        FootPt, Slot_B); // calculate the slot length along foot point-B direction
    Slotlength_before_ccp = SlotLen;
    if (SlotLen < MIN_PARK_LEN_PARALLEL)
    {
        USlotDetect_RunStep_ccp = 79;
        return 0;
    }
    else if (SlotLen > MAX_PARK_LEN_PARALLEL)
    {
        LOGD << "DROPPED the parallel slot because SlotLen is too long(>10)";
        USlotDetect_RunStep_ccp = 80;
        return -1;
    }

    // slot depth needs to be larger than a threshold
    if (Cal_Dis_Pt2Pt(Slot_E, Slot_D) < CurbParaSlotDepth_min ||
        Cal_Dis_Pt2Pt(Slot_B, Slot_C) < CurbParaSlotDepth_min)
    {
        LOGD << "DROPPED the parallel slot because depth is not enough(<1.2)";
        USlotDetect_RunStep_ccp = 81;
        return -1;
    }

    // distance of slot B and slot E along vertical curb direction can not be too large
    GetFootPt_PointAndVecLine(Slot_B, Vec_off_Curb, Slot_E, &FootPt);
    if (Cal_Dis_Pt2Pt(FootPt, Slot_B) > 1.5f)
    {
        LOGD << "DROPPED the parallel slot because distance of slot B and slot E along "
                "vertical curb direction is too large(>1.5)";
        USlotDetect_RunStep_ccp = 82;
        return -1;
    }

    GetFootPt_PointAndVecLine(Slot_B, Curb_Vec, Slot_E, &FootPt);
    SlotLen =
        Cal_Dis_Pt2Pt(FootPt, Slot_B); // calculate the BE distance along curb direction
    Slotlength_cal_ccp = SlotLen;

    d_len = SlotLen - VEHICLE_LEN;
    if (d_len < BIG_PARA_SLOT_LEN_DIFF)
    {
        div_x = d_len / 2 + REAR_SUSPENSION; // displacement that center the vehicle
    }
    else
    {
        div_x = (d_len - BIG_PARA_SEC_VEH_DIST) +
                REAR_SUSPENSION; // 1 meter ahead the last vehicle
    }

    m_TargPos.x = Slot_C.x + Vec_off_Curb.vx * (OffCurbDist + VEHICLE_WID / 2.0f) +
                  div_x * Curb_Vec.vx;
    m_TargPos.y = Slot_C.y + Vec_off_Curb.vy * (OffCurbDist + VEHICLE_WID / 2.0f) +
                  div_x * Curb_Vec.vy;
    m_TargPos.theta = atan2f(Curb_Vec.vy, Curb_Vec.vx);

    //	check whether target pose is within the slot
    if (IsTargPosInSlot(Slot_B, Slot_C, Slot_D, Slot_E, m_TargPos) == PK_OUTSIDE)
    {
        USlotDetect_RunStep_ccp = 83;
        return 0;
    }

    // 给出车槽附近的障碍物信息用于大屏动画显示，先简单以处理:与路沿平行
    /*for (i = 0; i < 2; i++)
    {
    Slot_A[i] = Slot_B[i] - Curb_Vec[i]*VEHICLE_LEN;
    Slot_F[i] = Slot_E[i] + Curb_Vec[i]*VEHICLE_LEN;
    }*/

    memcpy(&Slot_A, &Veh_1_Main_Line.pt1, sizeof(Slot_A));
    memcpy(&Slot_F, &Veh_2_Main_Line.pt2, sizeof(Slot_F));

    //	update target pose
    memcpy(TargPos, &m_TargPos, sizeof(m_TargPos));
    //	update slot object
    memcpy(&SlotObj->ptA, &Slot_A, sizeof(Slot_A));
    memcpy(&SlotObj->ptB, &Slot_B, sizeof(Slot_B));
    memcpy(&SlotObj->ptC, &Slot_C, sizeof(Slot_C));
    memcpy(&SlotObj->ptD, &Slot_D, sizeof(Slot_D));
    memcpy(&SlotObj->ptE, &Slot_E, sizeof(Slot_E));
    memcpy(&SlotObj->ptF, &Slot_F, sizeof(Slot_F));
    USlotDetect_RunStep_ccp = 84;
    return 1; // return success flag
}

// Check whether line segments from sensor fusion module interfere with vehicle
int IsTargPosInsertFusionObj_A(VehPos_T TargPos, LineSeg_T *Fusion_Obj,
                               int Fusion_ObjDir[], int Fusion_ObjNum,
                               PK_SlotShapeType SlotShap)
{
    Rect_T CornerPt; //	vehicle corners
    int CurLinePro;
    int i, StartIndex;
    Get_Veh_CornerPt(TargPos, &CornerPt, 0);
    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_LEFT_VERT)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&g_Side_FS_Obj);
    }
    else
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&g_Side_FS_Obj);
    }
    StartIndex = SF_OBJ_NUM - g_Side_FS_Obj.num;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        CurLinePro = (Fusion_ObjDir[i] / 1000) % 10;
        if (CurLinePro > 0 && CurLinePro <= 3 &&
            RectClipSegment(CornerPt, Fusion_Obj[i], NULL) ==
                1) //	line segment that interfere with vehicle needs to be valid
        {
            return 1; //	line segment interferes with vehicle
        }
    }
    return 0;
}

//	20170511：calculate vertical slot parameters without a curb line
//  return: 1-success, 2-Error
int VertSlotCal_NoCurb_V2(SLOT_DIR SlotDir, LineSeg_T Veh_1_Main_Line,
                          LineSeg_T Veh_2_Main_Line, VehPos_T *TargPos,
                          SlotObj_T *SlotObj)
{
    const float Safe_dist_curb =
        0.6f; //   safe distance from vehicle to curb after parking
    float Veh_Theta_1, Veh_Theta_2, Aver_Theta, TargTheta;
    Vec2_T TargVec, Slot_Vec;
    Point_T FootPt, Pt_2, VirtCurbPt;
    Point_T Slot_A, Slot_B, Slot_C, Slot_D, Slot_E, Slot_F; //	slot corners
    Point_T Slot_B_prime;                                   //	BE direction
    float SlotLen, d_len, div_x = 0;
    Relation_T dir;
    VehPos_T m_TargPos; //	target pose local variable

    // no curb in a vertical slot
    Veh_Theta_1 = atan2f(Veh_1_Main_Line.pt2.y - Veh_1_Main_Line.pt1.y,
                         Veh_1_Main_Line.pt2.x - Veh_1_Main_Line.pt1.x);
    Veh_Theta_2 = atan2f(Veh_2_Main_Line.pt2.y - Veh_2_Main_Line.pt1.y,
                         Veh_2_Main_Line.pt2.x - Veh_2_Main_Line.pt1.x);
    if (Round_PI(fabsf(Veh_Theta_2 - Veh_Theta_1)) >
        30.0f * PI / 180.0f) // if the angles of two vehicle lines differ to much, return
    {
        LOGD << "DROPPED the vetical slot because angles differ too much(>30)";
        USlotDetect_RunStep_ccp = 60;
        return 0;
    }
    Aver_Theta = Get_AverAngle_W(
        Veh_Theta_1, Veh_Theta_2, 0.5f,
        0.5f); // average angle of front vehicle line and rear vehicle line
    if (SlotDir == SLOT_DIR_LEFT)
    {
        TargTheta = Aver_Theta - PI / 2.0f; // rotate 90 degree clockwise
    }
    else
    {
        TargTheta = Aver_Theta + PI / 2.0f; // rotate 90 degree anti-clockwise
    }
    TargVec.vx = cosf(TargTheta);
    TargVec.vy = sinf(TargTheta);

    memcpy(&Slot_A, &Veh_1_Main_Line.pt1, sizeof(Slot_A));
    memcpy(&Slot_B, &Veh_1_Main_Line.pt2, sizeof(Slot_B));
    memcpy(&Slot_E, &Veh_2_Main_Line.pt1, sizeof(Slot_E));
    memcpy(&Slot_F, &Veh_2_Main_Line.pt2, sizeof(Slot_F));
    // displacement along x axis of target pose can not be too large
    GetFootPt_PointAndVecLine(Slot_B, TargVec, Slot_E,
                              &FootPt); // E点到B点的垂足（FootPt - 垂线的交点g）
    if (Cal_Dis_Pt2Pt(FootPt, Slot_B) >
        URadar_MaxVehLinesBias) //	20170908：if this method does not work, release the
                                // tjreshold
    {
        LOGD << "DROPPED the vetical slot because Cal_Dis_Pt2Pt(FootPt,Slot_B) too "
                "big(>1.6)";
        USlotDetect_RunStep_ccp = 61;
        return 0;
    }

    //  target pose calculation
    Slot_Vec.vx = cosf(Aver_Theta);
    Slot_Vec.vy = sinf(Aver_Theta); // pointing from vehicle 1st to vehicle 2nd
    GetFootPt_PointAndVecLine(Slot_B, Slot_Vec, Slot_E, &FootPt);
    SlotLen = Cal_Dis_Pt2Pt(
        FootPt, Slot_B); // project BE to slot vector direction and calculate the distance
    Slotlength_before_ccp = SlotLen;
    Slotlength_cal_ccp    = SlotLen;
    if (SlotLen < MIN_PARK_LEN_VERTICAL || SlotLen > MAX_PARK_LEN_VERTICAL)
    {
        LOGD << "DROPPED the vetical slot because SlotLen is too large or too small(>5.1 "
                "or <2.46), actually: "
             << SlotLen;
        USlotDetect_RunStep_ccp = 62;
        return 0;
    }
    d_len = SlotLen - VEHICLE_WID;
    if (d_len < BIG_VERT_SLOT_LEN_DIFF && d_len > 0)
    {
        div_x = SlotLen / 2; // center the vehicle
    }

    Slot_B_prime.x = Slot_B.x + Slot_Vec.vx;
    Slot_B_prime.y = Slot_B.y + Slot_Vec.vy;
    Get_Dist_Dir_Pt2PointLine(Slot_B, Slot_B_prime, Slot_E, NULL, &dir);
    // GetFootPt_PointAndVecLine(Slot_B,TargVec,Slot_E,FootPt);
    if ((SlotDir == SLOT_DIR_LEFT && dir == PK_ON_LEFT) ||
        (SlotDir == SLOT_DIR_RIGHT && dir == PK_ON_RIGHT))
    {
        // memcpy(Pt_1,Slot_B,sizeof(Slot_B));
        if (d_len > BIG_VERT_SLOT_LEN_DIFF)
        {
            div_x = SlotLen - VEHICLE_WID / 2.0f -
                    BIG_VERT_SEC_VEH_DIST; // keep 0.8m away from the second vehicle
        }

        Pt_2.x = Slot_B.x + Slot_Vec.vx * div_x; // middle point
        Pt_2.y = Slot_B.y + Slot_Vec.vy * div_x;
    }
    else
    {
        // GetFootPt_PointAndVecLine(Slot_B,TargVec,Slot_E,FootPt);
        // memcpy(Pt_1,FootPt,sizeof(FootPt));
        if (d_len > BIG_VERT_SLOT_LEN_DIFF)
        {
            div_x = BIG_VERT_SEC_VEH_DIST +
                    VEHICLE_WID / 2.0f; // keep 0.8m away from the second vehicle
        }

        Pt_2.x = Slot_E.x - Slot_Vec.vx * div_x; // middle point
        Pt_2.y = Slot_E.y - Slot_Vec.vy * div_x;
    }

    Pt_2.x = (Slot_B.x + Slot_E.x) / 2;
    Pt_2.y = (Slot_B.y + Slot_E.y) / 2;

    m_TargPos.x     = Pt_2.x + (VEHICLE_LEN - REAR_SUSPENSION) * (-TargVec.vx);
    m_TargPos.y     = Pt_2.y + (VEHICLE_LEN - REAR_SUSPENSION) * (-TargVec.vy);
    m_TargPos.theta = TargTheta; // target orientations

    // calculate virtual curb

    VirtCurbPt.x = m_TargPos.x - TargVec.vx * (REAR_SUSPENSION + Safe_dist_curb);
    VirtCurbPt.y = m_TargPos.y - TargVec.vy * (REAR_SUSPENSION + Safe_dist_curb);
    GetFootPt_PointAndVecLine(VirtCurbPt, Slot_Vec, Slot_B, &Slot_C);
    GetFootPt_PointAndVecLine(VirtCurbPt, Slot_Vec, Slot_E, &Slot_D);

    //	check if target pose is inside slot
    if (IsTargPosInSlot(Slot_B, Slot_C, Slot_D, Slot_E, m_TargPos) == PK_OUTSIDE)
    {
        LOGD << "DROPPED the vetical slot because target pose is outside slot";
        USlotDetect_RunStep_ccp = 63;
        return 0;
    }
    //	update target pose
    memcpy(TargPos, &m_TargPos, sizeof(m_TargPos));
    //	update slot object
    memcpy(&SlotObj->ptA, &Slot_A, sizeof(Slot_A));
    memcpy(&SlotObj->ptB, &Slot_B, sizeof(Slot_B));
    memcpy(&SlotObj->ptC, &Slot_C, sizeof(Slot_C));
    memcpy(&SlotObj->ptD, &Slot_D, sizeof(Slot_D));
    memcpy(&SlotObj->ptE, &Slot_E, sizeof(Slot_E));
    memcpy(&SlotObj->ptF, &Slot_F, sizeof(Slot_F));
    USlotDetect_RunStep_ccp = 64;
    return 1; // return success flag
}

//	translate target pose according to slot object
int UpDateTargPos_Trans(VehPos_T *TargPos, SlotObj_T NewSlotObj,
                        PK_SlotShapeType SlotShap)
{
    Vec2_T Slot_Vec, OffCurbVec;
    Point_T Slot_B, Slot_E, FootPt, PtInNewTargPos, PtInOldTargPos;
    float SlotTheta;
    float SlotLen, d_len, div_x;
    float TargPosTransDist = 0;
    int Ptdir              = 0;
    Relation_T re_Ptdir;

    VehPos_T mTargPos;
    Point_T targ_position;
    float mSign = 0;
    memcpy(&mTargPos, TargPos, sizeof(mTargPos));
    memcpy(&Slot_B, &NewSlotObj.ptB, sizeof(Slot_B));
    memcpy(&Slot_E, &NewSlotObj.ptE, sizeof(Slot_E));
    if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_LEFT_PARA)
    {
        mSign = -1.0f;
        //		SlotDir = SLOT_DIR_LEFT;
    }
    else
    {
        mSign = 1.0f;
        //		SlotDir = SLOT_DIR_RIGHT;
    }
    //	calculate slot angle
    if (SlotShap == PK_SLOT_LEFT_VERT) // left vertical slot
    {
        SlotTheta = TargPos->theta + PI / 2.0f; // rotate 90 degree anti-clockwise
    }
    else if (SlotShap == PK_SLOT_RIGHT_VERT) // right vertical slot
    {
        SlotTheta = TargPos->theta - PI / 2.0f; // rotate 90 degree clockwise
    }
    else //	parallel slot
    {
        SlotTheta = TargPos->theta;
    }

    //   calculate target pose
    Slot_Vec.vx   = cosf(SlotTheta);
    Slot_Vec.vy   = sinf(SlotTheta); // point from 1st vehicle to 2nd vehicle
    OffCurbVec.vx = -mSign * Slot_Vec.vy;
    OffCurbVec.vy = mSign * Slot_Vec.vx;
    GetFootPt_PointAndVecLine(Slot_B, Slot_Vec, Slot_E, &FootPt);
    SlotLen =
        Cal_Dis_Pt2Pt(FootPt, Slot_B); // project E on slot_vec direction and calculate
                                       // the distance from the foot point to B
    Slotlength_before_ccp = SlotLen;
    Slotlength_cal_ccp    = SlotLen;

    //============== Vertical slot target pose calculation	===============//
    if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
    {
        if (SlotLen < MIN_PARK_LEN_VERTICAL || SlotLen > MAX_PARK_LEN_VERTICAL)
        {
            USlotDetect_RunStep_ccp = 62;
            return 0;
        }
        d_len = SlotLen - VEHICLE_WID;
        if (d_len < BIG_VERT_SLOT_LEN_DIFF && d_len > 0)
        {
            div_x = SlotLen / 2.0f; // center the pose
        }
        else if (d_len > BIG_VERT_SLOT_LEN_DIFF)
        {
            div_x = (d_len - BIG_VERT_SEC_VEH_DIST) +
                    VEHICLE_WID / 2; // keep 0.8m away from 2nd vehicle
        }
        else
        {
            return 0;
        }
    }
    else //=============	target pose of parallel slot calculation ==============//
    {
        if (SlotLen < MIN_PARK_LEN_PARALLEL || SlotLen > MAX_PARK_LEN_PARALLEL)
        {
            USlotDetect_RunStep_ccp = 90;
            return 0;
        }
        d_len = SlotLen - VEHICLE_LEN;
        if (d_len < BIG_PARA_SLOT_LEN_DIFF && d_len > 0)
        {
            div_x = d_len / 2 + REAR_SUSPENSION; // center the vehicle
        }
        else if (d_len >= BIG_PARA_SLOT_LEN_DIFF)
        {
            div_x = (d_len - BIG_PARA_SEC_VEH_DIST) +
                    REAR_SUSPENSION; // keep 1m away from front vehicle
        }
        else
        {
            return 0;
        }
    }

    PtInNewTargPos.x = Slot_B.x + Slot_Vec.vx * div_x;
    PtInNewTargPos.y = Slot_B.y + Slot_Vec.vy * div_x;
    PtInOldTargPos.x = TargPos->x + OffCurbVec.vx;
    PtInOldTargPos.y = TargPos->y + OffCurbVec.vy;
    memcpy(&targ_position, TargPos, sizeof(targ_position));
    Get_Dist_Dir_Pt2PointLine(targ_position, PtInOldTargPos, PtInNewTargPos,
                              &TargPosTransDist, &re_Ptdir);
    if (re_Ptdir == PK_ON_LEFT)
    {
        Ptdir = -1;
    }
    else if (re_Ptdir == PK_ON_RIGHT)
    {
        Ptdir = 1;
    }
    //	translation

    mTargPos.x += Ptdir * mSign * TargPosTransDist * Slot_Vec.vx;
    mTargPos.y += Ptdir * mSign * TargPosTransDist * Slot_Vec.vy;
    //	check if target pose is inside the slot. Check if vehicle interfere with the slot
    if ((IsTargPosInSlot(NewSlotObj.ptB, NewSlotObj.ptC, NewSlotObj.ptD, NewSlotObj.ptE,
                         mTargPos) == PK_OUTSIDE) ||
        (IsTargPosVehInSertSlot(mTargPos, NewSlotObj) == 1))
    {
        return 0;
    }
    //	update target pose
    memcpy(TargPos, &mTargPos, sizeof(mTargPos));
    return 1;
}

//	20170717：shrink the slot if there are obstacles lying on both side of the vehicle. if
// obstacles invade too much, skip the slot. 	20171008：omit obstacles of short length
//	return: 0-skip the slot, 1-update the slot
int SlotCheckForSmallObj(SlotObj_T *SlotObj, VehPos_T *TargPos, LineSeg_T *MidObj,
                         int MidObjNum, PK_SlotShapeType SlotShap,
                         LineSeg_T Veh_1_MainLine, LineSeg_T Veh_2_MainLine)
{
    int i, j;
    Point_T PtA, PtB, PtC, PtD, PtE, PtF, FootPt, cur_pt, old_C, old_D;
    float dist_B = 0, dist_E = 0, sideDist_B = 0, sideDist_E = 0;
    float max_dist_B = 0, max_dist_E = 0;
    Vec2_T VecAB, VecFE, SlotVec, vec_CB, vec_DE; // OffCurbVec,
    float SlotTheta = TargPos->theta;             //	along searching direction
    float SlotLen   = 0;

    LineSeg_T TempCurbLines[CURB_LINE_NUM_MAX];
    int CurbNum = 0;
    SLOT_DIR SlotDir;
    Point_T CurbStart_Pt, CurbEnd_Pt;
    LineSeg_T MidObj_B[MAX_VEH_LINE_NUM],
        MidObj_E[MAX_VEH_LINE_NUM]; //	buffers of middle objects next to BE
    int MidObjNum_B = 0, MidObjNum_E = 0;
    float MidObjLen = 0;
    int OffCurbDir  = 0; // sign value representing left or right. -1:left, 1:right
    float OutSideDist_EF_max = -1e2f; //	EF Offset
    float OutSideDist_AB_max = -1e2f; //	AB Offset
    float MidObjOffCurbFlag  = 1.0f;
    Relation_T pt_dir, Re_OffCurbDir, BC_narrowSlotDir,
        DE_narrowSlotDir; //	direction that narrow the slot
    SlotObj_T temp_slotobj;
    VehPos_T temp_targpos;
    memcpy(&temp_slotobj, SlotObj, sizeof(temp_slotobj)); //	initialization
    memcpy(&temp_targpos, TargPos, sizeof(temp_targpos));
    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_LEFT_VERT)
    {
        SlotDir          = SLOT_DIR_LEFT;
        OffCurbDir       = -1;
        BC_narrowSlotDir = PK_ON_RIGHT;
        DE_narrowSlotDir = PK_ON_RIGHT;
        Re_OffCurbDir    = PK_ON_RIGHT;
    }
    else
    {
        SlotDir          = SLOT_DIR_RIGHT;
        OffCurbDir       = 1;
        BC_narrowSlotDir = PK_ON_LEFT;
        DE_narrowSlotDir = PK_ON_LEFT;
        Re_OffCurbDir    = PK_ON_LEFT;
    }
    if (SlotShap == PK_SLOT_LEFT_VERT) //	vertical slot
    {
        SlotTheta += PI / 2.0f;
    }
    else if (SlotShap == PK_SLOT_RIGHT_VERT)
    {
        SlotTheta -= PI / 2.0f;
    }
    SlotVec.vx = cosf(SlotTheta);
    SlotVec.vy = sinf(SlotTheta);
    // OffCurbVec.vx = -OffCurbDir*SlotVec.vy;
    // OffCurbVec.vy = OffCurbDir*SlotVec.vx;
    memcpy(&PtA, &SlotObj->ptA, sizeof(PtA));
    memcpy(&PtB, &SlotObj->ptB, sizeof(PtB));
    memcpy(&PtC, &SlotObj->ptC, sizeof(PtC));
    memcpy(&PtD, &SlotObj->ptD, sizeof(PtD));
    memcpy(&PtE, &SlotObj->ptE, sizeof(PtE));
    memcpy(&PtF, &SlotObj->ptF, sizeof(PtF));
    memcpy(&old_C, &PtC, sizeof(old_C));
    memcpy(&old_D, &PtD, sizeof(old_D));

    VecAB.vx  = PtB.x - PtA.x;
    VecAB.vy  = PtB.y - PtA.y;
    VecFE.vx  = PtE.x - PtF.x;
    VecFE.vy  = PtE.y - PtF.y;
    vec_CB.vx = PtB.x - PtC.x;
    vec_CB.vy = PtB.y - PtC.y;
    vec_DE.vx = PtE.x - PtD.x;
    vec_DE.vy = PtE.y - PtD.y;

    Normalize_Vec2(&VecAB); //	normalize vector
    Normalize_Vec2(&VecFE);
    Normalize_Vec2(&vec_CB);
    Normalize_Vec2(&vec_DE);
    for (i = 0; i < MidObjNum; i++) //	sort based on the distance
    {
        Get_Dist_Dir_Pt2PointLine(PtB, PtC, MidObj[i].pt1, &dist_B,
                                  NULL); // GetDistAlongAB(PtA,PtB,&MidObj[i][0]);
        Get_Dist_Dir_Pt2PointLine(PtD, PtE, MidObj[i].pt2, &dist_E,
                                  NULL); // GetDistAlongAB(PtF,PtE,&MidObj[i][2]);
        if (dist_B < dist_E)
        {
            memcpy(&MidObj_B[MidObjNum_B++], &MidObj[i], sizeof(LineSeg_T));
        }
        else
        {
            memcpy(&MidObj_E[MidObjNum_E++], &MidObj[i], sizeof(LineSeg_T));
        }
    }
    //	Offfset calculation
    if (MidObjNum_B > 0) //	calculate in sequence
    {
        for (i = 0; i < MidObjNum_B; i++)
        {
            MidObjLen = Get_Segment_Len(MidObj_B[i]);
            //==========calculate AB displacement================//
            if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
            {
                for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        memcpy(&cur_pt, &MidObj_B[i].pt1, sizeof(cur_pt));
                    }
                    else
                    {
                        memcpy(&cur_pt, &MidObj_B[i].pt2, sizeof(cur_pt));
                    }

                    Get_Dist_Dir_Pt2PointLine(PtA, PtB, cur_pt, &sideDist_B, &pt_dir);
                    MidObjOffCurbFlag =
                        -OffCurbDir; // MidObjOffCurbFlag = -OffCurbDir*pt_dir;
                    if (pt_dir == PK_ON_LEFT)
                    {
                        MidObjOffCurbFlag = -MidObjOffCurbFlag;
                    }
                    sideDist_B *= MidObjOffCurbFlag;
                    // OutSideDist_AB_max *= MidObjOffCurbFlag;
                    if (sideDist_B > OutSideDist_AB_max && MidObjLen >= SlotMidObjLen_min)
                    {
                        if ((MidObjOffCurbFlag < 0 &&
                             MidObjLen >= URadar_MinMidObjLen_SmallObjCheck) ||
                            MidObjOffCurbFlag >
                                0) //	move outward directly or move
                                   // inward when the length is long enough
                        {
                            OutSideDist_AB_max = sideDist_B;
                        }
                    }
                }
            }
            else //	AB offset calculation of parallel slot
            {
                for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        memcpy(&cur_pt, &MidObj_B[i].pt1, sizeof(cur_pt));
                    }
                    else
                    {
                        memcpy(&cur_pt, &MidObj_B[i].pt2, sizeof(cur_pt));
                    }
                    Get_Dist_Dir_Pt2PointLine(PtA, PtB, cur_pt, &sideDist_B, &pt_dir);
                    if (Re_OffCurbDir == pt_dir)
                    {
                        if (sideDist_B > OutSideDist_AB_max)
                        {
                            OutSideDist_AB_max = sideDist_B;
                        }
                    }
                }
            }

            //==========BC offset calculation================//

            Get_Dist_Dir_Pt2PointLine(PtA, PtB, MidObj_B[i].pt1, &sideDist_B, NULL);
            //	For parallel slot, obj-to-AB distance must be large than VEHICLE_WID. For
            // vertical slot, middle object length must be larger than a threshold
            if ((sideDist_B < VEHICLE_WID &&
                 (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)) ||
                ((SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT) &&
                 MidObjLen >=
                     SlotMidObjLen_min)) // no side distance requirement for vertical slot
            {
                Get_Dist_Dir_Pt2PointLine(
                    PtB, PtC, MidObj_B[i].pt2, &dist_B,
                    &pt_dir); // GetDistAlongAB(PtA,PtB,&MidObj_B[i][2]);	//	use
                              // further point to calculate side property and use nearer
                              // point to calculate distance
                if (dist_B > max_dist_B && pt_dir == BC_narrowSlotDir)
                {
                    max_dist_B = dist_B;
                }
            }
            else if (sideDist_B >= VEHICLE_WID &&
                     (SlotShap == PK_SLOT_LEFT_PARA ||
                      SlotShap ==
                          PK_SLOT_RIGHT_PARA)) //	line object might be a curb of a wall
            {
                if (CurbNum < MAX_VEH_LINE_NUM)
                {
                    memcpy(&TempCurbLines[CurbNum++], &MidObj_B[i],
                           sizeof(LineSeg_T)); //	collect line objects that might be
                                               // curbs or wall
                }
            }
            // else if(dist_B >= nearDist)// line segments exist in the mid center area
            //{
            //	return 0;
            // }
        }
    }

    if (MidObjNum_E > 0) //	calculate in reverse order
    {
        for (i = MidObjNum_E - 1; i >= 0; i--)
        {
            MidObjLen = Get_Segment_Len(MidObj_E[i]);
            //==========EF Offset calculation================//
            if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
            {
                for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        memcpy(&cur_pt, &MidObj_E[i].pt1, sizeof(cur_pt));
                    }
                    else
                    {
                        memcpy(&cur_pt, &MidObj_E[i].pt2, sizeof(cur_pt));
                    }
                    Get_Dist_Dir_Pt2PointLine(PtE, PtF, cur_pt, &sideDist_E, &pt_dir);
                    MidObjOffCurbFlag =
                        -OffCurbDir; // MidObjOffCurbFlag = -OffCurbDir*pt_dir;
                    if (pt_dir == PK_ON_LEFT)
                    {
                        MidObjOffCurbFlag = -MidObjOffCurbFlag;
                    }
                    sideDist_E *= MidObjOffCurbFlag;
                    if (sideDist_E > OutSideDist_EF_max && MidObjLen >= SlotMidObjLen_min)
                    {
                        if ((MidObjOffCurbFlag < 0 &&
                             MidObjLen >= URadar_MinMidObjLen_SmallObjCheck) ||
                            MidObjOffCurbFlag >
                                0) //	move outward directly or move
                                   // inward when the length is long enough
                        {
                            OutSideDist_EF_max = sideDist_E;
                        }
                    }
                }
            }
            else //	EF offset calculation of parallel slot
            {
                for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        memcpy(&cur_pt, &MidObj_E[i].pt1, sizeof(cur_pt));
                    }
                    else
                    {
                        memcpy(&cur_pt, &MidObj_E[i].pt2, sizeof(cur_pt));
                    }
                    Get_Dist_Dir_Pt2PointLine(PtE, PtF, cur_pt, &sideDist_E, &pt_dir);
                    if (Re_OffCurbDir == pt_dir) //	1st point of the line segment
                    {
                        if (sideDist_E > OutSideDist_EF_max)
                        {
                            OutSideDist_EF_max = sideDist_E;
                        }
                    }
                }
            }
            //==========DE offset calculation================//
            Get_Dist_Dir_Pt2PointLine(PtF, PtE, MidObj_E[i].pt2, &sideDist_E, NULL);
            //	//	For parallel slot, obj-to-AB distance must be large than VEHICLE_WID.
            // For vertical slot, middle object length must be larger than a threshold
            if ((sideDist_E < VEHICLE_WID &&
                 (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)) ||
                ((SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT) &&
                 MidObjLen >=
                     SlotMidObjLen_min)) // no side distance requirement for vertical slot
            {
                Get_Dist_Dir_Pt2PointLine(
                    PtD, PtE, MidObj_E[i].pt1, &dist_E,
                    &pt_dir); // GetDistAlongAB(PtF,PtE,&MidObj_E[i][0]);	//	use
                              // further point to calculate side property and use nearer
                              // point to calculate distance
                if (dist_E > max_dist_E && pt_dir == DE_narrowSlotDir)
                {
                    max_dist_E = dist_E;
                }
            }
            else if (sideDist_E >= VEHICLE_WID &&
                     (SlotShap == PK_SLOT_LEFT_PARA ||
                      SlotShap ==
                          PK_SLOT_RIGHT_PARA)) //	line object might be a curb or wall
            {
                if (CurbNum < MAX_VEH_LINE_NUM)
                {
                    memcpy(&TempCurbLines[CurbNum++], &MidObj_E[i],
                           sizeof(LineSeg_T)); //	collect the line objects
                }
            }
            // else if(dist_E >= nearDist)//	line segments exist in the mid center area
            //{
            //	return 0;
            // }
        }
    }

    if (CurbNum > 0 && g_CurbLinesNum == 0) //	deal with curb objects
    {
        if ((SlotShap == PK_SLOT_LEFT_PARA ||
             SlotShap == PK_SLOT_RIGHT_PARA)) //	calculate curb for parallel slot
        {
            if (CurbPointCal(SlotDir, TempCurbLines, CurbNum, Veh_1_MainLine,
                             Veh_2_MainLine, &CurbStart_Pt,
                             &CurbEnd_Pt)) //	curb calculation correct
            {
                ParalSlotCal_WithCurb_V2(SlotDir, CurbStart_Pt, CurbEnd_Pt,
                                         Veh_1_MainLine, Veh_2_MainLine, TargPos,
                                         SlotObj);
            }
        }
        else
        {
            return 0; //	skip the vertical slot if a curb is found
        }
    }

    //	update target pose
    dist_B = max_dist_B;
    dist_E = max_dist_E;
    if (OutSideDist_AB_max < URadar_ABEFMinOffset) // no translation
    {
        OutSideDist_AB_max = 0;
    }
    if (OutSideDist_EF_max < URadar_ABEFMinOffset) // no translation
    {
        OutSideDist_EF_max = 0;
    }
    if (dist_B > 0 || dist_E > 0)
    {
        //	update slot
        PtA.x += vec_CB.vx * OutSideDist_AB_max;
        PtA.y += vec_CB.vy * OutSideDist_AB_max; //	A only moves in AB direction
        PtB.x += dist_B * VecAB.vx + vec_CB.vx * OutSideDist_AB_max; //	B
        PtB.y += dist_B * VecAB.vy + vec_CB.vy * OutSideDist_AB_max;
        PtC.x += dist_B * VecAB.vx;
        PtC.y += dist_B * VecAB.vy; //		C only moves in AB direction
        PtD.x += dist_E * VecFE.vx;
        PtD.y += dist_E * VecFE.vy;                                  // D
        PtE.x += dist_E * VecFE.vx + vec_DE.vx * OutSideDist_EF_max; // E
        PtE.y += dist_E * VecFE.vy + vec_DE.vy * OutSideDist_EF_max;
        PtF.x += vec_DE.vx * OutSideDist_EF_max;
        PtF.y += vec_DE.vy * OutSideDist_EF_max; //  F only moves in EF direction

        //	calculate the cross point after new points are computed
        Get_CrossPt_PointLine(PtB, PtC, old_C, old_D, &PtC);
        Get_CrossPt_PointLine(PtD, PtE, old_C, old_D, &PtD);
        //	heck if new slot length is within valid range
        GetFootPt_PointAndVecLine(PtB, SlotVec, PtE, &FootPt);
        SlotLen = Cal_Dis_Pt2Pt(PtB, FootPt);
        if (((SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT) &&
             (SlotLen < MIN_PARK_LEN_VERTICAL || SlotLen > MAX_PARK_LEN_VERTICAL)) ||
            ((SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA) &&
             (SlotLen < MIN_PARK_LEN_PARALLEL || SlotLen > MAX_PARK_LEN_PARALLEL)))
        {
            return 0;
        }
        // assign value
        memcpy(&temp_slotobj.ptA, &PtA, sizeof(PtA)); //	A
        memcpy(&temp_slotobj.ptB, &PtB, sizeof(PtB)); //	B
        memcpy(&temp_slotobj.ptC, &PtC, sizeof(PtC)); //	C
        memcpy(&temp_slotobj.ptD, &PtD, sizeof(PtD)); //	D
        memcpy(&temp_slotobj.ptE, &PtE, sizeof(PtE)); //	E
        memcpy(&temp_slotobj.ptF, &PtF, sizeof(PtF)); //	F

        //	update target pose
        if (UpDateTargPos_Trans(&temp_targpos, temp_slotobj, SlotShap) == 0)
        {
            return 0;
        }
        //	check whether new pose interferes with slot
        if (IsTargPosVehInSertSlot(temp_targpos, temp_slotobj))
        {
            return 0;
        }
        // update slot
        memcpy(SlotObj, &temp_slotobj, sizeof(temp_slotobj));
        memcpy(TargPos, &temp_targpos, sizeof(temp_targpos));
    }

    return 1;
}

void Loop_SlotInfo(SlotInfo_T *slot_array, int max_slot_num, SlotInfo_T newslot)
{
    int i;
    for (i = 0; i < max_slot_num - 1; i++)
    {
        memcpy(&slot_array[i], &slot_array[i + 1], sizeof(SlotInfo_T));
    }
    memcpy(&slot_array[max_slot_num - 1], &newslot, sizeof(SlotInfo_T));
}

float CumSumSegmentLen(PK_SlotShapeType slotshap, SlotObj_T *slotobj)
{
    float Cumsum = 0;
    int segment_num, i;
    LineSeg_T segment, segment_EF;
    Point_T slot_E, slot_F;
    if (slotshap == PK_SLOT_LEFT_PARA || slotshap == PK_SLOT_LEFT_VERT)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(&g_FS_RearObsInfo_A);
    }
    else
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(&g_FS_RearObsInfo_A);
    }
    segment_num = g_FS_RearObsInfo_A.num;
    memcpy(&slot_E, &slotobj->ptE, sizeof(slot_E));
    memcpy(&slot_F, &slotobj->ptF, sizeof(slot_F));
    segment_EF.pt1 = slot_E;
    segment_EF.pt2 = slot_F;

    for (i = SF_OBJ_NUM - 1; i > SF_OBJ_NUM - segment_num - 1; i--)
    {
        memcpy(&segment, &g_FS_RearObsInfo_A.obj[i], sizeof(segment));

        Cumsum += LenTwoSegmentsOverlap(segment_EF, segment);
    }

    return Cumsum;
}

void Add_U_Radar_SlotInfo(SlotObj_T *slotobj, VehPos_T *targpos,
                          PK_SlotShapeType slotshap, int slotobj_attr[5])
{
    // check slot if exist in g_Left_U_Slot_Array and g_Right_U_Slot_Array add by
    // xls@2022.5.21
#if 0
    Multi_Slot_Array_T Multi_Slots_Temp;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&Multi_Slots_Temp);
    if (slotshap == PK_SLOT_LEFT_VERT || slotshap == PK_SLOT_LEFT_PARA)
    {
        if(check_slot_exist(slotobj,Multi_Slots_Temp.multiArray,Multi_Slots_Temp.multiNum,slotshap)){//存在返回1，否则返回
            LOGD << "Drop the slot, because it has exist in the g_Left_U_Slot_Array";
            return;
        }
    }else{
        if(check_slot_exist(slotobj,Multi_Slots_Temp.multiArray,Multi_Slots_Temp.multiNum,slotshap)){//存在返回1，否则返回
            LOGD << "Drop the slot, because it has exist in the g_Right_U_Slot_Array";
            return;
        }
    }
#endif

    SlotInfo_T newslot;
    float curpos[4];
    float rx;
    VehPos_T VehPos;
    float Cumsum, LenOfEF;
    Point_T slot_E, slot_F;
    LineSeg_T segment_EF;
    RTE_PK_Location_Get_CurPos(curpos);
    memcpy(&VehPos, curpos, sizeof(VehPos));
    memcpy(&newslot.slotobj, slotobj, sizeof(SlotObj_T));
    memcpy(&newslot.targpos, targpos, sizeof(VehPos_T));
    memcpy(newslot.slotobj_attr, slotobj_attr, 5 * sizeof(int));
    newslot.slotshap       = slotshap;
    newslot.is_vision_slot = 0;
    rx                     = Project_PosTo1stPos_rx(VehPos, *targpos);
    memcpy(&slot_E, &slotobj->ptE, sizeof(slot_E));
    memcpy(&slot_F, &slotobj->ptF, sizeof(slot_F));
    segment_EF.pt1 = slot_E;
    segment_EF.pt2 = slot_F;
    LenOfEF        = Get_Segment_Len(segment_EF);
    Cumsum         = CumSumSegmentLen(slotshap, slotobj);
    if ((rx < URadar_MaxProjRx_SlotCal && rx > URadar_MinProjRx_SlotCal) ||
        Cumsum > 0.5f * LenOfEF) //	make sure the uradar slot won't be detected ahead of a
                                 // vision slot in the same place
    {
        if (slotshap == PK_SLOT_LEFT_VERT || slotshap == PK_SLOT_LEFT_PARA)
        {
            if (g_Left_U_Slot_Num < MAX_U_SLOT_SIDE_NUM)
            {
                memcpy(&g_Left_U_Slot_Array[g_Left_U_Slot_Num], &newslot,
                       sizeof(SlotInfo_T));
                g_Left_U_Slot_Num++;
            }
            else
            {
                Loop_SlotInfo(g_Left_U_Slot_Array, MAX_U_SLOT_SIDE_NUM, newslot);
            }
        }
        else
        {
            if (g_Right_U_Slot_Num < MAX_U_SLOT_SIDE_NUM)
            {
                memcpy(&g_Right_U_Slot_Array[g_Right_U_Slot_Num], &newslot,
                       sizeof(SlotInfo_T));
                g_Right_U_Slot_Num++;
            }
            else
            {
                Loop_SlotInfo(g_Right_U_Slot_Array, MAX_U_SLOT_SIDE_NUM, newslot);
            }
        }
    }
    else
    {
        LOGD << "Didn't add the slot due vehicle is too near or too far from the target "
                "pos to make sure the uradar slot won't be detected ahead of a vision "
                "slot in the same place!";
    }
}
/*****************************************************************************************************************
 * Add Single Boudary U Radar Slot; xls@2022.6.10
 * ***************************************************************************************************************/
void Add_Single_Boundary_Slot(SlotObj_T current_slotobj, VehPos_T current_targpos,
                              PK_SlotShapeType current_slotshape)
{
    // Add_U_Radar_SlotInfo
    SlotInfo_T newslot;
    int current_slotobj_attr[5];
    for (int j = 0; j < 5; j++)
    {
        current_slotobj_attr[j] = 1;
    }
    current_slotobj_attr[2] = 0;

    memcpy(&newslot.slotobj, &current_slotobj, sizeof(SlotObj_T));
    memcpy(&newslot.targpos, &current_targpos, sizeof(VehPos_T));
    memcpy(newslot.slotobj_attr, &current_slotobj_attr, 5 * sizeof(int));
    newslot.slotshap       = current_slotshape;
    newslot.is_vision_slot = 0;
    if (current_slotshape == PK_SLOT_LEFT_VERT || current_slotshape == PK_SLOT_LEFT_PARA)
    {
        if (g_Left_U_Slot_Num < MAX_U_SLOT_SIDE_NUM)
        {
            memcpy(&g_Left_U_Slot_Array[g_Left_U_Slot_Num], &newslot, sizeof(SlotInfo_T));
            g_Left_U_Slot_Num++;
        }
        else
        {
            Loop_SlotInfo(g_Left_U_Slot_Array, MAX_U_SLOT_SIDE_NUM, newslot);
        }
    }
    else
    {
        if (g_Right_U_Slot_Num < MAX_U_SLOT_SIDE_NUM)
        {
            memcpy(&g_Right_U_Slot_Array[g_Right_U_Slot_Num], &newslot,
                   sizeof(SlotInfo_T));
            g_Right_U_Slot_Num++;
        }
        else
        {
            Loop_SlotInfo(g_Right_U_Slot_Array, MAX_U_SLOT_SIDE_NUM, newslot);
        }
    }
}
/*************************************************************************************************/

/*	Searching all the possible ultrasonic radar slots at one side */
void FindSlot_Dir_V3(FusionObj_T *FS_Obj, SLOT_DIR SlotDir)
{
    int Fusion_ObjNum = FS_Obj->num;
    VehPos_T TargPos;
    SlotObj_T SlotObj;
    int SlotObjAttr[5];
    PK_SlotShapeType SlotShap = PK_SLOT_NO; //	1-left parallel, 2-right parallel, 3-left
                                            // vertical, 4-right vertical
    FIND_SLOT_ST SLOT_ST = FIND_SLOT_FIRST_VEHICLE_INIT;
    int i, j;
    LineSeg_T CurLine;
    Point_T CurbStart_Pt, CurbEnd_Pt;
    Relation_T CurVehLineDir;
    //	line direction, 1-left, 2-right, 3-front, 4-back
    float VehTheta_1, VehTheta_2; //	angle of 1st vehicle line and 2nd vehicle line
    LineSeg_T Veh_1_MainLine, Veh_2_MainLine; //	1st vehicle line and 2nd vehicle line
    int SlotCal_Result = 0;                   // resylt：0-Success；1-Error
    int FirstVehDir[MAX_VEH_LINE_NUM],
        SecondVehDir[MAX_VEH_LINE_NUM]; //	vehicle direction set
    LineSeg_T FirstVehLines[MAX_VEH_LINE_NUM],
        SecondVehLines[MAX_VEH_LINE_NUM];          //	vehicle line set
    LineSeg_T MidObjLines[MAX_VEH_LINE_NUM];       //	middle object line set
    int FirstVehLineNum = 0, SecondVehLineNum = 0; //	number of vehicle lines
    int MidObjNum     = 0;                         //	number middle object line
    int FirstVehObjID = 0, SecVehObjID = 0,
        CurVehObjID = 0; //	ID of 1st vehicle line, 2nd vehicle line and current line
    int CurVehPro;       //	line property. 1-close vehicle, 2-curb, 3-far vehicle, 4-5510
    int SecVehStartLineIndex    = 0;    //	Index of line object for searching 2nd vehicle
    float SingleObjLen          = 0;    //	length of a single object
    const float ValidVehLen_min = 0.8f; //	minimum length for a valid vehicle line

    const float ParalVehLen_min = 2.6f; // minimum length for a valid parallel slot
    float TwoLineEndPtDist      = 0;    //	distance of two adjacent lines
    float ToLineDist            = 0;
    int VehLinePro_check        = -1; // checking result of current line
    LineSeg_T LastVehLine;
    HMI_ButtonInfo l_HmiButtonInfo = {0};
    RTE_PK_SM_GetHmiBtInfo(&l_HmiButtonInfo);

    int StartIndex = 0;

    if (SlotDir == SLOT_DIR_LEFT)
    {
        CurVehLineDir = PK_ON_LEFT;
        //	valid_lineseg_dir = 1;
    }
    else
    {
        CurVehLineDir = PK_ON_RIGHT;
        //	valid_lineseg_dir = 2;
    }
    // initialize curb line number
    g_CurbLinesNum = 0;
    StartIndex     = Fusion_ObjNum > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - Fusion_ObjNum;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        // LineDir = FS_Obj->attr[i]/10000;		//	get ten thousand bit
        CurVehPro = (FS_Obj->attr[i] / 1000) %
                    10; //	get thousand bit//
                        // 线段属性：1-近车身(<2600)；2-路沿；3-远车身；4-5510；
        memcpy(&CurLine, &FS_Obj->obj[i], sizeof(CurLine));
        CurVehObjID = FS_Obj->attr[i] % 100; // get the rest number for object ID
        // if (LineDir != valid_lineseg_dir)			//	the segment must be at the
        // side of SlotDir
        //{
        //	continue;
        // }
        switch (SLOT_ST)
        {
            case FIND_SLOT_FIRST_VEHICLE_INIT:
                if (CurVehPro == 1) //	vehicle line found
                {
                    USlotDetect_RunStep_ccp = 24;
                    FirstVehObjID           = CurVehObjID;
                    if (FirstVehLineNum <
                        MAX_VEH_LINE_NUM) //	prevent out-of-range problem
                    {
                        memcpy(&FirstVehLines[FirstVehLineNum], &CurLine,
                               sizeof(CurLine));
                        FirstVehDir[FirstVehLineNum] = FS_Obj->attr[i];
                        FirstVehLineNum++;
                        memcpy(&LastVehLine, &CurLine, sizeof(LastVehLine));
                        //	LastVehLinePro = CurVehPro;
                    }
                    else
                    {
                        VehLinesArrayShiftByOne(FirstVehLines, &CurLine);
                    }
                    SLOT_ST = FIND_SLOT_FIRST_VEHICLE;
                }
                break;

            case FIND_SLOT_FIRST_VEHICLE:
                if (CurVehObjID == FirstVehObjID) // 1st vehicle line found
                {
                    LOGD << "FIND 1st vehicle line!";
                    USlotDetect_RunStep_ccp = 25;
                    if (FirstVehLineNum <
                        MAX_VEH_LINE_NUM) //	prevent out-of-range problem
                    {
                        memcpy(&FirstVehLines[FirstVehLineNum], &CurLine,
                               sizeof(CurLine));
                        FirstVehDir[FirstVehLineNum] = FS_Obj->attr[i];
                        FirstVehLineNum++;
                    }
                    else
                    {
                        VehLinesArrayShiftByOne(FirstVehLines, &CurLine);
                        VehDirArrayShiftByOne(FirstVehDir, FS_Obj->attr[i]);
                    }
                }
                else
                {
                    USlotDetect_RunStep_ccp = 26;
                    // new logic: collect lines that belong to a single vehicle even
                    // though their ID is not the same
                    TwoLineEndPtDist = Cal_Dis_Pt2Pt(
                        FirstVehLines[FirstVehLineNum - 1].pt2, CurLine.pt1);
                    Get_Dist_Dir_Pt2SegLine(FirstVehLines[FirstVehLineNum - 1],
                                            CurLine.pt1, &ToLineDist, NULL);
                    if (TwoLineEndPtDist < URadar_TwoPtDist1 ||
                        (TwoLineEndPtDist < URadar_TwoPtDist2 &&
                         ToLineDist < URadar_MaxPtToLineDist)) //
                    {
                        if (FirstVehLineNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&FirstVehLines[FirstVehLineNum], &CurLine,
                                   sizeof(CurLine));
                            FirstVehDir[FirstVehLineNum] = FS_Obj->attr[i];
                            FirstVehLineNum++;
                        }
                        else
                        {
                            VehLinesArrayShiftByOne(FirstVehLines, &CurLine);
                            VehDirArrayShiftByOne(FirstVehDir, FS_Obj->attr[i]);
                        }
                    }
                    else
                    {
                        TwoLineEndPtDist = Cal_Dis_Pt2Pt(
                            FirstVehLines[0].pt1, FirstVehLines[FirstVehLineNum - 1].pt2);
                        if (TwoLineEndPtDist >
                            ValidVehLen_min) //	length of 1st vehicle line must be larger
                                             // than a threshold
                        {
                            USlotDetect_RunStep_ccp = 27;
                            i--; //	recover the index
                            SLOT_ST = FIND_SLOT_SECOND_VEHICLE_INIT;
                            //	1st vehicle line calculation, not cosidering what type of
                            // the slot
                            GetVehMainLine(FirstVehLines, FirstVehLineNum, FirstVehDir,
                                           &VehTheta_1, &Veh_1_MainLine, SlotDir, 0, 1);
                        }
                        else
                        {
                            USlotDetect_RunStep_ccp = 28;
                            i--;                  //	recover the index
                            g_CurbLinesNum   = 0; //	clear number
                            FirstVehLineNum  = 0;
                            SecondVehLineNum = 0;
                            MidObjNum        = 0; //
                            SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                        }
                    }
                }
                /*******************************************************************************************************
                 *  according to requirement from BYD, if there is no obj on the left or
                 * right add slot to park; xls@2022.6.7
                 * ******************************************************************************************************/
                if (l_HmiButtonInfo.HmiSinglrSlotSta)
                {
                    if (i == SF_OBJ_NUM - 1 || SLOT_ST == FIND_SLOT_SECOND_VEHICLE_INIT)
                    {
                        GetVehMainLine(FirstVehLines, FirstVehLineNum, FirstVehDir,
                                       &VehTheta_1, &Veh_1_MainLine, SlotDir, 9999,
                                       1); // 9999 can force to use average as direction
                        PK_SlotShapeType current_slotshape;
                        SlotObj_T current_slotobj;
                        VehPos_T current_targpos;
                        int check_space_result = check_vehicle_space(
                            Veh_1_MainLine, SlotDir, &current_slotshape, &current_slotobj,
                            &current_targpos);
                        if (check_space_result == 1)
                        {
                            Add_Single_Boundary_Slot(current_slotobj, current_targpos,
                                                     current_slotshape);
                        }
                        // 以下部分为检测车辆左侧车辆的左侧或右侧车辆的右侧是否右足够空间停车
                        int check_other_space_result = check_vehicle_other_space(
                            Veh_1_MainLine, SlotDir, &current_slotshape, &current_slotobj,
                            &current_targpos);
                        if (check_other_space_result == 1)
                        {
                            Add_Single_Boundary_Slot(current_slotobj, current_targpos,
                                                     current_slotshape);
                        }
                    }
                }
                /*******************************************************************************************************/
                break;

            case FIND_SLOT_SECOND_VEHICLE_INIT:
                if (CurVehPro == 1) // vehicle line found
                {
                    VehLinePro_check =
                        CheckVehLinePro(Veh_1_MainLine, CurLine, CurVehLineDir);
                    if (VehLinePro_check ==
                        2) //	last vehicle line is correct while current vehicle line is
                           // wrong. skip current line and keep searching.
                    {
                        if (g_CurbLinesNum < CURB_LINE_NUM_MAX)
                        {
                            memcpy(&g_CurbLines[g_CurbLinesNum++], &CurLine,
                                   sizeof(CurLine)); //	add current line to curb buffer
                        }
                    }
                    else if (VehLinePro_check ==
                             1) //	last vehicle line is wrong while current vehicle line
                                // is correct, so use current line as 1st vehicle line and
                                // keep searching.
                    {
                        i--;
                        SLOT_ST = FIND_SLOT_FIRST_VEHICLE_INIT;
                    }
                    else if (VehLinePro_check == 3)
                    {
                        if (MidObjNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&MidObjLines[MidObjNum], &CurLine, sizeof(CurLine));
                            MidObjNum++;
                        }
                    }
                    else if (VehLinePro_check == 4)
                    {
                        if (FirstVehLineNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&FirstVehLines[FirstVehLineNum], &CurLine,
                                   sizeof(CurLine));
                            FirstVehDir[FirstVehLineNum] = FS_Obj->attr[i];
                            FirstVehLineNum++;
                        }
                        else
                        {
                            VehLinesArrayShiftByOne(FirstVehLines, &CurLine);
                            VehDirArrayShiftByOne(FirstVehDir, FS_Obj->attr[i]);
                        }
                        GetVehMainLine(FirstVehLines, FirstVehLineNum, FirstVehDir,
                                       &VehTheta_1, &Veh_1_MainLine, SlotDir, 0, 1);
                    }
                    else if (VehLinePro_check == 5)
                    {
                        USlotDetect_RunStep_ccp = 47;
                        SecVehStartLineIndex    = i;
                        SecVehObjID             = CurVehObjID;
                        if (SecondVehLineNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&SecondVehLines[SecondVehLineNum], &CurLine,
                                   sizeof(CurLine));
                            SecondVehDir[SecondVehLineNum] = FS_Obj->attr[i];
                            SecondVehLineNum++;
                        }
                        if (i == SF_OBJ_NUM - 1)
                        {
                            SingleObjLen =
                                Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                              SecondVehLines[SecondVehLineNum - 1].pt2);
                            if (SingleObjLen >
                                ValidVehLen_min) //	if length is greater than the
                                                 // threshold, then enter next stage
                            {
                                i--; //	go to final computation stage
                                SLOT_ST = FIND_SLOT_CONFIRM;
                            }
                        }
                        else
                        {
                            SLOT_ST = FIND_SLOT_SECOND_VEHICLE;
                        }
                    }
                }
                else if (CurVehPro == 2 || CurVehPro == 3) // find curb
                {
                    USlotDetect_RunStep_ccp = 31;
                    if (g_CurbLinesNum < CURB_LINE_NUM_MAX)
                    {
                        memcpy(&g_CurbLines[g_CurbLinesNum++], &CurLine, sizeof(CurLine));
                    }
                }

                break;

            case FIND_SLOT_SECOND_VEHICLE:
                USlotDetect_RunStep_ccp = 49;
                if (CurVehObjID == SecVehObjID)
                {
                    LOGD << "FIND 2nd vehicle line!";
                    USlotDetect_RunStep_ccp = 50;
                    if (SecondVehLineNum < MAX_VEH_LINE_NUM)
                    {
                        memcpy(&SecondVehLines[SecondVehLineNum], &CurLine,
                               sizeof(CurLine));
                        SecondVehDir[SecondVehLineNum] = FS_Obj->attr[i];
                        SecondVehLineNum++;
                    }
                    if (i == SF_OBJ_NUM - 1 &&
                        Fusion_ObjNum >
                            0) //	enter final stage when current line is the last line.
                    {
                        SingleObjLen =
                            Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                          SecondVehLines[SecondVehLineNum - 1].pt2);
                        if (SingleObjLen >
                            ValidVehLen_min) //	check whether length is long enough
                        {
                            i--; //	go to final stage
                            SLOT_ST = FIND_SLOT_CONFIRM;
                        }
                    }
                }
                else
                {
                    // new logic: collect lines that belong to a single vehicle even
                    // though their ID is not the same
                    TwoLineEndPtDist = Cal_Dis_Pt2Pt(
                        SecondVehLines[SecondVehLineNum - 1].pt2, CurLine.pt1);
                    Get_Dist_Dir_Pt2SegLine(SecondVehLines[SecondVehLineNum - 1],
                                            CurLine.pt1, &ToLineDist, NULL);
                    if (TwoLineEndPtDist < URadar_TwoPtDist1 ||
                        (TwoLineEndPtDist < URadar_TwoPtDist3 &&
                         ToLineDist <
                             URadar_MaxPtToLineDist)) //	dividing one line segment into
                                                      // two parts will produce a gap
                                                      // between them. Two line segments
                                                      // with a gap shorter than 0.5m will
                                                      // be concatenated.
                    {
                        if (SecondVehLineNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&SecondVehLines[SecondVehLineNum], &CurLine,
                                   sizeof(CurLine));
                            SecondVehDir[SecondVehLineNum] = FS_Obj->attr[i];
                            SecondVehLineNum++;
                        }
                        if (i == SF_OBJ_NUM - 1 &&
                            Fusion_ObjNum > 0) //	when current line is the last one
                        {
                            SingleObjLen =
                                Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                              SecondVehLines[SecondVehLineNum - 1].pt2);
                            if (SingleObjLen > ValidVehLen_min) //	check the length
                            {
                                i--; //	go to final stage
                                SLOT_ST = FIND_SLOT_CONFIRM;
                            }
                        }
                    }
                    else
                    {
                        USlotDetect_RunStep_ccp = 51;
                        i--; // recover the index
                        SingleObjLen =
                            Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                          SecondVehLines[SecondVehLineNum - 1].pt2);
                        if (SingleObjLen >
                            ValidVehLen_min) //	check the length for 2nd vehicle line
                        {
                            USlotDetect_RunStep_ccp = 52;
                            SLOT_ST                 = FIND_SLOT_CONFIRM;
                        }
                        else
                        {
                            USlotDetect_RunStep_ccp = 53;
                            if (MidObjNum + SecondVehLineNum <
                                MAX_VEH_LINE_NUM) //	consider current line as a mid
                                                  // object
                            {
                                memcpy(&MidObjLines[MidObjNum], SecondVehLines,
                                       SecondVehLineNum * sizeof(LineSeg_T));
                                MidObjNum += SecondVehLineNum;
                            }

                            SecondVehLineNum = 0;
                            SLOT_ST          = FIND_SLOT_SECOND_VEHICLE_INIT;
                        }
                    }
                }
                break;

            case FIND_SLOT_CONFIRM:
                USlotDetect_RunStep_ccp = 54;
                LOGD << "Last-Check to comfirm slot!";
                // 2nd vehicle line length must be long enough
                if (Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                  SecondVehLines[SecondVehLineNum - 1].pt2) <
                    ValidVehLen_min)
                {
                    LOGD << "DROPPED the slot because 2nd vehicle line is too "
                            "short(<0.8)!";
                    USlotDetect_RunStep_ccp = 38;
                    i--;                  //	recover the index
                    g_CurbLinesNum   = 0; //	clear
                    FirstVehLineNum  = 0;
                    SecondVehLineNum = 0;
                    MidObjNum        = 0; //
                    SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                    continue;
                }
                //	slot length must be long enough
                if (Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                  FirstVehLines[FirstVehLineNum - 1].pt2) < 2.0f)
                {
                    LOGD << "DROPPED the slot because slot length is too short(<2.0)! "
                            "Actually:"
                         << Cal_Dis_Pt2Pt(SecondVehLines[0].pt1,
                                          FirstVehLines[FirstVehLineNum - 1].pt2);
                    USlotDetect_RunStep_ccp = 39;
                    i = SecVehStartLineIndex - 1; //	start from 2nd vehicle line set
                    g_CurbLinesNum   = 0;         //	clear
                    FirstVehLineNum  = 0;
                    SecondVehLineNum = 0;
                    MidObjNum        = 0;
                    SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                    continue;
                }
                USlotDetect_RunStep_ccp = 55;

                // calculate 2nd vehicle line while not considering what the slot type is,
                // i.e., vertical or parallel.
                GetVehMainLine(SecondVehLines, SecondVehLineNum, SecondVehDir,
                               &VehTheta_2, &Veh_2_MainLine, SlotDir, 0, 2);

                //	check whether middle objects are curbs
                CheckMidObjIsCurb(Veh_1_MainLine, MidObjLines, &MidObjNum, g_CurbLines,
                                  &g_CurbLinesNum);

                // filter noises in curb lines
                FilterCurbNoiseBasedOnPosition(Veh_1_MainLine, Veh_2_MainLine,
                                               MidObjLines, &MidObjNum, g_CurbLines,
                                               &g_CurbLinesNum);

                //	curb calculation
                if (g_CurbLinesNum > 0 &&
                    CurbPointCal(SlotDir, g_CurbLines, g_CurbLinesNum, Veh_1_MainLine,
                                 Veh_2_MainLine, &CurbStart_Pt, &CurbEnd_Pt) ==
                        0) //   use average angle as the direction of curb if there are
                           //   multiple curb lines.
                {
                    USlotDetect_RunStep_ccp = 57;

                    // if curb calculation fail then save curb object to middle object
                    // buffer
                    for (j = 0; j < g_CurbLinesNum; j++)
                    {
                        if (MidObjNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&MidObjLines[MidObjNum], &g_CurbLines[j],
                                   sizeof(LineSeg_T));
                            MidObjNum++;
                        }
                    }
                    g_CurbLinesNum = 0;
                }

                //	parallel slot calculatin
                GetVehMainLine(FirstVehLines, FirstVehLineNum, FirstVehDir, &VehTheta_1,
                               &Veh_1_MainLine, SlotDir, VEHICLE_LEN,
                               1); //	calculate 1st vehicle line
                GetVehMainLine(SecondVehLines, SecondVehLineNum, SecondVehDir,
                               &VehTheta_2, &Veh_2_MainLine, SlotDir, VEHICLE_LEN,
                               2); //	calculate 2nd vehicle line
                // TODO:  adjust vehicle main line accordint to the envelope diagram of
                // song plus URadar which collected by Long Yunxiang
                LOGD << "Before adjust Veh_1_MainLine pt1:(" << Veh_1_MainLine.pt1.x
                     << "," << Veh_1_MainLine.pt1.y << ")";
                LOGD << "Before adjust Veh_1_MainLine pt2:(" << Veh_1_MainLine.pt2.x
                     << "," << Veh_1_MainLine.pt2.y << ")";
                LOGD << "Before adjust Veh_2_MainLine pt1:(" << Veh_2_MainLine.pt1.x
                     << "," << Veh_2_MainLine.pt1.y << ")";
                LOGD << "Before adjust Veh_2_MainLine pt2:(" << Veh_2_MainLine.pt2.x
                     << "," << Veh_2_MainLine.pt2.y << ")";
                adjust_veh_1_mainline(&Veh_1_MainLine);
                adjust_veh_2_mainline(&Veh_2_MainLine);
                LOGD << "After adjust Veh_1_MainLine pt1:(" << Veh_1_MainLine.pt1.x << ","
                     << Veh_1_MainLine.pt1.y << ")";
                LOGD << "After adjust Veh_1_MainLine pt2:(" << Veh_1_MainLine.pt2.x << ","
                     << Veh_1_MainLine.pt2.y << ")";
                LOGD << "After adjust Veh_2_MainLine pt1:(" << Veh_2_MainLine.pt1.x << ","
                     << Veh_2_MainLine.pt1.y << ")";
                LOGD << "After adjust Veh_2_MainLine pt2:(" << Veh_2_MainLine.pt2.x << ","
                     << Veh_2_MainLine.pt2.y << ")";

                if (Get_Segment_Len(Veh_1_MainLine) > ParalVehLen_min &&
                    Get_Segment_Len(Veh_2_MainLine) >
                        ParalVehLen_min) //	check the length of both vehicle lines
                {
                    if (g_CurbLinesNum > 0)
                    {
                        SlotCal_Result = ParalSlotCal_WithCurb_V2(
                            SlotDir, CurbStart_Pt, CurbEnd_Pt, Veh_1_MainLine,
                            Veh_2_MainLine, &TargPos, &SlotObj);
                    }
                    else
                    {
                        SlotCal_Result = ParalSlotCal_NoCurb_V2(
                            SlotDir, Veh_1_MainLine, Veh_2_MainLine, &TargPos, &SlotObj);
                    }
                }
                if (SlotCal_Result == -1) //	wrong result. go back to initial stage
                {
                    USlotDetect_RunStep_ccp = 40;
                    i                       = SecVehStartLineIndex -
                        1;                //	start from the index of 2nd vehicle lines
                    g_CurbLinesNum   = 0; //	clear
                    FirstVehLineNum  = 0;
                    SecondVehLineNum = 0;
                    MidObjNum        = 0;
                    SlotCal_Result   = 0;
                    SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                    continue;
                }
                else if (SlotCal_Result == 1) //	parallel slot
                {

                    USlotDetect_RunStep_ccp = 41;

                    if (SlotDir == SLOT_DIR_LEFT)
                    {
                        SlotShap = PK_SLOT_LEFT_PARA;
                    }
                    else
                    {
                        SlotShap = PK_SLOT_RIGHT_PARA;
                    }

                    if (SlotShap !=
                        PK_SLOT_NO) //	check whether vehicle interfere with obstacles
                    { /* due to adjustment of mainline comment out by xls@2022.5.16
                      if (MidObjNum > 0 &&
                      SlotCheckForSmallObj(&SlotObj,&TargPos,MidObjLines,
                      MidObjNum,SlotShap, Veh_1_MainLine, Veh_2_MainLine) == 0)
                      {
                          LOGD << "DROPPED the parallel slot because  interfere with
                      obstacles!"; USlotDetect_RunStep_ccp = 58; SlotShap = PK_SLOT_NO;
                      }
                      if(SlotShap != PK_SLOT_NO &&
                      IsTargPosInsertFusionObj_A(TargPos,FS_Obj->obj, FS_Obj->attr,
                      Fusion_ObjNum, SlotShap) == 1)
                      {
                          LOGD << "DROPPED the parallel slot because  interfere with
                      obstacles!"; USlotDetect_RunStep_ccp = 59; SlotShap = PK_SLOT_NO;
                      }	*/
                        if (SlotShap != PK_SLOT_NO)
                        {
                            for (j = 0; j < 5; j++)
                            {
                                SlotObjAttr[j] = 1; //	parallel slot with curb
                            }
                            if (g_CurbLinesNum == 0)
                            {
                                SlotObjAttr[2] = 0; //	parallel slot without curb
                            }
                            if (Rear_Side_Radar_Check_Slot(TargPos, SlotShap) == 0)
                            {
                                Add_U_Radar_SlotInfo(&SlotObj, &TargPos, SlotShap,
                                                     SlotObjAttr);
                            }
                        }
                        if (i != SF_OBJ_NUM - 1) // 20171009： check whether current line
                                                 // is the last line
                        {
                            i = SecVehStartLineIndex -
                                1; //	start from the index of second vehicle
                        }
                    }
                }
                else if (SlotCal_Result == 0) //	vertical slot calculation
                {
                    //	calculate 1st vehicle line
                    GetVehMainLine(FirstVehLines, FirstVehLineNum, FirstVehDir,
                                   &VehTheta_1, &Veh_1_MainLine, SlotDir, VEHICLE_WID, 1);
                    //	calculate 2nd vehicle line
                    GetVehMainLine(SecondVehLines, SecondVehLineNum, SecondVehDir,
                                   &VehTheta_2, &Veh_2_MainLine, SlotDir, VEHICLE_WID, 2);
                    // TODO:  adjust vehicle main line accordint to the envelope diagram
                    // of song plus URadar which collected by long yunxiang
                    LOGD << "Before adjust Veh_1_MainLine pt1:(" << Veh_1_MainLine.pt1.x
                         << "," << Veh_1_MainLine.pt1.y << ")";
                    LOGD << "Before adjust Veh_1_MainLine pt2:(" << Veh_1_MainLine.pt2.x
                         << "," << Veh_1_MainLine.pt2.y << ")";
                    LOGD << "Before adjust Veh_2_MainLine pt1:(" << Veh_2_MainLine.pt1.x
                         << "," << Veh_2_MainLine.pt1.y << ")";
                    LOGD << "Before adjust Veh_2_MainLine pt2:(" << Veh_2_MainLine.pt2.x
                         << "," << Veh_2_MainLine.pt2.y << ")";
                    adjust_veh_1_mainline(&Veh_1_MainLine);
                    adjust_veh_2_mainline(&Veh_2_MainLine);
                    LOGD << "After adjust Veh_1_MainLine pt1:(" << Veh_1_MainLine.pt1.x
                         << "," << Veh_1_MainLine.pt1.y << ")";
                    LOGD << "After adjust Veh_1_MainLine pt2:(" << Veh_1_MainLine.pt2.x
                         << "," << Veh_1_MainLine.pt2.y << ")";
                    LOGD << "After adjust Veh_2_MainLine pt1:(" << Veh_2_MainLine.pt1.x
                         << "," << Veh_2_MainLine.pt1.y << ")";
                    LOGD << "After adjust Veh_2_MainLine pt2:(" << Veh_2_MainLine.pt2.x
                         << "," << Veh_2_MainLine.pt2.y << ")";

                    if (g_CurbLinesNum > 0) // move curb lines to midobj buffer
                    {
                        if (MidObjNum + g_CurbLinesNum < MAX_VEH_LINE_NUM)
                        {
                            memcpy(&MidObjLines[MidObjNum], g_CurbLines,
                                   g_CurbLinesNum * sizeof(LineSeg_T));
                            MidObjNum += g_CurbLinesNum;
                        }
                    }
                    SlotCal_Result = VertSlotCal_NoCurb_V2(
                        SlotDir, Veh_1_MainLine, Veh_2_MainLine, &TargPos, &SlotObj);
                    if (SlotCal_Result == 0)
                    {
                        LOGD << "DROPPED the vertical slot because VertSlotCal_NoCurb_V2 "
                                "ERROR!";
                        USlotDetect_RunStep_ccp = 43;
                        i                       = SecVehStartLineIndex -
                            1;                //	start from the index of second vehicle
                        g_CurbLinesNum   = 0; //	clear
                        FirstVehLineNum  = 0;
                        SecondVehLineNum = 0;
                        MidObjNum        = 0;
                        SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                        continue;
                    }
                    else
                    {
                        USlotDetect_RunStep_ccp = 44;
                        if (SlotDir == SLOT_DIR_LEFT)
                        {
                            SlotShap = PK_SLOT_LEFT_VERT;
                        }
                        else
                        {
                            SlotShap = PK_SLOT_RIGHT_VERT;
                        }

                        if (SlotShap != PK_SLOT_NO) //	update slot according to obstacle
                                                    // and check interference
                        {
                            USlotDetect_RunStep_ccp = 45;
                            /* due to adjustment of mainline comment out by xls@2022.5.16
                            if (MidObjNum > 0 &&
                            SlotCheckForSmallObj(&SlotObj,&TargPos,MidObjLines,
                            MidObjNum,SlotShap, Veh_1_MainLine, Veh_2_MainLine) == 0)
                            {
                                LOGD << "DROPPED the vertical slot because  interfere with
                            obstacles!"; SlotShap = PK_SLOT_NO;
                            }
                            if(SlotShap != PK_SLOT_NO &&
                            IsTargPosInsertFusionObj_A(TargPos,FS_Obj->obj, FS_Obj->attr,
                            Fusion_ObjNum, SlotShap) == 1)
                            {
                                LOGD << "DROPPED the vertical slot because  interfere with
                            obstacles!"; USlotDetect_RunStep_ccp = 46; SlotShap =
                            PK_SLOT_NO;
                            }*/
                            if (SlotShap != PK_SLOT_NO)
                            {
                                for (j = 0; j < 5; j++)
                                {
                                    SlotObjAttr[j] = 1;
                                }
                                SlotObjAttr[2] = 0;
                                if (Rear_Side_Radar_Check_Slot(TargPos, SlotShap) == 0)
                                {
                                    Add_U_Radar_SlotInfo(&SlotObj, &TargPos, SlotShap,
                                                         SlotObjAttr);
                                }
                            }

                            if (i !=
                                SF_OBJ_NUM - 1) // 20171009：	if this is the last line
                                                // then there is no need to recover the
                                                // index to the start of 2nd vehicle lines
                            {
                                i = SecVehStartLineIndex -
                                    1; // start from the 2nd vehicle index
                            }
                        }
                    }
                }

                g_CurbLinesNum   = 0; //	clear
                FirstVehLineNum  = 0;
                SecondVehLineNum = 0;
                MidObjNum        = 0;
                SlotCal_Result   = 0;
                SLOT_ST          = FIND_SLOT_FIRST_VEHICLE_INIT;
                break;
            default:
                break;
        }
    }
}

// check relationship between the input line segment and curb line
// return: -1 wrong line,ignore current line; 0- curb line; 1- vehicle line; 2- ignore
// curb lines,start from current line
int CheckLinePro_Curb(LineSeg_T curb_line, LineSeg_T segment_line, SLOT_DIR SlotDir)
{
    float rx, ry, ry1, ry2, dy;
    VehPos_T curb_pos;
    const float sameObj_SideDist    = 0.8f;
    const float curb_veh_side_dist  = 1.6f;
    const float curb_veh_x_dist_max = 2.0f; //
    Convert_Line_to_Pos(curb_line, &curb_pos);
    rx =
        Project_PointTo1stPos_rx(curb_pos, segment_line.pt1) - Get_Segment_Len(curb_line);
    ry1 = Project_PointTo1stPos_ry(curb_pos, segment_line.pt1);
    ry2 = Project_PointTo1stPos_ry(curb_pos, segment_line.pt2);
    ry  = (ry1 + ry2) / 2.0f;
    dy  = fabsf(ry);
    if (rx > curb_veh_x_dist_max)
    {
        return 2;
    }
    else
    {
        if (dy < sameObj_SideDist)
        {
            return 0;
        }
        else
        {
            if (dy > curb_veh_side_dist && ((SlotDir == SLOT_DIR_LEFT && ry < 0) ||
                                            (SlotDir == SLOT_DIR_RIGHT && ry > 0)))
            {
                return 1;
            }
            else
            {
                return -1;
            }
        }
    }
}

// check relationship between the input line segment and vehicle line
// return: -1 wrong line,ignore current line, start from next line; 0- vehicle line; 1-
// curb line; 2- ignore vehicle lines,start from current line
int CheckLinePro_Vehicle(LineSeg_T veh_line, LineSeg_T segment_line, SLOT_DIR SlotDir)
{
    float rx, ry, ry1, ry2, dy;
    VehPos_T veh_pos;
    const float sameObj_SideDist    = 0.8f;
    const float curb_veh_side_dist  = 1.6f;
    const float curb_veh_x_dist_max = 2.0f; //
    Convert_Line_to_Pos(veh_line, &veh_pos);
    rx  = Project_PointTo1stPos_rx(veh_pos, segment_line.pt1) - Get_Segment_Len(veh_line);
    ry1 = Project_PointTo1stPos_ry(veh_pos, segment_line.pt1);
    ry2 = Project_PointTo1stPos_ry(veh_pos, segment_line.pt2);
    ry  = (ry1 + ry2) / 2.0f;
    dy  = fabsf(ry);
    if (rx > curb_veh_x_dist_max)
    {
        return 2;
    }
    else
    {
        if (dy < sameObj_SideDist)
        {
            return 0;
        }
        else
        {
            if (dy > curb_veh_side_dist && ((SlotDir == SLOT_DIR_LEFT && ry > 0) ||
                                            (SlotDir == SLOT_DIR_RIGHT && ry < 0)))
            {
                return 1;
            }
            else
            {
                return -1;
            }
        }
    }
}

#if 0
//	Detect slots with one car and curb
void Find_SingleVeh_Slot(FusionObj_T* FS_Obj,SLOT_DIR SlotDir)
{
	int Fusion_ObjNum = FS_Obj->num;
	VehPos_T TargPos;
	SlotObj_T SlotObj;

	PK_SlotShapeType SlotShap = PK_SLOT_NO;//	1-左平行；2-右平行；3-左垂直；4-右垂直；平行车位与RearLine_Pro，垂直车位与VirtLine_Pro刚好对应上
	FIND_SLOT_ST SLOT_ST = FIND_SLOT_FIRST_VEHICLE_INIT;
	int i;
	LineSeg_T CurLine;
	Point_T CurbStart_Pt, CurbEnd_Pt;
	Relation_T CurVehLineDir;
	int LineDir;	//	线段方位：1-左；2-右；3-前；4-后；
	float VehTheta_1,VehTheta_2;	//	前后两车的方向角
	LineSeg_T Veh_1_MainLine,Veh_2_MainLine;	//	前后两车的计算线段
	int SlotCal_Result = 0; // 车位计算结果：0-正常；1-异常；
	int VehDir[MAX_VEH_LINE_NUM];	//	车辆轮廓线段属性集合
	LineSeg_T VehLines[MAX_VEH_LINE_NUM];	//	车辆轮廓的线段集合
	LineSeg_T CurbLines[MAX_VEH_LINE_NUM];	//	车位中间出现的障碍物线段
	int VehLineNum = 0;	//	车辆轮廓线段数量
	int CurbLineNum = 0;	//	车位中间出现的障碍物线段
	int VehObjID,CurbObjID,CurID;	//	当前车身线段的编号（同一个车身聚类有同一个编号）
	int CurVehPro;	//	线段属性：1-近车身；2-路沿；3-远车身；4-5510；
	int VehStartLineIndex;	//	第二辆车起始索引号
	float SingleObjLen = 0;	//	一个独立物体的长度
	const float ValidVehLen_min = 0.8f;	//	0.8m以内的独立物体
	const float ValidCurbLen_min = 0.6f;	//	认为路沿存在的最小长度
	const float ParalVehLen_min = 2.6f; // 构成平行车位的车辆最小长度
	float TwoLineEndPtDist = 0;	//	两条直线的首末点距离
	float ToLineDist = 0;
	int LinePro_check = -1; // 当前车身线段属性的check结果
	LineSeg_T LastObjLine;
	int LastLinePro = -1;
	int StartIndex = 0;
	int valid_lineseg_dir = 0;
	LineSeg_T CurbSeg,VehSeg;
	float curb_len,vehicle_len;
	const float min_len_cal = 1.0f;	//	the minimum dist of curb or vehicle to judge other segment line
	const float curb_len_valid = 2.5f;	//	the minimun dist of curb to detect a slot
	const float veh_len_valid = 3.0f;	//	the minimun dist of vehicle to detect a slot

	if (SlotDir == SLOT_DIR_LEFT)
	{
		CurVehLineDir = PK_ON_LEFT;
		valid_lineseg_dir = 1;
	}
	else
	{
		CurVehLineDir = PK_ON_RIGHT;
		valid_lineseg_dir = 2;
	}
	// 路沿线段初始化
	g_CurbLinesNum = 0;
	for(i = 0; i < Fusion_ObjNum; i++)
	{
		LineDir = FS_Obj->attr[i]/10000;		//	取万位
		CurVehPro = (FS_Obj->attr[i]/1000)%10;	//	取千位
		memcpy(&CurLine,&FS_Obj->obj[i],sizeof(CurLine));
		CurID = FS_Obj->attr[i] % 100;		// 求余十百位,得到物体编号
		if (LineDir != valid_lineseg_dir)			//	the segment must be at the side of SlotDir
		{
			continue;
		}
		switch (SLOT_ST)
		{
		case FIND_CURB_OR_VEHICLE_INIT:
			// do the init
			CurbLineNum = 0;
			VehLineNum = 0;
			vehicle_len = 0;
			curb_len = 0;
			if(CurVehPro == 1)	//	找到车身线段
			{
				USlotDetect_RunStep_ccp = 24;
				VehObjID = CurID;
				SLOT_ST = FIND_VEHICLE;
			}
			else if (CurVehPro == 3)
			{
				CurbObjID = CurID;
				SLOT_ST = FIND_CURB;
			}
			break;

		case FIND_CURB:
			if (CurID == CurbObjID)
			{
				if (CurbLineNum < MAX_VEH_LINE_NUM)
				{
					memcpy(&CurbLines[CurbLineNum],&CurLine,sizeof(CurLine));
					VehDir[CurbLineNum] = FS_Obj->attr[i];
					CurbLineNum++;
					memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
					LastLinePro = CurVehPro;
				}
				else
				{
					VehLinesArrayShiftByOne(CurbLines,&CurLine);
				}
				curb_len = Curb_SegLineCal(SlotDir,CurbLines,CurbLineNum,&CurbSeg);
				if (curb_len > curb_len_valid && vehicle_len > veh_len_valid)	//	find a vehicle before
				{
					i--;
					SLOT_ST = FIND_VEHICLE_CURB_SLOT;
				}
			}
			else
			{
				if (curb_len > min_len_cal)	//	curb line segment is long enough to judge other line segments
				{
					LinePro_check = CheckLinePro_Curb(CurbSeg,CurLine,SlotDir);
					if (LinePro_check == 0)
					{
						if (CurbLineNum < MAX_VEH_LINE_NUM)
						{
							memcpy(&CurbLines[CurbLineNum],&CurLine,sizeof(CurLine));
							VehDir[CurbLineNum] = FS_Obj->attr[i];
							CurbLineNum++;
							memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
							LastLinePro = CurVehPro;
						}
						else
						{
							VehLinesArrayShiftByOne(CurbLines,&CurLine);
						}
						curb_len = Curb_SegLineCal(SlotDir,CurbLines,CurbLineNum,&CurbSeg);
					}
					else if (LinePro_check == 1)
					{
						if (curb_len > curb_len_valid)
						{
							if (VehLineNum == 0)	//	first find the curb
							{
								VehObjID = CurID;
								memcpy(&VehLines[VehLineNum],&CurLine,sizeof(CurLine));
								VehDir[VehLineNum] = FS_Obj->attr[i];
								VehLineNum++;
								memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
								LastLinePro = CurVehPro;
								SLOT_ST = FIND_VEHICLE;
							}
							else	//	first find the vehicle
							{
								vehicle_len = Curb_SegLineCal( SlotDir,VehLines,VehLineNum,&VehSeg);
								if (vehicle_len > veh_len_valid)
								{
									i--;
									SLOT_ST = FIND_VEHICLE_CURB_SLOT;
								}
								else
								{
									SLOT_ST = FIND_VEHICLE;
								}
							}
						}
						else
						{
							i--;
							SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
						}
					}
					else if(LinePro_check == 2)
					{
						i--;
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
					else
					{
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
				}
				else	//	the curb segment is not long enough,we can only use the property of current line segment to judge
				{
					if(CurVehPro == 3)
					{
						if (CurbLineNum < MAX_VEH_LINE_NUM)
						{
							memcpy(&CurbLines[CurbLineNum],&CurLine,sizeof(CurLine));
							VehDir[CurbLineNum] = FS_Obj->attr[i];
							CurbLineNum++;
							memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
							LastLinePro = CurVehPro;
						}
						else
						{
							VehLinesArrayShiftByOne(CurbLines,&CurLine);
						}
						curb_len = Curb_SegLineCal(SlotDir,CurbLines,CurbLineNum,&CurbSeg);
					}
					else if (CurVehPro == 1)
					{
						if (curb_len > curb_len_valid)
						{
							VehObjID = CurID;
							if (VehLineNum < MAX_VEH_LINE_NUM)	//	防止越界
							{
								memcpy(&VehLines[VehLineNum],&CurLine,sizeof(CurLine));
								VehDir[VehLineNum] = FS_Obj->attr[i];
								VehLineNum++;
								memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
								LastLinePro = CurVehPro;
							}
							else
							{
								VehLinesArrayShiftByOne(VehLines,&CurLine);
							}
							SLOT_ST = FIND_VEHICLE;
						}
						else
						{
							i--;
							SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
						}
					}
				}
			}
			break;

		case FIND_VEHICLE:
			if (CurID == VehObjID)
			{
				if (VehLineNum < MAX_VEH_LINE_NUM)	//	防止越界
				{
					memcpy(&VehLines[VehLineNum],&CurLine,sizeof(CurLine));
					VehDir[VehLineNum] = FS_Obj->attr[i];
					VehLineNum++;
					memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
					LastLinePro = CurVehPro;
				}
				else
				{
					VehLinesArrayShiftByOne(VehLines,&CurLine);
				}
				vehicle_len = Curb_SegLineCal( SlotDir,VehLines,VehLineNum,&VehSeg);
				if (vehicle_len > veh_len_valid && curb_len > curb_len_valid)	//	first find a curb
				{
					i--;
					SLOT_ST = FIND_CURB_VEHICLE_SLOT;
				}
			}
			else
			{
				if (vehicle_len > min_len_cal)
				{
					LinePro_check = CheckLinePro_Vehicle(VehSeg,CurLine, SlotDir);
					if (LinePro_check == 0)
					{
						if (VehLineNum < MAX_VEH_LINE_NUM)	//	防止越界
						{
							memcpy(&VehLines[VehLineNum],&CurLine,sizeof(CurLine));
							VehDir[VehLineNum] = FS_Obj->attr[i];
							VehLineNum++;
							memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
							LastLinePro = CurVehPro;
						}
						else
						{
							VehLinesArrayShiftByOne(VehLines,&CurLine);
						}
						vehicle_len = Curb_SegLineCal( SlotDir,VehLines,VehLineNum,&VehSeg);
						if (vehicle_len > veh_len_valid && curb_len > curb_len_valid)	//	first find a curb
						{
							i--;
							SLOT_ST = FIND_CURB_VEHICLE_SLOT;
						}
					}
					else if (LinePro_check == 1)
					{
						if (CurbLineNum > 0)// first find a curb
						{
							if (vehicle_len < veh_len_valid)
							{
								i--;
								SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
							}
							else	//	first find a vehicle
							{
								if (CurbLineNum < MAX_VEH_LINE_NUM)
								{
									memcpy(&CurbLines[CurbLineNum],&CurLine,sizeof(CurLine));
									VehDir[CurbLineNum] = FS_Obj->attr[i];
									CurbLineNum++;
									memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
									LastLinePro = CurVehPro;
								}
								else
								{
									VehLinesArrayShiftByOne(CurbLines,&CurLine);
								}
								CurbObjID = CurID;
								SLOT_ST = FIND_CURB;
							}
						}
					}
					else if (LinePro_check == 2)
					{
						i--;
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
					else
					{
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
				}
				else	//	the vehicle segment is not long enough,we can only use the property of current line segment to judge
				{
					if(CurVehPro == 1)
					{
						if (VehLineNum < MAX_VEH_LINE_NUM)	//	防止越界
						{
							memcpy(&VehLines[VehLineNum],&CurLine,sizeof(CurLine));
							VehDir[VehLineNum] = FS_Obj->attr[i];
							VehLineNum++;
							memcpy(&LastObjLine,&CurLine,sizeof(LastObjLine));
							LastLinePro = CurVehPro;
						}
						else
						{
							VehLinesArrayShiftByOne(VehLines,&CurLine);
						}
						vehicle_len = Curb_SegLineCal( SlotDir,VehLines,VehLineNum,&VehSeg);
						if (vehicle_len > veh_len_valid && curb_len > curb_len_valid)	//	first find a curb
						{
							i--;
							SLOT_ST = FIND_CURB_VEHICLE_SLOT;
						}
					}
					else if (CurVehPro == 3)
					{
						i--;
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
					else
					{
						SLOT_ST = FIND_CURB_OR_VEHICLE_INIT;
					}
				}
			}
			break;

		case FIND_VEHICLE_CURB_SLOT:

			break;

		case FIND_CURB_VEHICLE_SLOT:

			break;

		}
	}
}

#endif

int FilterCurbNoiseBasedOnPosition(LineSeg_T firVehLine, LineSeg_T secVehLine,
                                   LineSeg_T *midObj, int *midObjNum,
                                   LineSeg_T *curbLines, int *curbLineNum)
{
    if (*curbLineNum < 1)
    {
        return 0;
    }

    int i, j = 0;
    float dist = 0.0, refDist = 0.0f, distLenSquare = 0.0f, lenSquare = 0.0f,
          curbLenSqua = 0.0f, refDistInv = 0.0f;
    float curbDist[CURB_LINE_NUM_MAX] = {0.0f};
    LineSeg_T refLine;
    LineSeg_T newCurbLines[CURB_LINE_NUM_MAX];
    Point_T ptSet[4];
    Point_T centerPt;

    memset(curbDist, 0, sizeof(curbDist));
    memset(newCurbLines, 0, sizeof(newCurbLines));

    // cal reference line using min least square
    memcpy(ptSet, &(firVehLine.pt1), sizeof(Point_T));
    memcpy(ptSet + 1, &(firVehLine.pt2), sizeof(Point_T));
    memcpy(ptSet + 2, &(secVehLine.pt1), sizeof(Point_T));
    memcpy(ptSet + 3, &(secVehLine.pt2), sizeof(Point_T));

    if (LineFittingLeastSquare(ptSet, 4, &refLine) == 0)
    {
        return 0;
    }

    for (i = 0; i < *curbLineNum; ++i)
    {
        // cal center point
        centerPt.x = (curbLines[i].pt1.x + curbLines[i].pt2.x) * 0.5;
        centerPt.y = (curbLines[i].pt1.y + curbLines[i].pt2.y) * 0.5;

        Get_Dist_Dir_Pt2SegLine(refLine, centerPt, &dist, NULL);

        // save the distance
        curbDist[i] = dist;

        curbLenSqua = Get_Segment_Len(curbLines[i]);
        curbLenSqua = curbLenSqua * curbLenSqua;

        distLenSquare += curbLenSqua * dist;
        lenSquare += curbLenSqua;
    }

    refDist = distLenSquare / lenSquare;

    // check reference distance validity
    if (refDist < 1e-8f)
    {
        return 0;
    }

    refDistInv = 1 / refDist;

    // Execute filtering
    for (i = 0; i < *curbLineNum; ++i)
    {
        if ((refDist - curbDist[i]) * refDistInv >
            0.2f) // noise that meets the condition: refDist - curbDist / refDist > 20%
                  // and still keep the curb line with a greater distance
        {
            if (*midObjNum < MAX_VEH_LINE_NUM)
            {
                memcpy(midObj + *midObjNum, curbLines + i, sizeof(LineSeg_T));
                *midObjNum = *midObjNum + 1;
            }
        }
        else
        {
            memcpy(newCurbLines + j, curbLines + i, sizeof(LineSeg_T));
            ++j;
        }
    }

    *curbLineNum = j;
    memcpy(curbLines, newCurbLines, sizeof(newCurbLines));

    return 1;
}

RECAL_DIR URadar_SlotDetect(void)
{
    // define local variables
    float curpos[4];
    const float ds_min = 0.5f; //  re-detect the slot if the vehicle moves this distance
    int MoveFlag       = 0;
    RECAL_DIR Cal_dir  = CAL_NO;
    int cal_num        = 0;
    RTE_PK_Location_Get_CurPos(curpos);

    //  start detection when there are new lines added.
    if (fabsf(curpos[3] - g_last_path) > ds_min)
    {
        g_last_path = curpos[3];
        MoveFlag    = 1;
    }

    // init
    memset(&g_Side_FS_Obj, 0, sizeof(g_Side_FS_Obj));
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&g_Side_FS_Obj);
    if (g_Side_FS_Obj.num_accu != L_Last_FS_Num_A ||
        (MoveFlag == 1 && L_Last_FS_Num_A > 0))
    {
        L_Last_FS_Num_A   = g_Side_FS_Obj.num_accu; //  update
        g_Left_U_Slot_Num = 0;                      // clear the number of last detection
        FindSlot_Dir_V3(&g_Side_FS_Obj, SLOT_DIR_LEFT); //  find left slot
        Cal_dir = CAL_LEFT;
        cal_num++;
    }
    // the following two lines of code added by xls@2022.5.20
    Multi_Slot_Array_T Multi_Slots_Temp;
    RTE_PK_SlotDetect_Get_Multi_SlotInfo(&Multi_Slots_Temp);
    FusionObj_T Left_Side_FS_Obj_trimmed =
        get_rid_of_excess_obj(Multi_Slots_Temp.multiArray, Multi_Slots_Temp.multiNum,
                              &g_Side_FS_Obj, TRIM_LEFT);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left(&Left_Side_FS_Obj_trimmed);

    // init
    memset(&g_Side_FS_Obj, 0, sizeof(g_Side_FS_Obj));
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&g_Side_FS_Obj);

    if (g_Side_FS_Obj.num_accu != R_Last_FS_Num_A ||
        (MoveFlag == 1 && R_Last_FS_Num_A > 0))
    {
        R_Last_FS_Num_A    = g_Side_FS_Obj.num_accu; //  update
        g_Right_U_Slot_Num = 0;                      // clear the number of last detection
        FindSlot_Dir_V3(&g_Side_FS_Obj, SLOT_DIR_RIGHT);
        Cal_dir = CAL_RIGHT;
        cal_num++;
    }
    // the following two lines of code added by xls@2022.5.20
    FusionObj_T Right_Side_FS_Obj_trimmed =
        get_rid_of_excess_obj(Multi_Slots_Temp.multiArray, Multi_Slots_Temp.multiNum,
                              &g_Side_FS_Obj, TRIM_RIGHT);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right(&Right_Side_FS_Obj_trimmed);

    if (cal_num == 2)
    {
        Cal_dir = CAL_BOTH;
    }
    return Cal_dir;
}
