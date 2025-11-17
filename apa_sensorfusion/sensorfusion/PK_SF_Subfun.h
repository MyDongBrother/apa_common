/**
 * @file PK_SF_Subfun.h
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
#ifndef PK_SF_SUBFUN_H
#define PK_SF_SUBFUN_H

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "Platform_Types.h"
#include "PK_Utility.h"
#include "PK_SF.h"
#include "PK_SF_A_FreeSpace.h"
/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

/** Version and module identification */
#define GEOMETRY_VENDOR_ID (0u)
#define GEOMETRY_MODULE_ID (0u)

/** Component Version Information */
#define PK_SF_SUBFUN_MAJOR_VERSION (3u)
#define PK_SF_SUBFUN_MINOR_VERSION (0u)
#define PK_SF_SUBFUN_PATCH_VERSION (14u)

extern float sf_ultrasMotorPos[USS_ECHO_INDEX_MAX][4];

#define PI_GEO 3.1415927f

#ifdef __cplusplus
extern "C"
{
#endif

    void sort(float *a, int left, int right);
    float DistToPt(float a, float b, float c, float xt, float yt);
    void Check_IMU_Data(void);

    /*********************************************************************************************************
    **  geometric function using struct defenition
    **  all the functions are copied from the above one and using index 2
    *********************************************************************************************************/
    uint8 LineFitting_LimitedSquare2(Point_T Pt[], int num, Line_T *pline);
    float DistToPt2(Line_T line, Point_T pt);
    uint8 GetNormalPt2(Line_T line, Point_T pt, Point_T *pt_normal);
    uint8 Getlineabc2(Point_T Pt0, Point_T Pt1, Line_T *pline);
    int GetPtLoc2(Point_T Pt1, Point_T Pt2, Point_T Pt_P);

    // The AB process can be used.
    uint8 CalLineSegAttribute(Point_T Pt[], int num, Point_T *pPt_0, Point_T *pPt_1,
                              Point_T *pPt_max, Point_T *pPt_min, Line_T *pLine,
                              float *pMean_X, float *pStd_offset);

    // A process use
    // uint8 CalLineSegAttribute_1sigama(Point_T Pt[], int num,Point_T *pPt_0,Point_T
    // *pPt_1, Point_T *pPt_max, Point_T *pPt_min, Line_T *pLine,float *pStd_offset);

    /*********************************************************************************************************
    **  sub functions
    *********************************************************************************************************/
    void CalAllDetected_12(float udata_dir[12], float udata_indir[8],
                           Point_T *pcurudata_detected);
    void UDetectCal(Point_T Pt0, float theta0, float deltax, float deltay, float aerfa,
                    float dis_detected, Point_T *pPt_detected_t);
    int GetObsLocation2(float aerfa, float deltax, float deltay, float invalidTh,
                        float Radio, float localx, float localy, float theta,
                        float *x_detected_out, float *y_detected_out);

    void CalMeanStdValue(float arr[], int len, float *pMean, float *pStd);

    void CalMaxMinValue(float arr[], int len, float *pMax, float *pMin);

    uint8 CalOutline_deviation_V3(Udata_Info_T obs, int firstIdx, int realnum,
                                  float StdOffsetTh, float udeltaTh, float pathdeltaTh,
                                  int Udata_Idx, int *pContour_num,
                                  LineSeg_T *pContour_arr, float *pDeviation);

    void CalObs_part_onlyonce(Udata_Info_T *U_info, int *obj_num, int Udata_Idx,
                              float STDOFFSET, float UDELTA, float PATHDELTA,
                              U_FusionObj_T *pFusionObj);

    void CalObs_whole(Udata_Info_T *U_info, int *obj_num, int Udata_Idx, uint8 isfront,
                      float STDOFFSET, float UDELTA, float PATHDELTA,
                      U_FusionObj_T *pFusionObj);

    void CalObsMainDir(const LineSeg_T contour_arr[], int contour_num,
                       float obs_meantheta, int *pmaindir);

    int JudgeObsClass_V2(Udata_Info_T obs, int firstIdx, int realnum, float obs_meantheta,
                         LineSeg_T contour_arr[CONTOUR_NUMMAX], int contour_num);

    void CalSubOutline_V2(float curlocalx, float curlocaly, uint8 be_XYAxisInterchanged,
                          Point_T PtArr[TIMENUM], int num, Point_T *pPt_1_out,
                          Point_T *pPt_2_out);

    uint8 CalLineSegAttribute_1sigama(Point_T Pt[], int num, Point_T *pPt_0,
                                      Point_T *pPt_1, Point_T *pPt_max, Point_T *pPt_min,
                                      Line_T *pLine, float *pStd_offset);

    void ClearObs(Udata_Info_T *U_info, float cur_fsl, float cur_fsl_2nd,
                  float cur_fsl_peak, float cur_fsl_peak_2nd, float cur_fsl_spd,
                  Point_T curudata, float curpath, float curtheta);

    void GetFilterData_V3();

    void JudgeRearSeg_Bevaild(
        /*U_FusionObj_T * FusionObjLeftRear_Cur,*/ Udata_Info_T Rear_info,
        FusionObj_T FusionObj_RTE, uint8 isleft, uint8 *bedeleted);

#ifdef __cplusplus
}
#endif

#endif /* ifndef GEOMETRY_H */
/*********************************************************************************************************
**                                                                          End Of File:
*Geometry.h
*********************************************************************************************************/
