/**
 * @file PK_SF_A_FreeSpace.h
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
#ifndef PK_SF_A_FREESPACE
#define PK_SF_A_FREESPACE

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Utility.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "Rte_Types.h"
/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

/** Version and module identification */
#define PK_SF_VENDOR_ID (0u)
#define PK_SF_MODULE_ID (0u)

/** Component Version Information */
#define PK_SF_A_FS_SW_MAJOR_VERSION (3u)
#define PK_SF_A_FS_SW_MINOR_VERSION (0u)
#define PK_SF_A_FS_SW_PATCH_VERSION (26u)

/*********************************************************************************************************
** Definition of structures
*********************************************************************************************************/
// Ultrasound data structure
#define TIMENUM        100 // definition of the max size of UdataInfo
#define CONTOUR_NUMMAX 20  // definition of the max size of the obj number in each cycle

#define NOISEPERCENT_TH \
    30U // definition of the noise percent,when the detected noise is larger than this
        // threshold, sf will be hang up.

// CalAllDetected_12
#define SIDE_INVALID      4800
#define FrontRear_INVALID 2550

typedef struct _U_FusionObj_T
{
    int obj_num;
    int obj_attr[CONTOUR_NUMMAX];
    LineSeg_T obj[CONTOUR_NUMMAX];
    float pathstart[CONTOUR_NUMMAX];
    int is_Wholeobj[CONTOUR_NUMMAX];
    int is_over2dot5[CONTOUR_NUMMAX];
    float ang[CONTOUR_NUMMAX];
} U_FusionObj_T;

// ClearPartObj & UpdateObj
// keep 32 obj outlines
typedef struct _U_FusionObj32_T
{
    int obj_num;
    int obj_attr[SF_OBJ_NUM];
    LineSeg_T obj[SF_OBJ_NUM];
    float pathstart[SF_OBJ_NUM];
    int is_Wholeobj[SF_OBJ_NUM];
    int is_over2dot5[SF_OBJ_NUM];
    float ang[SF_OBJ_NUM];
} U_FusionObj32_T;

typedef struct _Udata_Info_T
{
    Point_T obsPt[TIMENUM];   // 目标坐标
    float value[TIMENUM];     // ultrasonic distance
    float value_2nd[TIMENUM]; // ultrasonic scd distance
    float curtheta[TIMENUM];  // head angle
    uint8 peak[TIMENUM];
    uint8 peak_2nd[TIMENUM];
    int num;
    float pathstart;
    float start_spd;
    float stop_spd;
} Udata_Info_T;

// Obstacle Attribute Judgment
typedef enum
{
    NEAR_VEH_BODY = 1000,
    CURB          = 2000,
    FAR_VEH_BODY  = 3000,
    INVALID_5510  = 4000,
} PK_OBS_ATTRIBUTE_QIAN;

enum
{
    INDEX_FSR = 0,
    INDEX_FSL,
    INDEX_RSL,
    INDEX_RSR,
};

/*********************************************************************************************************
** Definition of glboal variables
*********************************************************************************************************/
extern float cur_fsr;
extern float cur_fsr_2nd; // The value of quadratic echo data
extern float cur_fsr_peak;
extern float cur_fsr_peak_2nd;
extern float cur_fsr_spd;
extern float cur_fsl;
extern float cur_fsl_2nd; // The value of quadratic echo data
extern float cur_fsl_peak;
extern float cur_fsl_peak_2nd;
extern float cur_fsl_spd;
extern float cur_rsr;
extern float cur_rsr_2nd; // The value of quadratic echo data
extern float cur_rsr_peak;
extern float cur_rsr_peak_2nd;
extern float cur_rsr_spd;
extern float cur_rsl;
extern float cur_rsl_2nd; // The value of quadratic echo data
extern float cur_rsl_peak;
extern float cur_rsl_peak_2nd;
extern float cur_rsl_spd;
/*********************************************************************************************************
** Definition of exported function like macros
*********************************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /**************************************************************************
    Description		: main function of free space detection
    Owner			:JSF
    Modified Date	:2018.06.12
    Parameter[In]	:None
    Parameter[Out]	:None
    Return			:None
    ***************************************************************************/
    void PK_SF_A_FreeSpace(void);

    void PK_SF_A_FreeSpace_Clear(void);

    void PK_SF_A_FreeSpace_OffClear(void);

    void PK_SF_A_FreeSpace_Init(void);

    void PK_SF_A_Vision(void);

    void PK_SF_A_URadar(void);

    void PK_SF_A_URadar_Left(U_FusionObj_T *FusionObjLeft_Cur);

    void PK_SF_A_URadar_Right(U_FusionObj_T *FusionObjRight_Cur);

    void PK_SF_A_URadar_LeftRear(U_FusionObj_T *FusionObjLeftRear_Cur);

    void PK_SF_A_URadar_RightRear(U_FusionObj_T *FusionObjRightRear_Cur);

    void PK_SF_A_URadar_Log(void);

    /**************************************************************************
    Description		:functions for update in each cycle
    Owner			:JSF
    Modified Date	:2018.06.19
    Parameter[In]	:None
    Parameter[Out]	:None
    Return			:None
    ***************************************************************************/
    void ClearPartObj(U_FusionObj32_T *FusionObj, int in_CalObs_whole);

    void UpdateObj(U_FusionObj32_T *FusionObj, U_FusionObj_T FusionObj_Cur);

    void UpdateObj_V2(U_FusionObj32_T *FusionObj, const U_FusionObj_T &FusionObj_Cur,
                      uint8 beUpdated_ByOdo, uint8 *beLooped_Flag, float *beLooped_S);

    void UpdateObj_CCP(FusionObj_T *FusionObj, U_FusionObj_T FusionObj_Cur);

#ifdef __cplusplus
}
#endif

#endif /* ifndef PK_SF */
/*********************************************************************************************************
**                                                                          End Of File:
*PK_SF.h
*********************************************************************************************************/
