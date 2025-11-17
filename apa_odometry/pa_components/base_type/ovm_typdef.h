/*****************************************************************************
 C O P Y R I G H T
--------------------------------------------------------------------*//**\file
*//*--------------------------------------------------------------------------
 N O T E S                                                          *//**\file
\brief         [brief]
\details       [detailed description]
Target system  NEC V850 Fx3/Fx4
Compiler       GHS Multi for V800 (>=V4.2.3 Patch01)
*//*--------------------------------------------------------------------------
 A U T H O R   I D E N T I T Y                                      *//**\file
 \author       gra1lr
*****************************************************************************/

#ifndef OVM_TYPDEF_H
#define OVM_TYPDEF_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#ifndef ENABLE_MATHLIB_ONLY
#include <pa_components/base_type/base_typedef.h>
#include <pa_components/mathlib/mathlib_typdef.h>
#include <pa_components/parameter/gen_par_switches.h>
#else
#include "base_typedef.h"

#include "../mathlib/mathlib_typdef.h"
#include "../parameter/gen_par_switches.h"

#endif

/*--------------------------------------------------------------------------*/
/*- set compiler switches                                                  -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilized in this file) are defined         -*/
/*--------------------------------------------------------------------------*/
#ifndef GS_OD_OBJHEIGHT_CLASSIFICATION
#error('GS_OD_OBJHEIGHT_CLASSIFICATION not defined')
#endif

#ifndef GS_BITFLD_ORDER
#error('GS_BITFLD_ORDER not defined')
#endif

#ifndef GS_BITFLD_ORDER_NORMAL
#error('GS_BITFLD_ORDER_NORMAL not defined')
#endif
/*--------------------------------------------------------------------------*/
/*- exported symbolic constants                                            -*/
/*--------------------------------------------------------------------------*/

// possible vehicle directions
#define DIR_FRONT 0
#define DIR_REAR  1
#define DIR_LEFT  2
#define DIR_RIGHT 3

#define gd_POS_INFINITE_si16 (SInt16)(0)

#define gd_DIST_INFINITE_ui16     (UInt16)(5100)
#define gd_DIST_INFINITE_CM_ui16  (UInt16)(gd_DIST_INFINITE_ui16 / 10)
#define gd_DIST_INFINITE_SQR_ui32 (UInt32)((UInt32)gd_DIST_INFINITE_ui16 * (UInt32)gd_DIST_INFINITE_ui16)
#define gd_DIST_MINIMUM_ui16      (UInt16)(150)
#define gd_DIST_MINIMUM_SQR_ui16  (UInt32)((UInt32)gd_DIST_MINIMUM_ui16 * (UInt32)gd_DIST_MINIMUM_ui16)

#ifndef GD_OD_MAX_NUMBER_OBJECTS_UI8
// ! Be careful about array access violation when modifying
#define GD_OD_MAX_NUMBER_OBJECTS_UI8 ((UInt8)20)
#endif

#ifndef gd_OD_MAX_NUMBER_MEMO_OBJECTS_ui8
#define gd_OD_MAX_NUMBER_MEMO_OBJECTS_ui8 ((UInt8)20)
#endif

#define md_MaxFrontFovRange_ui8 (UInt8)(GS_AP_PDC_NUM_DISP_AREA_FRONT * 4)
#define md_MaxRearFovRange_ui8  (UInt8)(GS_AP_PDC_NUM_DISP_AREA_REAR * 4)
#define md_MaxSideFovRange_ui8  (UInt8)(GS_AP_PDC_NUM_DISP_AREA_SIDE * 4)

#define md_MaxFovRange_ui8                                                                                                \
    (UInt8)((md_MaxFrontFovRange_ui8 > md_MaxRearFovRange_ui8)                                                            \
                ? ((md_MaxFrontFovRange_ui8 > md_MaxSideFovRange_ui8) ? md_MaxFrontFovRange_ui8 : md_MaxSideFovRange_ui8) \
                : ((md_MaxRearFovRange_ui8 > md_MaxSideFovRange_ui8) ? md_MaxRearFovRange_ui8 : md_MaxSideFovRange_ui8))

#define gd_MAX_NUMBER_CONTOURPNTS_FRONT_BUMPER_ui8 ((UInt8)6)
#define gd_MAX_NUMBER_CONTOURPNTS_REAR_BUMPER_ui8  ((UInt8)6)
#define gd_MAX_NUMBER_CONTOURPNTS_SIDE_BUMPER_ui8  ((UInt8)5)
#define gd_MAX_NUMBER_CONTOURPNTS_ui8 \
    ((UInt8)((gd_MAX_NUMBER_CONTOURPNTS_FRONT_BUMPER_ui8 + gd_MAX_NUMBER_CONTOURPNTS_REAR_BUMPER_ui8 + gd_MAX_NUMBER_CONTOURPNTS_SIDE_BUMPER_ui8) * 2))

#define gd_MaxConvexHullNum_ui8 ((md_MaxFovRange_ui8 > gd_MAX_NUMBER_CONTOURPNTS_ui8) ? md_MaxFovRange_ui8 : gd_MAX_NUMBER_CONTOURPNTS_ui8)

#define gd_EP_PDC_MAX_NUMBER_OBJECTS_ui8 ((UInt8)5)

#define gd_EP_PDC_MAX_DIRECTIONS_ui8 ((UInt8)2)

#ifndef gd_OD_OBJ_DEBUG_INFO_SWITCH
    #define gd_OD_OBJ_DEBUG_INFO_SWITCH SW_OFF
#endif // !gd_OD_OBJ_DEBUG_INFO_SWITCH


/*--------------------------------------------------------------------------*/
/*- exported macros                                                        -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- exported data types                                                    -*/
/*--------------------------------------------------------------------------*/

/**
OD object type
*/
#if (GS_OD_OBJHEIGHT_CLASSIFICATION == SW_ON)
// Object height status
typedef enum
{
    UNKNOWN_enm = 0,
    HIGH_enm    = 1,
    LOW_enm     = 2
} gType_ObjHeightStatus_en;

// ObjectHeightValidationStatus
typedef enum
{
    NotValid_enm  = 0,
    ValidLOW_enm  = 1,
    ValidHIGH_enm = 2,
    Uncertain_enm = 3
} gType_ObjHeightValidStatus_en;
#endif //( GS_OD_OBJHEIGHT_CLASSIFICATION == SW_ON )

typedef enum
{
    OD_NONE_enm             = 0, /**< OD Object Type: no object */
    OD_POINT_enm            = 1, /**< OD Object Type: point like object */
    OD_STRAIGHT_0CORNER_enm = 2, /**< OD Object Type: open wall object */
    OD_STRAIGHT_1CORNER_enm = 3, /**< OD Object Type: one side closed wall object */
    OD_STRAIGHT_2CORNER_enm = 4  /**< OD Object Type: both side closed wall object */
} gType_ovmOdObjectType_en;

/**
OD Echo type
*/
typedef enum
{
    OD_DE_enm = 0,
    OD_CE_enm = 1
} gType_ovmEchoType_en;

/**
OD 2nd Echo Validation type
*/
typedef enum
{
    OD_SECOND_ECHO_UNCERTAIN_enm = 0,
    OD_SECOND_ECHO_VALID_enm     = 1,
    OD_SECOND_ECHO_NOT_VALID_enm = 2
} gType_ovm2ndEchoValid_en;

/**
OD HeightClassification type
*/
typedef enum
{
    OD_HEIGHT_TRAVERSABLE_enm = 0,
    OD_HEIGHT_LOW_enm         = 1,
    OD_HEIGHT_HIGH_enm        = 2,
    OD_HEIGHT_UNKNOWN_enm     = 3
} gType_ovmOdObjectHeight_en;

/**
OD object origin
*/
typedef enum
{
    OD_ORIGIN_NONE_enm   = 0,
    OD_ORIGIN_DE_enm     = 1,
    OD_ORIGIN_CE_enm     = 2,
    OD_ORIGIN_DEDE_enm   = 3,
    OD_ORIGIN_DECE_enm   = 4,
    OD_ORIGIN_DEDECE_enm = 5,
    OD_ORIGIN_MERGE_enm  = 6
} gType_ovmOdObjectOrigin_en;

/**
OD object last update
*/
typedef enum
{
    OD_UPDATE_NONE_enm      = 0, /**<  */
    OD_UPDATE_FRONT_enm     = 1, /**< object currently measured by front sensor  */
    OD_UPDATE_FOV_FRONT_enm = 2, /**< object placed inside front FoV, but not currently measured */
    OD_UPDATE_REAR_enm      = 3, /**<  */
    OD_UPDATE_FOV_REAR_enm  = 4, /**<  */
    OD_UPDATE_TRACKING_enm  = 5  /**<  */
} gType_ovmOdObjectUpdateState_en;

typedef enum
{
    OOC_NOT_EXIST_enm = 0,

    OOC_NOT_ON_COURSE_enm       = 1,
    OOC_NOT_ON_COURSE_LEFT_enm  = 2,
    OOC_NOT_ON_COURSE_RIGHT_enm = 3,

    OOC_ON_COURSE_enm            = 4,
    OOC_ON_COURSE_FRONT_enm      = 5,
    OOC_ON_COURSE_CORNER_FL_enm  = 6,
    OOC_ON_COURSE_SIDE_LEFT_enm  = 7,
    OOC_ON_COURSE_CORNER_RL_enm  = 8,
    OOC_ON_COURSE_REAR_enm       = 9,
    OOC_ON_COURSE_CORNER_RR_enm  = 10,
    OOC_ON_COURSE_SIDE_RIGHT_enm = 11,
    OOC_ON_COURSE_CORNER_FR_enm  = 12
} gType_ovmOocObjectOnCourse_en;

/**
 */
typedef struct
{
    UInt16 X_ui16;
    UInt16 Y_ui16;
} gType_ovmPointError_st;

/**
OD object state flags
*/
typedef struct
{
#if (GS_BITFLD_ORDER == GS_BITFLD_ORDER_NORMAL)
    Bitfield16 UniformObjMove_bf1 : 1;        /**< indicates OD object shows a uniform
                                                 movement */
    Bitfield16 MeasUpdate_bf1 : 1;            /**< indicates OD object position is currently
                                                 updated by new measurements */
    Bitfield16 ExistProbUp_bf1 : 1;           /**< indicates OD object existence
                                                 probability tendency is growing up */
    Bitfield16 HeightProbUp_bf1 : 1;          /**< indicates OD object height probability
                                                 tendency is growing up */
    Bitfield16 ByDeOnly_bf1 : 1;              /**< indicates that OD object is generated by a
                                                 De-only measurement object */
    Bitfield16 ByCeOnly_bf1 : 1;              /**< indicates that OD object is generated by a
                                                 Ce-only measurement object */
    Bitfield16 ByDeTrck_bf1 : 1;              /**< indicates that OD object is generated by
                                                 De-tracking */
    Bitfield16 ObjUnreliable_bf1 : 1;         /**< indicates that OD object seems to be
                                                 unreliable */
    Bitfield16 ObjInsideFoV_bf1 : 1;          /**< indicated that OD object is probable
                                                 inside the FoV */
    Bitfield16 ClusteringNotFinished_bf1 : 1; /**< indicates that clustering for OD
                                                 object is not completely finished */
    Bitfield16 ByDeBufProc_bf1 : 1;           /**< indicated that OD object is generated by
                                                 DeBufProcessing */
    Bitfield16 Traversable_bf1 : 1;           /**< indicates OD object is traversable */
    Bitfield16 PosUnreliable_bf1 : 1;         /**< indicates that position of OD object
                                                 is unreliable */
    Bitfield16 DynamicObjMove_bf1 : 1;        /**< indicates that object is moving non
                                                 stationary  */
    Bitfield16 MemoryObj_bf1 : 1;             /**< Indicates whether the object is a real-time
                                                   obstacle or a memory obstacle */
    Bitfield16 reserved_bf1 : 1;
#else
    Bitfield16 reserved_bf1 : 2;
    Bitfield16 DynamicObjMove_bf1 : 1;        /**< indicates that object is moving non
                                                 stationary  */
    Bitfield16 PosUnreliable_bf1 : 1;         /**< indicates that position of OD object
                                                 is unreliable */
    Bitfield16 Traversable_bf1 : 1;           /**< indicates OD object is traversable */
    Bitfield16 ByDeBufProc_bf1 : 1;           /**< indicated that OD object is generated by
                                                 DeBufProcessing */
    Bitfield16 ClusteringNotFinished_bf1 : 1; /**< indicates that clustering for OD
                                                 object is not completely finished */
    Bitfield16 ObjInsideFoV_bf1 : 1;          /**< indicated that OD object is probable
                                                 inside the FoV */
    Bitfield16 ObjUnreliable_bf1 : 1;         /**< indicates that OD object seems to be
                                                 unreliable */
    Bitfield16 ByDeTrck_bf1 : 1;              /**< indicates that OD object is generated by
                                                 De-tracking */
    Bitfield16 ByCeOnly_bf1 : 1;              /**< indicates that OD object is generated by a
                                                 Ce-only measurement object */
    Bitfield16 ByDeOnly_bf1 : 1;              /**< indicates that OD object is generated by a
                                                 De-only measurement object */
    Bitfield16 HeightProbUp_bf1 : 1;          /**< indicates OD object height probability
                                                 tendency is growing up */
    Bitfield16 ExistProbUp_bf1 : 1;           /**< indicates OD object existence
                                                 probability tendency is growing up */
    Bitfield16 MeasUpdate_bf1 : 1;            /**< indicates OD object position is currently
                                                 updated by new measurements */
    Bitfield16 UniformObjMove_bf1 : 1;        /**< indicates OD object shows a uniform
                                                 movement */
#endif
} gType_ovmOdObjectStateFlags_st;

/**
OD object position flags
*/
typedef struct
{
#if (GS_BITFLD_ORDER == GS_BITFLD_ORDER_NORMAL)
    Bitfield8 VehFront_bf1 : 1;  /**< indicates that OD object has a location
                                    point at vehicle front side  */
    Bitfield8 VehRear_bf1 : 1;   /**< indicates that OD object has a location point
                                    at vehicle rear side   */
    Bitfield8 VehLeft_bf1 : 1;   /**< indicates that OD object has a location point
                                    at vehicle left side   */
    Bitfield8 VehRight_bf1 : 1;  /**< indicates that OD object has a location
                                    point at vehicle right side  */
    Bitfield8 VehInside_bf1 : 1; /**< indicates that OD object is at least partly
                                    located inside vehicle contour  */
    Bitfield8 reserved_bf1 : 3;
#else
    Bitfield8 reserved_bf1 : 3;
    Bitfield8 VehInside_bf1 : 1; /**< indicates that OD object is at least partly
                                    located inside vehicle contour  */
    Bitfield8 VehRight_bf1 : 1;  /**< indicates that OD object has a location
                                    point at vehicle right side  */
    Bitfield8 VehLeft_bf1 : 1;   /**< indicates that OD object has a location point
                                    at vehicle left side   */
    Bitfield8 VehRear_bf1 : 1;   /**< indicates that OD object has a location point
                                    at vehicle rear side   */
    Bitfield8 VehFront_bf1 : 1;  /**< indicates that OD object has a location
                                    point at vehicle front side  */
#endif
} gType_ovmOdObjectPositionFlags_st;

#if(gd_OD_OBJ_DEBUG_INFO_SWITCH == SW_ON)

typedef enum
{
    OD_PDC_NONE_enm       = 0, // Object Type: no object
    OD_PDC_DE_enm         = 1, // Object Type: Direct-Echo Object
    OD_PDC_DE_DE_enm      = 2, // Object Type: Double Direct-Echo Object
    OD_PDC_CE_enm         = 3, // Object Type: Cross-Echo Object
    OD_PDC_DE_CE_enm      = 4, // Object Type: Direct-Echo-Cross-Echo Object
    OD_PDC_ROUND_POST_enm = 5, // Object Type: Round Post
    OD_PDC_WALL_enm       = 6  // Object Type: Wall
} gType_PdcObjType_en;

typedef enum
{
    OD_ORIGIN_NEWOBJ_enm = 0,       //Obj Origin: New Real Obj 
    OD_ORIGIN_SPLITOBJ_enm = 1,     //Obj Origin: SPLIT Real Obj 
    OD_ORIGIN_MATCH_enm = 2,        //Obj Origin: Real Obj Match
    OD_ORIGIN_REAL_MERGE_enm = 3,   //Obj Origin: Real Obj Merge Real Obj Merge
    OD_ORIGIN_MEMO_MERGE_enm = 4    //Obj Origin: Real Obj Merge Memo Obj Merge
}gType_OdObjOrigin_en;

typedef enum
{
    OD_HL_DE_PYTHAGOREAN_enm      = 0,   /**< 高属性来源于回波勾股定理关系*/
    OD_HL_DE_DOUBLE_enm           = 1,   /**< 高属性来源于回波双倍关系*/
    OD_HL_DE_THREE_enm            = 2,   /**< 高属性来源于回波4/5关系*/
    OD_HL_DE_FOUR_enm             = 3,   /**< 高属性来源于回波3/10关系*/
    OD_HL_DE_HEIGHT_enm           = 4,   /**< 高属性来源于回波高度*/
    OD_HL_DE_POINT_BREAKPOINT_enm = 5,   /**< 点障碍物高属性来源于DE断点*/
    OD_HL_DE_LINE_BREAKPOINT_enm  = 6,   /**< 线障碍物高属性来源于DE断点*/
    OD_HL_CE_PYTHAGOREAN_enm      = 7,   /**< 高属性来源于回波勾股定理关系*/
    OD_HL_CE_DOUBLE_enm           = 8,   /**< 高属性来源于回波双倍关系*/
    OD_HL_CE_THREE_enm            = 9,   /**< 高属性来源于回波4/5关系*/
    OD_HL_CE_FOUR_enm             = 10,  /**< 高属性来源于回波3/10关系*/
    OD_HL_CE_HEIGHT_enm           = 11,  /**< 高属性来源于回波高度*/
    OD_HL_CE_POINT_BREAKPOINT_enm = 12,  /**< 点障碍物高属性来源于CE断点*/
    OD_HL_CE_LINE_BREAKPOINT_enm  = 13,  /**< 线障碍物高属性来源于CE断点*/
    OD_HL_ROUNDPOST_enm           = 14,  /**< 高属性来源于RoundPost类型*/
    OD_HL_RATIO_enm               = 15,  /**< 高属性来源于侧边雷达回波占比*/
    OD_HL_REAL_MERGE_enm          = 16,  /**< 高低来源于Real Object Merge Real Object*/ 
    OD_HL_MEMO_MERGE_enm          = 17,  /**< 高低来源于Memory Object Merge Real Object*/
    OD_HL_MAINTAIN_enm            = 18   /**< 高低来源于属性记忆*/

}gType_OdObjHLSource_en;

typedef enum
{
    OD_ECHO_NONE_enm        = 0,    /**< 无回波*/
    OD_ECHO_STD_STD_enm     = 1,    /**< 第一回波STD,第二回波STD*/
    OD_ECHO_STD_ADV_enm     = 2,    /**< 第一回波STD,第二回波ADV*/
    OD_ECHO_ADV_STD_enm     = 3,    /**< 第一回波ADV,第二回波STD*/
    OD_ECHO_ADV_ADV_enm     = 4     /**< 第一回波ADV,第二回波ADV*/
}gType_OdObjHLEchoCombineInfo_en;

typedef enum
{
    OD_MODIFY_NONE_enm      = 0,    /**< 无更新来源*/
    OD_MODIFY_BUMPER_enm    = 1,    /**< 更新来源于保险杠*/
    OD_MODIFY_MEMORANGE_enm = 2,    /**< 更新来源于记忆范围*/
    OD_MODIFY_FOVRANGE_enm  = 3,    /**< 更新来源于可探测范围*/
    OD_MODIFY_MERGE_enm     = 4     /**< 更新来源于Memory Object Cross Real Object Merge*/
}gType_OdMemoObjUpdateSource_en;

typedef struct
{
    //关联雷达编号
    UInt8 ObjRelSensAId_ui8;    /**< 组合对左侧雷达A编号*/
    UInt8 ObjRelSensBId_ui8;    /**< 组合对右侧雷达B编号*/
    //定位类型
    UInt8 TypeClassify_ui8;     /**< 初始分类 (gType_PdcObjType_en)*/ 
    UInt8 PosDegrade_ui8;       /**< 定位降级类型 (gType_PdcObjType_en)*/ 
    UInt8 PreMatch_ui8;         /**< 预匹配过滤类型 (gType_PdcObjType_en)*/ 
    UInt8 PdcObjType_ui8;       /**< 最终类型 (gType_PdcObjType_en)*/
    //定位来源
    UInt8 ObjOrigin_ui8;        /**< 障碍物形成来源(gType_OdObjOrigin_en-bit0:NEWOBJ,bit1:SPLITOBJ,bit2:MATCH)*/
    //原始回波
    UInt16 RawDe1_ui16;         /**< 组合对左侧雷达A直接回波数据*/
    UInt16 RawDe2_ui16;         /**< 组合对右侧雷达B直接回波数据*/
    UInt16 RawCe_ui16;          /**< 组合对间接回波数据*/
    // object distances to sensor A and B
    UInt16 ObjDistSensA_ui16;   /**< 组合对三角定位左侧雷达距离*/
    UInt16 ObjDistSensB_ui16;   /**< 组合对三角定位右侧雷达距离*/
    // the calculated object positions per sensor pair
    SInt16 ObjPosProb1X_si16;   /**< 可能性定位坐标1X*/
    SInt16 ObjPosProb2X_si16;   /**< 可能性定位坐标2X*/
    SInt16 ObjPosProb3X_si16;   /**< 可能性定位坐标3X*/
    SInt16 ObjPosProb1Y_si16;   /**< 可能性定位坐标1Y*/
    SInt16 ObjPosProb2Y_si16;   /**< 可能性定位坐标2Y*/
    SInt16 ObjPosProb3Y_si16;   /**< 可能性定位坐标3Y*/
   
}gType_ObjPdcData_st;
typedef struct
{
   //高低属性更新时相关回波信息
   UInt8 HL_DECombineInfo_ui8;           /**< 高低属性第一直接回波第二直接回波发波来源 (gType_OdObjHLEchoCombineInfo_en)*/
   UInt8 HL_CECombineInfo_ui8;           /**< 高低属性第一间接回波第二间接回波发波来源 (gType_OdObjHLEchoCombineInfo_en)*/
   UInt16 HL_JudgeFirstDe_ui16;          /**< 高低属性第一直接回波距离*/
   UInt16 HL_JudgeFirstDeHeight_ui16;    /**< 高低属性第一直接回波高度*/
   UInt16 HL_JudgeSecondDe_ui16;         /**< 高低属性第二直接回波距离*/
   UInt16 HL_JudgeSecondDeHeight_ui16;   /**< 高低属性第二直接回波高度*/
   UInt16 HL_JudgeFirstCe_ui16;          /**< 高低属性第一间接回波距离*/
   UInt16 HL_JudgeFirstCeHeight_ui16;    /**< 高低属性第一间接回波高度*/
   UInt16 HL_JudgeSecondCe_ui16;         /**< 高低属性第二间接回波距离*/
   UInt16 HL_JudgeSecondCeHeight_ui16;   /**< 高低属性第二间接回波高度*/
   //高低属性进入断点相关回波信息
   UInt16 HL_JudgeDeBreakPoint_ui16;     /**< 高低属性断点直接回波距离*/
   UInt16 HL_JudgeCeBreakPoint_ui16;     /**< 高低属性断点间接回波距离*/
   UInt32 HL_Source_ui32;                /**< 高低属性更新来源 (gType_OdObjHLSource_en-bit0:PYTHAGOREAN,bit1:DOUBLE...)*/

}gType_ObjHLUpdate_st;

typedef struct
{
   UInt8 MemoPosUpdateSource_ui8;       /**< 记忆障碍物位置变换来源 (gType_OdMemoObjUpdateSource_en)*/
}gType_MemoPosUpdate_st;


typedef struct 
{
   gType_ObjPdcData_st ObjPdcData_st;       /**< 障碍物定位点信息*/
   gType_ObjHLUpdate_st ObjHLUpdate_st;     /**< 障碍物高低属性信息*/
   gType_MemoPosUpdate_st MemoPosUpdate_st; /**< 记忆障碍物更新信息*/ 
   
}gType_DebugInfo_st;

#endif

/**
OD object data
*/
typedef struct
{
    // OD object position respective error values
    gType_mtlPoint_st P1_st;
    gType_mtlPoint_st P2_st;
    gType_ovmPointError_st ErrorP1_st;
    gType_ovmPointError_st ErrorP2_st;

    // Dist to Line of Last point which has mached with one of the wall points
    // Negativ if point was left to wall line
    SInt16 Dist2LineOfLastP1Match_si16;
    SInt16 Dist2LineOfLastP2Match_si16;

    // OD object attributes
    gType_ovmOdObjectStateFlags_st StateFlags_st;
    gType_ovmOdObjectPositionFlags_st PositionFlags_st;

    gType_ovmOdObjectOrigin_en Origin_en;
    gType_ovmOdObjectUpdateState_en Update_en;
    gType_ovmOdObjectType_en Type_en;

    UInt16 Length_ui16;
    UInt8 ExistProb_ui8;
    UInt8 ExistProbStandby_ui8;
    gType_ovmOdObjectHeight_en HeightStatus_en; // if GS_OD_OBJHEIGHT_CLASSIFICATION ==
                                                // SW_OFF always unknown
    
    gType_ovmOdObjectHeight_en HeightStatusStandby_en;

#if (GS_OD_OBJHEIGHT_CLASSIFICATION == SW_ON)
    // HeightProbVariables
    UInt8 HeightProb_ui8;
    UInt8 HeightProbStandby_ui8;
    gType_ObjHeightStatus_en HeightStatusForWallSplit_en;
    gType_ObjHeightValidStatus_en HeightValid_en;
    UInt8 ValidationCounter_ui8;
    gType_mtlPoint_st SplitPoint_st;
    gType_mtlPoint_st SplitReferencePoint_st;
    UInt8 NearRangeEchoCounter_ui8;
    UInt8 TriggerDirectionForWallSplit_ui8;
    UInt8 NumOf1stEcho_ui8;
    UInt8 NumOf2ndValidEcho_ui8;
    UInt8 PercentageOfHeight_ui8;
    UInt32 EchoHeighTotal_ui32[6]; // 0: 0~60cm  1: 60~90cm  2:90~110cm   3: 110~130cm  4: 130~150cm  4: 150~250cm
    UInt16 EchoHeighAvage_ui16[6];
    UInt16 EchoHeighCnt_ui16[6];
#endif // #if ( GS_OD_OBJHEIGHT_CLASSIFICATION == SW_ON )

    UInt8 ID_ui8;
    UInt8 NeedUpdateCnt_ui8; // Line change point count

    // OD object distances to vehicle contour
    UInt16 Dist_ui16;
    UInt16 StartTrackDist_ui16;
    UInt16 ODObjTrckCycl_ui16;
    gType_mtlPoint_st ClosestVehContourPnt_st;
#if(gd_OD_OBJ_DEBUG_INFO_SWITCH == SW_ON)
    //OD Object Debug Info
    gType_DebugInfo_st OdDebugInfo_st;
#endif

} gType_ovmOdObject_st;

/** OOC information for an OD object. See OOC design for details */
typedef struct
{
    /**! distance to collision [cm], calculated with current driving curvature
      from collision point on the object to collision point on the vehicle contour
      (as given by APG vehicle parameters). If an object is not on course (but in
      scope), a virtual distance to collision is calculated. See OOC design for
      details.*/
    UInt16 DistanceToCollision_ui16;
    SInt16 DistanceToCourse_si16; //<! Distance to course [cm], if object is on
                                  // course sign is negative

#if (GS_PP_FUNC_NRA == SW_ON)
    SInt16 KappaToRight_si16; //<! curvature to pass an obstacle to the right of
                              // the obstacle [1/(2^14 m)]
    SInt16 KappaToLeft_si16;  //<! curvature to pass an obstacle to the left
                              //[1/(2^14 m)]
#endif

    /**! collision point on the object [cm]. Given in vehicle coordinate system
     * (longitudinal axis = X-axis) */
    gType_mtlPoint_st PointOfCollisionObject_st;
    /**! collision point on the vehicle contour [cm]. Given in vehicle coordinate
     * system */
    gType_mtlPoint_st PointOfCollisionVehicle_st;

    gType_ovmOocObjectOnCourse_en OOC_en; //!< is object on course/in scope? See OOC design for details
} gType_ovmOocInfo_st;

/*--------------------------------------------------------------------------*/
/*- external objects declaration                                           -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- prototypes of exported functions                                       -*/
/*--------------------------------------------------------------------------*/

#endif
