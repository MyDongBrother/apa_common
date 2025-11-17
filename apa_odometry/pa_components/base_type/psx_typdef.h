/*****************************************************************************
 C O P Y R I G H T
-------------------------------------------------------------------
 Copyright (c) 2001 by Frovision.          All rights reserved.

 This file is property of Frovision. Any unauthorized copy, use or
 distribution is an offensive act against international law and may be
 prosecuted under federal law. Its content is company confidential.
------------------------------------------------------------------------------
*****************************************************************************/
/*! */

// the following line specifies a documentation section for MPS
//!\addtogroup COMPONENT_MPS MPS documentation

#ifndef PSX_INCLUDE_H
#define PSX_INCLUDE_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- set compiler switches                                                  -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilised in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

#define gd_MinOMAPSId_ui8 ((UInt8)249)
#define gd_MaxOMAPSId_ui8 ((UInt8)252)

#define gd_PSx_MAX_XOFFSET_BOTTOMPOINTANDCORNERS_ui8 ((UInt8)80)

#ifdef ENABLE_MATHLIB_ONLY
// Sensor configuration
// (do never change these values!!!)
#define gd_PSDSENSOR_LEFT_ui8 ((UInt8)0)
// sensor front left
#define gd_PSDSENSOR_RIGHT_ui8 ((UInt8)1)
// sensor left rear
#define gd_PSDSENSOR_LEFT_REAR_ui8 ((UInt8)2)
// sensor right rear
#define gd_PSDSENSOR_RIGHT_REAR_ui8 ((UInt8)3)

/*   the internal sensor numbers
     PSx:

            ---------------
            |             |
  left --> (1)           (6) <-- right
            |             |
            |             |
            |             |
            -------x------- HA
            |             |
           (12)          (7)
            |             |
            ---------------
*/

#define gd_PARKING_SLOT_LEFT_ui8  ((UInt8)0)
#define gd_PARKING_SLOT_RIGHT_ui8 ((UInt8)1)

// Invalid ID
// In case the values of the output-struct is not valid, the
// identification-number (ID) will be set to the value "gd_PSD_ID_INVALID_ui8"
// Example for the use of an ID:
// a) PS-Output of psd                      : "ParkingSlot_pst.PS_ID_ui8"
// b) Object-Output of the side-protection  : "SPR_OutputData_pst.ID_ui8"
#define gd_PSD_ID_INVALID_ui8 ((UInt8)0)

#define gd_PSDADI_OBJLISTSIZE_ui8 (UInt8)(gd_PSDCLU_LIST_SIZE_ui8 + (UInt8)1)

#define gd_PSDSENSOR_LEFT_ui8 ((UInt8)0)
// sensor front left
#define gd_PSDSENSOR_RIGHT_ui8 ((UInt8)1)
// sensor left rear
#define gd_PSDSENSOR_LEFT_REAR_ui8 ((UInt8)2)
// sensor right rear
#define gd_PSDSENSOR_RIGHT_REAR_ui8 ((UInt8)3)

#define gd_PARKING_SLOT_LEFT_ui8  ((UInt8)0)
#define gd_PARKING_SLOT_RIGHT_ui8 ((UInt8)1)

#define gd_PSDPara_Veh_FrontSensorXDist_ui16 \
    g_parGetParaReadAccess_pst(PSD_Veh)->FrontSensorXDist_ui16
#define gd_PSDPara_Veh_FrontSensorYDist_ui16 \
    g_parGetParaReadAccess_pst(PSD_Veh)->FrontSensorYDist_ui16
#define gd_PSDPara_Veh_FrontSensorAngle_ui8 \
    g_parGetParaReadAccess_pst(PSD_Veh)->FrontSensorAngle_ui8
#define gd_PSDPara_Veh_RearSensorXDist_si16 \
    g_parGetParaReadAccess_pst(PSD_Veh)->RearSensorXDist_si16
#define gd_PSDPara_Veh_RearSensorYDist_ui16 \
    g_parGetParaReadAccess_pst(PSD_Veh)->RearSensorYDist_ui16
#define gd_PSDPara_Veh_RearSensorAngle_ui8 \
    g_parGetParaReadAccess_pst(PSD_Veh)->RearSensorAngle_ui8
#endif
// Type of Searching status
typedef enum
{
    APA_SEARCHING_OFF_enm             = 0,
    APA_SEARCHING_ALL_enm             = 1,
    APA_SEARCHING_MAX_enm             = 2,
} gType_APA_SEARCHING_en;

// Type of PARKING status
typedef enum
{
    APA_PARKING_OFF_enm             = 0,
    APA_PARKING_ACTIVE_enm          = 1,
    APA_PARKING_UNFINISH_enm        = 2,
    APA_PARKING_FINISH_enm          = 3,
    APA_PARKING_MAX_enm             = 4,
} gType_APA_PARKING_en;

typedef enum
{
    g_psxPSTypeNone_enm     = 0, /**< Parking Slot type Default  */
    g_psxPSTypeParallel_enm = 1, /**< Parking Slot type Parallel */
    g_psxPSTypeCross_enm    = 2, /**< Parking Slot type Cross    */
    g_psxPSTypeDiagonal_enm = 3  /**< Parking Slot type Diagonal */
} gType_psxPSType_en;

typedef enum
{
    g_psxLatRefVirtual_enm = 0, /**< No real lateral reference object is        */
    g_psxLatRefLow_enm     = 1, /**< The lateral reference is low and based on  */
    g_psxLatRefHigh_enm    = 2  /**< The lateral reference is high and based    */
} gType_psxLatRefType_en;

typedef enum
{
    g_psxRefVirtual_enm = 0, /**< No real reference object is present. */
    g_psxRefReal_enm    = 1, /**< reference is based on a real object.
                               CPSC: Reference is based on a long object (to be
                               considered for alignment).    This is also the default
                               Ref type until the type can be qualified better */
    g_psxRefShort_enm = 2,   /**< OMA remeasurement of reference is based on a sole short
                                object.   This could be anything, but it's probably not a car.
                                Short or   unstructured object will be treated differently in
                                the CPSC   alignment rules. */
    g_psxRefPillar_enm = 3   /**< the reference is based on a short object with a larger
                                object   behind it. This is a typical OD object constellation
                                for a pillar   with a parking car behind. The reference will
                                be placed on the closer   (short) object to avoid danger of
                                collision. NOTE: Currently   unsupported. */
} gType_psxRefType_en;

// aly1kor: Note !!!! Do not change the below enum values 0->Left, 1->Right
typedef enum
{
    g_psxLeftSide_enm  = 0, /**< The left side of the vehicle */
    g_psxRightSide_enm = 1, /**< The right side of the vehicle */
    g_psxBothSide_enm  = 2  /**< Both Sides, MPS compatibility */
} gType_psxSide_en;

typedef enum
{
    g_psxPSNone_enm       = 0, /**< Dummy or invalid value */
    g_psxPSUnsuitable_enm = 1, /**< Parking space is too small */
    // g_psxPSSmall_enm        = 2, // Parking space suitable but small, forward
    // drive required g_psxPSSmallPosOk_enm   = 3, // Parking space suitable but
    // small, no forward drive required
    g_psxPSOk_enm      = 4, /**< Parking space is suitable, forward drive required */
    g_psxPSOkPosOk_enm = 5  /**< Parking space is suitable, no forward drive required */
} gType_psxParkable_en;

typedef enum
{
    g_psxPSOriginal_enm   = 0, /**< initial value before first OMA adaption */
    g_psxPSUnmodified_enm = 1, /**< value after an initial OMA adaption but
                                  without modification in current cycle */
    g_psxPSModified_enm = 2 /**< value after OMA adaption/modification in current cycle */
} gType_psxPSState_en;

typedef enum
{
    BOTTOM_NONE_enm    = 0,
    BOTTOM_LOW_enm     = 1,
    BOTTOM_HIGH_enm    = 2,
    BOTTOM_UNKNOWN_enm = 3
} gType_PSX_BottomHeightType_en;
// Type of an object
typedef enum
{
    OBJECT_CAR_enm          = 0,
    OBJECT_CURBSTONE_enm    = 1,
    OBJECT_ROUND_OBJECT_enm = 2,
    OBJECT_UNKNOWN_enm      = 3,
    OBJECT_DUMMY_enm        = 4, // contains start segment with minimum length
    OBJECT_NONE_enm         = 5,
    OBJECT_WALL_enm         = 6
} gType_PSX_ObjectType_en;

/**
State of update of a parking space boundary
*/
typedef enum
{
    g_psxRefUndefined_enm = 0, /**< Default value */
    g_psxRefOriginal_enm  = 1, /**< PS data is original data as given from MPS, not
                                  yet updated by OMA */
    g_psxRefUpdated_enm = 2    /**< PS data is updated by OMA */
} gType_psxRefState_en;

typedef enum
{
    g_psxPSNotOnCurb_enm     = 0, /**< PS is not on curbstone                       */
    g_psxPSOnCurb2Wheels_enm = 1, /**< PS is on curbstone; 2 wheels on curbstone    */
    g_psxPSOnCurb4Wheels_enm = 2  /**< PS is on curbstone; 4 wheels on curbstone    */
} gType_psxIsOnCurb_en;

/** Generic parking space structure.
 * This data is passed to the APG. It is a common structure
 * for PSD parking spaces and OMA updated parking spaces.
 * IMP!!!!!!! When new elements are added in the struct, gType_psxPS_st
 * remember to copy it in m_mpsProcessInputPSData_vd()
 * PSX design document can be
 */
typedef struct
{
    UInt8 ID_ui8; //!< Numeric ID of the parking space, Even Ids for Right and Odd
                  //!< for Left |Scale[Unit]: - | Range.Min: 0 | Range.Max: 255 |

    /**<  Coordinates of parking-space corner 1 (parking space start). Unit: [cm]
     */
    gType_mtlPoint_st PS1stCorner_st;

    /**< Coordinates of parking-space corner 2 (parking space end) coordinate. */
    /**< Are set to (0/0) before remeasurement. Unit: [cm] */
    gType_mtlPoint_st PS2ndCorner_st;
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // parking-space conditions:
    UInt16 Length_ui16;      /**< Length of the PS, Unit: [cm] */
    UInt16 Depth_ui16;       /**< Depth of the PS, Unit: [cm]  */
    SInt16 Orientation_si16; /**< Parking slot angle for Diagonal Parking, 0=>
                                Cross PSs, |Scale[Unit]: 1/2^12 [rad]|, Input from
                                PSl Orientation of the PS for MPS |Scale[Unit]:
                                1/2^10 [rad]| */
    SInt16 Curvature_si16;   /**< Curvature of the PS, Unit: [1/2^14 m] */
    gType_psxRefType_en
        Obj1stRefType_en; /**< Object1 Reference Type, Virtual or Real Object */
    gType_psxRefType_en
        Obj2ndRefType_en; /**< Object2 Reference Type, Virtual or Real Object */
    gType_psxLatRefType_en
        LatRefType_en;            /**< Lateral reference Type, Virtual, Low or high */
    gType_psxSide_en Side_en;     /**< Parking Slot Sdie */
    gType_psxPSType_en PSType_en; /**< Parking Slot Type */
    gType_psxIsOnCurb_en PSIsOnCurb_en; /**< Info on Curb Stone */
    //--------------------------------------------------------------------------

    /**< Orientation of the first object in 2^12 [rad] in OMA coordinates. */
    /**< Subtract pi/2 to transform the angle into the APG parking space
     * coordinate system. */
    SInt16 Obj1stOrientation_si16;

    /**< Orientation of the second object in 2^12 [rad] in OMA coordinates (see
     * Obj1stOrientation_si16). */
    SInt16 Obj2ndOrientation_si16;

    //--------------------------------------------------------------------------
} gType_psxPS_st;

typedef struct
{
    gType_mtlPoint_st
        Obj1stCorner_pst[gd_PSM_MaxNumOfObjects_ui8]; //!< track data in VHM coordinates,
                                                      //!< Array size must be >=
                                                      //!< gd_PSDADI_OBJLISTSIZE_ui8
    gType_mtlPoint_st
        Obj2ndCorner_pst[gd_PSM_MaxNumOfObjects_ui8]; //!< track data in VHM coordinates,
                                                      //!< Array size must be >=
                                                      //!< gd_PSDADI_OBJLISTSIZE_ui8
    UInt8 ObjectID_ui8[gd_PSM_MaxNumOfObjects_ui8]; //!< PSD Object ID

    SInt16 SPosObj1stCorner_si16
        [gd_PSM_MaxNumOfObjects_ui8]; //!< Appliable only if
                                      //!< GS_PSD_SPOS_INFO_DYN_OBJECTS
                                      //!< = SW_ON
    SInt16 SPosObj2ndCorner_si16
        [gd_PSM_MaxNumOfObjects_ui8]; //!< Appliable only if
                                      //!< GS_PSD_SPOS_INFO_DYN_OBJECTS
                                      //!< = SW_ON

#if 0
  UInt8             UntransformedSPosObj1_ui8;
  UInt8             UntransformedSPosObj2_ui8;                             //!< Bit info for objects stored in ObjectID_ui8, Bit Set => Static(PSDCLA_OBJECT_FINISHED_enm), Reset => Dynamic(), 
  //!< Bit0 => Byte0 in ObjectID_ui8... Note: if gd_MPS_MaxNumOfObjects_ui8 > 8, then the information will be incorrect
#endif

    UInt8 IsStaticObject_ui8; //!< Bit info for objects stored in ObjectID_ui8,
                              //!< Bit Set => Static(PSDCLA_OBJECT_FINISHED_enm),
                              //!< Reset => Dynamic(),
    //!< Bit0 => Byte0 in ObjectID_ui8... Note: if gd_MPS_MaxNumOfObjects_ui8 > 8,
    //!< then the information will be incorrect
    UInt8 IsMergedObject_ui8; //!< Bit info for objects stored in ObjectID_ui8,
                              //!< Bit Set => PSD has merged Object, Reset => Not
                              //!< merged object,
    //!< Bit0 => Byte0 in ObjectID_ui8... Note: if gd_MPS_MaxNumOfObjects_ui8 > 8,
    //!< then the information will be incorrect
    UInt8 CurrentIdx_ui8; //!< index of current element
    UInt8 Number_ui8;     //!< current number of elements in the buffer

} gType_psxObjectData_st;

typedef struct
{
    gType_mtlPoint_st Track_pst[gd_PSM_SPRMaxNumOfPoints_ui8]; //!< track data in VHM
                                                               //!< coordinates
    UInt8 PrevID_ui8;             //!< PSD ID of previous request
    UInt32 SPRVirtualPoints_ui32; //!< Bit info for points stored in Track_pst,
                                  //!< Bit Set => Virtual, Reset => Real, Bit0 =>
                                  //!< Byte0 in Track_pst... Note: if
                                  //!< gd_PSM_SPRMaxNumOfPoints_ui8 > 32, then the
                                  //!< information will be incorrect
    UInt8 CurrentIdx_ui8;         //!< index of current element
    UInt8 Number_ui8;             //!< current number of elements in the buffer

} gType_psxSPRData_st;

typedef struct
{
    gType_mtlPoint_st PS_A_st;                   //!< global coordinates A
    gType_mtlPoint_st PS_B_st;                   //!< global coordinates B
    gType_mtlPoint_st PS_C_st;                   //!< global coordinates C
    gType_mtlPoint_st PS_D_st;                   //!< global coordinates D
    gType_mtlPoint_st PS_E_st;                   //!< global coordinates E
    gType_mtlPoint_st PS_F_st;                   //!< global coordinates F
    gType_PSX_ObjectType_en Object1Type_en;      //!< Object1 Type [E,A]
    gType_PSX_BottomHeightType_en BottomType_en; //!< Bottom  Type [C,D]
    gType_PSX_ObjectType_en Object2Type_en;      //!< Object2 Type [B,F]
    gType_psxPS_st SelectedUssPs_st;
    gType_mtlPoint_st curbStoneStartPoint; //!< curbstone start point
    gType_mtlPoint_st curbStoneEndPoint;   //!< curbstone end point
} gType_psm4psuSelectedUSSPSData_st;

#ifdef ENABLE_MATHLIB_ONLY

// structure of a OBB (Oriented-Bounding-Box):
typedef struct
{
    // Center-Point of the OBB
    gType_mtlPoint_st Center_st;

    // Width of the OBB
    SInt16 Width_si16;

    // Height of the OBB
    SInt16 Height_si16;

    // Orientation of the OBB
    SInt16 Orientation_si16;

} gType_psdOBB_st;

// structure with US and corresponding vehicle model data
typedef struct
{
    // X-position of the new coord.sys-origin in the VHO-coord.sys
    // Scaling: 1 | Unit: cm | Phys.Min: 0 | Phys.Max: 32767.0 |
    // Impl.Min: -32768 | Impl.Max: 32767
    SInt16 Origin_X_si16;

    // X-position of the Onew coord.sys-origin in the VHO-coord.sys
    // Scaling: 1 | Unit: cm | Phys.Min: 0 | Phys.Max: 32767.0 |
    // Impl.Min: -32768 | Impl.Max: 32767
    SInt16 Origin_Y_si16;

    // sine of the orientation of the new coord.sys-origin in the VHO-coord.sys
    // Scaling: 1024 | Unit: rad | Phys.Min: -3*pi | Phys.Max: 3*pi |
    // Impl.Min: -32768 | Impl.Max: 32767
    SInt16 SIN_Orientation1024_si16;

    // cosine of the orientation of the new coord.sys-origin in the VHO-coord.sys
    // Scaling: 1024 | Unit: rad | Phys.Min: -3*pi | Phys.Max: 3*pi |
    // Impl.Min: -32768 | Impl.Max: 32767
    SInt16 COS_Orientation1024_si16;
} gType_PSDMATH2_CoordSys_st;

// structure of a regression-line:
typedef struct
{
    // First point of the regression-line
    gType_mtlPoint_st Point1st_st;

    // Sum of the x-deltas of all points used for the regression-line
    // (x-delta = xPoint - Point1st_st.X)
    SInt32 SumDeltaX_si32;

    // Sum of the y-deltas of all points used for the regression-line
    // (y-delta = xPoint - Point1st_st.Y)
    SInt32 SumDeltaY_si32;

    // Sum of the xx-deltas of all points used for the regression-line
    // (xx-delta = (xPoint - Point1st_st.X)*(xPoint - Point1st_st.X) )
    SInt32 SumDeltaXX_si32;

    // Sum of the yy-deltas of all points used for the regression-line
    // (yy-delta = (yPoint - Point1st_st.Y)*(yPoint - Point1st_st.Y) )
    SInt32 SumDeltaYY_si32;

    // Sum of the xy-deltas of all points used for the regression-line
    // (xy-delta = (xPoint - Point1st_st.X)*(yPoint - Point1st_st.Y) )
    SInt32 SumDeltaXY_si32;

    // Number of points used for the regression-line
    UInt8 Num_ui8;
} gType_psdRegression_st;

// actual car position
extern SInt16 g_PSD_ActXPosCar_si16;
extern SInt16 g_PSD_ActYPosCar_si16;
extern SInt16 g_PSD_ActSPosCar_si16;
extern SInt16 g_PSD_ActPhiCar_si16;

#endif

#endif
