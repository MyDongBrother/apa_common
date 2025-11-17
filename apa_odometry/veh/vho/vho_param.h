/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_param.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_BASE_H
#define VHO_BASE_H

#define LS_VHO_DIST_FROM_FRONT_WIC_ACKERMANN_STEER 1
#define LS_VHO_DIST_FROM_FRONT_WIC_AVG             2
#define LS_VHO_DIST_FROM_RECT_VEH_FRONT_WIC        3
#define LS_VHO_DIST_FROM_FRONT_WIC                 LS_VHO_DIST_FROM_FRONT_WIC_AVG

#define LS_VHO_DIST_FROM_REAR_WIC_ACKERMANN_STEER 4
#define LS_VHO_DIST_FROM_REAR_WIC_AVG             5
#define LS_VHO_DIST_FROM_RECT_VEH_REAR_WIC        6
#define LS_VHO_DIST_FROM_REAR_WIC                 LS_VHO_DIST_FROM_REAR_WIC_AVG

#define LS_VHO_DIST_V

/*-----------------------------------------------------------------------------
| setting for autocalibration of absolute tyre radii
\----------------------------------------------------------------------------*/
#define LS_VHO_AUTOCALIB_TYRE_RADII SW_OFF

/*-----------------------------------------------------------------------------
| setting for the velocity dependent scaling of curve radii
\----------------------------------------------------------------------------*/
#define LS_VHO_VEL_ADAPTIVE_CURVATURE SW_OFF

#ifndef VHO_USSPROC_EXTRAPOLATION_RESTRICTED
#define VHO_USS_PROC_EXTRAPOLATION_RESTRICTED SW_ON
#endif

//
//  unit            : km/h
//  resolution      : 1 bit = 0.1km/h
//
#define gd_VHOShutdownVelocitySigRes_ui16 ((UInt16)900)

//
//  unit            :
//  resolution      :
//  allowed range   :
//
//
#define md_MinWheelCircumferenceMM_ui32 ((UInt32)1500)

//
//  unit            :
//  resolution      :
//  allowed range   :
//
//
#define md_MaxWheelCircumferenceMM_ui32 ((UInt32)2500)

//  every cycle the x- and y-coordinates are updated based on either
//  straight line or circle arc approximation; if adaptive configuration
//  is chosen the decision is taken dynamically based on the yaw angle
//  change of the curent cycle; below the yaw angle change defined by
//  this parameter straight line approximation is used; in any other case
//  circle arc approximation is used
//  unit            : rad
//  resolution      : 1 bit = 1/4194304 rad
//  allowed range   : 73204,...,219613 (1掳,...,3掳)
//  default         : 146408 (2掳)
//
#define md_CurveApproxMinDPsi_ui32 ((UInt32)146408)

//  min time between two consecutive writing operations to the EEPROM
//  unit            : ms
//  resolution      : -
//  allowed range   : 600000,...,1200000
//  default         : 900000  == 15min
//
#define md_VHOMinTimeBetweenEEPROMWriteOpMS_ui32 ((UInt32)900000)

//  max velocity allowed for EEPROM write access
//  unit            : m/s
//  resolution      : 1 bit = 1/256 m/s
//  allowed range   : 1066...2133 (15-30km/h)
//  default         : 1422  == 20km/h
//
#define md_VHOMaxVeloEEPROMWriteOpMS_ui16 ((UInt16)1422)

//  number of measurements for each sensor that can be stored
//  unit            : -
//  Resolution      : -
//  allowed range   : 1,2,3,4,5
//  default         : 1 (based on the assumption that the PSD is called fast
//  enough
//                    to process one measurement before the next one will be
//                    available
//
#define gd_VHO_NUM_RM_SIZE_PER_SENSOR_ui8 ((UInt8)5)

//  maximal age of an USS measurement that shall be stored (it is
//  not possible to calculate a precise position for a measurement
//  that is too old)
//  unit            : milliseconds
//  Resolution      : 1 bit = 1 millisecond
//  allowed range   : 100,...,300
//  default         : 200
//
#define md_MaxUSSDeltaTimeMS_ui16 ((UInt16)200)

//  when the current and the last position are used to estimate positions by
//  interpolation or extrapolation this parameter defines the maximal time
//  difference between both positions
//  unit            : milliseconds
//  Resolution      : 1 bit = 1 millisecond
//  allowed range   : 100,...,300
//  default         : 300
//
#define md_MaxDeltaTimePositionMS_ui16 ((UInt16)300)

//  maximal time that two consecutive positions will be extrapolated in the past
//  or in the future
//  unit            : milliseconds
//  Resolution      : 1 bit = 1 millisecond
//  allowed range   : 100,...,500
//  default         : 200
//
#define md_MaxToleranceMS_si32 ((SInt32)200)

//  size of the time window that is used to filter the yaw rate
//  unit            : milliseconds
//  Resolution      : 1 bit = 1 millisecond
//  allowed range   : 100,200,...,500
//  default         : 200
//
#define gd_TIME_WINDOW_YR_FILT_MS_ui16 ((UInt16)200)

//  polynome coefficients that are used for velocity dependent scaling of
//  the curve radius
//  unit            : -
//  resolution      : 1 bit = 0.00001
//  default         : C_3 = 470, C_2 = 300, C_1 = 99535
//
#define md_VHOPolCoeff_3_ui32 ((UInt32)470)
#define md_VHOPolCoeff_2_ui32 ((UInt32)300)
#define md_VHOPolCoeff_1_ui32 ((UInt32)99535)

#define md_TimeBeforeStartupSigPreProcMS_ui32 ((UInt32)2000)

/******************************************************************************/
/*    DEFINES NEEDED FOR TYRE TOLERANCE COMPENSATION                          */
/******************************************************************************/

//  number of measurements that have to be gathered to complete one tyre
//  tolerance compensation cycle unit            : - resolution      : - allowed
//  range   : 200..1000 default         : 300
//
#define md_VCALTTCMinNumberOfMeasurements_ui16 ((UInt16)300)

//  maximal deviation between the biggest and the smallest radius of the tyres
//  considered in the tyre tolerance compensation; if this threshold is exceeded
//  the ttc will be set to unstable
//  unit            : %
//  resolution      : 1 bit = 1%
//  allowed range   : 2..10
//  default         : 6
//
#define md_VCALTTCMaxDevMinMaxVelPercent_ui8 ((UInt8)6)

//  minimal deviation between the wic length stored in EEPROM and a newly
//  calculated wic length determined by the ttc required to perform an EEPROM
//  write access unit            : % resolution      : 1 bit = 1/16% allowed
//  range   : 1/16%..2% default         : 3 (0.1875%)
//
#define md_VCALTTCMinDevOldNewWicLengthForEEPROMaccessPercentF4_ui8 ((UInt8)3)

//  maximal deviation between the current wic length and a newly calculated wic
//  length determined by the ttc unit            : % resolution      : 1 bit =
//  1/16% allowed range   : 1/16%..2% default         : 8 (0.5%)
//
#define md_VCALTTCMaxDevOldNewWicLengthPercentF4_ui8 ((UInt8)6)

//  weight of the newly calibrated wic length when a new value is calculated as
//  a weighted average of the old and the new value unit            : -
//  resolution      : -
//  allowed range   : 1..10
//  default         : 1
//
#define md_VCALTTCWeightNewValue_si32 ((SInt32)1)

//  weight of the current wic length when a new value is calculated as a
//  weighted average of the old and the new value unit            : - resolution
//  : - allowed range   : 1..10 default         : 1
//
#define md_VCALTTCWeightOldValue_si32 ((SInt32)1)

//  minimal vehicle velocity allowed to run tyre tolerance compensation
//
//  unit            : km/h
//  resolution      : 1 bit = 0.25 km/h
//  allowed range   : 10..40
//  default         : 17
//
#define md_VCALTTCMinVelocityMPERSF2_ui16 ((UInt16)17)

// maximal vehicle velocity allowed to run tyre tolerance compensation
//
//  unit            : km/h
//  resolution      : 1 bit = 0.25 km/h
//  allowed range   : 41..100
//  default         : 88
//
#define md_VCALTTCMaxVelocityMPERSF2_ui16 ((UInt16)88)

//  maximal vehicle acceleration allowed to run tyre tolerance compensation
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.5m/s^2
//  allowed range   : 1..20
//  default         : 2
//
#define md_VCALTTCMaxAccAbsMPERSPOW2F4_ui16 ((UInt16)2)

//  maximal yaw rate allowed to run tyre tolerance compensation (for an offset
//  corrected yaw rate signal)
//
//  unit            : 掳/s
//  resolution      : 1 bit = 0.01掳/s
//  allowed range   : 0..100
//  default         : 50
//
#define md_VCALTTCMaxYawRateAbs_ui16 ((UInt16)50)

//  minimal time that all the ttc admission conditions have to be fulfilled
//  before ttc will start
//
//  unit            : milliseconds
//  resolution      : 1 bit = 1ms
//  allowed range   : 100..1000
//  default         : 400
//
#define md_VCALTTCMinTimeMSAdmissionCond_ui16 ((UInt16)400)

/******************************************************************************/
/*    WIC SIGNAL PROCESSING                                                   */
/******************************************************************************/

//  max. possible acceleration (pos. as well as neg.) that can occur with normal
//  driving conditions
//
//  unit            : m/s^2
//  resolution      : 1 bit = 1m/s^2
//  allowed range   : 5..20
//  default         : 15
//
#define md_VCALWicMaxAcc_ui32 ((UInt32)15)

/******************************************************************************/
/*    DEFINES NEEDED FOR STEERING WHEEL ANGLE OFFSET CALIBRATION              */
/******************************************************************************/

//  max. yaw rate allowed to run steering wheel offset calibration
//
//  unit            : 掳/s
//  resolution      : 1 bit = 0.01掳/s
//  allowed range   : 1000..2000
//  default         : 1500
//
#define md_VCALSWAYawRateHighThres_si32 ((SInt32)1500)

//  min. vehicle velocity allowed to run steering wheel offset calibration
//
//  unit            : kmh
//  resolution      : 1 bit = 0.01kmh
//  allowed range   : 1500..3000
//  default         : 2300
//
#define md_VCALSWAVeloLowThres_ui16 ((UInt16)2300)

//  max. vehicle velocity allowed to run steering wheel offset calibration
//
//  unit            : kmh
//  resolution      : 1 bit = 0.01kmh
//  allowed range   : 5000..10000
//  default         : 8200
//
#define md_VCALSWAVeloHighThres_ui16 ((UInt16)8200)

//  max. steering wheel angle (pos. or neg.) allowed to run steering wheel
//  offset calibration
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 6000..8000
//  default         : 7500 (300掳)
//
#define md_VCALSWA_SWAHighThres_ui16 ((UInt16)7500)

//  min. steering wheel angle (pos. or neg.) allowed to run steering wheel
//  offset calibration
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 100..200
//  default         : 150 (6掳)
//
#define md_VCALSWA_SWALowThres_ui16 ((UInt16)150)

//  maximum deviation of relative error of right and left side to finish
//  a calibration cycle
//
//  unit            : %
//  resolution      : 1 bit = 1%
//  allowed range   : 1..5
//  default         : 3
//
#define md_VCALSWAOffsMaxDiffPercent_ui8 ((UInt8)3)

//
//  unit            : 掳
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALSWAAddtlSWA_ui16 ((UInt16)125)

//  weightage of old and new calibration value when a new value is present
//
//  unit            : -
//  resolution      : -
//  allowed range   : 1..5
//  default         : old: 3, new: 1
//
#define md_VCALSWAOffsWeightOldValue_si16 ((SInt16)3)
#define md_VCALSWAOffsWeightNewValue_si16 ((SInt16)1)

// max. difference allowed between measured and estimated steering wheel angle
// to be considered in steering wheel offset calculation
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 500..1000
//  default         : 765 (30.625掳)
//
#define md_VCALSWAMaxDiffMeasEst_si16 ((SInt16)765)

//  min time that has to be driven before a steering wheel offset calibration
//  can be finished
//  unit            : milliseconds
//  resolution      : 1 bit = 1ms
//  allowed range   : 100000..600000
//  default         : 150000 (2.5min)
//
#define md_VCALSWAOffsMinTimeUntilEvalMS_ui32 ((UInt32)150000)

//  min number of measurements needed for an autocalibration cycle of the
//  steering wheel angle offset
//
//  unit            : -
//  resolution      : 1 bit = 1 cycle
//  allowed range   : 1000..2000
//  default         : 1500
//
#define md_VCALSWAOffsMinNumMeas_ui16 ((UInt16)1500)

// max difference of the estimated steering wheel angle offsets of two
// consecutive calibration cycles
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 25..100
//  default         : 50 (2掳)
//
#define md_VCALSWAOffsMaxJitter_ui16 ((UInt16)50)

//  amplitude of the zero point noise of the yaw rate sensor
//
//  unit            : degrees per second
//  resolution      : 1 bit = 0.01掳/s
//  allowed range   : 1,...,30
//  default         : 15 (0.15掳/s)
//
#define md_VHOYawRateZeroPointNoise_ui32 ((UInt32)15)

// max difference of the estimated steering wheel angle offsets of two
// consecutive calibration cycles
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 5..30
//  default         : 10 (0.4掳)
//
#define md_VCALSWAOffsMinDevEEPROMaccess_ui16 ((UInt16)10)

/******************************************************************************/
/*    DEFINES NEEDED FOR ROLLING DIRECTION RECOGNITION - based on yaw rate    */
/******************************************************************************/

//  minimal velocity that the vehicle has to drice to realize the YawRate thres-
//  hold at given minimum steering wheel angle
//  unit            : km/h
//  resolution      : 1 bit = 0.1 km/h
//  allowed range   : 18, ...,30
//  default         : 24
#define md_VHOYawRateDirVeloThres_ui16 ((UInt16)24)

//  minimal steering wheel angle that has to be set before the rolling direction
//  will be considered to be valid
//  unit            : degrees
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 250,...,1000 (10掳,...,40掳)
//  default         : 500 (20掳)
//
#define md_VHOYawRateDirSWAThreshold_ui16 ((UInt16)500)

//  minimal time that the steering wheel angle has to be either only positive or
//  only negative before the rolling direction will considered to be valid
//  unit            : milliseconds
//  resolution      : 1 bit = 1ms
//  allowed range   : 200,...,500
//  default         : 400
//
#define md_VHOYawRateDirTimeWindowSWA_ui16 ((UInt16)400)

/******************************************************************************/
/*    DEFINES NEEDED FOR ROLLING DIRECTION RECOGNITION - based on longit. acc */
/******************************************************************************/

//  max. negative acceleration allowed to start a new direction change pattern
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   : -50,...,-10
//  default         : -20 (-0.2m/s^2)
//
#define md_VHOAccWicNegThres_si16 ((SInt16) - 20)

//  min. positive acceleration allowed to end a direction change pattern
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   : 10,...,50
//  default         : 20 (0.2m/s^2)
//
#define md_VHOAccWicPosThres_si16 ((SInt16)20)

//  min. difference between start and end point of a direction change
//  pattern to consider it valid
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   : 50,...,100
//  default         : 70 (0.7m/s^2)
//
#define md_VHOAccWicMinDelta_ui16 ((UInt16)70)

//  max. decrease of acceleration that is allowed to happen inside a
//  direction change pattern
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   : 10,...,40
//  default         : 30 (0.3m/s^2)
//
#define md_VHOAccWicMaxFallback_si16 ((SInt16)30)

//  max. acceleration gradient allowed to end a direction change pattern
//
//  unit            : m/s^3
//  resolution      : 1 bit = 0.01m/s^3
//  allowed range   : 100,...,500
//  default         : 400 (4m/s^3)
//
#define md_VHOAccWicMaxGradient_si16 ((SInt16)400)

//  max. duration of a direction change pattern
//
//  unit            : milliseconds
//  resolution      : 1 bit = 1ms
//  allowed range   : 500,...,1000ms
//  default         : 700
//
#define md_VHODirChangeTimeThres_ui16 ((UInt16)1800)

//
//  unit            : m/s^2
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHOMaxDistAccFromMin_ui16 ((UInt16)10)

//  max. velocity allowed inside direction change pattern
//
//  unit            : m/s
//  resolution      : 1 bit = 1/256m/s
//  allowed range   : 0..256
//  default         : 71
//
#define md_VHODirChangeMaxVel_ui16 ((UInt16)150)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointMaxVel_ui16 ((UInt16)25)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointInitialTimeMS_ui16 ((UInt16)200)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroMinCalibTimeMS_ui16 ((UInt16)300)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointTimeBasePointEvalMS_ui16 ((UInt16)300)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointMaxDistCycleMM_ui16 ((UInt16)300)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointMaxDistBeforeDiscardMM_ui16 ((UInt16)2000)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirZeroPointWeightBaseValueMS_ui16 ((UInt16)150)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VHODirIntegVelThresMax_ui16 ((UInt16)25)

//  detection +/- threshold; above/below this threshold the driving direction
//  will be estimated by comparison the gradients of the acc signal from the
//  sensor and the acc signal based on the wic signal
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   : 0,...,256
//  default         : 50 (0.5 m/s^2)
//
#define md_VHODirAccGradDetectThresh_ui8 ((UInt8)50)

//  window size to estimate driving direction based on the gradient of of the
//  acc signal from the sensor and the acc signal based on the wic signal
//
//  unit            :
//  resolution      :
//  allowed range   : 0,...,256
//  default         : 3
//
#define md_VHODirAccGradDetectWindow_ui8 ((UInt8)3)

/******************************************************************************/
/*    DEFINES NEEDED FOR YAW RATE OFFSET CALIBRATION                          */
/******************************************************************************/

// duration of the initial break-in phase of the yaw rate sensor
// (zero point of yaw rate less stable before end of break-in phase)
//  unit            : milliseconds
//  resolution      : 1 bit = 1 ms
//  allowed range   : 60000..480000
//  default         : 240000 (= 4 min)
//
#define md_VCALYRInitialBreakinPhaseMS_ui32 ((UInt32)240000)

// yaw rate offset correction will calculate the average yaw rate over a
// time frame of specified duration; Parameters are used to define the
// min and max size of the time frame
//  unit            : milliseconds
//  resolution      : 1 bit = 1ms
//  allowed range   : min (100..800), max(1500..3000)
//  default         : min: 500, max: 3000
//
#define md_VCALYRMinTimeFrameMS_ui16 ((UInt16)500)
#define md_VCALYRMaxTimeFrameMS_ui16 ((UInt16)3000)

//  weightage of old and new yaw rate offset when a new value is present
//
//  unit            : -
//  resolution      : -
//  allowed range   : 1..5
//  default         : old: 3, new: 1
//
#define md_VCALYRWeightOldValue_si16 ((SInt16)1)
#define md_VCALYRWeightNewValue_si16 ((SInt16)1)

// time that the vehicle has to remain in stillstand before
// the yaw rate offset will be calibrated (it takes time after
// stillstand until resonation will be zero)
//  unit            : milliseconds
//  resolution      : 1 bit = 1 ms
//  allowed range   : 300..800
//  default         : 650
//
#define md_VCALYRStillstTimeThresMS_ui16 ((UInt16)650)

//  max. reliable yaw rate offset (max. offset from PH-spec + x)
//  unit            : 掳/s
//  resolution      : 1 bit = 0.01掳/s
//  allowed range   : 250..500
//  default         : 300
//
#define md_VCALMinMaxYROffset_si16 ((SInt16)300)

//  max. reliable yaw rate while vehicle is standing still
//  above this value the error handler will be called.
//  unit            : 掳/s
//  resolution      : 1 bit = 0.01掳/s
//  allowed range   : 250..500
//  default         : 350
//
#define md_VCALMaxYRWhileStandingStill_si16 ((SInt16)350)

//  very high SWA start stand still to SWA difference every cycle time at
//  standing still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed
//  range   : 10000 ... 18750   (400掳...750掳) default         : 12500 (500掳)
//
#define md_VCALVeryHighSWADifferenceAbs_ui16 ((UInt16)12500)

//  high SWA start stand still to SWA difference every cycle time at standing
//  still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed range   :
//  5000 ... 7500   (200掳...300掳) default         : 6250            (250掳)
//
#define md_VCALHighSWADifferenceAbs_ui16 ((UInt16)6250)

//  mid SWA start stand still to SWA difference every cycle time at standing
//  still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed range   :
//  2500 ... 3750   (100掳...150掳) default         : 3125            (125掳)
//
#define md_VCALMidSWADifferenceAbs_ui16 ((UInt16)3125)

//  very high SWA start stand still reduction threshold every cycle time at
//  standing still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed
//  range   : 375 ... 625   (15掳...25掳) default         : 500           (20掳)
//
#define md_VCALVeryHighSWAStandstillRedThres_si16 ((SInt16)500)

//  high SWA start stand still reduction threshold every cycle time at standing
//  still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed range   :
//  200 ... 375   (8掳...15掳) default         : 250           (10掳)
//
#define md_VCALHighSWAStandstillRedThres_si16 ((SInt16)250)

//  mid SWA start stand still reduction threshold every cycle time at standing
//  still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed range   :
//  100 ... 200   (4掳...8掳) default         : 125           (5掳)
//
#define md_VCALMidSWAStandstillRedThres_si16 ((SInt16)125)

//  low SWA start stand still reduction threshold every cycle time at standing
//  still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed range   :
//  50  ... 100   (2掳...4掳) default         : 75            (3掳)
//
#define md_VCALLowSWAStandstillRedThres_si16 ((SInt16)75)

//  very low SWA start stand still reduction threshold every cycle time at
//  standing still unit            : 掳 resolution      : 1 bit = 0.04掳 allowed
//  range   : 50  ... 100   (0掳...1掳) default         : 5             (0.2掳)
//
#define md_VCALVeryLowSWAStandstillRedThres_si16 ((SInt16)5)

//  max allowed SWA changes while vehicle is standing still
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 375 ... 3750   (15掳... 150掳)
//  default         : 750            (30掳)
//
#define md_VCALMaxSWAChangesStandingStill_ui16 ((UInt16)750)

/******************************************************************************/
/*    DEFINES NEEDED FOR TYRE RADII CALIBRATION                               */
/******************************************************************************/

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreVeloLowThres_ui16 ((UInt16)500)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreVeloHighThres_ui16 ((UInt16)5000)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreYawRateHighthres_si32 ((SInt32)3500)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreSALowThres_si16 ((SInt16)1072)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreMinNumberOfMeas_ui16 ((UInt16)1000)

// min time that has to be driven before evaluation of tyre radii
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreMinTimeUntilEvalMS_ui32 ((UInt32)600000)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreMinNumMeasPerSlot_ui16 ((UInt16)50)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreOffsWeightOldValue_si16 ((SInt16)3)

//
//  unit            :
//  resolution      : 1 bit =
//  allowed range   :
//  default         :
//
#define md_VCALTyreOffsWeightNewValue_si16 ((SInt16)1)

/******************************************************************************/
/*    DEFINES NEEDED FOR DIRECTION RECOGNITION                                */
/******************************************************************************/
#define md_VHODirIntegMaxDistAfterStandstillMMF8_ui32 ((UInt32)64000)

//  the following three parameters specify the conditions to evaluate the gear
//  for the direction recognition
//
//  unit            : m/s^3
//  resolution      : 1 bit = 0.01m/s^3
//  allowed range   :
//  default         : 180
//
#define md_VHODirGearAxGradThres_ui16 ((UInt16)180)
//
//  unit            : m/s^3
//  resolution      : 1 bit = 0.01m/s^3
//  allowed range   :
//  default         : 0
//
#define md_VHODirGearAccWICGradThres_si16 ((SInt16)0)
//
//  unit            : m/s^2
//  resolution      : 1 bit = 0.01m/s^2
//  allowed range   :
//  default         : 100
//
#define md_VHODirGearAccWICThres_si16 ((SInt16)100)

//  number of cycles that the conditions for the gear criteria have to be
//  fulfilled
//
//  unit            : -
//  resolution      : 1 bit = 1 cycle
//  allowed range   :
//  default         : 2
//
#define md_VHODirGearNumCycles_ui8 ((UInt8)2)

//  after a change pattern has been detected the gradient criteria has to be
//  considered unstable; it will be locked for a time period specified by this
//  parameter
//
//  unit            : ms
//  resolution      : 1 bit = 1ms
//  allowed range   :
//  default         : 1300
//
#define md_VHODirCtrlAccGradLockTimeMS_ui16 ((UInt16)1300)

//  first speed threshold for direction recognition; above that speed limit it
//  is assumed that a change of direction is impossible => no evaluation
//
//  unit            : m/s
//  resolution      : 1 bit = 1/256 m/s
//  allowed range   :
//  default         : 568 (8 km/h)
//
#define md_VHODirVelThres1MperSF8_ui16 ((UInt16)568)

//  second speed threshold for direction recognition; above that speed limit the
//  gear information is used to set the driving direction
//
//  unit            : m/s
//  resolution      : 1 bit = 1/256 m/s
//  allowed range   :
//  default         : 1422 (20 km/h)
//
#define md_VHODirVelThres2MperSF8_ui16 ((UInt16)1422)

//  steering wheel angle threshold; if the norm of the steering wheel angle is
//  above this threshold the direction recognition control will generally prefer
//  the yaw angle criteria to criteria based on the acceleration gradient
//
//  unit            : 掳
//  resolution      : 1 bit = 0.04掳
//  allowed range   : 1250...12500
//  default         : 5000 (200掳)
//
#define md_VHODirCtrlSWAThres_ui16 ((UInt16)5000)

// FURTHER DEFINITIONS AND MACROS
#define md_VHOYawRateDirYRThres_ui32 ((UInt32)3 * md_VHOYawRateZeroPointNoise_ui32)

#define gd_VHOUSS_RM_SIZE_ui8 (gd_VHO_NUM_RM_SIZE_PER_SENSOR_ui8)
// #define gd_RM_SIZE_VEL_ui8         ((UInt8)
//(gd_TIME_WINDOW_VEL_CALC_MS_ui16/(UInt16)gd_VHO_WIC_CYCLETIME_MS_ui32))
#define md_VHOYawRateDirSWACntThres_ui16          \
    ((UInt8)(md_VHOYawRateDirTimeWindowSWA_ui16 / \
             (UInt16)g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16))

#define LS_VHO_YES 1
#define LS_VHO_NO  2

#define LS_VHO_INCLUDE_PROTOTYPES LS_VHO_NO

// cycletime for virtual WIC timestamps in case of degredation mode
// extrapolation and WIC timeout
#define gd_VHO_DEGMODEEXTRAPOL_VIRT_WIC_CYCLETIME_MS_ui32 ((UInt32)100)

/******************************************************************************/
/*   DEFINES NEEDED for Elephant Test                                         */
/******************************************************************************/

//  Minimum number of tires that have to move be certain that vehicle moves
//
//  unit            : -
//  resolution      : -
//  allowed range   : 1..4
//  default         : 2
//
#define md_VHO_MinNumberMovTires_ui8 ((UInt8)2)

//  Maximum number of allowed WIC changes second per wheel for still detecting
//  standstill
//
//  unit            : WIC changes per second
//  resolution      : -
//  allowed range   : 1..3
//  default         : 2
//
#define md_VHO_MaxWICDelta_ui8 ((UInt8)2)

//  Length of time window
//
//  unit            : ms
//  resolution      : 1 bit = 1 ms
//  allowed range   :
//  default         : 1000
//
#define md_VHO_TimeThreshold1_ui16 ((UInt16)1000)

//  Length of extended time window
//
//  unit            : ms
//  resolution      : 1 bit = 1 ms
//  allowed range   :
//  default         : 1500
//
#define md_VHO_TimeThreshold2_ui16 ((UInt16)1500)

#endif // #ifndef VHO_BASE_H
