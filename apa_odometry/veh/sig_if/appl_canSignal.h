
#ifndef APPL_CANSIGNAL_H
#define APPL_CANSIGNAL_H

/*--------------------------------------------------------------------------*/
/*                                                                          */
/*include files                                                              */
/*                                                                          */
/*--------------------------------------------------------------------------*/
//
#include <pa_components/base_type/global_include.h>

typedef struct g_RX_Signals
{

    // wic
    UInt16 ESC_FLWheelSpeedRC;
    UInt16 ESC_FRWheelSpeedRC;
    UInt16 ESC_RLWheelSpeedRC;
    UInt16 ESC_RRWheelSpeedRC;
    // wic tick
    UInt16 FL_WicTick_ui16;
    UInt16 FR_WicTick_ui16;
    UInt16 RL_WicTick_ui16;
    UInt16 RR_WicTick_ui16;

    // wic valid   0:valid  1: invalid
    UInt8 ESC_FLWheelSpeedRC_valid;
    UInt8 ESC_FRWheelSpeedRC_valid;
    UInt8 ESC_RLWheelSpeedRC_valid;
    UInt8 ESC_RRWheelSpeedRC_valid;
    // wheel direction
    // 0: standstill  1: forward 2: backward  3:invalid
    UInt8 ESC_FLWheelDirection;
    UInt8 ESC_FRWheelDirection;
    UInt8 ESC_RLWheelDirection;
    UInt8 ESC_RRWheelDirection;

    // gear
    // 0:P  1:R  2:N 3:D  4:invalid
    UInt8 Gear_ui8;

    // steer angle
    // 0.01 degree
    SInt32 SteerWheelAngle_si32;
    // steer angle valid
    // 0: valid   1: invalid
    UInt8 SteerWheelAngle_valid;

    // wheel speed: 0.01km/h
    UInt16 ESC_FLWheelSpeedKPH_ui16;
    UInt16 ESC_FRWheelSpeedKPH_ui16;
    UInt16 ESC_RLWheelSpeedKPH_ui16;
    UInt16 ESC_RRWheelSpeedKPH_ui16;
    // wheel speed valid
    // 0: valid  1: invalid
    UInt8 ESC_FLWheelSpeed_valid;
    UInt8 ESC_FRWheelSpeed_valid;
    UInt8 ESC_RLWheelSpeed_valid;
    UInt8 ESC_RRWheelSpeed_valid;

    // vehicle speed : 0.1km/h
    UInt16 VehicleSpeed_ui16;
    // 0: valid  1: invalid
    UInt8 VehicleSpeed_valid;

    // 0.01 m/s^2
    SInt16 LateralAcce_si16;
    // 0: valid  1: invalid
    UInt8 LateralAcce_valid;
    // 0.01 m/s^2
    SInt16 LongitAcce_si16;
    // 0: valid  1: invalid
    UInt8 LongitAcce_valid;

    // 0.01 degree/s
    SInt16 YawRate_si16;
    // 0: valid  1: invalid
    UInt8 YawRate_valid;

    // 1 degree
    SInt8 Temperature_si8;
    // 0: valid  1: invalid
    UInt8 TemperatureInvalid;

    // 0: middle  1: left 2: right 3: WarningLights
    UInt8 TurnIndicator;

    // power mode
    // 0: off  1: acc on  2: IGN on
    UInt8 PowerMode;
    // 0: valid  1: invalid
    UInt8 PowerMode_Valid;

    // epb status
    // 0: realse 1: close 2: in pocess 3:default value
    UInt8 EPBStatus;
    // engine status
    // 0: stop  1: running  2:invalid
    UInt8 EngineStatus;

    // 0:no rain  1: light rain 2: middle rain  3: heavy rain  4:invalid
    UInt8 RainfallLevel;
    // 0: stop 1: slow 2: middle 3:fast 4 :invalid
    UInt8 BCM_WiperSts;

    // 0: not present  1: trailer hitch present
    UInt8 TrailerHitchPresent_ui8;
    // 0: not attached  1: attached
    UInt8 TrailerAttached_ui8;

    // 0: Ready  1: Active 2: aborted  3: inactive  4: error
    UInt8 EPS_status;
    // 0: Ready  1: Active 2: aborted  3: inactive  4: error
    UInt8 ESC_Status;

} gType_RX_Signals;

typedef struct gT_ApplTXSignals_st
{
    UInt8 noSignal;

} gType_TX_Signals;

typedef struct gT_ApplSignals_st
{
    gType_TX_Signals TXSignals_st;
    gType_RX_Signals RXSignals_st;

} gType_ApplSignals_st;

extern gType_ApplSignals_st g_ApplCanSig_st;

#endif