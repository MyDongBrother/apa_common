/*
 * ApaCtrlCarBody.h
 *
 *  Created on: 2021年9月1日
 *      Author: longyunx
 */

#ifndef APACTRL_APACTRLCARBODY_H_
#define APACTRL_APACTRLCARBODY_H_
#include "ApaCtrlVarDef.h"
#include "CanIf.h"

#define ANGLE_MAX           12600 // 480 deg
#define ANGLE_MIN           3000  //-480 deg
#define ANGLE_SPD           50    // deg/s
#define TARGET_ANGLE_OFFSET 7800
#define TARGET_ANGLE_STEP   120

#define PATH_CNT 6

#define TIRE_CIRC         218 // CM
#define PLUSS_NUM_ONE_CYC 96
#define PLUSS_EEROR_CNT   50

#define CHANGE_PLUSS_CNT_DEF 300

typedef enum
{
    SCU_STAS_Invalid = 0,
    SCU_STAS_standby,
    SCU_STAS_available,
    SCU_STAS_inhibit,
    SCU_STAS_reserved
} ScuApaSts;

typedef enum
{
    SCU_INH_NoOperation = 0,
    SCU_INH_NormalOperation,
    SCU_INH_DriverGearInterference,
    SCU_INH_reserved
} ScuApaInh;

enum
{
    TIRE_NO_FL = 0,
    TIRE_NO_FR,
    TIRE_NO_RL,
    TIRE_NO_RR,
    TIRE_NO_MAX
};

typedef struct
{
    uint16_t preValue;
    uint16_t currValue;
    uint16_t addValue;
} DiffValUpdate;

typedef struct
{
    uint8_t preStatus;
    uint8_t currStatus;
} ChgStatus;

enum
{
    INVALID = 0,
    VALID   = 1
};

extern void Can_InitSendInfo();
#endif /* APACTRL_APACTRLCARBODY_H_ */
