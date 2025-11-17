#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include "Std_Types.h"
#include "SystemPara.h"
#include "PK_Calibration_Config.h"
#include "PK_Calibration_Debug.h"

void PK_Calibration();

int PK_CalibrationPrint(const int index, char buff[128]);

int PK_VehConfigType();

int PK_Calibration_GetDebugValue(const char *key, char buff[64]);

#endif
