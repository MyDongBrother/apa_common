#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "MathFunc.h"
#include "Rte.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void PK_Location(void);

int PK_PreLocation(const uint32_t preMs, RD_PosStamp *posStamp);

#endif
