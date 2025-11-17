#ifndef ETHENET_VISION_H
#define ETHENET_VISION_H
#include "Rte_Types.h"

using VisionMsgRecvPtr = void *(*)(void *paras);

VisionMsgRecvPtr GetVisionLoopFunc();

#endif