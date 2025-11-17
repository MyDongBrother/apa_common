#ifndef PK_J3ETHNET_H_
#define PK_J3ETHNET_H_

#include "Rte_Func.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

void Eth_PushCanData(const uint16_t *canId, const uint8_t data[][8],
                     const uint32_t number);

void Eth_InitVisionSlot();

void Eth_InitUlatroSlot();

void Eth_InitLedDisplay();

void Eth_InitVisionSta();

void Eth_InitHeartBeat();

#endif