#ifndef RECORD_BUFFER_HEADER__
#define RECORD_BUFFER_HEADER__
#include "Std_Types.h"

void *Buff_Init(uint8_t *buff, const int buffLen, FILE *logFd);

void Buff_UnInit(void *bufFd);

int Buff_Put(void *bufFd, const char *data, const int dataLen);

int Buff_Get(void *bufFd, uint8_t *data, const int dataLen);

int Buff_Len(void *bufFd);

#endif
