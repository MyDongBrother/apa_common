#ifndef APACTRL_LIBAPACTRL_H_
#define APACTRL_LIBAPACTRL_H_
#include "Std_Types.h"

#define candata_bytes 8 /**< CAN数据长度，固定字节长度 */

/**
 * @enum CurrApaSta
 * @brief APA（自动泊车辅助）当前状态枚举
 */
typedef enum
{
    APA_STANDBY = 0,
    APA_ACTIVE,
    APA_COMPLETE,
    APA_DISTAN
} CurrApaSta;

extern void Can_PushData(const uint16_t canId, const uint8_t data[candata_bytes]);

extern int Apa_CanInit(const char *logName);

#endif /* APACTRL_LIBAPACTRL_H_ */
