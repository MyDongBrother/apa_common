#include "Rte.h"

void BuildAvmCanData1(uint8_t data[10])
{
    /**
     * @brief AVM 相关 CAN 数据结构体
     */
    struct can_data
    {
        uint16_t canId; /**< CAN 消息 ID */

        unsigned spdS : 1;   /**< 车速信号状态位：0-无效，1-有效 */
        unsigned epsS : 1;   /**< 转向信号状态位：0-无效，1-有效 */
        unsigned yawS : 1;   /**< 横摆角速度信号状态位：0-无效，1-有效 */
        unsigned : 1;        /**< 保留位 */
        unsigned spdV : 12;  /**< 车速值（单位和缩放系数需参照协议） */
        signed epsV : 12;    /**< 转角值（单位：一般为角度或转向盘角度） */
        signed yawV : 12;    /**< 横摆角速度值（单位：一般为 deg/s 或 rad/s） */
        unsigned lledS : 1;  /**< 左转向灯状态位：0-无效，1-有效 */
        unsigned lledV : 2;  /**< 左转向灯状态值：0-关闭，1-开启，2-闪烁等 */
        unsigned rledS : 1;  /**< 右转向灯状态位：0-无效，1-有效 */
        unsigned rledV : 2;  /**< 右转向灯状态值：0-关闭，1-开启，2-闪烁等 */
        unsigned brakeS : 1; /**< 制动状态信号位：0-无效，1-有效 */
        unsigned brakeV : 2; /**< 制动状态值：0-无制动，1-轻踩，2-重踩等 */
        unsigned gearS : 1;  /**< 挡位信号状态位：0-无效，1-有效 */
        unsigned gearV : 2;  /**< 当前挡位：0-P，1-R，2-N，3-D 等 */
        unsigned : 3;        /**< 保留位（用于字节对齐） */

        uint8_t stamp; /**< 时间戳（通常用于同步或时序判断） */
    } avmCanData;

    avmCanData.canId = 0x6400;
    // BIT0: Speed status 0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.spdS = RTE_BSW_Get_ESC_VehSpdVD();
    // BIT1: Eps Angle status 0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.epsS = (RTE_BSW_Get_EPS_AngleSt() == 0) ? 1 : 0;
    // BIT2: yaw status 0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.yawS = (RTE_BSW_Get_EPS_AngleSt() == 0) ? 1 : 0;

    // BIT4-BIT15: Vehicle_speed  0.06875km/h [0; 281.4625] km/h Default:0xFFF"
    avmCanData.spdV = (uint16_t)(RTE_BSW_Get_ESC_VehSpd() / 0.06875);

    // BIT16-BIT27: AngleValue 1dag [+/-780] dag Default:0xFFF
    avmCanData.epsV = (int16_t)(RTE_BSW_Get_EPS_Angle());

    // BIT28-BIT39: yaw 0.1dag [+/-180] dag Default:0x0
    float value     = RTE_BSW_Get_ESC_YAW_RATE() / 0.002132603;
    avmCanData.yawV = (int16_t)(value);

    // BIT40: Left turn lights status  0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.lledS = 1;

    // BIT41-42: Left turn lights value  Default:0x0
    avmCanData.lledV = 0;

    // BIT43: Right turn lights status  0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.rledS = 1;

    // BIT44-45: Right turn lights value Default:0x0
    avmCanData.rledV = 0;

    // BIT46: Brake status  0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.brakeS = 1;

    // BIT47-48: Brake value  [0-7] Reserved  Init:0x0 / Default:0x0
    avmCanData.brakeV = 0;

    // BIT49: Gear status  0x0:Valid 0x1:Invalid Init:0x0 / Default:0x1
    avmCanData.gearS = 0;

    // BIT50-52: Gear value  [0-5]  0:GEAR_REAL_NONE, 1:GEAR_REAL_D,  2:GEAR_REAL_N,  3:
    // GEAR_REAL_R, 4: GEAR_REAL_P, GEAR_REAL_MAX
    avmCanData.gearV = RTE_BSW_Get_CurrentGear();

    // BIT53-55: Reserved 0
    // BIT56-63: Time stamp 1ms [0-255] ms Default: 0
    avmCanData.stamp = RTE_BSW_Get_CurTime() & 0xff;
    memcpy(data, &avmCanData, 10);
}

void BuildAvmCanData2(uint8_t data[10])
{
    struct can_data
    {
        uint16_t canId;
        unsigned fl : 12;
        unsigned fr : 12;
        unsigned rl : 12;
        unsigned rr : 12;
        uint8_t stamp;
    } WheelCountData;

    WheelCountData.canId = 0x6401;

    // 0 ~ 1022
    uint16_t value    = RTE_BSW_Get_WheelCounter_FL();
    WheelCountData.fl = (uint16_t)(value);

    value             = RTE_BSW_Get_WheelCounter_FR();
    WheelCountData.fr = (uint16_t)(value);

    value             = RTE_BSW_Get_WheelCounter_RL();
    WheelCountData.rl = (uint16_t)(value);

    value             = RTE_BSW_Get_WheelCounter_RR();
    WheelCountData.rr = (uint16_t)(value);

    WheelCountData.stamp = RTE_BSW_Get_CurTime() & 0xff;
    memcpy(data, &WheelCountData, 10);
}
