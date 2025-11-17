#ifndef J3SPIAPI_H
#define J3SPIAPI_H

#ifdef __cplusplus
extern "C"
{
#endif

    struct can_header
    {
        int64_t time_stamp;
        uint8_t counter;
        uint8_t frame_num;
        uint8_t channel;
        uint8_t type;
        uint8_t reserve[4];
    };

    int spientry();
    void spi_pushdata(const uint16_t *canId, const uint8_t data[][8],
                      const uint32_t number);

#ifdef __cplusplus
}
#endif

#endif