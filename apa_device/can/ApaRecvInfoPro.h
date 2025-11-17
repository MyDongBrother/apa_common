/*
 * CanDataType.h
 *
 *  Created on:
 *      Author:
 */

#ifndef APA_CAN_RECV_INFO_H_
#define APA_CAN_RECV_INFO_H_
#include <stdint.h>

typedef union {
    uint8_t byte;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b1 : 1;
        uint8_t b2 : 1;
        uint8_t b3 : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
    } DataL1Bit1;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b23 : 2;
        uint8_t b45 : 2;
        uint8_t b67 : 2;
    } DataL2Bit01;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b12 : 2;
        uint8_t b34 : 2;
        uint8_t b56 : 2;
        uint8_t b7 : 1;
    } DataL2Bit23;
    struct
    {
        uint8_t b02 : 3;
        uint8_t b35 : 3;
        uint8_t b67 : 2;
    } DataL3Bit02;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b13 : 3;
        uint8_t b46 : 3;
        uint8_t b7 : 1;
    } DataL3Bit13;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b24 : 3;
        uint8_t b57 : 3;
    } DataL3Bit24;
    struct
    {
        uint8_t b03 : 4;
        uint8_t b47 : 4;
    } DataL4Bit03;
    struct
    {
        uint8_t b04 : 5;
        uint8_t b57 : 3;
    } DataL5Bit04;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b15 : 5;
        uint8_t b67 : 2;
    } DataL5Bit15;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b26 : 5;
        uint8_t b7 : 1;
    } DataL5Bit26;
    struct
    {
        uint8_t b02 : 3;
        uint8_t b37 : 5;
    } DataL5Bit37;
    struct
    {
        uint8_t b05 : 6;
        uint8_t b67 : 2;
    } DataL6Bit05;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b16 : 6;
        uint8_t b7 : 1;
    } DataL6Bit16;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b27 : 6;
    } DataL6Bit27;
    struct
    {
        uint8_t b06 : 7;
        uint8_t b7 : 1;
    } DataL7Bit06;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b17 : 7;
    } DataL7Bit17;
} DataOneByte;

extern void Can_InitRecvInfo(const char *logName);

#endif /* CANDATATYPE_H_ */
