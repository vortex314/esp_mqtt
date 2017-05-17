#include <stdint.h>
#ifndef DWM1000_Message_H_
#define DWM1000_Message_H_

typedef union {
    uint8_t buffer[];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        union  {
            struct {
                uint8_t crc[2];
            } __attribute__ ((packed)) poll;
            struct {
                uint8_t activity;
                uint8_t activityParameter[2];
                uint8_t crc[2];
            } __attribute__ ((packed)) resp;
            struct {
                uint8_t pollTimestamp[4];
                uint8_t respTimestamp[4];
                uint8_t finalTimestamp[4];
                uint8_t crc[2];
            } __attribute__ ((packed)) finalMsg;
        };
    } __attribute__ ((packed)) ;
} __attribute__ ((packed)) DwmMsg;

typedef  union {
    uint8_t buffer[];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t crc[2];
    } ;
} __attribute__ ((packed)) PollMsg ;

typedef  union {
    uint8_t buffer[];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t activity;
        uint8_t activityParameter[2];
        uint8_t crc[2];
    } ;
} __attribute__ ((packed)) RespMsg ;

typedef  union {
    uint8_t buffer[];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t pollTimestamp[4];
        uint8_t respTimestamp[4];
        uint8_t finalTimestamp[4];
        uint8_t crc[2];
    } ;
} __attribute__ ((packed)) FinalMsg ;

#define FUNC_POLL_MSG  0x21
#define FUNC_RESP_MSG  0x10
#define FUNC_FINAL_MSG  0x23
#define ACT_CODE 0x2

#endif
