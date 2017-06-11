/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_H_
#define DWM1000_H_
#include <Log.h>
#include <EventBus.h>
#include <Peripheral.h>
#include <DWM1000_Message.h>


extern "C" {
#include <spi.h>
#include <gpio_c.h>
#include <espmissingincludes.h>
#include <osapi.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
}

#define MAX_MESSAGE 5

typedef void (*InterruptHandler)(void* instance);
typedef enum {
    FT_BLINK,FT_POLL,FT_RESP,FT_FINAL,FT_UNKNOWN
} FrameType;

class DWM1000
{
    uint32_t _count;
    Spi _spi;
/*    DWM1000* _me;
    uint32_t _interrupts;
    uint32_t _polls;
    bool interrupt_detected;
    */

    uint8_t _longAddress[8];
    uint16_t _shortAddress;

    dwt_config_t _config;
    uint8_t _channel;
    uint8_t _prf;
    uint8_t _preambleLength;
    uint8_t _dataRate;
    uint8_t _pacSize;
    uint8_t _sequence;

public:
    DWM1000();
    virtual ~DWM1000();
    void mode(uint32_t m);
    void setup();
    void resetChip();
    void initSpi();
    void enableIsr();
    void onEvent(Cbor& msg);
    bool isRespMsg();
    void sendFinalMsg();
    void setShortAddress(uint16_t address);
    void setLongAddress(uint8_t address[]);
    static FrameType getFrameType(DwmMsg& msg);
    static FrameType getFrameType(uint8_t fc[]);
    void createBlinkFrame(BlinkMsg& blink);
    void createPollMsg(PollMsg& pollMsg,uint16_t address);
    void createRespMsg(RespMsg& respMsg,PollMsg& pollMsg);
    void tune();

private:

    DwmMsg _rcvMsg[MAX_MESSAGE];


};

#endif /* DWM1000_H_ */
