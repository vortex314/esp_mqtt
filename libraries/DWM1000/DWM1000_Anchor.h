/*
 * DWM1000_Anchor_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Anchor_H_
#define DWM1000_Anchor_H_

#include <EventBus.h>
#include <Peripheral.h>
#include <DWM1000_Message.h>

class DWM1000_Anchor: public Actor
{
    uint32_t _count;
    Spi _spi;
    DigitalIn _irq;
    uint32_t _interrupts;
    uint32_t _polls;
    uint32_t _finals;
    uint32_t _errs;
    uint32_t _missed;
    uint8_t _lastSequence;
    static DWM1000_Anchor* _anchor;
    enum { WAIT_POLL, WAIT_FINAL } _state;
    bool interrupt_detected ;
    uint32_t _frame_len;
    RespMsg respMsg;
    DwmMsg dwmMsg;
    Str _panAddress;
public:
    DWM1000_Anchor(const char* name);
    virtual ~DWM1000_Anchor();
    void mode(uint32_t m);
    void setup();
    void resetChip();
    void initSpi();
    void onEvent(Cbor& msg);
    void enableIsr();
    static void my_dwt_isr();
    bool isInterruptDetected();
    bool clearInterrupt();
    void sendReply();
    void calcFinalMsg();
    int sendRespMsg();
    bool isPollMsg();
    bool isFinalMsg();
};

#endif /* DWM1000_Anchor_Tag_H_ */
