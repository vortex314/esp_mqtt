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
#include <DWM1000.h>

class DWM1000_Anchor: public Actor,public DWM1000
{
    uint32_t _count;
    Spi _spi;
    DigitalIn _irq;
    uint32_t _interrupts;
    uint32_t _polls;
    uint32_t _finals;
    uint32_t _blinks;
    uint32_t _resps;
    uint32_t _errs;
    uint32_t _missed;
    uint8_t _lastSequence;
    static DWM1000_Anchor* _anchor;
//   enum { WAIT_POLL, WAIT_FINAL } _state;
    bool interrupt_detected ;
    uint32_t _frame_len;
    PollMsg _pollMsg;
    RespMsg _respMsg;
    FinalMsg _finalMsg;
    BlinkMsg _blinkMsg;
    DwmMsg _dwmMsg;
    Str _panAddress;
    Cbor _irqEvent;
    bool _hasIrqEvent;
    uint32_t _distanceInCm;
    uint8_t _blinkSequence;
    enum { SND_BLINK=H("SND_BLINK"),
           RCV_POLL=H("RCV_POLL"),
           SND_RESP=H("SND_RESP"),
           RCV_FINAL=H("RCV_FINAL")
         } _state;

public:
    DWM1000_Anchor(const char* name);
    virtual ~DWM1000_Anchor();
    void mode(uint32_t m);
    void setup();
//    void resetChip();
//    void initSpi();
    void onEvent(Cbor& msg);
//    void enableIsr();

    void sendReply();
    void calcFinalMsg();
    int sendRespMsg();
    bool getPollMsg(const  dwt_callback_data_t* signal);
    bool getFinalMsg(const  dwt_callback_data_t* signal);
    bool isPollMsg();
    bool isFinalMsg();
    void loop();
    static void rxcallback(const  dwt_callback_data_t* event) ;
    static void txcallback(const  dwt_callback_data_t* event) ;
    void onRxd(const  dwt_callback_data_t* signal);
    void onTxd(const  dwt_callback_data_t* signal);
    void onDWEvent(const  dwt_callback_data_t* event);
    FrameType readMsg(const dwt_callback_data_t* signal);
};

#endif /* DWM1000_Anchor_Tag_H_ */
