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
    float _distance;
 //   uint32_t _distanceInCm;
    uint8_t _blinkSequence;
    typedef  enum { RCV_ANY=H("RCV_ANY"),
                    RCV_POLL=H("RCV_POLL"),
                    RCV_FINAL=H("RCV_FINAL")
                  } State;
    State _state;
    Timeout _blinkTimer;


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

    void loop();
    void update(uint16_t srcAddress,uint8_t sequence);
    static void rxcallback(const  dwt_callback_data_t* event) ;
    static void txcallback(const  dwt_callback_data_t* event) ;

    void FSM(const dwt_callback_data_t* signal);
    void onDWEvent(const  dwt_callback_data_t* event);
    FrameType readMsg(const dwt_callback_data_t* signal);
    void sendBlinkMsg();
    void handleFinalMsg();


};

#endif /* DWM1000_Anchor_Tag_H_ */
