/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Tag_H_
#define DWM1000_Tag_H_
#include <Log.h>
#include <EventBus.h>
#include <Peripheral.h>
#include <DWM1000.h>
#include <DWM1000_Message.h>
#include <map>

#define ANCHOR_EXPIRE_TIME 10000
class RemoteAnchor
{
public:
    uint16_t _address;
    uint64_t _expires;
    uint8_t _sequence;
    bool expired() {
        return Sys::millis()>_expires;
    }
    void update(uint8_t sequence) {
        _expires = Sys::millis()+ANCHOR_EXPIRE_TIME;
        if ( sequence > (_sequence+1)) INFO(" dropped %d frames from 0x%X",sequence-_sequence-1,_address);
        _sequence=sequence;
    }
    RemoteAnchor(uint16_t address,uint8_t sequence) {
        _address=address;
        _expires = Sys::millis()+ANCHOR_EXPIRE_TIME;
        _sequence = sequence;

    }
};

class DWM1000_Tag: public Actor,public DWM1000
{
    uint32_t _count;
    Spi _spi;
    static DWM1000_Tag* _tag;
    uint32_t _interrupts;
    uint32_t _polls;
    bool interrupt_detected;
    uint32_t _resps;
    uint32_t _blinks;
    uint32_t _finals;
    uint32_t _frame_len;
    PollMsg _pollMsg;
    FinalMsg _finalMsg;
    DwmMsg _dwmMsg;
    Str _anchors;
    uint32_t _anchorIndex;
    uint32_t _anchorMax;
    Str _panAddress;
    uint8_t _rxdSequence;
    std::map<uint16_t,RemoteAnchor> anchors;
    enum { RCV_BLINK=H("RCV_BLINK"),SND_POLL=H("SND_POLL"),RCV_RESP=H("RCV_RESP"),SND_FINAL=H("SND_FINAL") } _state;
public:
    DWM1000_Tag(const char* name);
    virtual ~DWM1000_Tag();
    void mode(uint32_t m);
    void setup();
    void loop();
//    void resetChip();
    void initSpi();
    static void my_dwt_isr(void *);
    bool isInterruptDetected();
    bool clearInterrupt();
    void enableIsr();
    void onEvent(Cbor& msg);
    bool isRespMsg();
    void sendFinalMsg();
    static void rxcallback(const  dwt_callback_data_t* event) ;
    static void txcallback(const  dwt_callback_data_t* event) ;
    void onRxd(const  dwt_callback_data_t* signal);
    void onTxd(const  dwt_callback_data_t* signal);
    void testStl();
private:
    bool _rxd;
    bool _txd;
};

#endif /* DWM1000_Tag_H_ */
