#include "IrqTester.h"
#include <Arduino.h>

IrqTester* _gIrq;
void isr()
{
    _gIrq->_deltaIrq = micros() - _gIrq->_startIrq;
    _gIrq->_irqCount++;
}

IrqTester::IrqTester(const char* name) :Actor(name)
{
    _gIrq=this;
}

IrqTester::~IrqTester()
{
}

#define PIN_IRQ_IN D2// PIN == D2 == GPIO4
#define PIN_IRQ_OUT D1 // PIN == D1 == GPIO5

// loopback between D1 and D2 == test speed delay of IRQ ==> result 4-5 µSec

void IrqTester::setup()
{

    eb.onDst(id()).call(this);
    pinMode(PIN_IRQ_OUT, OUTPUT);// OUTPUT
    pinMode(PIN_IRQ_IN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_IRQ_IN),isr,RISING);

    _reportInterval=1000;
    _pollInterval=10;
    _reportCount=_reportInterval;
    _irqCount=0;

    timeout(5000);
}



void IrqTester::onEvent(Cbor& msg)
{
    Bytes out(0);
    if ( timeout() ) {
        timeout(_pollInterval);
        _startIrq = micros();
        digitalWrite(PIN_IRQ_OUT, 1);// PULL HIGH
        Sys::delay(10);// 10ms
        digitalWrite(PIN_IRQ_OUT, 0);// PUT LOW



        _reportCount -= _pollInterval;
        if ( _reportCount<0 ) {
            _reportCount = _reportInterval;
            eb.event(id(),H("irqCount")).addKeyValue(H("data"),_irqCount).addKeyValue(H("public"),true);
            eb.send();
            eb.event(id(),H("delta")).addKeyValue(H("data"),_deltaIrq).addKeyValue(H("public"),true);
            eb.send();
            eb.event(id(),H("startIrq")).addKeyValue(H("data"),_startIrq).addKeyValue(H("public"),true);
            eb.send();
            eb.event(H("services"),H("irqTester")).addKeyValue(H("#data"),id()).addKeyValue(H("public"),true);
            eb.send();
        }
    } /*else if ( eb.isRequest(H("set"))) {
        if ( msg.getKeyValue(H("$outBytes"),_outBytes) ) {
            *(uint32_t*)_address = _value;
            eb.reply().addKeyValue(H("$outBytes"),_outBytes);
            eb.send();
        };
        if ( msg.getKeyValue(H("pollInterval"),_pollInterval) ) {
            eb.reply().addKeyValue(H("pollInterval"),_pollInterval);
            eb.send();
        };
        if ( msg.getKeyValue(H("reportInterval"),_reportInterval) ) {
            eb.reply().addKeyValue(H("reportInterval"),_reportInterval);
            eb.send();
        };
    }*/ else {
        eb.reply().addKeyValue(H("error"),EINVAL).addKeyValue(H("error-msg"),"invalid request");
        eb.send();
    }
};
