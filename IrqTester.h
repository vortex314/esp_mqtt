#ifndef IRQTESTER_H
#define IRQTESTER_H

#include <EventBus.h>
#include <Peripheral.h>
#include <Bytes.h>

class IrqTester : public Actor
{
    public:
    uint64_t _startIrq;
    uint64_t _deltaIrq;
    uint64_t _irqCount;
    uint32_t _pollInterval;
    uint32_t _reportInterval;
    int32_t _reportCount;
    
public:
    IrqTester(const char* name);
    ~IrqTester();
   
    void setup() ;
    void onEvent(Cbor& msg) ;
};

#endif // IRQTESTER_H
