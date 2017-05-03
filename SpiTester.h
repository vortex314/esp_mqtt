#ifndef SPITESTER_H
#define SPITESTER_H

#include <EventBus.h>
#include <Peripheral.h>
#include <Bytes.h>

class SpiTester : public Actor
{
    uint32_t _address;
    uint32_t _value;
    Bytes _outBytes;
    Bytes _inBytes;
    Spi _spi;
    uint32_t _pollInterval;
    uint32_t _reportInterval;
    int32_t _reportCount;
    
public:
    SpiTester(const char* name);
    ~SpiTester();
   
    void setup() ;
    void onEvent(Cbor& msg) ;
};

#endif // SPITESTER_H
