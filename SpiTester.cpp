#include "SpiTester.h"


SpiTester::SpiTester(const char* name) :Actor(name),_outBytes(100),_inBytes(100),_spi(1)
{
}

SpiTester::~SpiTester()
{
}

uint8_t setEui[]={0x81,0,1,2,3,4,5,6,7};
Bytes setEuiResp(10);
uint8_t getEui[]={0x01,0,1,2,3,4,5,6,7};
Bytes getEuiResp(10);

void SpiTester::setup()
{
    eb.onDst(id()).call(this);
    _reportInterval=1000;
    _pollInterval=10;
    _reportCount=_reportInterval;
    for(int i=0;i<5;i++) _outBytes.write((uint8_t)0);
/*    _outBytes.write((uint8_t)0xFF);
    _outBytes.write((uint8_t)0);
    _outBytes.write((uint8_t)0x81);
    _outBytes.write((uint8_t)0x00);
    _outBytes.write((uint8_t)0xAA);*/
    timeout(5000);
    INFO(" before init");
    _spi.setClock(10000);
    _spi.setHwSelect(true);
    _spi.setMode(SPI_MODE_PHASE1_POL0);
    _spi.setLsbFirst(false);
    _spi.init();
        int pin = D1;	// RESET PIN == D1 == GPIO5
    pinMode(pin, 1);// OUTPUT
    digitalWrite(pin, 0);// PULL LOW
    Sys::delay(10);// 10ms
    digitalWrite(pin, 1);// PUT HIGH
    INFO(" after init");
}

void SpiTester::onEvent(Cbor& msg)
{
    Bytes out(0);
    if ( timeout() ) {
        timeout(_pollInterval);
        
        setEuiResp.clear();
        out.map(setEui,sizeof(setEui));
        _spi.exchange(setEuiResp,out);
        
        getEuiResp.clear();
        out.map(getEui,sizeof(getEui));
        _spi.exchange(getEuiResp,out);
        
        _reportCount -= _pollInterval;
        if ( _reportCount<0 ) {
            _reportCount = _reportInterval;
            eb.event(id(),H("setEuiResp")).addKeyValue(H("$data"),setEuiResp).addKeyValue(H("public"),true);
            eb.send();
            eb.event(id(),H("getEuiResp")).addKeyValue(H("$data"),getEuiResp).addKeyValue(H("public"),true);
            eb.send();
            eb.event(id(),H("inBytes")).addKeyValue(H("$data"),_inBytes).addKeyValue(H("public"),true);
            eb.send();
            eb.event(id(),H("outBytes")).addKeyValue(H("$data"),_outBytes).addKeyValue(H("public"),true);
            eb.send();
            eb.event(H("services"),H("spiTester")).addKeyValue(H("#data"),id()).addKeyValue(H("public"),true);
            eb.send();
        }
    } else if ( eb.isRequest(H("set"))) {
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
    } else {
        eb.reply().addKeyValue(H("error"),EINVAL).addKeyValue(H("error-msg"),"invalid request");
        eb.send();
    }
};
