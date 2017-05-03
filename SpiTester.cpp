#include "SpiTester.h"


SpiTester::SpiTester(const char* name) :Actor(name),_outBytes(100),_inBytes(100),_spi(1)
{
}

SpiTester::~SpiTester()
{
}

void SpiTester::setup()
{
    eb.onDst(id()).call(this);
    _reportInterval=5000;
    _pollInterval=1;
    _reportCount=_reportInterval;
    _outBytes.write((uint8_t)0xFF);
    _outBytes.write((uint8_t)0);
    _outBytes.write((uint8_t)0xAA);
    _outBytes.write((uint8_t)0x00);
    _outBytes.write((uint8_t)0x11);
    timeout(5000);
    INFO(" before init");
    _spi.init();
    _spi.setClock(10000);
    INFO(" after init");
}

void SpiTester::onEvent(Cbor& msg)
{
    if ( timeout() ) {
        timeout(_pollInterval);
        _spi.exchange(_inBytes,_outBytes);
        _reportCount -= _pollInterval;
        if ( _reportCount<0 ) {
            _reportCount = _reportInterval;
            eb.event(id(),H("inBytes")).addKeyValue(H("$data"),_inBytes).addKeyValue(H("public"),true);
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
