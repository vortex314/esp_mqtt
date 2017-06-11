#include <Peripheral.h>
#include <deca_device_api.h>

Spi* _gSpi;     // to support irq 


void spi_set_global(Spi* spi) {
    _gSpi = spi;
}


void spi_cs_select()
{
    digitalWrite(15, 0);
}

void spi_cs_deselect()
{
    digitalWrite(15, 1);
}

Bytes out(100);
Bytes in(100);

//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" int writetospi(uint16 hLen, const uint8 *hbuff, uint32 bLen,
                          const uint8 *buffer)
{

    out.clear();
    out.write((uint8_t*)hbuff,0,hLen);
    out.write((uint8_t*)buffer,0,bLen);
    _gSpi->exchange(in,out);
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" int readfromspi(uint16 hLen, const uint8 *hbuff, uint32 bLen, uint8 *buffer)
{
    out.clear();
    out.write((uint8_t*)hbuff,0,hLen);
    out.write((uint8_t*)buffer,0,bLen);
    _gSpi->exchange(in,out);
    in.offset(hLen);
    while(in.hasData()) {
        *buffer++ = in.read();
    }
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_low()
{
    _gSpi->setClock(1000000);
    _gSpi->init();
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_high()
{
   _gSpi->setClock(4000000);
   _gSpi->init();
}

extern "C" decaIrqStatus_t decamutexon() {
    noInterrupts();
}

extern "C" void decamutexoff(decaIrqStatus_t s) {
    interrupts();
}



