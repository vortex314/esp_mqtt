#include "Peripheral.h"
#include <Log.h>
#include "spi.h"
#include "Sys.h"
//#include "Logger.h"

#define CLOCK_80MHZ 80000000

DigitalIn::DigitalIn(uint32_t pin)
{
    _pin=pin;
    _interruptFunction =0;
    _instance=0;
    _port=0;
    _mode=0;
}

DigitalIn::~DigitalIn()
{
}

void DigitalIn::setMode(uint32_t mode)
{
    _mode=mode;
}

void DigitalIn::setInterruptHandler(InterruptFunction f,void* instance)
{
    _interruptFunction = f;
    _instance=instance;
}

void DigitalIn::init()
{
    pinMode(_pin,INPUT);
    if ( _mode & DIN_INT_ENABLED) {
//        attachInterrupt(digitalPinToInterrupt(_pin),_interruptFunction,CHANGE);
    }
}





Spi::Spi(uint32_t base)
{
    _base = base;
    _spi_no = base;
    _clock = 1000000;
    _mode = SPI_MODE_PHASE0_POL0;
    _hwSelect = true;
}

Spi::~Spi()
{
}

void Spi::setMode(uint32_t mode)
{
    _mode=mode;
}

void Spi::setClock(uint32_t clock)
{
    _clock=clock;
}


void Spi::init()
{

    if (_spi_no > 1)
        return; //Only SPI and HSPI are valid spi modules.
//================== set GPIO

    uint32 clock_div_flag = 0;
    if (_clock == 80000000 ) {
        clock_div_flag = 0x0001;
    }

    if (_spi_no == SPI) {
        WRITE_PERI_REG(PERIPHS_IO_MUX, 0x005 | (clock_div_flag << 8)); //Set bit 8 if 80MHz sysclock required
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CLK_U, 1);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CMD_U, 1);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U, 1);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, 1);
    } else if (_spi_no == HSPI) {
        WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105 | (clock_div_flag << 9)); //Set bit 9 if 80MHz sysclock required
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2); //GPIO12 is HSPI MISO pin (Master Data In)
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2); //GPIO13 is HSPI MOSI pin (Master Data Out)
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2); //GPIO14 is HSPI CLK pin (Clock)
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2); //GPIO15 is HSPI CS pin (Chip Select / Slave Select)
    }
//======================== SPI_CLOCK_REG

    uint32_t cntdiv = 8;
    uint32_t prediv =  ( CLOCK_80MHZ / cntdiv ) / _clock;
    if (_clock == CLOCK_80MHZ) {
        WRITE_PERI_REG(SPI_CLOCK(_spi_no), SPI_CLK_EQU_SYSCLK);
    } else {
        WRITE_PERI_REG(SPI_CLOCK(_spi_no),
                       (((prediv-1)&SPI_CLKDIV_PRE)<<SPI_CLKDIV_PRE_S) //
                       | (((cntdiv-1)&SPI_CLKCNT_N)<<SPI_CLKCNT_N_S) //
                       | ((((cntdiv>>1)-1)&SPI_CLKCNT_H)<<SPI_CLKCNT_H_S) //
                       | (((cntdiv-1)&SPI_CLKCNT_L)<<SPI_CLKCNT_L_S)//
                      );
    }
//======================= SPI_CTRL_REG

    uint32_t spi_ctrl =0;
    spi_ctrl += SPI_WR_BIT_ORDER | SPI_RD_BIT_ORDER;
    WRITE_PERI_REG(SPI_CTRL(_spi_no), spi_ctrl);
//======================= SPI_CTRL2_REG

    uint32_t spi_ctrl2=0;
    spi_ctrl2 += ( 2 << SPI_MOSI_DELAY_MODE_S );    // mode 0 and clock below 80MHZ, see ESP32
    if ( _mode == SPI_MODE_PHASE0_POL1 || _mode==SPI_MODE_PHASE1_POL1) {
        spi_ctrl2 += SPI_CK_OUT_HIGH_MODE << SPI_CK_OUT_HIGH_MODE_S ;
    } else {
        spi_ctrl2 += SPI_CK_OUT_LOW_MODE << SPI_CK_OUT_LOW_MODE_S;
    };
    WRITE_PERI_REG(SPI_CTRL2(_spi_no), spi_ctrl2);
//======================= SPI_USER

    uint32_t spi_user=0;            //======= phase
    if (_mode == SPI_MODE_PHASE0_POL0 || _mode==SPI_MODE_PHASE0_POL1) {
        spi_user += SPI_CK_OUT_EDGE ;
    }
    if ( _hwSelect ) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2); //GPIO15 is HSPI CS pin (Chip Select / Slave Select)
        spi_user +=  SPI_CS_SETUP|SPI_CS_HOLD;
    } else {
        pinMode(15, OUTPUT);
    }
    spi_user += SPI_USR_MOSI | SPI_DOUTDIN; // out & in at the same time
    WRITE_PERI_REG(SPI_USER(_spi_no), spi_user);
//    spi_user += SPI_WR_BYTE_ORDER | SPI_RD_BYTE_ORDER; // big endian
    uint32_t spi_user2=0;
    WRITE_PERI_REG(SPI_USER2(_spi_no), spi_user2);
//================================= SPI_PIN

    uint32_t spi_pin=0;
    if (_mode == SPI_MODE_PHASE0_POL1 || _mode==SPI_MODE_PHASE1_POL0) {
        spi_pin += SPI_IDLE_EDGE ;
        spi_pin += SPI_CS2_DIS | SPI_CS1_DIS;
    }
//=================================

    INFO(" SPI_CTRL  0x%08X",READ_PERI_REG(SPI_CTRL(HSPI)));
    INFO(" SPI_CTRL1 0x%08X",READ_PERI_REG(SPI_CTRL1(HSPI)));
    INFO(" SPI_CTRL2 0x%08X",READ_PERI_REG(SPI_CTRL2(HSPI)));
    INFO(" SPI_CLOCK 0x%08X",READ_PERI_REG(SPI_CLOCK(HSPI)));
    INFO(" SPI_USER  0x%08X",READ_PERI_REG(SPI_USER(HSPI)));
    INFO(" SPI_USER1  0x%08X",READ_PERI_REG(SPI_USER1(HSPI)));
    INFO(" SPI_USER2  0x%08X",READ_PERI_REG(SPI_USER2(HSPI)));
}

void Spi::setHwSelect(bool select)
{
    _hwSelect = select;
}

bool Spi::busy()
{
    return READ_PERI_REG(SPI_CMD(_spi_no))&SPI_USR;
}

//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
void bytesToWords(uint32_t* pW, Bytes& out)
{
    union {
        uint32_t word;
        uint8_t bytes[4];
    };
    uint32_t byteIndex=0;
    uint32_t wordIndex=0;
    uint32_t index=0;

    out.offset(0);
    word=0;
    while(out.hasData()) {
        word = word << 8;
        word += out.read();
        byteIndex++;
        if ( byteIndex %4 == 0 ) {
            *pW = word;
            wordIndex++;
            word=0;
            pW++;
        }
    }
    *pW= word;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
void wordsToBytes(uint32_t* pW, Bytes& in, uint32_t length)
{
    union {
        uint32_t word;
        uint8_t bytes[4];
    };
    uint32_t byteIndex=0;
    word= *pW;
    while(length) {
        in.write(word & 0xFF);
        word = word>> 8;
        byteIndex++;
        if ( byteIndex %4 == 0 ) {
            word = *pW;
        }
        length--;
    }
}

Erc Spi::exchange(Bytes& in,Bytes& out)
{
    union {
        uint32_t w;
        uint8_t b[4];
    } word;
    if (out.length() >64) {
        WARN("SPI Buffer to big !")
        return ENOSPC;
    }
// wait ready
    while(busy());
// write data to buffer
    bytesToWords((uint32_t*)SPI_W0(_spi_no),out);
    WRITE_PERI_REG(SPI_USER1(_spi_no), ((out.length()*8)-1) << SPI_USR_MOSI_BITLEN_S); // bit length
// trigger start
    SET_PERI_REG_MASK(SPI_CMD(_spi_no), SPI_USR);
// wait finish
    while(busy());
// get data from buffer
    in.clear();
    wordsToBytes((uint32_t*)SPI_W0(_spi_no),in,out.length());
    return E_OK;
}

void spi_cs_select()
{
    digitalWrite(15, 0);
}

void spi_cs_deselect()
{
    digitalWrite(15, 1);
}



void spi_set_rate_low_old()
{
    spi_clock(HSPI, SPI_CLK_PREDIV, 20);
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
void spi_set_rate_high_old()
{
    spi_clock(HSPI, SPI_CLK_PREDIV, SPI_CLK_CNTDIV);
}

/*//////////////////////////////////////////////////////////////////////////////*/
/*	uint32_t din_bits = hLen * 8;
 uint32_t dout_bits = bLen * 8;

 while (spi_busy(HSPI))
 ; //wait for SPI to be ready

 //########## Enable SPI Functions ##########//
 //disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
 CLEAR_PERI_REG_MASK(SPI_USER(HSPI),
 SPI_USR_MOSI|SPI_USR_MISO|SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_DUMMY);
 SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_DOUTDIN); // LMR set full duplex
 SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MISO); //enable MISO function in SPI module
 SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI); //enable MOSI function in SPI module

 //########## Setup Bitlengths ##########//
 WRITE_PERI_REG(SPI_USER1(HSPI),
 ((dout_bits-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S | //Number of bits to Send
 ((din_bits-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S);//Number of bits to receive

 //########## Setup DOUT data ##########//
 if (dout_bits) {
 //copy data to W0
 uint32_t offset = bytesToWord(SPI_W0(HSPI), hbuff, hLen);
 bytesToWord(SPI_W0(HSPI), buffer,bLen);
 }

 SET_PERI_REG_MASK(SPI_CMD(HSPI), SPI_USR); //########## Begin SPI Transaction ##########//

 //########## Return DIN data ##########//
 wordsToBytes(SPI_W0(HSPI),);
 if (din_bits) {
 while (spi_busy(HSPI))
 ;	//wait for SPI transaction to complete

 if (READ_PERI_REG(SPI_USER(HSPI)) & SPI_RD_BYTE_ORDER) {
 return READ_PERI_REG(SPI_W0(HSPI)) >> (32 - din_bits); //Assuming data in is written to MSB. TBC
 } else {
 return READ_PERI_REG(SPI_W0(HSPI)); //Read in the same way as DOUT is sent. Note existing contents of SPI_W0 remain unless overwritten!
 }
 }*/


//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" int writetospi(uint16 hLen, const uint8 *hbuff, uint32 bLen,
                          const uint8 *buffer)
{
/*    if ( (hLen+bLen) > 64 ) {
        INFO("SPI write buffer too big %d %d ",hLen,bLen);
        return -1;
    }
    digitalWrite(15, 0);
    memcpy(o._spi_out,hbuff,hLen);
    memcpy(&o._spi_out[hLen],buffer,bLen);
    __spi.transferBytes(o._spi_out,i._spi_in,hLen+bLen);
    digitalWrite(15, 1);
    return 0;*/
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" int readfromspi(uint16 hLen, const uint8 *hbuff, uint32 bLen, uint8 *buffer)
{
/*    if ( (hLen+bLen) > 64 ) {
        INFO("SPI read buffer too big %d %d ",hLen,bLen);
        return -1;
    }
    digitalWrite(15, 0);
    memcpy(o._spi_out,hbuff,hLen);
    memcpy(&o._spi_out[hLen],buffer,bLen);
    __spi.transferBytes(o._spi_out,i._spi_in,hLen+bLen);
    memcpy(buffer,&i._spi_in[hLen],bLen);
    digitalWrite(15, 1);
    return 0; */
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_low()
{
//    __spi.setFrequency(400000);
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_high()
{
//   __spi.setFrequency(1000000);
}

/*
int writetospi(uint16 hLen, const uint8 *hbuff, uint32 bLen,
		const uint8 *buffer);
int readfromspi(uint16 hLen, const uint8 *hbuff, uint32 bLen, uint8 *buffer);
void spi_set_rate_low();
void spi_set_rate_high();
*/