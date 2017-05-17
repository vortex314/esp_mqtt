#include "Peripheral.h"
#include <Log.h>
#include "spi.h"
#include "Sys.h"
//#include "Logger.h"



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

//=====================================  S P I =============================================

#define CLOCK_80MHZ 80000000



Spi::Spi(uint32_t base)
{
    _base = base;
    _spi_no = base;
    _clock = 1000000;
    _mode = SPI_MODE_PHASE0_POL0;
    _hwSelect = true;
    _lsbFirst = false;
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

void Spi::setLsbFirst(bool lsbFirst)
{
    _lsbFirst = lsbFirst;
}


void Spi::init()
{

    if (_spi_no > 1)
        return; //Only SPI and HSPI are valid spi modules.
//================== set GPIO

    uint32 clock_div_flag = 0;
    if (_clock == CLOCK_80MHZ ) {
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
    };
//======================= SPI_CTRL_REG  needs to be set before spi_clk otherwise fails

    uint32_t spi_ctrl =0;
    if ( _lsbFirst ) {
        spi_ctrl += SPI_WR_BIT_ORDER | SPI_RD_BIT_ORDER;
    }
    WRITE_PERI_REG(SPI_CTRL(_spi_no), spi_ctrl);
//======================== SPI_CLOCK_REG

    uint32_t cntdiv = 8;
    uint32_t prediv =  ( CLOCK_80MHZ / cntdiv ) / _clock;
    DEBUG(" reg : %X clock : %d cntdiv : %d predic : %d ",SPI_CLOCK(_spi_no),_clock,cntdiv,prediv);
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
    INFO(" SPI_CLOCK 0x%08X",READ_PERI_REG(SPI_CLOCK(_spi_no)));

//======================= SPI_USER

    uint32_t spi_user=0;            //======= phase
    if (_mode == SPI_MODE_PHASE1_POL0 || _mode==SPI_MODE_PHASE0_POL1) {
        spi_user += SPI_CK_OUT_EDGE ;
    }
    if ( _hwSelect ) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2); //GPIO15 is HSPI CS pin (Chip Select / Slave Select)
        spi_user +=  SPI_CS_SETUP|SPI_CS_HOLD;
    } else {
        pinMode(15, OUTPUT);
    }
    spi_user += SPI_USR_MOSI | SPI_DOUTDIN; // out & in at the same time
//    spi_user += SPI_WR_BYTE_ORDER | SPI_RD_BYTE_ORDER; // big endian
    WRITE_PERI_REG(SPI_USER(_spi_no), spi_user);


//======================= SPI_CTRL2_REG needs to be set after SPI_USER


    uint32_t spi_ctrl2=0;
//    spi_ctrl2 += ( 2 << SPI_MOSI_DELAY_MODE_S );    // mode 0 and clock below 80MHZ, see ESP32
        spi_ctrl2 += SPI_CK_OUT_HIGH_MODE << SPI_CK_OUT_HIGH_MODE_S ;
    if ( _mode == SPI_MODE_PHASE0_POL1 || _mode==SPI_MODE_PHASE1_POL1) {
        spi_ctrl2 += SPI_CK_OUT_HIGH_MODE << SPI_CK_OUT_HIGH_MODE_S ;
    } else {
        spi_ctrl2 += SPI_CK_OUT_LOW_MODE << SPI_CK_OUT_LOW_MODE_S;
    };
    spi_ctrl2 += (2 << SPI_MISO_DELAY_MODE_S); // add delay for going trough mux , see ESP32 ref manual
    WRITE_PERI_REG(SPI_CTRL2(_spi_no), spi_ctrl2);
//================================= SPI_USER2
    uint32_t spi_user2=0;
    WRITE_PERI_REG(SPI_USER2(_spi_no), spi_user2);

//================================= SPI_PIN

    uint32_t spi_pin=0;
    if (_mode == SPI_MODE_PHASE0_POL1 || _mode==SPI_MODE_PHASE1_POL1) {
        spi_pin += SPI_IDLE_EDGE ;
    }
    spi_pin += SPI_CS2_DIS | SPI_CS1_DIS;
    WRITE_PERI_REG(SPI_PIN(_spi_no), spi_pin);
//=================================

/*    INFO(" SPI_CTRL  0x%08X",READ_PERI_REG(SPI_CTRL(_spi_no)));
    INFO(" SPI_CTRL1 0x%08X",READ_PERI_REG(SPI_CTRL1(_spi_no)));
    INFO(" SPI_CTRL2 0x%08X",READ_PERI_REG(SPI_CTRL2(_spi_no)));
    INFO(" SPI_CLOCK 0x%08X",READ_PERI_REG(SPI_CLOCK(_spi_no)));
    INFO(" SPI_USER  0x%08X",READ_PERI_REG(SPI_USER(_spi_no)));
    INFO(" SPI_USER1  0x%08X",READ_PERI_REG(SPI_USER1(_spi_no)));
    INFO(" SPI_USER2  0x%08X",READ_PERI_REG(SPI_USER2(_spi_no)));*/
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
        uint8_t bytes[4]; // little endian b[3] = MSB
    };
    uint32_t byteIndex=0;

    out.offset(0);
    word=0;
    while(out.hasData()) {
        bytes[byteIndex] = out.read();
        byteIndex++;
        if ( byteIndex %4 == 0 ) {
            byteIndex=0;
            *pW = word;
            pW++;
        }
    }
    if ( byteIndex ) { // some bytes to left align
        *pW= word;
    }
//    INFO(" last word : %X ",word);
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
        in.write(bytes[byteIndex]);
        byteIndex++;
        if ( byteIndex %4 == 0 ) {
            pW++;
            word = *pW;
            byteIndex=0;
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
        WARN("SPI Buffer too big !")
        return ENOSPC;
    }
// wait ready
    while(busy());
// write data to buffer
    bytesToWords((uint32_t*)SPI_W0(_spi_no),out);
    uint32_t spi_user1=0;
    spi_user1 = ((out.length()*8)-1) << SPI_USR_MOSI_BITLEN_S;
//    spi_user1 += 1 << SPI_USR_ADDR_BITLEN_S;
    WRITE_PERI_REG(SPI_USER1(_spi_no), spi_user1); // bit length
// trigger start
    SET_PERI_REG_MASK(SPI_CMD(_spi_no), SPI_USR);
// wait finish
    while(busy());
// get data from buffer
    in.clear();
    wordsToBytes((uint32_t*)SPI_W0(_spi_no),in,out.length());
    return E_OK;
}
