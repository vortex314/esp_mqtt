/*
 * DWM1000_Anchor.cpp
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#include <DWM1000_Anchor.h>
#include <Log.h>
#include <decaSpi.h>
#include <Config.h>
#include <Metric.h>

extern "C" {
#include <spi.h>
#include <gpio_c.h>
#include <espmissingincludes.h>
#include <osapi.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
}

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t dwt_config_xxx = {  //
    2, // Channel number.
    DWT_PRF_64M, // Pulse repetition frequency.
    DWT_PLEN_1024, /* Preamble length. */
    DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    1, /* Use non-standard SFD (Boolean) */
    DWT_BR_850K, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

///

/* Frames used in the ranging process. See NOTE 2 below. */

static uint8 rx_poll_msg[] = { //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,  // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'W', 'A',   // byte 5/6: destination address, see NOTE 3 below.
    'V', 'E',   // byte 7/8: source address, see NOTE 3 below.
    0x21, // byte 9: function code (specific values to indicate which message it is in the ranging process).
    0, 0
};  // All messages end with a 2-byte checksum automatically set by DW1000.

static uint8 tx_resp_msg[] = { //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,  // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'V', 'E',   // byte 5/6: destination address, see NOTE 3 below.
    'W', 'A',   // byte 7/8: source address, see NOTE 3 below.
    0x10, // byte 9: function code (specific values to indicate which message it is in the ranging process). [Response message]
    0x02, // byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
    0, 0, // byte 11/12: activity parameter, not used for activity code 0x02.
    0, 0
};  // All messages end with a 2-byte checksum automatically set by DW1000.

static uint8 rx_final_msg[] = { //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,          // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'W', 'A',   // byte 5/6: destination address, see NOTE 3 below.
    'V', 'E',   // byte 7/8: source address, see NOTE 3 below.
    0x23, // byte 9: function code (specific values to indicate which message it is in the ranging process). [Final message]
    0, 0, 0, 0, // byte 10 -> 13: poll message transmission timestamp
    0, 0, 0, 0, // byte 14 -> 17: response message reception timestamp
    0, 0, 0, 0, // byte 18 -> 21: final message transmission timestamp.
    0, 0
}; // All messages end with a 2-byte checksum automatically set by DW1000.

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 32
//static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536



/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500 /* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define FINAL_RX_TIMEOUT_UUS 3300   /* Receive final timeout. See NOTE 5 below. */

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

DWM1000_Anchor* DWM1000_Anchor::_anchor;

DWM1000_Anchor::DWM1000_Anchor(const char* name) :
    Actor(name), _spi(HSPI), _irq(D2), _panAddress(3), _irqEvent(100)
{
    _count = 0;
    _interrupts = 0;
    _polls = 0;
    _finals = 0;
    _anchor = this;
    _hasIrqEvent = false;
}

DWM1000_Anchor::~DWM1000_Anchor()
{

}

//_________________________________________________ IRQ handler

uint64_t startIsr;
uint64_t delta;
uint8_t seq_nbr ;

bool DWM1000_Anchor::isPollMsg()
{

    return _dwmMsg.function == FUNC_POLL_MSG  ;

}

bool DWM1000_Anchor::isFinalMsg()
{
//    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return _dwmMsg.function == FUNC_FINAL_MSG  ;
//    return  ( (FinalMsg*)rx_buffer)->function == FUNC_FINAL_MSG  ;
//    return memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0;
}

int DWM1000_Anchor::sendRespMsg()
{
    uint32 resp_tx_time;


    poll_rx_ts = get_rx_timestamp_u64(); /* Retrieve poll reception timestamp. */

    /* Set send time for response. See NOTE 8 below. */
    resp_tx_time = (poll_rx_ts
                    + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

    if ( seq_nbr > (uint8_t)(_lastSequence + 1))
        _anchor->_missed++;
    _lastSequence = seq_nbr;

    dwt_writetxdata(sizeof(_respMsg), _respMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_respMsg), 0);


    /* Write and send the response message. See NOTE 9 below.*/
    /*    tx_resp_msg[ALL_MSG_SN_IDX] = seq_nbr;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0);*/
    /* Set expected delay and timeout for final message reception. */
    dwt_setdelayedtrxtime(resp_tx_time);
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);


    delta =  micros() - startIsr;
    if ( dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) < 0) {
        return -1;
    }
    return 0;
}

//===================================================================================

void DWM1000_Anchor::calcFinalMsg()
{
    uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db;
    int64 tof_dtu;

    /* Retrieve response transmission and final reception timestamps. */
    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();

    /* Get timestamps embedded in the final message. */
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_POLL_TX_TS_IDX],
                     &poll_tx_ts);
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_RESP_RX_TS_IDX],
                     &resp_rx_ts);
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_FINAL_TX_TS_IDX],
                     &final_tx_ts);

    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
    poll_rx_ts_32 = (uint32) poll_rx_ts;
    resp_tx_ts_32 = (uint32) resp_tx_ts;
    final_rx_ts_32 = (uint32) final_rx_ts;
    Ra = (double) (resp_rx_ts - poll_tx_ts);
    Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
    Da = (double) (final_tx_ts - resp_rx_ts);
    Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64) ((Ra * Rb - Da * Db)
                       / (Ra + Rb + Da + Db));

    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;
    _distanceInCm = distance * 100.0;
    if ( !_hasIrqEvent) {
        _irqEvent.addKeyValue(EB_DST, id())
        .addKeyValue(EB_SRC, id())
        .addKeyValue(EB_REQUEST, H("IRQ"))
        .addKeyValue(H("distance"), distance)
        .addKeyValue(H("sequence"), seq_nbr);
        _hasIrqEvent = true;
    }

}

//===================================================================================





//===================================================================================
//_________________________________________________ INITIALIZE SPI
//

uint16_t btow(uint8_t* b)
{
    return  b[0] + ((uint16_t)b[1] << 8);
}

void wtob(uint8_t* b, uint16_t w)
{
    b[0] = w & 0xFF;
    w >>= 8;
    b[1] = w & 0xFF;
}
Timer anchorRxcallbackTime("rxcallback time",10);
Str anchorLogStr(100);
//_________________________________________________ IRQ handler
void DWM1000_Anchor::rxcallback(const  dwt_callback_data_t* signal)
{
    anchorRxcallbackTime.start();
    _anchor->onRxd(signal);
    anchorRxcallbackTime.stop();
}

void DWM1000_Anchor::onRxd( const dwt_callback_data_t* signal)
{
    INFO("RXD %2X:%2X",signal->fctrl[0],signal->fctrl[1]);
    uint32_t frameLength = signal->datalength;
    if ( signal->event == DWT_SIG_RX_OKAY ) {
        _interrupts++;
        status_reg = signal->status;
        if ( frameLength < sizeof(_dwmMsg)) {

            anchorRxcallbackTime.start();
//           frame_length.update(frameLength);
            dwt_readrxdata(_dwmMsg.buffer, frameLength, 0);

            anchorLogStr =" data : ";
            anchorLogStr.appendHex(_dwmMsg.buffer,frameLength,':');
            INFO(anchorLogStr.c_str());
            FrameType ft= DWM1000::getFrameType(_dwmMsg);

            seq_nbr = _anchor->_dwmMsg.sequence;

            if (ft==FT_POLL) {
                PollMsg& poll=*(PollMsg*)&_dwmMsg;
                _polls++;
                createRespMsg(_respMsg,poll);
                if ( _anchor->sendRespMsg() < 0) {
                    _anchor->_errs++;
                }
//                frame_seq_nb++;
            } else if (_anchor-> isFinalMsg() ) {
                _anchor->_finals++;
                _anchor->calcFinalMsg();
                dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
                dwt_rxenable(0); /* Activate reception immediately. */

            } else {
                _anchor->_errs += 1000;
            }
        }  else  {
            ERROR(" frame too big ");
        }
    } else if ( signal->event == DWT_SIG_RX_PTOTIMEOUT || signal->event == DWT_SIG_RX_SFDTIMEOUT) {
        INFO(" RXD timeout");
        dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
        dwt_rxenable(0); /* Activate reception immediately. */
    } else {
        INFO(" unknown event : %d",signal->event);
    }
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO, 1);  // enable

}

void DWM1000_Anchor::txcallback( const dwt_callback_data_t* signal)
{

}

//===================================================================================
void DWM1000_Anchor::setup()
{
//_________________________________________________INIT SPI ESP8266

    DWM1000::setup();
    INFO("DWM1000 ANCHOR started.");
    dwt_setcallbacks(txcallback,rxcallback);
    dwt_setautorxreenable(true);
    dwt_setdblrxbuffmode(false);
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO, 1); // enable
    attachInterrupt(digitalPinToInterrupt(D2), dwt_isr, RISING);

    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    /*    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);*/

    _count = 0;

    eb.onDst(id()).call(this);
    timeout(5000);

}

Metric<uint32_t> opsTime("DW1000 Action",10);
uint64_t _startTime;

Bytes byts(100);
Str hex(100);
extern "C" char* bytesToHex(uint8_t* pb, uint32_t len);
static int _timeoutCounter = 0;
void DWM1000_Anchor::onEvent(Cbor& msg)
{
    Bytes bytes(0);
    Str str(100);
    uint32_t sys_mask;
    int erc;
    static uint32_t _oldInterrupts = 0;
    PT_BEGIN()

BLINK : {
        dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
        dwt_rxenable(0); /* Activate reception immediately. */


        while(true) {
            BlinkMsg blinkMsg;
            createBlinkFrame(blinkMsg);
            _startTime=micros();
            dwt_writetxdata(sizeof(blinkMsg), blinkMsg.buffer, 0);
            dwt_writetxfctrl(sizeof(blinkMsg), 0);
            erc =dwt_starttx(DWT_START_TX_IMMEDIATE);
            opsTime.update(micros()-_startTime);

            str.appendHex(blinkMsg.buffer,sizeof(blinkMsg),':');
            INFO(str.c_str());
            INFO("BLINK : erc %d , sequence : %d ",erc,_blinkSequence);
            timeout(1000);
            PT_YIELD_UNTIL(timeout());
            dwt_write32bitreg(SYS_STATUS_ID,  SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
            dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO , 1); // enable
            sys_mask = dwt_read32bitreg(SYS_MASK_ID);
            INFO(" SYS_MASK : %X ", sys_mask);
        }
    }


WAIT_POLL: {
//       dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
//       dwt_rxenable(0); /* Activate reception immediately. */
        while (true) { /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
            dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
            dwt_rxenable(0); /* Activate reception immediately. */
            dwt_write32bitreg(SYS_STATUS_ID,  SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
            dwt_setinterrupt(DWT_INT_RFCG , 1); // enable
            sys_mask = dwt_read32bitreg(SYS_MASK_ID);
            INFO(" SYS_MASK : %X ", sys_mask);

            timeout(300);/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
            PT_YIELD_UNTIL(timeout() || eb.isRequest(H("IRQ")));
            double dist = 0.0;
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            INFO( " SYS_STATUS : %X seq : %d" , status_reg , seq_nbr);
            if ( status_reg & SYS_STATUS_CLKPLL_LL ) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CLKPLL_LL );
                INFO(" clearing PLL lock bit ")
            };

            if ( eb.isRequest(H("IRQ")) && msg.getKeyValue(H("distance"), dist)) {



                eb.publicEvent(id(), H("interrupts")).addKeyValue(H("$data"), _interrupts);
                eb.send();
                bytes.map(_dwmMsg.buffer, _frame_len);
                /*                eb.publicEvent(id(), H("poll")).addKeyValue(H("$data"), bytes);
                                eb.send();
                                bytes.map(tx_resp_msg, sizeof(tx_resp_msg));
                                eb.publicEvent(id(), H("resp")).addKeyValue(H("$data"), bytes);
                                eb.send();
                                bytes.map(respMsg.buffer, sizeof(respMsg));
                                eb.publicEvent(id(), H("respMsg")).addKeyValue(H("$data"), bytes);
                                eb.send();*/
                eb.publicEvent(id(), H("distance")).addKeyValue(EB_DATA, dist);
                eb.send();
                /*                eb.publicEvent(id(), H("distanceInCm")).addKeyValue(EB_DATA, _distanceInCm);
                                eb.send();
                                eb.publicEvent(id(), H("finals")).addKeyValue(EB_DATA, _finals);
                                eb.send();
                                eb.publicEvent(id(), H("polls")).addKeyValue(EB_DATA, _polls);
                                eb.send();
                                eb.publicEvent(id(), H("errs")).addKeyValue(EB_DATA, _errs);
                                eb.send();*/
                eb.publicEvent(id(), H("delta")).addKeyValue(EB_DATA, delta);
                eb.send();
                eb.publicEvent(id(), H("missed")).addKeyValue(EB_DATA, _missed);
                eb.send();

                _oldInterrupts = _interrupts;

            } else if ( timeout()) {
                eb.publicEvent(id(), H("alive")).addKeyValue(EB_DATA, true);
                eb.send();
            }

        }
    }
    PT_END()
    ;
}

Timer anchorLoopTime("loopTime ",100000);
uint64_t anchorLastMicros=micros();

void DWM1000_Anchor::loop()
{
    /*   anchorLoopTime.stop();
       if ( digitalRead(D2))
           dwt_isr();
       anchorLoopTime.start();
        * */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
