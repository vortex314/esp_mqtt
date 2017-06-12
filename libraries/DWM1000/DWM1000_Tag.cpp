/*
* DWM1000_Tag.cpp
*
*  Created on: Feb 12, 2016
*      Author: lieven
*/

#include <DWM1000_Tag.h>
#include <Log.h>
#include <Gpio.h>
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



/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */

static uint8 tx_poll_msg[] = { //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,  // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'W', 'A',  // byte 5/6: destination address, see NOTE 3 below.
    'V', 'E',  // byte 7/8: source address, see NOTE 3 below.
    0x21, // byte 9: function code (specific values to indicate which message it is in the ranging process).
    0, 0
};  // All messages end with a 2-byte checksum automatically set by DW1000.

static uint8 rx_resp_msg[] = { //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,  // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'V', 'E',  // byte 5/6: destination address, see NOTE 3 below.
    'W', 'A',  // byte 7/8: source address, see NOTE 3 below.
    0x10, // byte 9: function code (specific values to indicate which message it is in the ranging process). [Response message]
    0x02, // byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
    0, 0, // byte 11/12: activity parameter, not used for activity code 0x02.
    0, 0
};  // All messages end with a 2-byte checksum automatically set by DW1000.

static uint8 tx_final_msg[] = {  //
    0x41, 0x88, // byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
    0,          // byte 2: sequence number, incremented for each new frame.
    0xCA, 0xDE, // byte 3/4 : PAN ID (0xDECA)
    'W', 'A',  // byte 5/6: destination address, see NOTE 3 below.
    'V', 'E',  // byte 7/8: source address, see NOTE 3 below.
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
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */ #define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
* frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Time-stamps of frames transmission/reception, expressed in device time units.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

DWM1000_Tag* DWM1000_Tag::_tag = 0;

DWM1000_Tag::DWM1000_Tag(const char* name) :
    Actor(name), _spi(HSPI), _anchors(20), _panAddress(3)
{
    _count = 0;
    _tag = this;
    _interrupts = 0;
    _resps = 0;
    _polls = 0;
    _rxd=false;
    _txd=false;
    _state=RCV_BLINK;

}

DWM1000_Tag::~DWM1000_Tag()
{

}


bool DWM1000_Tag::isRespMsg()
{
    return _dwmMsg.function == FUNC_RESP_MSG  ;
//    rx_buffer[ALL_MSG_SN_IDX] = 0;
//    return (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) ;      // CHECK RESP MSG
}

void DWM1000_Tag::sendFinalMsg()
{
    uint32 final_tx_time;

    /* Retrieve poll transmission and response reception timestamp. */
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();

    /* Compute final message transmission time. See NOTE 9 below. */
    final_tx_time = (resp_rx_ts
                     + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME))
                    >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
    final_tx_ts = (((uint64) (final_tx_time & 0xFFFFFFFE)) << 8)
                  + TX_ANT_DLY;

    /* Write all timestamps in the final message. See NOTE 10 below. */
    /*  final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],
                        poll_tx_ts);
      final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],
                        resp_rx_ts);
      final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX],
                        final_tx_ts); */

    memcpy(_finalMsg.buffer, tx_final_msg, sizeof(_finalMsg));
    final_msg_set_ts(_finalMsg.pollTimestamp, poll_tx_ts);
    final_msg_set_ts(_finalMsg.respTimestamp, resp_rx_ts);
    final_msg_set_ts(_finalMsg.finalTimestamp, final_tx_ts);
    memcpy(_finalMsg.dst,_dwmMsg.src,2);
    memcpy(_finalMsg.src,_dwmMsg.dst,2);
    _finalMsg.sequence = _dwmMsg.sequence;

    dwt_writetxdata(sizeof(_finalMsg), _finalMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_finalMsg), 0);



    /* Write and send final message. See NOTE 7 below. */
//    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    /*    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
        dwt_writetxfctrl(sizeof(tx_final_msg), 0);*/
    dwt_starttx(DWT_START_TX_DELAYED);                          // SEND FINAL MSG
    /* Poll DW1000 until TX frame sent event set. See NOTE 8 below. */

    /* Increment frame sequence number after transmission of the final message (modulo 256). */
}

Timer rxcallbackTime("rxcallback time",10);
Metric<uint32_t> frame_length("frame_length ",10);

Str logStr(100);

void DWM1000_Tag::onRxd(const  dwt_callback_data_t* signal)
{
    uint32_t frameLength = signal->datalength;
    INFO(" state : %s",uid.label(_state));
    if ( _state == RCV_RESP && signal->event == DWT_SIG_RX_TIMEOUT ) {
        _state=RCV_BLINK;
        dwt_setrxtimeout(0);
        dwt_rxenable(false);
        return;
    };
    if ( signal->event == DWT_SIG_RX_OKAY ) {
        _interrupts++;
        status_reg = signal->status;
        if ( frameLength < sizeof(_dwmMsg)) {

            rxcallbackTime.start();
//          frame_length.update(frameLength);
            dwt_readrxdata(_dwmMsg.buffer, frameLength, 0);

            logStr =" RXD : ";
            logStr.appendHex(_dwmMsg.buffer,frameLength,':');
            INFO(logStr.c_str());
            FrameType ft= DWM1000::getFrameType(_dwmMsg);

            if ( signal->fctrl[0] == FC_1_BLINK && _state==RCV_BLINK ) {

                BlinkMsg* blink = (BlinkMsg*)&_dwmMsg;
                uint16_t address =  blink->sourceShort[1];
                address <<= 8 ;
                address += blink->sourceShort[0];

                if ( anchors.count(address)==0) {
                    anchors.emplace(address,RemoteAnchor(address,blink->sequence));
                    INFO(" new anchor : 0x%X",address);
                } else {
                    anchors.find(address)->second.update(blink->sequence);
                }
                _blinks++;
                createPollMsg(_pollMsg,address);
                logStr =" TXD poll : ";
                logStr.appendHex(_pollMsg.buffer,sizeof(_pollMsg),':');
                INFO(logStr.c_str());

                dwt_writetxdata(sizeof(_pollMsg), _pollMsg.buffer, 0);
                dwt_writetxfctrl(sizeof(_pollMsg), 0);
                _polls++;

                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                dwt_rxenable(true);
                dwt_starttx(DWT_START_TX_IMMEDIATE);
                _state = SND_POLL;
            } else if ( ft == FT_RESP && _state==RCV_RESP) {
                _resps++;
                sendFinalMsg();
                _finals++;
                _state = SND_FINAL;
            } else {
                INFO(" invalid state & frame combination : state=%d fctrl=[0x%X,0x%X]",_state,signal->fctrl[0],signal->fctrl[1]);
            }
        }
    } else {
        INFO(" unknown event : %d",signal->event);
    }
}

//_________________________________________________ IRQ handler
void DWM1000_Tag::rxcallback(const  dwt_callback_data_t* signal)
{
    INFO("RXD");
    _tag->onRxd(signal);
    rxcallbackTime.stop();
}

void DWM1000_Tag::onTxd(const  dwt_callback_data_t* signal)
{
    if ( _state == SND_POLL ) _state=RCV_RESP;
    if ( _state == SND_FINAL ) _state=RCV_BLINK;
}

void DWM1000_Tag::txcallback(const  dwt_callback_data_t* signal)
{
    INFO("TXD");
    _tag->onTxd(signal);
}



void DWM1000_Tag::setup()
{
//_________________________________________________INIT SPI ESP8266
    DWM1000::setup();
    INFO("DWM1000 TAG started.");

    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
    * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setcallbacks(txcallback,rxcallback);
    dwt_setautorxreenable(true);
    dwt_setdblrxbuffmode(false);
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_TFRS , 1); // enable
    attachInterrupt(digitalPinToInterrupt(D2), dwt_isr, RISING);
    _count = 0;
    timeout(5000);
}



void DWM1000_Tag::loop()
{

}

//
extern "C" char* bytesToHex(uint8_t* pb, uint32_t len);
static int _timeoutCounter = 0;
void DWM1000_Tag::onEvent(Cbor& msg)
{
    Bytes bytes(0);
    PT_BEGIN()

RECV : {
        while (true) {
            INFO("reset");
            dwt_setrxtimeout(0); /* Clear reception timeout to start next ranging process. */
            dwt_rxenable(0); /* Activate reception immediately. */
            dwt_setautorxreenable(true);
            timeout(10000);
            PT_YIELD_UNTIL(timeout());
            std::map<uint16_t,RemoteAnchor>::iterator it;
            for(it= anchors.begin(); it!=anchors.end(); ++it) {
                if ( it->second.expired()) {
                    INFO(" expired address : 0x%X ",it->first);
                    anchors.erase(it->first);
                }
            }
        }
    }

POLL_SEND: {
        while (true) {

            /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
            memcpy(_pollMsg.buffer, tx_poll_msg, sizeof(tx_poll_msg));
            memcpy(_pollMsg.src, _panAddress.data(), 2);
            memcpy(_pollMsg.dst, _anchors.data() + 2 * _anchorIndex, 2);
            _pollMsg.sequence = frame_seq_nb;
            dwt_writetxdata(sizeof(_pollMsg), _pollMsg.buffer, 0);
            dwt_writetxfctrl(sizeof(_pollMsg), 0);

            _polls++;
            /*          tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                      dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
                      dwt_writetxfctrl(sizeof(tx_poll_msg), 0); */
            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);


            /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
            * set by dwt_setrxaftertxdelay() has elapsed. */
            INFO( " Start TXF " );
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX); // Clear RX error events in the DW1000 status register.
//          dwt_setinterrupt(DWT_INT_TFRS, 0);
            dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO , 1);  // enable
            dwt_rxenable(1);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);                // SEND POLL MSG
            _timeoutCounter = 0;
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if ( status_reg & SYS_STATUS_CLKPLL_LL ) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CLKPLL_LL );
                INFO(" clearing PLL lock bit ")
            };
            INFO( " SYS_STATUS before : %X" , status_reg );


            /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
            timeout(1000);
            PT_YIELD_UNTIL(timeout() );                        // WAIT RESP MSG

            bytes.map(tx_poll_msg, sizeof(tx_poll_msg));
            eb.event(id(), H("poll")).addKeyValue(H("$data"), bytes).addKeyValue(H("public"), true);;
            eb.send();
            bytes.map(_pollMsg.buffer, sizeof(_pollMsg));
            eb.event(id(), H("pollMsg")).addKeyValue(H("$data"), bytes).addKeyValue(H("public"), true);;
            eb.send();
            bytes.map(_finalMsg.buffer, sizeof(_finalMsg));
            eb.event(id(), H("finalMsg")).addKeyValue(H("$data"), bytes).addKeyValue(H("public"), true);;
            eb.send();
            bytes.map(tx_final_msg, sizeof(tx_final_msg));
            eb.event(id(), H("tx_final")).addKeyValue(H("$data"), bytes).addKeyValue(H("public"), true);;
            eb.send();
            eb.event(id(), H("interrupts")).addKeyValue(H("$data"), _interrupts).addKeyValue(H("public"), true);;
            eb.send();
            bytes.map(rx_buffer, _frame_len);
            eb.event(id(), H("rxd")).addKeyValue(H("$data"), bytes).addKeyValue(H("public"), true);;
            eb.send();
            eb.event(id(), H("resps")).addKeyValue(H("data"), _resps).addKeyValue(H("public"), true);;
            eb.send();
            eb.event(id(), H("polls")).addKeyValue(H("data"), _polls).addKeyValue(H("public"), true);;
            eb.send();
            eb.publicEvent(id(), H("alive")).addKeyValue(EB_DATA, true);
            eb.send();
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);

            INFO( " SYS_STATUS after : %X" , status_reg );
            if ( status_reg == 0xDEADDEAD ) setup();
//            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); // Clear RX error events in the DW1000 status register.
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
            _anchorIndex++;
            if ( _anchorIndex == _anchorMax) {
                _anchorIndex = 0;
                frame_seq_nb++;
            }
        }
    }
    PT_END()
    ;
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
*        ts  timestamp value
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

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
*        message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*        ts  timestamp value
*
* @return none
*/
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
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
*    - a poll message sent by the initiator to trigger the ranging exchange.
*    - a response message sent by the responder allowing the initiator to go on with the process
*    - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
*      time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*    - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*    - byte 2: sequence number, incremented for each new frame.
*    - byte 3/4: PAN ID (0xDECA).
*    - byte 5/6: destination address, see NOTE 3 below.
*    - byte 7/8: source address, see NOTE 3 below.
*    - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*    - no more data
*    Response message:
*    - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
*    - byte 11/12: activity parameter, not used for activity code 0x02.
*    Final message:
*    - byte 10 -> 13: poll message transmission timestamp.
*    - byte 14 -> 17: response message reception timestamp.
*    - byte 18 -> 21: final message transmission timestamp.
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
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*    DW1000 API Guide for more details on the DW1000 driver functions.
************************************************************************************************************************************************/
