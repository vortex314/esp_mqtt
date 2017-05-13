#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <pins_arduino.h>
#include <Sys.h>
#include <Wifi.h>
#include <LedBlinker.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <mDNS.h>
#include <ctype.h>
#include <uart.h>
#include <Udp.h>
#include <ctype.h>
#include <cstdlib>
#include <Base64.h>
#include <Config.h>
#include <PubSubClient.h>
#include <EventBus.h>
#include <Mqtt.h>
#include <MqttJson.h>
#include <System.h>
#include <MqttJson.h>
#include <DWM1000_Tag.h>
#include <DWM1000_Anchor.h>
#include <Memory.h>
#include <SpiTester.h>
#include <IrqTester.h>
#include <Configurator.h>

uint32_t BAUDRATE = 115200;

Uid uid(200);
EventBus eb(2048,1024);
Log logger(256);




//________________________________________________Se_________________
#ifndef WIFI_SSID
#define WIFI_SSID "Merckx3"
#endif
#ifndef WIFI_PSWD
#define WIFI_PSWD "LievenMarletteEwoutRonald"
#endif
Str line(20);
class Timer : public Actor
{
    uint32_t _counter;
    uid_t _bl;
public:
    Timer():Actor("timer") {
        _counter=0;
        _bl =  H("bootloader");
    }
    void setup() {
        eb.onDst(H("timer")).call(this);
        timeout(10000);
    }

    void onEvent(Cbor& msg) {
        PT_BEGIN();
LOOP : {
            while(true) {
                eb.request(H("remote1"),H("connect"),id());
                eb.send();
                timeout(3000);
                PT_YIELD_UNTIL(timeout() || eb.isReplyCorrect(H("remote1"),H("connect")));
            }
        }
        PT_END();
    }
};




#include <main_labels.h>


Wifi wifi("wifi");
mDNS mdns(wifi);
Timer timer;
LedBlinker led;
Mqtt mqtt("mqtt",1024);
System systm("system");
MqttJson router("router",1024);
DWM1000_Tag dwm1000Tag("TAG");
DWM1000_Anchor dwm1000Anchor("ANCHOR");
Memory memory("memory");
Configurator configurator("config");
//SpiTester spiTester("spiTester");
//IrqTester irqTester("irqTester");
Config config;

extern void waitConfig();

void setup()
{


    Serial.begin(BAUDRATE, SerialConfig::SERIAL_8E1, SerialMode::SERIAL_FULL); // 8E1 for STM32
    Serial.setDebugOutput(false);
    Sys::delay(1000);
    INFO("version : " __DATE__ " " __TIME__);
    INFO("WIFI_SSID '%s'  ",WIFI_SSID);

    waitConfig();

    String hostname;
    Str strHostname(30),ssid(30),pswd(60);
    Sys::init();
    INFO("");
    char hn[20];

    sprintf(hn,"ESP%X",ESP.getChipId());
    hostname = hn;
    INFO(" hostname : %s",hn);
    Sys::hostname(hn);

    logger.level(Log::LOG_TRACE);

    config.get("wifi.ssid",ssid,"SSID");
    config.get("wifi.pswd",pswd,"PSWD");
    /*ssid = WIFI_SSID;
    pswd= WIFI_PSWD;
     * */
    hostname=hn;
    strHostname = hn;

    wifi.setConfig(ssid,pswd,strHostname);
    mdns.setConfig(strHostname,2000);
    INFO(" starting Wifi host : '%s' on SSID : '%s' '%s' ", wifi.getHostname(),
         wifi.getSSID(), wifi.getPassword());

    uint32_t port;
//    config.clear();

 /*   config.get("mqtt.host",strPswd,"limero.ddns.net");
    config.get("mqtt.port",port,1883);
    config.get("lpos.role",strPswd,"tag");
    config.get("lpos.address",strPswd,"T1");*/



    eb.onAny().call([](Cbor& msg) { // Log all events
        Str str(256);
        eb.log(str,msg);
        DEBUG("%s",str.c_str());
    });

    /*   eb.onEvent(systm.id(),EB_UID_IGNORE).call([](Cbor& msg) { // PUBLISH system events
           router.ebToMqtt(msg);
       }); */

    uid.add(labels,LABEL_COUNT);
    led.setMqtt(mqtt.id());
    led.setWifi(wifi.id());
//    mqtt.setWifi(wifi.id());
    wifi.setup();
    mdns.setup();
    timer.setup();
    memory.setup();
//    spiTester.setup();
//    irqTester.setup();
    configurator.setup();
    if ( strcmp(Sys::hostname(),"ESPCF5241")!=0) {
        dwm1000Anchor.setup();
        INFO("ANCHOR started");

    } else {
        dwm1000Tag.setup();
        INFO("TAG started");
    }


    mqtt.setup();
    router.setMqttId(mqtt.id());
    router.setup();

    led.setup();
    systm.setup();


//	eb.onEvent(H("system"),H("state")).subscribe(&router,(MethodHandler)&Router::ebToMqtt); // publisize timer-state events


    return;
}
extern "C"  void loop()
{
    eb.eventLoop();
    wifi.loop();
    mqtt.loop();
    mdns.loop();

}
