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
#include <Metric.h>

uint32_t BAUDRATE = 115200;

Uid uid(200);
EventBus eb(4096,1024);
Log logger(256);




//________________________________________________Se_________________
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PSWD
#define WIFI_PSWD "YOUR_PASSWORD"
#endif
Str line(20);



#include <main_labels.h>


Wifi wifi("wifi");
mDNS mdns(wifi);

LedBlinker led;
Mqtt mqtt("mqtt",1024);
System systm("system");
MqttJson router("mqttJson",1024);
DWM1000_Tag dwm1000Tag("TAG");
DWM1000_Anchor dwm1000Anchor("ANCHOR");
Memory memory("memory");
Configurator configurator("config");
//SpiTester spiTester("spiTester");
//IrqTester irqTester("irqTester");
Config config;

extern void waitConfig();

enum { NONE,TAG,ANCHOR } DWM1000Role=NONE;

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

    logger.level(Log::LOG_DEBUG);

    config.get("wifi.ssid",ssid,"SSID");
    config.get("wifi.pswd",pswd,"PSWD");

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

//    memory.setup();
//    spiTester.setup();
//    irqTester.setup();
    configurator.setup();

    Str role(3);
    config.get("lpos.role",role,"A");


    uint8_t mac[8];
    memset(mac,0,8);
    WiFi.macAddress(&mac[2]);
    if ( strcmp(role.c_str(), "T") == 0) {
        DWM1000Role = TAG;
        dwm1000Tag.setShortAddress(ESP.getChipId() & 0xFFFF);
        dwm1000Tag.setLongAddress(mac);
        dwm1000Tag.setup();
    } else { // otherwise an anchor
    DWM1000Role=ANCHOR;
        dwm1000Anchor.setShortAddress(ESP.getChipId() & 0xFFFF);
        dwm1000Anchor.setLongAddress(mac);
        dwm1000Anchor.setup();
    }


    mqtt.setup();
    router.setMqttId(mqtt.id());
    router.setup();

    led.setup();
    systm.setup();


//	eb.onEvent(H("system"),H("state")).subscribe(&router,(MethodHandler)&Router::ebToMqtt); // publisize timer-state events


    return;
}
Timer eventBusy("eventBusy",100000);
Timer eventIdle("eventIdle",100000);
extern "C"  void loop()
{
    eventIdle.stop();
    eventBusy.start();
    
    eb.eventLoop();
    wifi.loop();
    mqtt.loop();
    mdns.loop();
    if ( DWM1000Role==TAG)
        dwm1000Tag.loop();
    if ( DWM1000Role==ANCHOR)
        dwm1000Anchor.loop();
        
    eventBusy.stop();
    eventIdle.start();
}
