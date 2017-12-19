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
#include <WiFiUdp.h>
#include <SysLog.h>

bool stopHeap(Actor* a,const char* s);
void startHeap();

extern "C" {
#include "user_interface.h"

};

uint32_t BAUDRATE = 230400;

Uid uid(200);
EventBus eb(4096,1024);
Log logger(256);




//________________________________________________Se_________________

Str line(20);

#include <main_labels.h>


Wifi wifi("wifi");
mDNS mdns("mdns");

LedBlinker led;
Mqtt mqtt("mqtt",1024);
System systm("system");
MqttJson router("mqttJson",1024);
DWM1000_Tag dwm1000Tag("dwm1000");
DWM1000_Anchor dwm1000Anchor("dwm1000");
Memory memory("memory");
Configurator configurator("config");
//SpiTester spiTester("spiTester");
//IrqTester irqTester("irqTester");
Config config;

extern void waitConfig();
/*
// A UDP instance to let us send and receive packets over UDP*/
// WiFiUDP udpClient;

// Create a new syslog instance with LOG_KERN facility
Syslog *syslog;
Str udpHost(20);
uint32_t udpPort;


void syslogger(char* start,uint32_t length)
{
    if ( syslog==0) {
//       syslog = new Syslog(udpClient,udpHost.c_str() , udpPort, Sys::hostname(), "system", LOG_KERN);
    }
    start[length]='\0';
    syslog->log(start);
    Log::serialLog(start,length);
}

enum { NONE,TAG,ANCHOR } DWM1000Role=NONE;

#include <vector.h>
#include <error_handler.h>
//etl::vector<uint32_t,100> v;


void free_error_handler(const etl::exception& e)
{
    ERROR( "The error was %s in %s at %d ",e.what() ,e.file_name() ,e.line_number() );
}

Str strLog(256);

void setup()
{
    Serial.begin(BAUDRATE, SerialConfig::SERIAL_8E1, SerialMode::SERIAL_FULL); // 8E1 for STM32
    Serial.setDebugOutput(false);
    Sys::delay(1000);
//   v.push_back(3);
    /*    WiFi.setAutoConnect(false);
        WiFi.setAutoReconnect(false);
        WiFi.enableAP(false);
        WiFi.enableSTA(false);
        WiFi.mode(WIFI_OFF); */
    etl::error_handler::free_function callback(free_error_handler);
    etl::error_handler::set_callback(callback);

    config.load();
    waitConfig();
    eb.onAny().call([](Cbor& msg) { // Log all events -> first handler
        eb.log(strLog,msg);
        DEBUG("%s",strLog.c_str());
    });

    Str strHostname(30),logLevel(5),logOutput(5);
    Sys::init();

    char hn[20];
    sprintf(hn,"ESP%X",ESP.getChipId());

    config.setNameSpace("sys");
    config.get("host",strHostname,hn);

//    system_update_cpu_freq(160);

    Sys::hostname(strHostname.c_str());

    logger.level(Log::LOG_INFO);
    INFO("version : " __DATE__ " " __TIME__ " host %s",hn);

    config.setNameSpace("log");

    config.get("level",logLevel,"I");
    logger.setLogLevel(logLevel.peek(0));
    config.get("output",logOutput,"S");
    /*  cannot log through UDP too much overhead
    if ( logOutput.peek(0)=='U') {
        config.setNameSpace("syslog");
        config.get("host",udpHost,"192.168.0.150");
        config.get("port",udpPort,514);
        logger.setOutput(syslogger);
    };
    /* Cannot log through MQTT too much overhead or DWM1000 protocol
     * if ( logOutput.peek(0)=='M') {
        mqtt.setLog(true);
    }
    */


    wifi.setup();
    INFO(" starting Wifi host : '%s' on SSID : '%s' '%s' ", wifi.getHostname(),
         wifi.getSSID(), wifi.getPassword());



    uid.add(labels,LABEL_COUNT);
    led.setMqtt(mqtt.id());
    led.setWifi(wifi.id());
    mqtt.setWifi(wifi.id());
//    wifi.setup();
    mdns.setWifi(wifi.id());
    mdns.setup();

//    memory.setup();
//    spiTester.setup();
//    irqTester.setup();
    configurator.setup();

    Str role(3);
    config.setNameSpace("lpos");
    config.get("role",role,"A");


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

    // mqtt.setHost("limero.ddns.net")
    mqtt.setup();
    router.setMqttId(mqtt.id());
    router.setup();

    led.setup();
    systm.setup();

    config.save();


    return;
}


extern "C" void tcpCleanup();

extern "C"  void loop()
{


    startHeap();
    eb.eventLoop();
    stopHeap(0,"after eventLoop" );
    wifi.loop();
    stopHeap(0,"after wifi loop" );
    mqtt.loop();
    stopHeap(0,"after mqtt loop" );
    tcpCleanup();
    mdns.loop();
    if ( DWM1000Role == TAG ) {
        dwm1000Tag.loop();
    } else if ( DWM1000Role==ANCHOR) {
        dwm1000Anchor.loop();
    }

}
