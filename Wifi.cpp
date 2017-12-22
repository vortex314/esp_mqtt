/*
 * Wifi.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: lieven
 */

#include "Wifi.h"
#include <Config.h>
#include <Property.h>

const  char* getIP(){
  return  WiFi.localIP().toString().c_str();
}

Wifi::Wifi(const char* name) :
    Actor(name),_password(60),_ssid(30),_hostname(30)
{

}

Wifi::~Wifi()
{
}

void Wifi::init()
{

}

void Wifi::switchState(int st)
{
    if (st != Actor::state()) {
        Actor::state(st);
        if (st == H("connected")) {
            INFO(" Wifi Connected.");
            INFO(" ip : %s ", WiFi.localIP().toString().c_str());
            eb.event(id(),H("connected")).addKeyValue(H("data"),true);
            eb.send();
        } else  if (st == H("disconnected")) {
            WARN(" Wifi Disconnected.");
            eb.event(id(),H("disconnected")).addKeyValue(H("data"),true);
            eb.send();
        }
    }
}

void Wifi::setup()
{
    config.setNameSpace("wifi");
    config.get("ssid",_ssid,"Merckx3");
    config.get("password",_password,"LievenMarletteEwoutRonald");
    _hostname=Sys::hostname();
    state(0);
//    switchState(H("disconnected"));
    WiFi.hostname(_hostname.c_str());
//   WiFi.begin(_ssid.c_str(), _password.c_str());
    WiFi.enableSTA(true);
    Property<const char*>::build(getIP, id(), H("IP"), 20000);
}

void Wifi::loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG("Connecting to %s ... ",_ssid.c_str());
        WiFi.begin(_ssid.c_str(), _password.c_str());

        if (WiFi.waitForConnectResult() != WL_CONNECTED) {
            DEBUG(" still connecting ");
            return;
        } else
            INFO(" starting Wifi host : '%s' on SSID : '%s' PSWD : '%s' ", getHostname(),
                 getSSID(), getPassword());

    }
    if ( WiFi.status() == WL_CONNECTED )  {
        switchState(H("connected"));
    }
    else if (  WiFi.status() == WL_DISCONNECTED ) {
        switchState(H("disconnected"));
    } else  {
        ERROR(" unknown Wifi status%d ",WiFi.status());
    }
//   switchState(WiFi.status() == WL_CONNECTED ? H("connected") : H("disconnected"));
}

void Wifi::setConfig(Str& ssid, Str& password, Str& hostname)
{
    _ssid = ssid;
    _password = password;
    _hostname = hostname;
}

const char*  Wifi::getHostname()
{
    return Sys::hostname();
}

const char*  Wifi::getSSID()
{
    return _ssid.c_str();
}



const char*  Wifi::getPassword()
{
    return _password.c_str();
}
