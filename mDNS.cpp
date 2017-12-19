/*
 * mDNS.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: lieven
 */

#include "mDNS.h"
//MDNSResponder responder;

mDNS::mDNS(const char* name) :
    Actor(name) 
{
 
    
}

mDNS::~mDNS()
{
}

void mDNS::setWifi(uid_t wifi) {
    _wifi = wifi;
}


void mDNS::setup()
{
    eb.onEvent(_wifi,H("connected")).call(this,(MethodHandler)&mDNS::onWifiConnected);
}

void mDNS::onEvent(Cbor& cbor)
{
ERROR(" no events expected");
}

void mDNS::onWifiConnected()
{
    if (!MDNS.begin(WiFi.hostname().c_str(),WiFi.localIP())) {
        WARN("Error setting up MDNS responder!");
    }
    INFO(" MDNS publish %s ",WiFi.hostname().c_str());
}

void mDNS::loop()
{
    MDNS.update();
//   responder.update();
}

