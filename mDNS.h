/*
 * mDNS.h
 *
 *  Created on: Jul 2, 2016
 *      Author: lieven
 */

#ifndef MDNS_H_
#define MDNS_H_
#include <Actor.h>
#include <Wifi.h>
#include <ESP8266mDNS.h>
class mDNS :public Actor {
//	Str _service;
	uint16_t _port;
	uid_t _wifi;
public:
	mDNS(const char* name);
	virtual ~mDNS();
	void onEvent(Cbor& msg);
	void loop();
	void onWifiConnected();
	void setup();
    void setWifi(uid_t wifi);
};

#endif /* MDNS_H_ */
