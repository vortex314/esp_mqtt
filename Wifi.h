/*
 * Wifi.h
 *
 *  Created on: Jun 27, 2016
 *      Author: lieven
 */

#ifndef WIFI_H_
#define WIFI_H_

#include <EventBus.h>
#include <c_types.h>
#include <ESP8266WiFi.h>

class Wifi: public Actor {
	Str _ssid;
	Str _password;
	Str _hostname;
public:
	Wifi(const char* name);

	virtual ~Wifi();
	void init();
	void setup();
	void loop();
	inline bool connected() {
		return state()==H("connected");
	}
	void setConfig(Str& ssid,Str& password,Str& hostname);
	const char* getHostname();
	const char* getSSID();
	const char* getPassword();
	void switchState(int st);

};

#endif /* WIFI_H_ */
