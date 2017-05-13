/*
 * Config.h
 *
 *  Created on: Jul 12, 2016
 *      Author: lieven
 */

#ifndef ACTOR_CONFIG_H_
#define ACTOR_CONFIG_H_

#include <Json.h>


class Config {
	void initialize();
	void initMagic();
	bool checkMagic();
    Json json;

public:
	Config();
	virtual ~Config();

    void clear();
	void load();
	void save();

    bool hasKey(const char* key);
	void get(const char*, uint32_t &, uint32_t defaultValue);
	void get(const char*, Str&, const char* defaultValue);

	void set(const char*, uint32_t );
	void set(const char*, Str&);
    void set(const char* key, const char* value);
};

extern Config config;

#endif /* ACTOR_CONFIG_H_ */
