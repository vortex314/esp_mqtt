/*
 * Config.h
 *
 *  Created on: Jul 12, 2016
 *      Author: lieven
 */

#ifndef ACTOR_CONFIG_H_
#define ACTOR_CONFIG_H_

#include <Str.h>


class Config {
	void initialize();
	void initMagic();
	bool checkMagic();


public:
	Config();
	virtual ~Config();

	void load();
	void save();

	void get(const char*, uint32_t &, uint32_t defaultValue);
	void get(const char*, Str&, const char* defaultValue);

	void set(const char*, uint32_t &);
	void set(const char*, Str&);
};

extern Config config;

#endif /* ACTOR_CONFIG_H_ */
