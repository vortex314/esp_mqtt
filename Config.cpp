/*
 * Config.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: lieven
 */
#include <EEPROM.h>
#include "Config.h"
#include <EventBus.h>

#include <Sys.h>
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xDEADBEEF
#define KEY_SIZE 40
#define VALUE_SIZE 60

Config::Config() : json(512)
{

}

Config::~Config()
{

}


void Config::initMagic()
{
    uint32_t word = EEPROM_MAGIC;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(i, (uint8_t) (word & 0xFF));
        word >>= 8;
    }
}

bool Config::checkMagic()
{
    uint32_t word = 0;
    uint32_t i = 3;
    while (true) {
        word += EEPROM.read(i);
        if (i == 0)
            break;
        word <<= 8;
        i--;
    }
    if (word == EEPROM_MAGIC)
        return true;
    return false;
}

void Config::clear()
{
    DEBUG(" initialize EEPROM with empty config.");
    EEPROM.begin(EEPROM_SIZE);
    initMagic();
    int address = 4;
    EEPROM.write(address++, '{');
    EEPROM.write(address++, '}');
    EEPROM.write(address++, '\0');
    EEPROM.end();
}

void Config::load()
{
    if ( json.length() ) return;
    EEPROM.begin(EEPROM_SIZE);
    if (!checkMagic()) {
        DEBUG(" initialize EEPROM with empty config.");
        initMagic();
        int address = 4;
        EEPROM.write(address++, '{');
        EEPROM.write(address++, '}');
        EEPROM.write(address++, '\0');
    };
    json.clear();
    uint8_t b;
    int i=4;
    while (true) {
        b = EEPROM.read(i++);
        if (b == 0)
            break;
        json.write(b);
    }
    json.write((uint8_t)0);
    EEPROM.end();
//	DEBUG(str.c_str());
    json.parse();
    INFO(" json after load : %s",json.c_str());
}

void Config::save()
{
//	DEBUG(str.c_str());
    EEPROM.begin(EEPROM_SIZE);
    int address = 4;
    initMagic();
    for (int i = 0; i < json.length(); i++)
        EEPROM.write(address++, json.peek(i));
    EEPROM.write(address++, 0);
    ASSERT_LOG(EEPROM.commit());
    EEPROM.end();
}



bool copyExcept(Json& result,Json& input,const char *name)
{
    if (input.parse() != E_OK) {
        WARN(" invalid JSON ");
        return false;
    }
    Str key(30), value(60);
    input.rewind();
    result.clear();
    if ( input.getMap()) {
        result.addMap();
        while (true) {
            if (input.get(key)) {
                if ( key == name ) {
                    input.next();
                    continue;
                };
                result.addKey(key.c_str());
                if (input.getType() == Json::JSON_STRING) {
                    input.get(value);
                    result.add(value);
                } else if (input.getType() == Json::JSON_NUMBER) {
                    uint32_t d;
                    input.get(d);
                    result.add(d);
                } else if (input.getType() == Json::JSON_BOOL) {
                    bool flag;
                    input.get(flag);
                    result.add(flag);
                } else {
                    WARN(" invalid type ");
                }
            } else
                break;
        }
    } else {
        WARN(" no JSON OBject found ");
    }
    DEBUG(" result : %s ",result.c_str());
    DEBUG(" input : %s ",input.c_str());
    DEBUG(" except: %s ",name);
    return true;
}


void Config::set(const char* key,const char* value)
{
    Str str(0);
    str.map((uint8_t*)value,strlen(value));
    set(key,str);
}

bool Config::hasKey(const char* key) {
    json.rewind();
    return json.findKey(key);
}


void Config::set(const char* key, Str& value)
{
    load();
    Json output(EEPROM_SIZE);
    Str valueConfig(60);

//    DEBUG(" set %s:%s ",key,value.c_str());
//    DEBUG(" json before set : %s",json.c_str());
    json.rewind();
    if ( json.findKey(key) &&  json.get(valueConfig) && valueConfig ==  value) {

        return;
    }
    if ( copyExcept(output,json,key)) {
        output.addKey(key);
        output.add(value);
        output.addBreak();
        json=output;
    }
    INFO(" Config => SAVE  %s = %s",key,value.c_str());
    save();
    json.parse();
//    DEBUG(" json after set : %s",json.c_str());

}

void Config::set(const char* key, uint32_t value)
{
    load();
    Json output(EEPROM_SIZE);
    uint32_t valueConfig;

//    DEBUG(" set %s:%d ",key,value);
//    DEBUG(" json before set : %s",json.c_str());
    json.rewind();
    if ( json.findKey(key) &&  json.get(valueConfig) && valueConfig ==  value) {
        return;
    }
    if ( copyExcept(output,json,key)) {
        output.addKey(key);
        output.add(value);
        output.addBreak();
        json=output;
    }
    INFO(" Config => SAVE  %s = %d",key,value);
    save();
    json.parse();
}

void Config::get(const char* key, Str& value,
                 const char* defaultValue)
{

    load();
//	DEBUG(" input :%s",input.c_str());
    json.rewind();
    if (json.findKey(key)) {
        json.get(value);
    } else {
        value = defaultValue;
        set(key,value);
    }
    INFO(" Config => %s = %s ",key,value.c_str());
}


void Config::get(const char* key, uint32_t& value,
                 uint32_t defaultValue)
{
    load();
//	DEBUG(" input :%s",input.c_str());
    json.rewind();
    if (json.findKey(key)) {
        json.get(value);
    } else {
        value = defaultValue;
        set(key,value);
    }
    INFO(" Config => %s = %d ",key,value);
}
