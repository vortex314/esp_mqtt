#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include <EventBus.h>
#include <Config.h>

class Configurator : public Actor
{
    Str _key;
    Str _value;
public:
	Configurator(const char* name);
	~Configurator();
   
    void setup() ;
    void onEvent(Cbor& msg) ;
};

#endif // CONFIGURATOR_H
