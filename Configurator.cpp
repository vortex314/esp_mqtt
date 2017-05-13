#include "Configurator.h"

Configurator::Configurator(const char* name) : Actor(name),_key(30),_value(60)
{
}

Configurator::~Configurator()
{
}



void Configurator::setup()
{
    eb.onDst(id()).call(this);
    timeout(5000);
    INFO(" before init");

}

void Configurator::onEvent(Cbor& msg)
{
    uint32_t iValue;
    uint32_t error;
    if ( timeout() ) {
        timeout(5000);
        eb.publicEvent(id(),H("configurator")).addKeyValue(H("$data"),"ccc");
        eb.send();
    } else if ( eb.isRequest(H("set"))) {
        Cbor& reply = eb.reply();
        if ( msg.getKeyValue(H("key"),_key) && msg.getKeyValue(H("value"),_value) ) {
            eb.reply().addKeyValue(H("key"),_key).addKeyValue(H("value"),_value);
            config.set(_key.c_str(),_value);
            reply.addKeyValue(H("error"),0);
        } else {
            reply.addKeyValue(H("error"),EINVAL);
        }
        eb.send();
    } else if ( eb.isRequest(H("get"))) {
        Cbor& reply = eb.reply();
        if ( msg.getKeyValue(H("key"),_key) && config.hasKey(_key.c_str())) {
            config.get(_key.c_str(),_value,"");
            reply.addKeyValue(H("key"),_key).addKeyValue(H("value"),_value);
            reply.addKeyValue(H("error"),0);
        } else {
            reply.addKeyValue(H("error"),EINVAL);
        }
        eb.send();
    } else {
        eb.reply().addKeyValue(H("error"),EINVAL).addKeyValue(H("error-msg"),"invalid request");
        eb.send();
    }
};
