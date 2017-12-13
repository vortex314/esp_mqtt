#include "Mqtt.h"
#include <Cbor.h>
#include <EventBus.h>
#include <Config.h>

#include <unistd.h>
#include <fcntl.h>
Mqtt* Mqtt::_thisMqtt;
//--------------------------------------------------------------------------------------------------------
Mqtt::Mqtt(const char* name,uint32_t maxSize) : Actor(name),
    _host(40), _port(1883), _clientId(30), _user(20), _password(20), _willTopic(
        30), _willMessage(30), _willQos(0), _willRetain(false), _keepAlive(
            20), _cleanSession(1), _msgid(0),_prefix(20),_topic(TOPIC_LENGTH),_message(maxSize)
{
    _lastSrc=0;
    _thisMqtt=this;
}
//--------------------------------------------------------------------------------------------------------
Mqtt::~Mqtt()
{
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::setup()
{
    config.setNameSpace("mqtt");
    config.get("host",_host,"limero.ddns.net");
    config.get("port",_port,1883);
    config.get("clientId",_clientId,Sys::hostname());
    config.get("user",_user,"");
    config.get("password",_password,"");
    _willTopic = "src/";
    _willTopic += Sys::hostname();
    _willTopic += "/system/alive";
    config.get("mqtt.willTopic",_willTopic,_willTopic.c_str());
    config.get("willMessage",_willMessage,"false");
    config.get("keepAlive",_keepAlive,20);

    _wifi_client = new WiFiClient();
    _pubSub =  new PubSubClient(*_wifi_client);
    _pubSub->setServer(_host.c_str(), 1883);
    _pubSub->setCallback(callback);
    state(H("disconnected"));

    eb.onDst(id()).call(this);
    eb.onEvent(_wifi,EB_UID_ANY).call(this,(MethodHandler)&Mqtt::onWifiEvent);
    timeout(5000);
//	eb.onRequest(H("mqtt"),H("publish")).call(this,(MethodHandler)&Mqtt::publish);
//	eb.onRequest(H("mqtt"),H("subscribe")).call(this,(MethodHandler)&Mqtt::subscribe);
// 	eb.onRequest(H("mqtt"),H("connected")).call(this,(MethodHandler)&Mqtt::isConnected);
}

void Mqtt::onWifiEvent(Cbor&  msg)
{
    if ( eb.isEvent(_wifi,H("connected"))) {
        pubSubConnect();
    } else if ( eb.isEvent(_wifi,H("disconnected"))) {
        _pubSub->disconnect();
    }
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::onEvent(Cbor& msg)
{
    if ( eb.isRequest(id(),H("publish"))) {
        publish(msg);
    } else if ( eb.isRequest(id(),H("subscribe"))) {
        subscribe(msg);
    } else if ( eb.isRequest(id(),H("connected)"))) {
        isConnected(msg);
    } else if ( timeout() ) {
        if ( state()==H("disconnected") ) {
            _pubSub->disconnect();
            pubSubConnect();
        }
        timeout(5000);
    }  else {
        eb.defaultHandler(this,msg);
    }
}
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
void Mqtt::loop()
{
    _pubSub->loop();
    if ( _pubSub->state() != _client_state) {
        _client_state = _pubSub->state();
        if ( _client_state == MQTT_CONNECTED ) {
            state(H("connected"));
            INFO(" state changed : %s ",uid.label(state()));
        } else {
            state(H("disconnected"));
            INFO(" state changed : %s ",uid.label(state()));
            pubSubConnect();
        }
//       eb.publish(id(),state());
        //      eb.send();
    }

}
//--------------------------------------------------------------------------------------------------------
void Mqtt::onActorRegister(Cbor& cbor)
{
}

//--------------------------------------------------------------------------------------------------------
void Mqtt::wakeup()
{
    DEBUG(" wakeup ");
}


//--------------------------------------------------------------------------------------------------------
void Mqtt::loadConfig(Cbor& cbor)
{
    cbor.getKeyValue(H("host"), _host);
    cbor.getKeyValue(H("port"), _port);
    cbor.getKeyValue(H("client_id"), _clientId);
    cbor.getKeyValue(H("user"), _user);
    cbor.getKeyValue(H("password"), _password);
    cbor.getKeyValue(H("keep_alive"), _keepAlive);
    cbor.getKeyValue(H("clean_session"), _cleanSession);
    cbor.getKeyValue(H("will_topic"), _willTopic);
    cbor.getKeyValue(H("will_message"), _willMessage);
    cbor.getKeyValue(H("will_qos"), _willQos);
    cbor.getKeyValue(H("will_retain"), _willRetain);
    cbor.getKeyValue(H("prefix"),_prefix);
}
//--------------------------------------------------------------------------------------------------------
bool Mqtt::pubSubConnect()
{

    if ( _pubSub->connected()) return true;
    INFO( " Connecting to (%s : %d) will (%s : %s) with clientId : %s ",_host.c_str(),_port,_willTopic.c_str(),_willMessage.c_str(),_clientId.c_str());
    if ( _pubSub->connect (_clientId.c_str(), _user.c_str(), _password.c_str(), _willTopic.c_str(), _willQos, _willRetain, _willMessage.c_str())) {
        eb.publish(id(),H("connected"));
        state(H("connected"));
        return true;
    } else {
        eb.publish(id(),H("disconnected"));
        state(H("disconnected"));
        return false;
    }
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::callback(char* topic,byte* message,uint32_t length)
{
    INFO(" message arrived : [%s]",topic);
    Str tpc(topic);
    Str msg(message,length);
    eb.event(_thisMqtt->id(),H("published")).addKeyValue(H("topic"), tpc).addKeyValue(H("message"), msg); // TODO make H("mqtt") dynamic on name
    eb.send();
}
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
void Mqtt::isConnected(Cbor& cbor)
{
    cbor.getKeyValue(EB_SRC,_lastSrc);
    if ( state()==H("connected)") ) {
        eb.reply().addKeyValue(H("error"),0);
        eb.send();
    } else {
        eb.reply().addKeyValue(H("error"),_pubSub->state());
        eb.send();
    }
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::disconnect(Cbor& cbor)
{
    INFO("disconnect()");
//    eb.publish(id(),H("disconnected"));
    _pubSub->disconnect();
    eb.reply().addKeyValue(H("error"), (uint32_t) E_OK);
    eb.send();
}
//--------------------------------------------------------------------------------------------------------
void startHeap();
void stopHeap(Actor* a,const char* s);
//--------------------------------------------------------------------------------------------------------
void Mqtt::publish(Cbor& cbor)
{
    if (state()!=H("connected")) {
        eb.reply().addKeyValue(H("error_detail"),"not connected").addKeyValue(EB_ERROR, (uint32_t) ENOTCONN);
        eb.send();
        WARN(" not connected ");
        return;
    }
    if ( cbor.getKeyValue(H("topic"), _topic) && cbor.getKeyValue(H("message"), _message)) {
        bool retain=false;
        cbor.getKeyValue(H("retain"), retain);
        _pubSub->loop();    // to avoid timeouts  https://github.com/knolleary/pubsubclient/issues/151
        stopHeap(this,"loop1");
        if ( _pubSub->publish(_topic.c_str(),_message.data(),_message.length(),retain)) {
            stopHeap(this,"publish");
            _pubSub->loop();    // to avoid timeouts  https://github.com/knolleary/pubsubclient/issues/151
            eb.reply().addKeyValue(EB_ERROR, (uint32_t) E_OK);
            eb.send();
        } else {
            stopHeap(this,"error");
            eb.reply().addKeyValue(EB_ERROR, _pubSub->state());
            eb.send();
            /*           Cbor msg(0);
                       disconnect(msg);
                       eb.event(id(),H("disconnected"));
                       eb.send(); */
        }
    } else {
        eb.reply().addKeyValue(H("error"), _pubSub->state());
        eb.send();
    }
}

//======================================================
void mqttLog(char* start,uint32_t length)
{
    if (Mqtt::_thisMqtt
        && Mqtt::_thisMqtt->_pubSub
        && Mqtt::_thisMqtt->_pubSub->state() == MQTT_CONNECTED )
        Mqtt::_thisMqtt->log(start,length);
    else
        Log::serialLog(start,length);
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::setLog(bool on)
{
    if ( on ) {
        logger.setOutput(mqttLog);
    } else {
        logger.defaultOutput();
    }
}
//--------------------------------------------------------------------------------------------------------
Str logTopic(40);
void Mqtt::log(char* start,uint32_t length)
{
    if ( logTopic.length()==0) {
        logTopic = "src/";
        logTopic += Sys::hostname();
        logTopic += "/log";
    }
    _pubSub->publish(logTopic.c_str(),(uint8_t*)start,length,false);
    _pubSub->loop();
}
//--------------------------------------------------------------------------------------------------------
void Mqtt::subscribe(Cbor& cbor)
{
    cbor.getKeyValue(EB_SRC,_lastSrc);
    if (state() != H("connected")) {
        eb.reply().addKeyValue(H("error"), ENOTCONN);
        eb.send();
        return;
    }
    Str topic(TOPIC_LENGTH);
    int qos = 0;
    if ( cbor.getKeyValue(H("topic"), topic)  ) {
        cbor.getKeyValue(H("qos"), qos);
        DEBUG(" topic : %s qos : %d ",topic.c_str(),qos);
        if ( _pubSub->subscribe(topic.c_str(),qos) ) {
            eb.reply().addKeyValue(H("error"), E_OK);
            eb.send();
        } else {

            eb.reply().addKeyValue(H("error"), EFAULT);
            eb.send();
            WARN("NOK OK : EFAULT");
        }
    } else {
        eb.reply().addKeyValue(H("error"), EINVAL);
        eb.send();
        WARN("NOK OK : EINVAL");
    }
    _pubSub->loop(); // https://github.com/knolleary/pubsubclient/issues/98
}
//--------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------
