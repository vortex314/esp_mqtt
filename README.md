
https://docs.google.com/document/d/1fxgZ6O5Crq3wHmV1OV_fEfGh67ktuzFFL1c-V1uz-TM/pub

The target is to create a very modular local positioning system which is easy to deploy and integrate.
The tag measures the distance towards different anchors by measuring the time of flight for an electro magnetic wave

The modules are using a DWM1000 and an ESP8266.
The ESP8266 is driving the DWM1000 and sending the results through the use of MQTT to a MQTT broker
where any host can subscribe. The host needs to do the trilateration calculation , using the
known positions of the anchors. 

The tag will auto-discover the anchors as each anchor sends out on a regular base their
existence.

The cost of this system is low compared with some commercial products :
    - ESP8266 no
## Challenges
The following investigations tooks weeks to complete and make it work.

- The SPI interface of the ESP8266 is very complex and still badly documented. 
- The DWM1000 should have been mode 0 on SPI level, but seems CPOL=0,PHASE=1.
- Some bits are set on SPI ESP8266 by trial and error and don't really explain what they do. The duty cycle of the SPI_CK_OUT_HIGH defines if it reads correct or not. 
- The registers of the SPI need to be set in a specific orders or they don't contain the values that are written to them. SPI_CTRL and SPI_CTRL2 exhibit this strange behaviour.
- I suspect that I needed to navigate between the bugs of the SPI implementation.
- luckily the DW1000 protocol didn't require to send messages longer then 64 bytes, otherwise manual CS select was needed with concatenation of different parts, probably resulting in a slower implementation. 
- The examples provided by DWM1000 documentation, suppose that the message is handled in 150 micro sec. While this is nowhere stated. The example transmits at 115kb, which leads to 2.4 msec frame duration, the reply time for the anchor is 2.6 msec. The ESP8266 took 450 microsec to respond. So exceeding the reponse time, which make sthat the polling response is never send. When changing to 850KB the issue is gone. Pffff
The Sample provided by Decawave should provide a word of warning.
Still don't understand why the DW1000 is not SPI mode 0. While according to the doc it should be. 

## configuration parameters
As the device is intended for generic use , the parameters can be set at boot up time via serial port or via the mqtt protocol, once the mqtt config is established 

- wifi.ssid : network identification
- wifi.password : network authentication
- mqtt.host : MQTT broker host, DNS name or IP address
- mqtt.port : MQTT port
- lps.#role : tag | anchor
- lps.address : 2 chars for address : "T1"
- lps.anchors :  ["A1","A2"] in case of tag , anchors to address


dst/ESP1223.config/mqtt.host : test.mosquitto.org

is equivalent to send { "#dst":"ESP1223.config","#src":"me","request":"set","key":"mqtt.host","value":"test.mosquitto.org" }
response will arrive on "dst/me" : { "#dst":"me","#src":"ESP1223.config","#reply":"set","error":0,...}


dst/ESP1223.mqtt/host : test.mosquitto.org

mqtt.set(key:host,value:test.mosquitto.org);

subscribe("dst/ESP1223.service/#")

if ( contains(extraField) ) 
	makeSetRequestKV(extraField,mqttPayload)
"#request":"set"
"#src":"unknown"
"key":"extraField"
"value":mqttPayload
"id

demcu : 7 €
    - DWM1000 : 25 €
    - PCB and some components : 5 €
    - total : +/- 37 €

get started quickly : 
    - build the system according to instructions
    - download the binary 
    - configure through serial port the MQTT settings and the role of the system : T(ag) or A(chor)
    - enjoy !


# Limero Local Positioning System
The lpos system is based on EBos and uses an ESP8266 and DWM1000 to send local positioning information to a central mqtt server.
It uses two way ranging technique - TWR.


## TODO
- Make it configurable via serial port & mqtt and store in flash. - DONE
- Test in outdoors max distance measurements - first test : 20M
- Publish article 
- Pictures and movie
- Some math to calculate X/Y pos : (x-x1)^2 + (y-y1)^2 = R^2





