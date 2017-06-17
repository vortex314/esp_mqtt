
https://docs.google.com/document/d/1fxgZ6O5Crq3wHmV1OV_fEfGh67ktuzFFL1c-V1uz-TM/pub

# Limero Local Positioning System
The lpos system is based on EBos and uses an ESP8266 and DWM1000 to send local positioning information to a central mqtt server.
It uses two way ranging technique - TWR.

The device tags send regularly or on request their beacon signal to the anchor. In a poll-response-final message exchange the time of flight is determined and converted to meters. 
The purpose is that through trilateration the position of device on a plane is measured.

## Schematic
The schematic is made around a standard connector ( my own ), so I can always cut the board and re-use the parts. As I expected issues with ESP8266, it was a safe bet.
The connector also makes it possible to connect to other boards. As long as they follow the same interface.

## Board
Order here :



## TODO
- Make it configurable via serial port & mqtt and store in flash.
- Make address , baud rate configurable, coordinates  
- Test in outdoors max distance measurements
- Publish article
- Pictures and movie
- Some math to calculate X/Y pos : (x-x1)^2 + (y-y1)^2 = R^2


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





