
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
    - ESP8266 nodemcu : 7 €
    - DWM1000 : 25 €
    - PCB and some components : 5 €
    - total : +/- 37 €

get started quickly : 
    - build the system according to instructions
    - download the binary 
    - configure through serial port the MQTT settings and the role of the system : T(ag) or A(chor)
    - enjoy !

https://drive.google.com/open?id=1sEt6OF94nYKO7jOcpxjCYkwHNYLjgiLhi7eMlyxKfjA


## TODO
- Make it configurable via serial port & mqtt and store in flash. - DONE
- Test in outdoors max distance measurements - first test : 20M
- Publish article 
- Pictures and movie
- Some math to calculate X/Y pos : (x-x1)^2 + (y-y1)^2 = R^2





