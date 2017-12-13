# USER_LINK = -L../Common/Debug -lCommon -lmain -L../Ebos/Debug -lEbos 
ARDUINO_VARIANT = nodemcu

TTY ?= USB0
SERIAL_PORT ?= /dev/tty$(TTY)
UPLOAD_PORT = $(SERIAL_PORT)
SERIAL_BAUD=921600
UPLOAD_SPEED ?= 460800 # 230400
SERIAL_BAUD = 230400
ESP_ROOT=/home/lieven/esp8266
BUILD_ROOT=build
SKETCH=main.cpp
# uncomment and set the right serail baud according to your sketch (default to 115200)
#SERIAL_BAUD = 115200
# uncomment this to use the 1M SPIFFS mapping
#SPIFFS_SIZE = 1
USER_DEFINE = -Ilibraries/Common/src -I../pubsubclient/src -Ilibraries/Ebos -Ilibrariers/etl/src \
	-DMQTT_MAX_PACKET_SIZE=768 -DMQTT_MAX_TRANSFER_SIZE=900 -DESP8266 \
	-I../Arduino/libraries/SPI  -lc
USER_LINK =  -lmain  
OTA_IP = 192.168.1.184
OTA_PORT = 8266 
OTA_AUTH = password
USER_LIBS= pubsubsclient Common Ebos DWM1000 SPI EEPROM ArduinoJson etl
USRCDIRS =  src Common/src Ebos DWM1000 SPI etl
# MINICOM_LOG = minicom.log
# include ../Esp8266-Arduino-Makefile/esp8266Arduino.mk 
include ../Esp8266-Arduino-Makefile/espXArduino.mk
#make -f ../makeEspArduino/makeEspArduino.mk ESP_ROOT=~/esp8266"
