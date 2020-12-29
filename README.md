# Room-Sensor
Self-contained room monitoring sensors for CO2, temperature, and humidity on ESP32

## Framework
ESP-IDF release/v4.1

## Hardware
* Controller: Wemos Lolin32 development board with embedded OLED display (branded HW-724, but any board of this nature should be suitable)
* CO2 sensor: MH-Z19B
* Temp/Hum sensor: DHT11

## To Do
* Add DHT11 temperature/humidity sensor
* Configure OLED for local sensor display
* Add Wifi/MQTT for remote monitoring
	* Actually get MQTT to work
* Add wiring diagram
* Learn more about/optimize FreeRTOS task structure