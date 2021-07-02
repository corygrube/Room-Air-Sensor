# Room-Sensor
Self-contained room monitoring sensors for CO2, temperature, and humidity on ESP32

## Framework
ESP-IDF release/v4.3

## Hardware
* Controller: Wemos Lolin32 development board with embedded OLED display (branded HW-724, but any board of this nature should be suitable)
* CO2 sensor: MH-Z19B
* Temp/Hum sensor: DHT11

## To Do
* Features
	* Configure as Wifi station
	* Add MQTT for remote monitoring
	* Configure embedded OLED for local sensor display
* Optimizations
	* Rework get_CO2 to not rely on vTaskDelay (event handler?)
	* Improve error handling (/add it at all)
	* Consider revisiting tasks one day?
	* Add 5 minute(+/-?) startup delay for CO2 - readings often invalid/frozen on boot
	* Identify source of occasional 1DegC/1%RH readings from DHT11.
		* Presumably Either bad hardware, or timeout/error return from library
		* If they're a fact of life, implement retry logic
* Documentation
	* Draw wiring diagram (/PCB layout?)
		* Design circuit/PCB/Enclosure to make this a self-contained USB dongle
	* Improve comments
	* Add instructions for dependencies
	* Add instructions for wifi parameter config (MenuConfig)