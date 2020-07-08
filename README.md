# ESP32 Weather Station

This is the code for my weather station.
If basically keeps track of temperature, humidity, pressure, particles (pm10, pm2.5) and rain and pushes the data to MQTT.

The project is not Ardunino based. It is made using the [ESP-IDF SDK](https://github.com/espressif/esp-idf) with FreeRTOS.

## Hardware
 * ESP32
 * SDS011
 * BME280
 * Tipping bucket rain gague

## Measurments

The weatherstation starts a new measurment every 30 seconds. Every 10 measurments the SDS011 is activated for new PM readings. Since the SDS takes some time to get accurate results it runs for 30 seconds. Which also delays the measurment.

The data is pushed to a MQTT Broker (IP, Authentication and Topic set in the config) in a JSON format:
```json
{
	"humid": <humidity in %>, 
	"pressure": <pressure in hPa>, 
	"temp": <temperature in °C>, 
	"pm10": <pm10 value in µg/m³>, 
	"pm25": <pm2.5 value in µg/m³>, 
	"rain": <rain amount in ml/m²>
}
```

## Build the project

1. Download and install the [ESP-IDF SDK](https://github.com/espressif/esp-idf) you can find more information in the README of the SDK or [here](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/#step-1-install-prerequisites)
2. Configure the project (Refer to the section below)
3. Build the project with ``make`` and flash it to your ESP with ``make flash``

### Configuration
To configure the project run ``make menuconfig``.

If you select a menu entry/option you can press ``?`` to get more info. The keys will also be shown in on the bottom.

The Project configuration can be found in the menu ``Project (Weather) Configuration``.

In ``Networt`` set your WiFi SSID, Password. Optionally you can also change the Hostname.

Under ``MQTT`` you have to set the Broker URI (``mqtt://<ip or hostname>:<port>``) and the topic for weather data. If you use user authentication enable the option ``Use User Authentication`` and set the Username and Password.

Depending on your setup and board you might want to modify the ``Hardware`` configuration. Here you can set the Pin for the Status LED and invert the LED if necessary. You can also change the I2C settings (Pins and Clock), Uart Config for the SDS11 (Internal Uart number and pins), and more importantly the rain gague (Pin, Pull-up, Edge Trigger, Debounce, Measure period, Counts per 100ml and area).



