#ifndef STATION_H
#define STATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "constants.h"
#include "publish.h"
#include "sds011.h"
#include "bme_hpt.h"

#define STATION_CONFIG_PERIOD pdMS_TO_TICKS(60000)
#define STATION_CONFIG_TIMEOUT pdMS_TO_TICKS(35000)
#define STATION_CONFIG_PMWORK pdMS_TO_TICKS(30000)
#define STATION_CONFIG_PMINTERVAL 10
#define SDSUART_BUF_SIZE 1024

#define STATION_EVENT_TSTART 0x01 // Measurment Timer executed while measuring this bit is used for timeout during standby its used for scheduling
#define STATION_EVENT_TTOUT 0x02 // Measurment Timer executed while measuring this bit is used for timeout during standby its used for scheduling
#define STATION_EVENT_DAT_PM 0x04 // PM Sensor recieved Data
#define STATION_EVENT_DAT_PTH 0x08 // Preasure, Temerature, Humidity Sensor recived Data
#define STATION_EVENT_START_PTH 0x10 // Preasure, Temerature, Humidity Sensor recived Data

#define STATION_STATE_RUNNING 0x01 // Set if Measruement is in process
#define STATION_STATE_WITHPM 0x02 // Set if the PM Sensor is queued for measurement
#define STATION_STATE_READY 0x04 // Set if the Station is internaly and externaly ready to make masurments
#define STATION_STATE_CONNECTED 0x08 // Set if the outside APIs are ready to allow operation (WIFI & MQTT connected)
#define STATION_STATE_INITIALIZED 0x10 // Set if the station is inittialized
#define STATION_STATE_RAINMEASURED 0x20 // Set if the last masured rain amount is valid

void init_station();

void station_connected(uint8_t state);

#endif