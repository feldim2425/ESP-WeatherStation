#ifndef PUBLISH_H
#define PUBLISH_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "weather_data.h"
#include "ftoa.h"
#include "station.h"

#define MAX_PUBLISH_DATA_LENGTH 128

esp_err_t push_test_data(const char* data);

esp_err_t push_weather_data(const weather_data_t wdata);

esp_err_t init_publish();

esp_err_t uninit_publish();

#endif