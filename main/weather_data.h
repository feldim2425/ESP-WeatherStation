#ifndef WEATHER_DATA_H
#define WEATHER_DATA_H

#include <stdint.h>

typedef struct {
    uint8_t data_bits;
    float humidity;
    float pressue;
    float temperature;
    float pm10;
    float pm25;
    float rain;
} weather_data_t;

#define WEATHER_BIT_HUMIDITY 0x01
#define WEATHER_BIT_PRESSUE 0x02
#define WEATHER_BIT_TEMPERATURE 0x04
#define WEATHER_BIT_PM10 0x08
#define WEATHER_BIT_PM25 0x10
#define WEATHER_BIT_RAIN 0x20

#endif