#ifndef BME280_HPT_H
#define BME280_HPT_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define BME280_I2C_ADDR 0x76



#define BME280_REG_CALIBRATION_T_ADDRLEN 0x06
#define BME280_REG_CALIBRATION_T1 0x88
#define BME280_REG_CALIBRATION_T2 0x8A
#define BME280_REG_CALIBRATION_T3 0x8C
#define BME280_REG_CALIBRATION_P_ADDRLEN 0x12
#define BME280_REG_CALIBRATION_P1 0x8E
#define BME280_REG_CALIBRATION_P2 0x90
#define BME280_REG_CALIBRATION_P3 0x92
#define BME280_REG_CALIBRATION_P4 0x94
#define BME280_REG_CALIBRATION_P5 0x96
#define BME280_REG_CALIBRATION_P6 0x98
#define BME280_REG_CALIBRATION_P7 0x9A
#define BME280_REG_CALIBRATION_P8 0x9C
#define BME280_REG_CALIBRATION_P9 0x9E
#define BME280_REG_CALIBRATION_H_ADDRLEN1 0x02 // Acctually 1 but 0xA0 seems to be unsused which would come after P9
#define BME280_REG_CALIBRATION_H1 0xA1
#define BME280_REG_CALIBRATION_H_ADDRLEN2 0x07
#define BME280_REG_CALIBRATION_H2 0xE1
#define BME280_REG_CALIBRATION_H3 0xE3
#define BME280_REG_CALIBRATION_H4 0xE4
#define BME280_REG_CALIBRATION_H5 0xE5
#define BME280_REG_CALIBRATION_H6 0xE7


#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_DATA 0xF7
#define BME280_REG_TEMP_DATA 0xFA
#define BME280_REG_HUMID_DATA 0xFD

/*
 * VALUES for ctrl_meas
 * MACRO: BME280_REG_CTRL_MEAS
 * CONTROL R/W
 */
#define BME280_BIT_MODE_0 0x01
#define BME280_BIT_MODE_1 0x02
#define BME280_VAL_MODE_MAX (BME280_BIT_MODE_0 | BME280_BIT_MODE_1)
#define BME280_VAL_MODE(x) ((x * BME280_BIT_MODE_0) & BME280_VAL_MODE_MAX)
#define BME280_BIT_OSRS_P_0 0x04
#define BME280_BIT_OSRS_P_1 0x08
#define BME280_BIT_OSRS_P_2 0x10
#define BME280_VAL_OSRS_P_MAX (BME280_BIT_OSRS_P_0 | BME280_BIT_OSRS_P_1 | BME280_BIT_OSRS_P_2)
#define BME280_VAL_OSRS_P(x) ((x * BME280_BIT_OSRS_P_0) & BME280_VAL_OSRS_P_MAX)
#define BME280_BIT_OSRS_T_0 0x20
#define BME280_BIT_OSRS_T_1 0x40
#define BME280_BIT_OSRS_T_2 0x80
#define BME280_VAL_OSRS_T_MAX (BME280_BIT_OSRS_T_0 | BME280_BIT_OSRS_T_1 | BME280_BIT_OSRS_T_2)
#define BME280_VAL_OSRS_T(x) ((x * BME280_BIT_OSRS_T_0) & BME280_VAL_OSRS_T_MAX)

/*
 * VALUES for config
 * MACRO: BME280_REG_CONFIG
 * CONTROL R/W
 */
#define BME280_BIT_SPI3W_EN 0x01
#define BME280_VAL_SPI3W_EN(x) ((x) ? BME280_BIT_SPI3W_EN : 0x00)
#define BME280_BIT_FILTER_0 0x04
#define BME280_BIT_FILTER_1 0x08
#define BME280_BIT_FILTER_2 0x10
#define BME280_VAL_FILTER_MAX (BME280_BIT_FILTER_0 | BME280_BIT_FILTER_1 | BME280_BIT_FILTER_2)
#define BME280_VAL_FILTER(x) ((x * BME280_BIT_FILTER_0) & BME280_VAL_FILTER_MAX)
#define BME280_BIT_T_STB_0 0x20
#define BME280_BIT_T_STB_1 0x40
#define BME280_BIT_T_STB_2 0x80
#define BME280_VAL_T_STB_MAX (BME280_BIT_T_STB_0 | BME280_BIT_T_STB_1 | BME280_BIT_T_STB_2)
#define BME280_VAL_T_STB(x) ((x * BME280_BIT_T_STB_0) & BME280_VAL_T_STB_MAX)

/*
 * VALUES for status
 * MACRO: BME280_REG_STATUS
 * STATUS R-only
 */
#define BME280_BIT_IM_UPDATE 0x01
#define BME280_BIT_MEASURING 0x08

/*
 * VALUES for ctrl_hum
 * MACRO: BME280_REG_CTRL_HUM
 * CONTROL R/W
 */
#define BME280_BIT_OSRS_H_0 0x01
#define BME280_BIT_OSRS_H_1 0x02
#define BME280_BIT_OSRS_H_2 0x04
#define BME280_VAL_OSRS_H_MAX (BME280_BIT_OSRS_H_0 | BME280_BIT_OSRS_H_1 | BME280_BIT_OSRS_H_2)
#define BME280_VAL_OSRS_H(x) ((x * BME280_BIT_OSRS_H_0) & BME280_VAL_OSRS_H_MAX)


#define BME280_CALCP_hPa(x) ((x) / 25600.0)
#define BME280_CALCP_Pa(x) ((x) / 256.0)
#define BME280_CALCT_dC(x) ((x) / 100.0)
#define BME280_CALCH_pcRH(x) ((x) / 1024.0)

typedef enum {
    BME280_MODE_SLEEP = 0b00,
    BME280_MODE_FORCE = 0b01,
    BME280_MODE_NORMAL = 0b11
} Bme280_Mode_t;

typedef enum {
    BME280_OSRS_IGNORE = 0b000,
    BME280_OSRS_X1 = 0b001,
    BME280_OSRS_X2 = 0b010,
    BME280_OSRS_X4 = 0b011,
    BME280_OSRS_X8 = 0b100,
    BME280_OSRS_X16 = 0b101
} Bme280_Osrs_t;

typedef enum {
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_X2 = 0b001,
    BME280_FILTER_X4 = 0b010,
    BME280_FILTER_X8 = 0b011,
    BME280_FILTER_X16 = 0b100
} Bme280_Filter_t; 

typedef enum {
    BME280_TSTB_0ms5 = 0b000,
    BME280_TSTB_62ms5 = 0b001,
    BME280_TSTB_125ms = 0b010,
    BME280_TSTB_250ms = 0b011,
    BME280_TSTB_500ms = 0b100,
    BME280_TSTB_1000ms = 0b101,
    BME280_TSTB_2000ms = 0b110,
    BME280_TSTB_4000ms = 0b111
} Bme280_TStb_t;

typedef Bme280_Osrs_t Bme280_OsrsT_t;
typedef Bme280_Osrs_t Bme280_OsrsP_t;
typedef Bme280_Osrs_t Bme280_OsrsH_t;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint16_t dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calibration_t;

typedef struct {
    i2c_port_t port;
    uint8_t dConfig;
    uint8_t dCtrlMeas;
    uint8_t dCtrlHum;
    int32_t t_fine;
    bme280_calibration_t calib;
} bme280_instance_t;

void bme280_init(bme280_instance_t* inst, i2c_port_t port);

/*
 * Sets the default config values in the instance. DOES NOT SEND THEM TO THE MODULE!
 */
void bme280_config(bme280_instance_t*inst, Bme280_TStb_t standbyTime, Bme280_Filter_t filter);

/*
 * Sets the default ctrl_meas values in the instance. DOES NOT SEND THEM TO THE MODULE!
 * WARNING: setting the mode to FORCE here will start a measurment if you call bme_commitSetup.
 */
void bme280_ctrlMeas(bme280_instance_t*inst, Bme280_Mode_t mode, Bme280_OsrsT_t oversampleTemp, Bme280_OsrsT_t oversamplePreas);

void bme280_ctrlHum(bme280_instance_t*inst, Bme280_OsrsH_t oversampleHumid);

esp_err_t bme280_commitSetup(bme280_instance_t*inst);

esp_err_t bme280_setMode(bme280_instance_t*inst, Bme280_Mode_t mode);

uint8_t bme280_checkStatus(bme280_instance_t* inst);

esp_err_t bme280_readCalibration(bme280_instance_t* inst);

uint32_t bme280_getPressure(bme280_instance_t* inst, uint8_t updateTemperature);

int32_t bme280_getTemperature(bme280_instance_t* inst);

uint32_t bme280_getHumidity(bme280_instance_t* inst, uint8_t updateTemperature);

#endif