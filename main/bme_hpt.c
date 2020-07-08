#include "bme_hpt.h"
static const char *TAG = "BME-LIB";

const uint8_t _bme280_cal_h_addr[] = {0xA1, 0xE1, 0xE3, 0xE4, 0xE5, 0xE7};
const uint8_t _bme280_cal_h_shorts = 0b011010; 

#define BME_TOS20BIT(xl, l, h) (((int32_t)(h) << 12) | ((int32_t)(l) << 4) | ((int32_t)(xl) >> 4))

static esp_err_t send_bme280(i2c_port_t port, uint8_t* reg, uint8_t* data, uint8_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    for(uint i = 0; i < len; i++){
        i2c_master_write_byte(cmd, reg[i], true);
        i2c_master_write_byte(cmd, data[i], true);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t read_bme280(i2c_port_t port, uint8_t reg, uint8_t* data, uint8_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_I2C_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bme280_init(bme280_instance_t* inst, i2c_port_t port){
    inst->port = port;
    inst->dConfig = 0;
    inst->dCtrlMeas = 0;
    inst->dCtrlHum = 0;
}

void bme280_config(bme280_instance_t*inst, Bme280_TStb_t standbyTime, Bme280_Filter_t filter){
    inst->dConfig = BME280_VAL_T_STB(standbyTime) | BME280_VAL_FILTER(filter);
}

void bme280_ctrlMeas(bme280_instance_t*inst, Bme280_Mode_t mode, Bme280_OsrsT_t oversampleTemp, Bme280_OsrsT_t oversamplePreas){
    inst->dCtrlMeas = BME280_VAL_MODE(mode) | BME280_VAL_OSRS_T(oversampleTemp) | BME280_VAL_OSRS_P(oversamplePreas);
}

void bme280_ctrlHum(bme280_instance_t*inst, Bme280_OsrsH_t oversampleHumid){
    inst->dCtrlHum = BME280_VAL_OSRS_H(oversampleHumid);
}

esp_err_t bme280_commitSetup(bme280_instance_t*inst){
    return send_bme280(inst->port, (uint8_t[]){BME280_REG_CONFIG, BME280_REG_CTRL_MEAS, BME280_REG_CTRL_HUM}, (uint8_t[]){inst->dConfig, inst->dCtrlMeas, inst->dCtrlHum},3);
}

#define BME280_TOSHORT(l,h) ((int16_t)(l) | ((int16_t)(h) << 8))
#define BME280_TOUSHORT(l,h) ((uint16_t)(l) | ((uint16_t)(h) << 8))
/*
 * Calibration table can be take from the BME250 Datasheet Page 24-25
 * The calibration data is mostly stored in two continuous chunks.
 * All the Temperature and Pressure calibration is in Chunk 1 as well as the first value for Humidity
 * Humidity is mostly stored in chunk 2. 
 * There is however a one byte gap between the value for P9 and H1 in the firs Chunk
 */
esp_err_t bme280_readCalibration(bme280_instance_t*inst){
    const uint8_t dataLength1 = BME280_REG_CALIBRATION_T_ADDRLEN + BME280_REG_CALIBRATION_P_ADDRLEN + BME280_REG_CALIBRATION_H_ADDRLEN1;
    const uint8_t dataLength2 = BME280_REG_CALIBRATION_H_ADDRLEN2;
    bme280_calibration_t* calib = &(inst->calib);
    
    uint8_t data1[dataLength1];
    uint8_t data2[dataLength2];
    esp_err_t res = read_bme280(inst->port, BME280_REG_CALIBRATION_T1 , data1, dataLength1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    if(res != ESP_OK){
        return res;
    }
    res = read_bme280(inst->port, BME280_REG_CALIBRATION_H2 , data2, dataLength2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    if(res != ESP_OK){
        return res;
    }
   
    calib->dig_T1 = BME280_TOUSHORT(data1[0], data1[1]);
    calib->dig_T2 = BME280_TOSHORT(data1[2], data1[3]);
    calib->dig_T3 = BME280_TOSHORT(data1[4], data1[5]);
    calib->dig_P1 = BME280_TOUSHORT(data1[6], data1[7]);
    calib->dig_P2 = BME280_TOSHORT(data1[8], data1[9]);
    calib->dig_P3 = BME280_TOSHORT(data1[10], data1[11]);
    calib->dig_P4 = BME280_TOSHORT(data1[12], data1[13]);
    calib->dig_P5 = BME280_TOSHORT(data1[14], data1[15]);
    calib->dig_P6 = BME280_TOSHORT(data1[16], data1[17]);
    calib->dig_P7 = BME280_TOSHORT(data1[18], data1[19]);
    calib->dig_P8 = BME280_TOSHORT(data1[20], data1[21]);
    calib->dig_P9 = BME280_TOSHORT(data1[22], data1[23]);
    calib->dig_H1 = data1[25];
    calib->dig_H2 = BME280_TOSHORT(data2[0], data2[1]);
    calib->dig_H3 = data2[2];
    calib->dig_H4 = ((int16_t)data2[4] & 0x0F) | ((int16_t)data2[3] << 4);
    calib->dig_H5 = ((int16_t)data2[4] >> 4) | ((int16_t)data2[5] << 4);
    calib->dig_H6 = (int8_t) data2[6];

    return ESP_OK;
}

esp_err_t bme280_setMode(bme280_instance_t*inst, Bme280_Mode_t mode){
    uint8_t val = inst->dCtrlMeas &~ BME280_VAL_MODE_MAX;
    val |= BME280_VAL_MODE(mode);
    return send_bme280(inst->port, (uint8_t[]){BME280_REG_CTRL_MEAS}, (uint8_t[]){val},1);
}

uint8_t bme280_checkStatus(bme280_instance_t* inst){
    uint8_t data;
    esp_err_t res = read_bme280(inst->port, BME280_REG_STATUS, &data, 1);
    if(res!=ESP_OK){
        return 0xff;
    }
    return data;
}

/*
 * This function updates the temperature.
 * The temerature is stored as it is required for compoensation in humidity and pressure
 */
static void bme280_updateTFine(bme280_instance_t* inst, uint8_t* data){
    bme280_calibration_t* calib = &inst->calib;
    int32_t adc_T;
    int32_t var1, var2;
    adc_T = BME_TOS20BIT(data[2], data[1], data[0]);

    // Please don't ask me what that does
    // This is the compensation formula for temperature in the BME250 Datasheet (Page 25)
    var1 = ((((adc_T>>3) - ((int32_t)calib->dig_T1<<1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calib->dig_T1)) * ((adc_T>>4) - ((int32_t)calib->dig_T1))) >> 12) *
    ((int32_t)calib->dig_T3)) >> 14;
    inst->t_fine = var1 + var2;
}

uint32_t bme280_getPressure(bme280_instance_t* inst, uint8_t updateTemperature){
    uint8_t data[6];
    esp_err_t res = read_bme280(inst->port, BME280_REG_PRESS_DATA, data, updateTemperature ? 6 : 3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    if(res == ESP_OK && updateTemperature){
        bme280_updateTFine(inst, data + 3);
    }
    else if(res != ESP_OK){
        return 0x00;
    }

    // Please don't ask me what that does
    // This is the compensation formulas straight out of the BME250 Datasheet for Pressure (Page 25)
    bme280_calibration_t* calib = &inst->calib;
    int32_t adc_P = BME_TOS20BIT(data[2], data[1], data[0]);
    int64_t var1, var2, p;
    var1 = ((int64_t)inst->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1*(int64_t)calib->dig_P5)<<17);
    var2 = var2 + (((int64_t)calib->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3)>>8) + ((var1 * (int64_t)calib->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib->dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)calib->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7)<<4);
    return (uint32_t)p;
}

int32_t bme280_getTemperature(bme280_instance_t* inst){
    uint8_t data[3];
    esp_err_t res = read_bme280(inst->port, BME280_REG_TEMP_DATA, data, 3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    if(res == ESP_OK){
        bme280_updateTFine(inst, data);
    }
    // This part is actually part of the BME250 temerature compensation (Page 25)
    // It is seperated since t_fine is calculated in bme280_updateTFine
    return (inst->t_fine * 5 + 128) >> 8;
}

uint32_t bme280_getHumidity(bme280_instance_t* inst, uint8_t updateTemperature){
    uint8_t data[3];
    esp_err_t res = ESP_OK;
    if(updateTemperature) {
        res = read_bme280(inst->port, BME280_REG_TEMP_DATA, data, 3);
        ESP_ERROR_CHECK_WITHOUT_ABORT(res);
        if(res == ESP_OK){
            bme280_updateTFine(inst, data);
        }
        else {
            return 0x00;
        }
    }
    
    res = read_bme280(inst->port, BME280_REG_HUMID_DATA, data, 3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    if(res != ESP_OK){
        bme280_updateTFine(inst, data);
    }


    // Please don't ask me what that does
    // This is the compensation formula for humidity in the BME250 Datasheet (Page 25-26)
    bme280_calibration_t* calib = &inst->calib;
    int32_t adc_H = BME280_TOSHORT(data[1], data[0]);
    int32_t v_x1_u32r;
    v_x1_u32r = (inst->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) - (((int32_t)calib->dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((int32_t)calib->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib->dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)calib->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}