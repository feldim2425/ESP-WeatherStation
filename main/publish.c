#include "publish.h"

static esp_mqtt_client_handle_t _glob_client = NULL;
static const char *TAG = "Weather-MQTT";

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    //esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected to: %s", CONFIG_MQTT_BROKER_URI);
            station_connected(true);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT error");
            station_connected(false);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            station_connected(false);
        default:
            break;
    }
    return ESP_OK;
}

esp_err_t push_test_data(const char* data){
    if(_glob_client == NULL){
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t res = esp_mqtt_client_publish(_glob_client, CONFIG_MQTT_TOPIC, data, 0, 1, 0);
    return res;
}

esp_err_t push_weather_data(const weather_data_t wdata){
    if(_glob_client == NULL){
        return ESP_ERR_INVALID_STATE;
    }
    else if(wdata.data_bits == 0){
        return ESP_OK;
    }

    char data[MAX_PUBLISH_DATA_LENGTH+1] = {0};
    char buf[16] = {0};
    bool setOnce = false;
    strcat(data, "{");
    if(wdata.data_bits & WEATHER_BIT_HUMIDITY){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.humidity, buf, 2);
        sprintf(data + strlen(data), "\"humid\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }
    
    if(wdata.data_bits & WEATHER_BIT_PRESSUE){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.pressue, buf, 2);
        sprintf(data + strlen(data), "\"pressure\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }

    if(wdata.data_bits & WEATHER_BIT_TEMPERATURE){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.temperature, buf, 2);
        sprintf(data + strlen(data), "\"temp\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }

    if(wdata.data_bits & WEATHER_BIT_PM10){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.pm10, buf, 1);
        sprintf(data + strlen(data), "\"pm10\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }

    if(wdata.data_bits & WEATHER_BIT_PM25){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.pm25, buf, 1);
        sprintf(data + strlen(data), "\"pm25\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }
    
     if(wdata.data_bits & WEATHER_BIT_RAIN){
        if(setOnce){
            strcat(data, ",");
        }
        ftoa(wdata.rain, buf, 1);
        sprintf(data + strlen(data), "\"rain\":%s", buf);
        buf[0]=0;
        setOnce = true;
    }

    if(!setOnce){
        return ESP_OK;
    }
    strcat(data, "}");
    esp_err_t res = esp_mqtt_client_publish(_glob_client, CONFIG_MQTT_TOPIC, data, 0, 1, 0);
    return res;
}

esp_err_t init_publish(){
    if(_glob_client != NULL){
        return ESP_ERR_INVALID_STATE;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_MQTT_BROKER_URI,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    #if CONFIG_MQTT_USERAUTHENTICATED
        mqtt_cfg.username = CONFIG_MQTT_AUTH_USERNAME;
        mqtt_cfg.password = CONFIG_MQTT_AUTH_PASSWORD;
    #endif

    _glob_client = esp_mqtt_client_init(&mqtt_cfg);
    return esp_mqtt_client_start(_glob_client);
}

esp_err_t uninit_publish(){
    if(_glob_client == NULL){
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t res = esp_mqtt_client_stop(_glob_client);
    if(res == ESP_OK){
        res = esp_mqtt_client_destroy(_glob_client);
    }
    return res;
}