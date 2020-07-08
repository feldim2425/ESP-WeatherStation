/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
//#include "esp_heap_trace.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "constants.h"
#include "station.h"
#include "publish.h"



#define IP4_CONNECTED_BIT BIT0
#define IP6_CONNECTED_BIT BIT1
static const char *TAG = "Weather-MAIN";

//#define NUM_RECORDS 100
//static heap_trace_record_t trace_record[NUM_RECORDS];

static EventGroupHandle_t wifi_event_group = NULL;

/* ###############################
 * # EVENTS Section
 */

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
   system_event_info_t *info = &event->event_info;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, CONFIG_ESP_HOSTNAME);
        //esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11B);
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IP4_CONNECTED_BIT);
        ESP_LOGI(TAG, "got ip: %s",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        ESP_ERROR_CHECK(init_publish());
        station_connected(true);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IP6_CONNECTED_BIT);
        ESP_LOGI(TAG, "got ipv6: %s", ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip));
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
       
        ESP_LOGE(TAG, "Disconnect reason: %d", info->disconnected.reason);
        tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, CONFIG_ESP_HOSTNAME);
        esp_wifi_connect();
        
        xEventGroupClearBits(wifi_event_group, IP4_CONNECTED_BIT | IP6_CONNECTED_BIT);
        station_connected(false);
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* ###############################
 * # TASKS Section
 */


#if CONFIG_CONNECTION_STATUS_LED_INVERT
    #define STATUS_LED_ON 1
#else
    #define STATUS_LED_ON 0
#endif
void status_led_task(void* arg){
    gpio_set_level(CONFIG_CONNECTION_STATUS_LED, STATUS_LED_ON);
    while(true){
        if(wifi_event_group != NULL && (xEventGroupGetBits(wifi_event_group) & IP4_CONNECTED_BIT) != 0){
            gpio_set_level(CONFIG_CONNECTION_STATUS_LED, !STATUS_LED_ON);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        gpio_set_level(CONFIG_CONNECTION_STATUS_LED, STATUS_LED_ON);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

/* ###############################
 * # INIT Section
 */

void gpio_init(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT(CONFIG_CONNECTION_STATUS_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}


void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s",CONFIG_ESP_WIFI_SSID);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //ESP_ERROR_CHECK( heap_trace_init_standalone(trace_record, NUM_RECORDS) );

    wifi_init_sta();
    gpio_init();
    xTaskCreate(status_led_task, TASKNAME_STATUS_LED, 2048, NULL, TASKPRIO_STATUS_LED, NULL);

    init_station();
}
