#include "station.h"

static const char *TAG = "Weather-STATION";
static sds011_instance_t sensInstance;
static bme280_instance_t bmeInstance;

static TimerHandle_t sds_timout_tmr;
static TimerHandle_t measurement_tmr;
static TimerHandle_t pm_work_tmr;
static TimerHandle_t rain_period_tmr;
static TimerHandle_t rain_debounce_tmr;

static QueueHandle_t sds011_uart_queue;

static uint8_t measurement_state;
static uint8_t measurement_pmCountdown;
static EventGroupHandle_t measurement_group;
static weather_data_t weather_data;

static uint16_t rain_current;
static uint16_t rain_last;
static uint8_t rain_debounce = 1;

/*
 * FreeRTOS Task Callback
 * Handles Events via the sds011_uart_queue from the UART Driver
 */
static void sds011_uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(SDS011_RX_BUFFERLEN);
    size_t buflen = 0;
    size_t readable = 0;

    bzero(dtmp, SDS011_RX_BUFFERLEN);
    while (true)
    {
        // Waiting for UART event.
        if (xQueueReceive(sds011_uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {

            switch (event.type)
            {
            case UART_DATA:
                readable = (event.size > (SDS011_RX_BUFFERLEN - buflen)) ? SDS011_RX_BUFFERLEN - buflen : event.size;
                while (readable > 0)
                {
                    readable = (event.size > (SDS011_RX_BUFFERLEN - buflen)) ? SDS011_RX_BUFFERLEN - buflen : event.size;
                    uart_read_bytes(CONFIG_SDS011_UART, dtmp + buflen, readable, portMAX_DELAY);
                    buflen += readable;
                    buflen -= sds011_readMessage(&sensInstance, dtmp, buflen);
                    readable = (event.size > (SDS011_RX_BUFFERLEN - buflen)) ? SDS011_RX_BUFFERLEN - buflen : event.size;
                }
                break;

            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                uart_flush_input(CONFIG_SDS011_UART);
                xQueueReset(sds011_uart_queue);
                break;

            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                uart_flush_input(CONFIG_SDS011_UART);
                xQueueReset(sds011_uart_queue);
                break;

            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "uart parity error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "uart frame error");
                break;
            default:
                ESP_LOGW(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

/*
 * Callback for sds011 events generated when a message got sent or recieved.
 * Called by sds011.h
 */
static void sds011_handle_event(sds011_event_t *event)
{
    if (event->type == SDS011_EVENT_MESSAGEGOT)
    {
        if (event->message->valid == SDS011_MSGVALID_OK &&
            ((event->message->type == SDS011_MESSAGE_DATA && event->instance->queue) ||
             event->message->type != SDS011_MESSAGE_DATA))
        {
            xTimerStop(sds_timout_tmr, 0);
        }

        if (event->message->valid == SDS011_MSGVALID_OK)
        {
            ESP_LOGI(TAG, "[SDS011] Valid: Type=%d, Value=%d, PM25=%d, PM10=%d", event->message->type, event->message->value, event->message->pm25, event->message->pm10);
            if (event->message->type == SDS011_MESSAGE_DATA)
            {
                weather_data.pm10 = (float)(event->message->pm10) / 10.0;
                weather_data.pm25 = (float)(event->message->pm25) / 10.0;
                weather_data.data_bits |= WEATHER_BIT_PM10 | WEATHER_BIT_PM25;
                xEventGroupSetBits(measurement_group, STATION_EVENT_DAT_PM);
            }
        }
        else
        {
            ESP_LOGI(TAG, "[SDS011] Inalid: Reason=%d", event->message->valid);
        }
    }
    else if (event->type == SDS011_EVENT_MESSAGESENT)
    {
        xTimerStart(sds_timout_tmr, 0);
    }
}

/*
 * Helper function to end a measurment
 * Used by the measure_task.
 */
static void prvEndMeasurement(){
    measurement_state &= ~STATION_STATE_RUNNING & ~STATION_STATE_WITHPM;
    xTimerStop(pm_work_tmr, 0);
    ESP_LOGI(TAG, "[MEASURE] Stop");
    weather_data.data_bits = 0;
    xTimerChangePeriod(measurement_tmr, STATION_CONFIG_PERIOD, 0);
    if((measurement_state & STATION_STATE_READY) != 0){
        xTimerStart(measurement_tmr, 0);
    }
}

/*
 * Update the measurment state and timers.
 * Currently used during initialization and connect, disconnect, reconnect from mqtt.
 */
static void station_updateReady(){
    uint8_t ready = (measurement_state & (STATION_STATE_INITIALIZED | STATION_STATE_CONNECTED)) ==
    (STATION_STATE_INITIALIZED | STATION_STATE_CONNECTED);

    // The newly determined ready-state is not different from the current state no change required
    if((ready != 0) == ((measurement_state & STATION_STATE_READY) != 0)){
        return;
    }

    // Changed to ready therefore the state gets set
    // If the measurment is not running the weather_data gets reset, the timer period is updated and the timer gets started
    if(ready){
        measurement_state |= STATION_STATE_READY;
        if((measurement_state & STATION_STATE_RUNNING) == 0){
            weather_data.data_bits = 0;
            xTimerChangePeriod(measurement_tmr, STATION_CONFIG_PERIOD, 0);
            xTimerStart(measurement_tmr, 0);
        }
    }
    // Changed to ready therefore the state gets set
    // If the measurment is not running the timer gets stoped
    // The running check is required because stopping it during a measurment stops the timeout and not the measurment
    else {
        measurement_state &= ~STATION_STATE_READY;
        if((measurement_state & STATION_STATE_RUNNING) == 0){
            xTimerStop(measurement_tmr, 0);
        }
    }
}

/*
 * Task that starts, initialized, stops and sends the data for the measurments
 */
static void measure_task(void *arg)
{
    EventBits_t bits;

    ESP_ERROR_CHECK(sds011_setReport(&sensInstance, SDS011_REPORTMODE_QUERY));
    ESP_ERROR_CHECK(sds011_setSleep(&sensInstance, SDS011_SLEEPMODE_SLEEP));

    while(true){
        bits = xEventGroupWaitBits (measurement_group,
                                    STATION_EVENT_TIMER | STATION_EVENT_DAT_PM | STATION_EVENT_DAT_PTH,
                                    STATION_EVENT_TIMER | STATION_EVENT_DAT_PM | STATION_EVENT_DAT_PTH,
                                    false, portMAX_DELAY);
           
        // Station Timer is used to start and timeout a measurment
        if(bits & STATION_EVENT_TIMER){
            // The measurement is running therefore the timer is interpreted as a timeout
            if(measurement_state & STATION_STATE_RUNNING) {
                ESP_LOGI(TAG, "[MEASURE] Timeout with data: 0x%X", weather_data.data_bits);
                if(weather_data.data_bits != 0){
                    push_weather_data(weather_data);
                }
                prvEndMeasurement();
            }
            // Measurement is not running. New measurment gets scheduled if the station is ready
            else if((measurement_state & STATION_STATE_READY) != 0 ) {
                measurement_state |= STATION_STATE_RUNNING;

                // Increment the PM Countdown and schedule a PM measurement if the countdown reaches 0 
                // 2x if's instead of if-else is intentional otherwise the cycle would be every 11'th measurment
                if(measurement_pmCountdown > 0){
                    measurement_pmCountdown--;
                }
                if(measurement_pmCountdown == 0 || (measurement_state & STATION_STATE_WITHPM)){
                    measurement_state |= STATION_STATE_WITHPM;
                    measurement_pmCountdown = STATION_CONFIG_PMINTERVAL;
                }
                
                ESP_LOGI(TAG, "[MEASURE] Start / State = 0x%X / PM Countdown = %d",measurement_state, measurement_pmCountdown);
                xEventGroupSetBits(measurement_group, STATION_EVENT_START_PTH);

                // If the PM measurment is scheduled the SDS011 runs before a measurment is taken
                // Therefore it gets set into the Workmode and Querymode additionally the timer for the measurment gets started
                if(measurement_state & STATION_STATE_WITHPM){
                    ESP_ERROR_CHECK(sds011_setSleep(&sensInstance, SDS011_SLEEPMODE_WORK));
                    ESP_ERROR_CHECK(sds011_setReport(&sensInstance, SDS011_REPORTMODE_QUERY));
                    xTimerStart(pm_work_tmr, 0);
                }

                // Modify the period of the measurment timer for the timeout and restart the timer
                // Info: The period is modified again in prvEndMeasurement for the next measurement 
                xTimerChangePeriod(measurement_tmr, STATION_CONFIG_TIMEOUT, 0);
                xTimerStart(measurement_tmr, 0);
                
                // In case the rain gague collected some data the data gets updated immediatly
                // Depending on the settings this will always be the case or only at the beginning (first measurment) but it should never change back
                if(measurement_state & STATION_STATE_RAINMEASURED){
                    weather_data.rain = (float)rain_last / (((float)CONFIG_RGAGUE_CALIB_OPENING / 100000.0) * (float)CONFIG_RGAGUE_CALIB_COUNTS);
                    weather_data.data_bits |= WEATHER_BIT_RAIN;
                    ESP_LOGI(TAG, "Rain amount: %d", (uint16_t)(weather_data.rain * 10));
                }
            }
            // This else should never execute. If would be if the measurment timer is running but the ready state is reset and the station is not measuring
            // Since the state is only handled by station_updateReady which also starts and stops the timer if necessary this should not happen.
            else {
                ESP_LOGI(TAG, "[MEASURE] Start canceled (Not Ready) / State: 0x%X",measurement_state);
            }
        }
        // Timeout not reached (STATION_EVENT_TIMER not set and measure_state is Running) but there is new data
        else if((bits & (STATION_EVENT_DAT_PM | STATION_EVENT_DAT_PTH)) && (measurement_state & STATION_STATE_RUNNING)){
            uint8_t mask = WEATHER_BIT_PRESSUE | WEATHER_BIT_TEMPERATURE;
            // Add PM measurement to the mask if the measurement was scheduled in the code above
            if(measurement_state & STATION_STATE_WITHPM){
                mask |= WEATHER_BIT_PM10 | WEATHER_BIT_PM25;
            }
            // Check if weather_data contains all the data required for the scheduled measurements and end if thats the case
            if((weather_data.data_bits & mask) == mask){
                push_weather_data(weather_data);
                prvEndMeasurement();
            }
        }
    }
    vTaskDelete(NULL);
}

/*
 * This timer is used to measure the 30 seconds required by the dust sensor the settle on the values
 */
static void pmWork_callback(TimerHandle_t xTimer){
    if(measurement_state & STATION_STATE_RUNNING){
        ESP_ERROR_CHECK(sds011_requestQuery(&sensInstance));
        ESP_ERROR_CHECK(sds011_setSleep(&sensInstance, SDS011_SLEEPMODE_SLEEP));
    }
}

/*
 * This Timer callback is used to start a measurment and also to force a stop if one of the sensors isn't responding
 * Managing is done in the measure_task and station_updateReady
 */
static void measure_callback(TimerHandle_t xTimer)
{
    xEventGroupSetBits(measurement_group, STATION_EVENT_TIMER);
}

/*
 * Started when a message is sent to the sds011 to call a timeout if the time has passed and no response was received
 */
static void sds011_timeout_callback(TimerHandle_t xTimer)
{
    sds011_timeout(&sensInstance);
}

/*
 * Move the current counter to the last counter (used for calculation) and reset the current counter.
 */
static void rain_period_callback(TimerHandle_t xTimer){
    rain_last = rain_current;
    rain_current = 0;
    measurement_state |= STATION_STATE_RAINMEASURED;
    ESP_LOGI(TAG, "Rain Period End: %d", rain_last);
}

/*
 * Timer used to debounce the rain gague
 */
static void rain_debounce_callback(TimerHandle_t xTimer){
    rain_debounce = 0;
    ESP_LOGI(TAG, "Rain Count: %d", rain_current);
}

static void IRAM_ATTR rgague_interrupt(void* arg){
    if(rain_debounce == 0){
        rain_current++;
        rain_debounce = 1;
        xTimerStartFromISR(rain_debounce_tmr,0);
    }
}


/*
 * This tasks handles the initialization and measurments for the BME280
 * Communicated with the measurment tas via measurement_group
 */
static void bme280_i2c_task(void *arg)
{
    EventBits_t bits;

    vTaskDelay(pdMS_TO_TICKS(100));
    while(bme280_checkStatus(&bmeInstance) & BME280_BIT_IM_UPDATE) vPortYield();
    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_commitSetup(&bmeInstance));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_readCalibration(&bmeInstance));
    while(bme280_checkStatus(&bmeInstance) & BME280_BIT_MEASURING) vPortYield();

    while(true){
        bits = xEventGroupWaitBits (measurement_group,
                                    STATION_EVENT_START_PTH,
                                    STATION_EVENT_START_PTH,
                                    false, portMAX_DELAY);

        if(bits & STATION_EVENT_START_PTH){
            ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_setMode(&bmeInstance, BME280_MODE_FORCE));
            while(bme280_checkStatus(&bmeInstance) & BME280_BIT_MEASURING) vPortYield();
            int32_t temp = bme280_getTemperature(&bmeInstance);
            uint32_t press = bme280_getPressure(&bmeInstance, 0);
            uint32_t humid = bme280_getHumidity(&bmeInstance, 0);
            weather_data.temperature = BME280_CALCT_dC(temp);
            weather_data.pressue = BME280_CALCP_hPa(press);
            weather_data.humidity = BME280_CALCH_pcRH(humid);
            ESP_LOGI(TAG, "Temperature: %d / Pressure: %d / Humidity: %d", temp, press, humid);
            weather_data.data_bits |= WEATHER_BIT_PRESSUE | WEATHER_BIT_TEMPERATURE | WEATHER_BIT_HUMIDITY;
            xEventGroupSetBits(measurement_group, STATION_EVENT_DAT_PTH);
        }
    }
    
    vTaskDelete(NULL);
}

/*
 * Initialize the sds011 UART bus but DOES NOT sent any data
 * Init messages are sent in the measument task at the beginning
 */
static void init_sds011()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(CONFIG_SDS011_UART, &uart_config));
    #if CONFIG_SDS011_CHANGE_PINS
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_SDS011_UART, CONFIG_SDS011_TX, CONFIG_SDS011_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    #endif
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_SDS011_UART, SDSUART_BUF_SIZE * 2, SDSUART_BUF_SIZE * 2, 100, &sds011_uart_queue, 0));
    ESP_LOGI(TAG, "[UART Initialized] on %d", CONFIG_SDS011_UART);
    ESP_ERROR_CHECK(sds011_init(&sensInstance, CONFIG_SDS011_UART, sds011_handle_event, 10));
}

/*
 * Initialize the bme I2C bus but DOES NOT sent any data
 * Init messages are sent in the bme280_i2c_task at the beginning
 */
static void init_bme(){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_I2C_SDA;
    conf.scl_io_num = CONFIG_I2C_SCL;
    conf.master.clk_speed = CONFIG_I2C_CLOCK;
#if CONFIG_I2C_PULLUPS
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
#else
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
#endif
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0,0,0));
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    bme280_init(&bmeInstance, I2C_NUM_0);
    bme280_config(&bmeInstance, BME280_TSTB_1000ms, BME280_FILTER_OFF);
    bme280_ctrlMeas(&bmeInstance, BME280_MODE_SLEEP, BME280_OSRS_X1, BME280_OSRS_X1);
    bme280_ctrlHum(&bmeInstance, BME280_OSRS_X1);
}

/*
 * Initialize the rain gague pin and interrupts
 */
static void init_rgague(){
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BIT(CONFIG_RGAGUE_PIN);
    
    #if CONFIG_RGAGUE_PINMODE_PULL_UP
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    #elif CONFIG_RGAGUE_PINMODE_PULL_DOWN
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    #else
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    #endif
    
     #if CONFIG_RGAGUE_EDGECOUNT_RISING
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    #elif CONFIG_RGAGUE_EDGECOUNT_FALLING
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    #elif RGAGUE_EDGECOUNT_BOTH
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
    #else
    #error "No Rain Gague countedge configuration"
    #endif
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_RGAGUE_PIN, rgague_interrupt, (void*) CONFIG_RGAGUE_PIN);
    gpio_config(&io_conf);
}
/*
 * Initialize all tasks and timers used in this file
 */
static void init_station_tasks()
{
    sds_timout_tmr = xTimerCreate(TMRNAME_SDS011_TIMEOUT, pdMS_TO_TICKS(1000), false, TMRID_SDS011_TIMEOUT, sds011_timeout_callback);
    measurement_tmr = xTimerCreate(TMRNAME_MEASURE_TIMEOUT, STATION_CONFIG_PERIOD, false, TMRID_MEASURE_TIMEOUT, measure_callback);
    pm_work_tmr = xTimerCreate(TMRNAME_PM_WORKING, pdMS_TO_TICKS(30000), false, TMRID_PM_WORKING, pmWork_callback);
    rain_debounce_tmr = xTimerCreate(TMRNAME_RAIN_DEBOUNCE, pdMS_TO_TICKS(CONFIG_RGAGUE_DEBOUNCE), false, TMRID_RAIN_DEBOUNCE, rain_debounce_callback);
    rain_period_tmr = xTimerCreate(TMRNAME_RAIN_MEASURE, pdMS_TO_TICKS(CONFIG_RGAGUE_PERIOD * 1000), true, TMRID_RAIN_MEASURE, rain_period_callback);
    
    xTaskCreate(sds011_uart_event_task, TASKNAME_SDS011_UART, 2048, NULL, TASKPRIO_SDS011_UART, NULL);
    xTaskCreate(measure_task, TASKNAME_STATION_EVENT, 2048, NULL, TASKPRIO_STATION_EVENT, NULL);
    xTaskCreate(bme280_i2c_task, TASKNAME_BME_CTRL, 2048, NULL, TASKPRIO_BME_CTRL, NULL);
}

/*
 * Update the external ready state (Connected).
 * Used to stop and hold mesurments until they are usefull (can be sent to a server)
 * station_connected() is called by the sources for mqtt client to update the state.
 */
void station_connected(uint8_t state){
    if(state){
        measurement_state |= STATION_STATE_CONNECTED;
    }
    else {
        measurement_state &= ~STATION_STATE_CONNECTED;
    }
    
    station_updateReady();
}

/*
 * General initialization of this "library"
 */
void init_station()
{
    init_sds011();

    weather_data.data_bits = 0;
    measurement_pmCountdown = 0;

    measurement_group = xEventGroupCreate();
    init_station_tasks();
    init_bme();
    init_rgague();

    measurement_state |= STATION_STATE_INITIALIZED;
    station_updateReady();
    
    xTimerStart(rain_period_tmr,0);
    xTimerStart(rain_debounce_tmr,0);
}
