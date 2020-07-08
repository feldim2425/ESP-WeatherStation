#ifndef CONSTANTS_H
#define CONSTANTS_H

// #######################
// #    TASK NAMES
// #
#define TASKNAME_STATUS_LED "status_led_task"
#define TASKNAME_SDS011_UART "sds011_uart_task"
#define TASKNAME_STATION_EVENT "station_event_task"
#define TASKNAME_BME_CTRL "bme_ctrl_task"

// #######################
// #    TASK PRIORITY
// #
#define TASKPRIO_STATUS_LED 5
#define TASKPRIO_SDS011_UART 10
#define TASKPRIO_STATION_EVENT 7
#define TASKPRIO_BME_CTRL 8

// #######################
// #    TIMER NAMES
// #
#define TMRNAME_SDS011_TIMEOUT "sds011_timeout_tmr"
#define TMRNAME_MEASURE_TIMEOUT "measure_tmr"
#define TMRNAME_PM_WORKING "pm_working_tmr"
#define TMRNAME_RAIN_DEBOUNCE "rain_deb_tmr"
#define TMRNAME_RAIN_MEASURE "rain_meas_tmr"

// #######################
// #    TIMER IDS
// #
#define TMR_ID(id) (void*)id
#define TMRID_SDS011_TIMEOUT TMR_ID(0)
#define TMRID_MEASURE_TIMEOUT TMR_ID(1)
#define TMRID_PM_WORKING TMR_ID(2)
#define TMRID_RAIN_DEBOUNCE TMR_ID(3)
#define TMRID_RAIN_MEASURE TMR_ID(4)

// #######################
// #    MISC
// #


#endif