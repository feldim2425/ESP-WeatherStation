#ifndef SDS011_H
#define SDS011_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"

#define SDS011_RX_BUFFERLEN 10

#define SDS011_EVENT_UPDATE 1
#define SDS011_EVENT_RESPOND 2
#define SDS011_EVENT_TIMEOUT 4

#define SDS011_GET_EVENTGROUP(inst) (inst.waitingGroup)
#define SDS011_WAIT_FOR_UPDATE(inst, time) xEventGroupWaitBits(SDS011_GET_EVENTGROUP(inst), SDS011_EVENT_UPDATE, SDS011_EVENT_UPDATE, 0, time)

typedef enum {
    SDS011_SLEEPMODE_SLEEP = 0,
    SDS011_SLEEPMODE_WORK = 1
} sds011_sleepmode_t;

typedef enum {
    SDS011_REPORTMODE_ACTIVE = 0,
    SDS011_REPORTMODE_QUERY = 1
} sds011_reportmode_t;

typedef enum {
    SDS011_MESSAGE_DATA = 0,
    SDS011_MESSAGE_RESPONSE_SLEEP = 1,
    SDS011_MESSAGE_RESPONSE_REPORT = 2,
    SDS011_MESSAGE_RESPONSE_UNKNOWN = 255,
} sds011_messagetype_t;

typedef enum {
    SDS011_MSGVALID_OK = 0,
    SDS011_MSGVALID_WRONGSUM = 1,
    SDS011_MSGVALID_NOMSG = 2,
    SDS011_MSGVALID_UNKNOWNTYPE = 3
} sds011_msgvalidation_t;

typedef enum {
    SDS011_EVENT_MESSAGEGOT = 0,
    SDS011_EVENT_MESSAGESENT = 1,
} sds011_eventtype_t;

typedef struct {
    sds011_msgvalidation_t valid;
    sds011_messagetype_t type;
    uint8_t set; // Valid for types SDS011_MESSAGE_RESPONSE_*
    uint8_t value; // Valid for types SDS011_MESSAGE_RESPONSE_*
    uint16_t pm10; // Valid for types SDS011_MESSAGE_DATA*
    uint16_t pm25; // Valid for types SDS011_MESSAGE_DATA*
} sds011_message_t;

typedef struct {
    uart_port_t port;
    QueueHandle_t queue;
    EventGroupHandle_t waitingGroup;
    TimerHandle_t timeout;
    void* eventHandle;
    uint8_t queryMode;
} sds011_instance_t;

typedef struct {
    sds011_instance_t* instance;
    sds011_message_t* message;
    sds011_eventtype_t type;
} sds011_event_t;

typedef void (*sds011_handle_t)(sds011_event_t* eventInfo);

size_t sds011_readMessage(sds011_instance_t* inst, uint8_t* data, size_t len);

esp_err_t sds011_setSleep(sds011_instance_t* inst, sds011_sleepmode_t mode);

esp_err_t sds011_setReport(sds011_instance_t* inst, sds011_reportmode_t mode);

esp_err_t sds011_requestQuery(sds011_instance_t* inst);

uint8_t sds011_waitingResponse(sds011_instance_t* instance);

esp_err_t sds011_timeout(sds011_instance_t* instance);

esp_err_t sds011_init(sds011_instance_t* inst, uart_port_t port, sds011_handle_t handle, UBaseType_t quLen);

esp_err_t sds011_delete(sds011_instance_t* instance);

#endif