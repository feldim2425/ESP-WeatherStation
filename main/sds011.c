#include "sds011.h"
#include "esp_heap_trace.h"
static const char *TAG = "Weather-SDS011";

static uint8_t calculateChecksum(uint8_t* data, uint8_t len){
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++){
        sum += (uint16_t)data[i];
    }
    return sum & 0xff;
}

static esp_err_t sendMessage(sds011_instance_t* inst){
    uint8_t data[19];
    if(xQueueReceive(inst->queue, data, 0)){
        xEventGroupClearBits(inst->waitingGroup, SDS011_EVENT_RESPOND | SDS011_EVENT_TIMEOUT);
        size_t num = uart_tx_chars(inst->port, (char*)data, 19);
        if(num != 19){
            return ESP_ERR_INVALID_SIZE;
        }
        if(inst->eventHandle != NULL){
            sds011_event_t event = {
                .instance = inst,
                .message = NULL,
                .type = SDS011_EVENT_MESSAGESENT    
            };
            ((sds011_handle_t)inst->eventHandle)(&event);
        }
    }
    return ESP_OK;
}

static esp_err_t queueMessage(sds011_instance_t* inst, uint8_t* data){
    uint8_t txData[19];
    txData[0] = 0xaa;
    txData[1] = 0xb4;
    txData[17] = calculateChecksum(data, 15);
    txData[18] = 0xab;
    memcpy(txData + 2, data, 15);
    if(xQueueSend(inst->queue, txData, 0) == errQUEUE_FULL){
        return ESP_ERR_NO_MEM;
    }

    if(sds011_waitingResponse(inst)){
        ESP_LOGI(TAG, "[QUEUE MSG] Transmit queued");
    }
    else {
        ESP_LOGI(TAG, "[QUEUE MSG] Transmit immediate");
        return sendMessage(inst);
    }
    return ESP_OK;
}

size_t sds011_readMessage(sds011_instance_t* inst, uint8_t* data, size_t len){
    size_t off = 0;
    sds011_message_t message;
    message.valid = SDS011_MSGVALID_NOMSG;
    while(data[off] != 0xaa && off < len){
        off++;
    }

    if(data[off] != 0xaa){
        return off + 1;
    }
    else if((len-off) < 10){
        return off;
    }
    else if(data[off+9] != 0xab){
        return off + 10;
    }
    else if(data[off+8] != calculateChecksum(data+2, 6)){
        message.valid = SDS011_MSGVALID_WRONGSUM;
        if(inst->eventHandle != NULL){
            sds011_event_t event = {
                .instance = inst,
                .message = &message,
                .type = SDS011_EVENT_MESSAGEGOT
            };
            ((sds011_handle_t)inst->eventHandle)(&event);
        }
        return off + 10;
    }
    if(data[off+1] == 0xc0){
        message.valid = SDS011_MSGVALID_OK;
        message.type = SDS011_MESSAGE_DATA;
        message.pm25 = (uint16_t)data[off+2] | ((uint16_t)data[off+3] << 8);
        message.pm10 = (uint16_t)data[off+4] | ((uint16_t)data[off+5] << 8);
        if(inst->queryMode != 0) {
            xEventGroupSetBits(inst->waitingGroup, SDS011_EVENT_RESPOND);
        }
        if(inst->eventHandle != NULL){
            sds011_event_t event = {
                .instance = inst,
                .message = &message,
                .type = SDS011_EVENT_MESSAGEGOT
            };
            ((sds011_handle_t)inst->eventHandle)(&event);
        }
        sendMessage(inst);
    }
    else if(data[off+1] == 0xc5){
        switch(data[off+2]){
            case 2:
                message.type = SDS011_MESSAGE_RESPONSE_REPORT;
                inst->queryMode = data[off+4];
                break;
            case 6:
                message.type = SDS011_MESSAGE_RESPONSE_SLEEP;
                break;
            default:
                message.type = SDS011_MESSAGE_RESPONSE_UNKNOWN;
                break;
        }
        message.value = data[off+4];
        message.valid = SDS011_MSGVALID_OK;
        message.set = (data[off+3] != 0);
        if(message.set){
            xEventGroupSetBits(inst->waitingGroup, SDS011_EVENT_RESPOND);
        }
        if(inst->eventHandle != NULL){
            sds011_event_t event = {
                .instance = inst,
                .message = &message,
                .type = SDS011_EVENT_MESSAGEGOT
            };
            ((sds011_handle_t)inst->eventHandle)(&event);
        }
        sendMessage(inst);
    }
    else {
        message.valid = SDS011_MSGVALID_UNKNOWNTYPE;
        if(inst->eventHandle != NULL){
            sds011_event_t event = {
                .instance = inst,
                .message = &message,
                .type = SDS011_EVENT_MESSAGEGOT
            };
            ((sds011_handle_t)inst->eventHandle)(&event);
        }
    }
    return off + 10;
}

esp_err_t sds011_setSleep(sds011_instance_t* inst, sds011_sleepmode_t mode){
    uint8_t data[] = {6,1,mode, 0,0,0, 0,0,0, 0,0,0, 0,0xff,0xff};
    return queueMessage(inst, data);
}

esp_err_t sds011_setReport(sds011_instance_t* inst, sds011_reportmode_t mode){
    uint8_t data[] = {2,1,mode, 0,0,0, 0,0,0, 0,0,0, 0,0xff,0xff};
    return queueMessage(inst, data);
}

esp_err_t sds011_requestQuery(sds011_instance_t* inst){
    uint8_t data[] = {4,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0xff,0xff};
    return queueMessage(inst, data);
}

uint8_t sds011_waitingResponse(sds011_instance_t* instance){
    return (xEventGroupGetBits(instance->waitingGroup) & ( SDS011_EVENT_RESPOND | SDS011_EVENT_TIMEOUT )) == 0;
}

esp_err_t sds011_timeout(sds011_instance_t* instance){
    if(sds011_waitingResponse(instance)) {
        ESP_LOGI(TAG, "[MSG Timeout] Timeout called");
        xEventGroupSetBits(instance->waitingGroup, SDS011_EVENT_TIMEOUT);
        sendMessage(instance);
    }
    return ESP_OK;
}

esp_err_t sds011_init(sds011_instance_t* inst, uart_port_t port, sds011_handle_t handle,  UBaseType_t quLen){
    inst->port = port;
    inst->waitingGroup = xEventGroupCreate();
    xEventGroupSetBits(inst->waitingGroup, SDS011_EVENT_RESPOND);
    inst->queue = xQueueCreate(quLen, 19);
    inst->queryMode = 0;
    inst->eventHandle = handle;
    return ESP_OK;
}

esp_err_t sds011_delete(sds011_instance_t* instance){
    vEventGroupDelete(instance->waitingGroup);
    vQueueDelete(instance->queue);
    return ESP_OK;
}