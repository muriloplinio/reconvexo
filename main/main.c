/*
WIFI CASA - PITUBA
CONFIG_WIFI_SSID="LIVE TIM_5501_2G"
CONFIG_WIFI_PASSWORD="2px64kk44x"

WIFI CASA PATY
CONFIG_WIFI_SSID="MPS"
CONFIG_WIFI_PASSWORD="032081157"


 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Smart Thermostat v1.2.0
 * main.c
 * 
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file main.c
 * @brief simple MQTT publish, subscribe, and device shadows for use with AWS IoT EduKit reference hardware.
 *
 * This example takes the parameters from the build configuration and establishes a connection to AWS IoT Core over MQTT.
 *
 * Some configuration is required. Visit https://edukit.workshop.aws
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"
#include "I2c_device.h"
#include "core2forAWS.h"


#include "wifi.h"

#include "fft.h"


static const char *TAG = "MAIN";

#define I2CRELAYADDRESS 0x26

#define I2C_NO_REG   ( 1 << 30 )
#define addr 0X26
#define mode_Reg 0X10
#define relay_Reg 0X11

#define HEATING "HEATING"
#define COOLING "COOLING"
#define STANDBY "STANDBY"

#define VOLUME_DETECT_PUMP 10

#define STARTING_ROOMTEMPERATURE 0.0f
#define STARTING_SOUNDLEVEL 0x00
#define STARTING_HVACSTATUS STANDBY
#define STARTING_ROOMOCCUPANCY false

// Number of slices to split the microphone sample into
#define AUDIO_TIME_SLICES 60

#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 2000

#define MAX_TEXTAREA_LENGTH 1024

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

/*
Semaphore for sound levels
*/
SemaphoreHandle_t xMaxNoiseSemaphore;





/**********************
 *  STATIC VARIABLES
 **********************/

static lv_obj_t *out_txtarea;
static lv_obj_t *wifi_label;
static lv_obj_t *sw_valve1;
static lv_obj_t *sw_valve2;
static lv_obj_t *sw_valve3;
static lv_obj_t *sw_valve4;
static lv_obj_t *sw_valve5;
static lv_obj_t *sw_valve6;
static lv_obj_t *led_bomba;

float TempFerm01 = 0.0f;
float TempFerm02 = 0.0f;
float TempFerm03 = 0.0f;
bool Valve1Open = false;
bool Valve2Open = false;
bool Valve3Open = false;
bool Valve4Open = false;
bool Valve5Open = false;
bool Valve6Open = false;
bool PumpOn = false;
float temperature = STARTING_ROOMTEMPERATURE;
uint8_t soundBuffer = STARTING_SOUNDLEVEL;
uint8_t reportedSound = STARTING_SOUNDLEVEL;
char hvacStatus[7] = STARTING_HVACSTATUS;
bool roomOccupancy = STARTING_ROOMOCCUPANCY;
static bool shadowUpdateInProgress;

void ui_textarea_prune(size_t new_text_length){
    const char * current_text = lv_textarea_get_text(out_txtarea);
    size_t current_text_len = strlen(current_text);
    if(current_text_len + new_text_length >= MAX_TEXTAREA_LENGTH){
        for(int i = 0; i < new_text_length; i++){
            lv_textarea_set_cursor_pos(out_txtarea, 0);
            lv_textarea_del_char_forward(out_txtarea);
        }
        lv_textarea_set_cursor_pos(out_txtarea, LV_TEXTAREA_CURSOR_LAST);
    }
}

void ui_textarea_add(char *baseTxt, char *param, size_t paramLen) {
    if( baseTxt != NULL ){
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (param != NULL && paramLen != 0){
            size_t baseTxtLen = strlen(baseTxt);
            ui_textarea_prune(paramLen);
            size_t bufLen = baseTxtLen + paramLen;
            char buf[(int) bufLen];
            sprintf(buf, baseTxt, param);
            lv_textarea_add_text(out_txtarea, buf);
        } 
        else{
            lv_textarea_add_text(out_txtarea, baseTxt); 
        }
        xSemaphoreGive(xGuiSemaphore);
    } 
    else{
        ESP_LOGE(TAG, "Textarea baseTxt is NULL!");
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);

    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
                                const char *pReceivedJsonDocument, void *pContextData) {
    IOT_UNUSED(pThingName);
    IOT_UNUSED(action);
    IOT_UNUSED(pReceivedJsonDocument);
    IOT_UNUSED(pContextData);

    shadowUpdateInProgress = false;

    if(SHADOW_ACK_TIMEOUT == status) {
        ESP_LOGE(TAG, "Update timed out");
    } else if(SHADOW_ACK_REJECTED == status) {
        ESP_LOGE(TAG, "Update rejected");
    } else if(SHADOW_ACK_ACCEPTED == status) {
        ESP_LOGI(TAG, "Update accepted");
    }
}

void hvac_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    char * status = (char *) (pContext->pData);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - hvacStatus state changed to %s", status);
    }

    if(strcmp(status, HEATING) == 0) {
        ESP_LOGI(TAG, "setting side LEDs to red");
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0xFF0000);
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0xFF0000);
        Core2ForAWS_Sk6812_Show();

    } else if(strcmp(status, COOLING) == 0) {
        ESP_LOGI(TAG, "setting side LEDs to blue");
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0x0000FF);
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0x0000FF);
        Core2ForAWS_Sk6812_Show();
    } else {
        ESP_LOGI(TAG, "clearing side LEDs");
        Core2ForAWS_Sk6812_Clear();
        Core2ForAWS_Sk6812_Show();
    }
}

void occupancy_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - roomOccupancy state changed to %d", *(bool *) (pContext->pData));
    }
}


void ui_atualizacontexto() 
{
    if (Valve1Open){lv_btn_set_state(sw_valve1, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve1, LV_BTN_STATE_RELEASED);};
    if (Valve2Open){lv_btn_set_state(sw_valve2, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve2, LV_BTN_STATE_RELEASED);};
    if (Valve3Open){lv_btn_set_state(sw_valve3, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve3, LV_BTN_STATE_RELEASED);};
    if (Valve4Open){lv_btn_set_state(sw_valve4, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve4, LV_BTN_STATE_RELEASED);};
    if (Valve5Open){lv_btn_set_state(sw_valve5, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve5, LV_BTN_STATE_RELEASED);};
    if (Valve6Open){lv_btn_set_state(sw_valve6, LV_BTN_STATE_CHECKED_PRESSED);} else {lv_btn_set_state(sw_valve6, LV_BTN_STATE_RELEASED);};
}

void ComandarValvula(int numValvula, bool Status){
uint8_t my_data;
    switch ( numValvula )
  {
    case 1 :
        Valve1Open = Status;
        printf("Valve1Open\n");
        break;

    case 2 :
        Valve2Open = Status;
        printf("Valve2Open\n");
        break;

    case 3 :
        Valve3Open = Status;
        printf("Valve3Open\n");
        break;

    case 4 :
        Valve4Open = Status;
        printf("Valve4Open\n");
        break;
    case 5 :
        Valve5Open = Status;
        printf("Valve5Open\n");
        break;
    case 6 :
        Valve6Open = Status;
        printf("Valve6Open\n");
        break;
  }
my_data = 0;
if (Valve1Open){my_data = my_data +1;}
if (Valve2Open){my_data = my_data +2;}
if (Valve3Open){my_data = my_data +4;}
if (Valve4Open){my_data = my_data +8;}
if (Valve4Open){my_data = my_data +16;}
if (Valve4Open){my_data = my_data +32;}
if (Valve4Open){my_data = my_data +64;}



I2CDevice_t port_A_peripheral = Core2ForAWS_Port_A_I2C_Begin(I2CRELAYADDRESS, PORT_A_I2C_STANDARD_BAUD);
esp_err_t err = Core2ForAWS_Port_A_I2C_Write(port_A_peripheral, 0x11, &my_data, 1); 

//I2CDevice_t port_A_peripheral = Core2ForAWS_Port_A_I2C_Begin(0x70, PORT_A_I2C_STANDARD_BAUD);
//esp_err_t err = Core2ForAWS_Port_A_I2C_Write(port_A_peripheral, 0x01, 0x01, 1); 

//Core2ForAWS_Port_A_I2C_Close(port_A_peripheral);
//I2CDevice_t port_A_peripheral = Core2ForAWS_Port_A_I2C_Begin(I2CRELAYADDRESS, PORT_A_I2C_STANDARD_BAUD);
//esp_err_t err = Core2ForAWS_Port_A_I2C_Write(port_A_peripheral, 0x11, &my_data, 1); 

Core2ForAWS_Port_A_I2C_Close(port_A_peripheral);
ui_atualizacontexto();

}

void Valve1Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(1, *(bool *) pContext->pData);
        
    }
}

void Valve2Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(2, *(bool *) pContext->pData);

    }
}

void Valve3Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(3, *(bool *) pContext->pData);

    }
}

void Valve4Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(4, *(bool *) pContext->pData);
    }
}

void Valve5Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(5, *(bool *) pContext->pData);

    }
}

void Valve6Open_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
    IOT_UNUSED(pJsonString);
    IOT_UNUSED(JsonStringDataLen);

    if(pContext != NULL) {
        ESP_LOGI(TAG, "Delta - Valve1Open state changed to %d", *(bool *) (pContext->pData));
        ComandarValvula(6, *(bool *) pContext->pData);

    }
}
// helper function for working with audio data
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    long divisor = (in_max - in_min);
    if(divisor == 0){
        return -1; //AVR returns -1, SAM returns 0
    }
    return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

void microphone_task(void *arg) {
    static int8_t i2s_readraw_buff[1024];
    size_t bytesread;
    int16_t *buffptr;
    double data = 0;

    Microphone_Init();
    uint8_t maxSound = 0x00;
    uint8_t currentSound = 0x00;

    for (;;) {
        maxSound = 0x00;
        fft_config_t *real_fft_plan = fft_init(512, FFT_REAL, FFT_FORWARD, NULL, NULL);
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 1024, &bytesread, pdMS_TO_TICKS(100));
        buffptr = (int16_t *)i2s_readraw_buff;
        for (uint16_t count_n = 0; count_n < real_fft_plan->size; count_n++) {
            real_fft_plan->input[count_n] = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -1000, 1000);
        }
        fft_execute(real_fft_plan);

        for (uint16_t count_n = 1; count_n < AUDIO_TIME_SLICES; count_n++) {
            data = sqrt(real_fft_plan->output[2 * count_n] * real_fft_plan->output[2 * count_n] + real_fft_plan->output[2 * count_n + 1] * real_fft_plan->output[2 * count_n + 1]);
            currentSound = map(data, 0, 2000, 0, 256);
            if(currentSound > maxSound) {
                maxSound = currentSound;
            }
        }
        fft_destroy(real_fft_plan);

        // store max of sample in semaphore
        xSemaphoreTake(xMaxNoiseSemaphore, portMAX_DELAY);
        soundBuffer = maxSound;
        xSemaphoreGive(xMaxNoiseSemaphore);
    }
}

void ReadSensors(void *param){
    ESP_LOGI(TAG, "ReadSensors");
    for(;;){
        //ESP_LOGI(TAG, "Temperatura ADC converted to voltage: %d", Core2ForAWS_Port_B_ADC_ReadMilliVolts());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void aws_iot_task(void *param) {
    IoT_Error_t rc = FAILURE;

    char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
    size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);

    jsonStruct_t temperatureHandler;
    temperatureHandler.cb = NULL;
    temperatureHandler.pKey = "temperature";
    temperatureHandler.pData = &temperature;
    temperatureHandler.type = SHADOW_JSON_FLOAT;
    temperatureHandler.dataLength = sizeof(float);

    jsonStruct_t soundHandler;
    soundHandler.cb = NULL;
    soundHandler.pKey = "sound";
    soundHandler.pData = &reportedSound;
    soundHandler.type = SHADOW_JSON_UINT8;
    soundHandler.dataLength = sizeof(uint8_t);

    jsonStruct_t hvacStatusActuator;
    hvacStatusActuator.cb = hvac_Callback;
    hvacStatusActuator.pKey = "hvacStatus";
    hvacStatusActuator.pData = &hvacStatus;
    hvacStatusActuator.type = SHADOW_JSON_STRING;
    hvacStatusActuator.dataLength = strlen(hvacStatus)+1;

    jsonStruct_t roomOccupancyActuator;
    roomOccupancyActuator.cb = occupancy_Callback;
    roomOccupancyActuator.pKey = "roomOccupancy";
    roomOccupancyActuator.pData = &roomOccupancy;
    roomOccupancyActuator.type = SHADOW_JSON_BOOL;
    roomOccupancyActuator.dataLength = sizeof(bool);
    

    jsonStruct_t Valve1OpenActuator;
    Valve1OpenActuator.cb = Valve1Open_Callback;
    Valve1OpenActuator.pKey = "Valve1Open";
    Valve1OpenActuator.pData = &Valve1Open;
    Valve1OpenActuator.type = SHADOW_JSON_BOOL;
    Valve1OpenActuator.dataLength = sizeof(bool);

    jsonStruct_t Valve2OpenActuator;
    Valve2OpenActuator.cb = Valve2Open_Callback;
    Valve2OpenActuator.pKey = "Valve2Open";
    Valve2OpenActuator.pData = &Valve2Open;
    Valve2OpenActuator.type = SHADOW_JSON_BOOL;
    Valve2OpenActuator.dataLength = sizeof(bool);

    jsonStruct_t Valve3OpenActuator;
    Valve3OpenActuator.cb = Valve3Open_Callback;
    Valve3OpenActuator.pKey = "Valve3Open";
    Valve3OpenActuator.pData = &Valve3Open;
    Valve3OpenActuator.type = SHADOW_JSON_BOOL;
    Valve3OpenActuator.dataLength = sizeof(bool);

    jsonStruct_t Valve4OpenActuator;
    Valve4OpenActuator.cb = Valve4Open_Callback;
    Valve4OpenActuator.pKey = "Valve4Open";
    Valve4OpenActuator.pData = &Valve4Open;
    Valve4OpenActuator.type = SHADOW_JSON_BOOL;
    Valve4OpenActuator.dataLength = sizeof(bool);

    jsonStruct_t Valve5OpenActuator;
    Valve5OpenActuator.cb = Valve5Open_Callback;
    Valve5OpenActuator.pKey = "Valve5Open";
    Valve5OpenActuator.pData = &Valve5Open;
    Valve5OpenActuator.type = SHADOW_JSON_BOOL;
    Valve5OpenActuator.dataLength = sizeof(bool);

    jsonStruct_t Valve6OpenActuator;
    Valve6OpenActuator.cb = Valve6Open_Callback;
    Valve6OpenActuator.pKey = "Valve6Open";
    Valve6OpenActuator.pData = &Valve6Open;
    Valve6OpenActuator.type = SHADOW_JSON_BOOL;
    Valve6OpenActuator.dataLength = sizeof(bool);

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    // initialize the mqtt client
    AWS_IoT_Client iotCoreClient;

    ShadowInitParameters_t sp = ShadowInitParametersDefault;
    sp.pHost = HostAddress;
    sp.port = port;
    sp.enableAutoReconnect = false;
    sp.disconnectHandler = disconnect_callback_handler;

    sp.pRootCA = (const char *)aws_root_ca_pem_start;
    sp.pClientCRT = "#";
    sp.pClientKey = "#0";
    
    #define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS){
        ESP_LOGE(TAG, "Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    ui_textarea_add("\n\nDevice client Id:\n>> %s <<\n", client_id, CLIENT_ID_LEN);

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "Shadow Init");

    rc = aws_iot_shadow_init(&iotCoreClient, &sp);
    ESP_LOGE(TAG, "aws_iot_shadow_init: %d", rc);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_init returned error %d, aborting...", rc);
        abort();
    }

    ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
    scp.pMyThingName = client_id;
    scp.pMqttClientId = client_id;
    scp.mqttClientIdLen = CLIENT_ID_LEN;

    

    ESP_LOGI(TAG, "Shadow Connect");
    rc = aws_iot_shadow_connect(&iotCoreClient, &scp);
    ESP_LOGE(TAG, "aws_iot_shadow_connect: %d", rc);
 
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_shadow_connect returned error %d, aborting...", rc);
        abort();
    }
    ui_textarea_add("Connected to AWS IoT Device Shadow service", NULL, 0);

    xTaskCreatePinnedToCore(&microphone_task, "microphone_task", 4096, NULL, 1, NULL, 1);

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    
    rc = aws_iot_shadow_set_autoreconnect_status(&iotCoreClient, true);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for roomOccupancy
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &roomOccupancyActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for hvacStatus
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &hvacStatusActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve1Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve1OpenActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve2Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve2OpenActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve3Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve3OpenActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve4Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve4OpenActuator);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve5Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve5OpenActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
    // register delta callback for Valve6Open
    rc = aws_iot_shadow_register_delta(&iotCoreClient, &Valve6OpenActuator);
    ESP_LOGE(TAG, "aws_iot_shadow_register_delta: %d", rc);
    if(SUCCESS != rc) {ESP_LOGE(TAG, "Shadow Register Delta Error");}
  
  
    ESP_LOGI(TAG, "loop and publish changes");
    // loop and publish changes
    while(NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc) {


		//	erro em MQTT_RX_BUFFER_TOO_SHORT_ERROR = -32, tentei aumentar o buffer, mas entrou em loop de bug
            

        rc = aws_iot_shadow_yield(&iotCoreClient, 200);
        ESP_LOGE(TAG, "aws_iot_shadow_yield1: %d", rc);
        if(NETWORK_ATTEMPTING_RECONNECT == rc || shadowUpdateInProgress) {
            rc = aws_iot_shadow_yield(&iotCoreClient, 1000);
            ESP_LOGE(TAG, "aws_iot_shadow_yield2: %d", rc);
            // If the client is attempting to reconnect, or already waiting on a shadow update,
            // we will skip the rest of the loop.
            continue;
        }

        // START get sensor readings
        // sample temperature, convert to fahrenheit
        MPU6886_GetTempData(&temperature);
        temperature = (temperature * 1.8)  + 32 - 50;

        // sample from soundBuffer (latest reading from microphone)
        xSemaphoreTake(xMaxNoiseSemaphore, portMAX_DELAY);
        reportedSound = soundBuffer;
        xSemaphoreGive(xMaxNoiseSemaphore);

        PumpOn = reportedSound > VOLUME_DETECT_PUMP;
        if (PumpOn){
            lv_led_on(led_bomba);
        } else {
            lv_led_off(led_bomba);
        }
        

        // END get sensor readings

        ESP_LOGI(TAG, "*****************************************************************************************");
        ESP_LOGI(TAG, "On Device: roomOccupancy %s", roomOccupancy ? "true" : "false");
        ESP_LOGI(TAG, "On Device: hvacStatus %s", hvacStatus);
        ESP_LOGI(TAG, "On Device: temperature %f", temperature);
        ESP_LOGI(TAG, "On Device: sound %d", reportedSound);
        ESP_LOGI(TAG, "On Device: vave1open %d", Valve1Open);
        ESP_LOGI(TAG, "On Device: vave2open %d", Valve2Open);
        ESP_LOGI(TAG, "On Device: vave3open %d", Valve3Open);
        ESP_LOGI(TAG, "On Device: vave4open %d", Valve4Open);
        ESP_LOGI(TAG, "On Device: vave4open %d", Valve5Open);
        ESP_LOGI(TAG, "On Device: vave4open %d", Valve6Open);
        ESP_LOGI(TAG, "On Device: PumpOn %d", PumpOn);

        rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
        ESP_LOGE(TAG, "aws_iot_shadow_init_json_document: %d", rc);
        if(SUCCESS == rc) {
            rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 10, &temperatureHandler,
                                             &soundHandler, &roomOccupancyActuator, &hvacStatusActuator, &Valve1OpenActuator, 
                                             &Valve2OpenActuator, &Valve3OpenActuator, &Valve4OpenActuator, &Valve5OpenActuator, &Valve6OpenActuator);
            ESP_LOGE(TAG, "aws_iot_shadow_add_reported: %d", rc);
            if(SUCCESS == rc) {
                rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
                ESP_LOGE(TAG, "aws_iot_finalize_json_document: %d", rc);
                if(SUCCESS == rc) {
                    ESP_LOGI(TAG, "Update Shadow: %s", JsonDocumentBuffer);
                    rc = aws_iot_shadow_update(&iotCoreClient, client_id, JsonDocumentBuffer,
                                                ShadowUpdateStatusCallback, NULL, 4, true);
                    shadowUpdateInProgress = true;
                }
            }
        }
        ESP_LOGI(TAG, "*****************************************************************************************");
        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "An error occurred in the loop %d", rc);
    }

    ESP_LOGI(TAG, "Disconnecting");
    rc = aws_iot_shadow_disconnect(&iotCoreClient);
    ESP_LOGI(TAG, "Disconnected");

    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Disconnect error %d", rc);
    }

    vTaskDelete(NULL);
}

void ui_wifi_label_update(bool state){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    if (state == false) {
        lv_label_set_text(wifi_label, LV_SYMBOL_WIFI);
    } 
    else{
        char buffer[25];
        sprintf (buffer, "#0000ff %s #", LV_SYMBOL_WIFI);
        lv_label_set_text(wifi_label, buffer);
    }
    xSemaphoreGive(xGuiSemaphore);
}

void sw_valve1_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve1Open = !Valve1Open;
         ComandarValvula(1,Valve1Open);
    }
}

void sw_valve2_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve2Open = !Valve2Open;
         ComandarValvula(2,Valve2Open);
         
    }
}

void sw_valve3_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve3Open = !Valve3Open;
         ComandarValvula(3,Valve3Open);
    }     
}
void sw_valve4_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve4Open = !Valve4Open;
         ComandarValvula(4,Valve4Open);
    }
}

void sw_valve5_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve5Open = !Valve5Open;
         ComandarValvula(5,Valve5Open);
    }
}

void sw_valve6_cb(lv_obj_t * obj, lv_event_t event)
{
  if(event == LV_EVENT_CLICKED) {
         Valve6Open = !Valve6Open;
         ComandarValvula(6,Valve5Open);
    }
}

static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        printf("Button: %s\n", lv_msgbox_get_active_btn_text(obj));
    }
}

void tab_log_create(lv_obj_t *parent)
{
    out_txtarea = lv_textarea_create(parent, NULL);
    lv_obj_set_size(out_txtarea, 300, 180);//300
    lv_obj_align(out_txtarea, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -12);
    lv_textarea_set_max_length(out_txtarea, MAX_TEXTAREA_LENGTH);
    lv_textarea_set_text_sel(out_txtarea, false);
    lv_textarea_set_cursor_hidden(out_txtarea, true);
    lv_textarea_set_text(out_txtarea, "Starting Cloud Connected Blinky\n");

}


void tab_cantina_create(lv_obj_t *parent)
{


}



void tab_campo_create(lv_obj_t *parent)
{
    int x = 10; // posição x dos botões
    int y = 10; //posição y dos botões
    int s = 14; //espaçamento entre botões
    int h = 30; //altura do botão
    int c = 135; //comprimento dos botões
    sw_valve1 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve1, x, y);
	lv_obj_set_size(sw_valve1, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve1, sw_valve1_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve1, true);
    lv_btn_toggle(sw_valve1);
    lv_btn_set_fit2(sw_valve1, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label1;
    label1 = lv_label_create(sw_valve1, NULL);
    lv_label_set_text(label1, "RCX 1: 01:15");

     

    sw_valve2 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve2, x, y);
	lv_obj_set_size(sw_valve2, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve2, sw_valve2_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve2, true);
    lv_btn_toggle(sw_valve2);
    lv_btn_set_fit2(sw_valve2, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label3;
    label3 = lv_label_create(sw_valve2, NULL);
    lv_label_set_text(label3, "RCX 2: 00:00");

    sw_valve3 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve3, x, y);
	lv_obj_set_size(sw_valve3, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve3, sw_valve3_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve3, true);
    lv_btn_toggle(sw_valve3);
    lv_btn_set_fit2(sw_valve3, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label4;
    label4 = lv_label_create(sw_valve3, NULL);
    lv_label_set_text(label4, "RCX 3: 00:00");

    sw_valve4 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve4, x, y);
	lv_obj_set_size(sw_valve4, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve4, sw_valve4_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve4, true);
    lv_btn_toggle(sw_valve4);
    lv_btn_set_fit2(sw_valve4, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label6;
    label6 = lv_label_create(sw_valve4, NULL);
    lv_label_set_text(label6, "RCX 4:  00:00");

    x = 160; // posição x dos botões
    y = 10; //posição y dos botões

    sw_valve5 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve5, x, y);
	lv_obj_set_size(sw_valve5, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve5, sw_valve5_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve5, true);
    lv_btn_toggle(sw_valve5);
    lv_btn_set_fit2(sw_valve5, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label7;
    label7 = lv_label_create(sw_valve5, NULL);
    lv_label_set_text(label7, "RCX 5:  00:00");

    sw_valve6 = lv_btn_create(parent, NULL);      /*Add a button to the current screen*/
    lv_obj_set_pos(sw_valve6, x, y);
	lv_obj_set_size(sw_valve6, c, h);
    y = y + h + s;
    lv_obj_set_event_cb(sw_valve6, sw_valve6_cb); /*Assign a callback to the button*/
    lv_btn_set_checkable(sw_valve6, true);
    lv_btn_toggle(sw_valve6);
    lv_btn_set_fit2(sw_valve6, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_t * label8;
    label8 = lv_label_create(sw_valve6, NULL);
    lv_label_set_text(label8, "RCX 6:  00:00");

    ui_atualizacontexto();
    y = y + 5;
    x = x + 45;

    led_bomba  = lv_led_create(parent, NULL);
    //lv_obj_align(led_bomba, NULL, LV_ALIGN_CENTER, 40, 0);
    lv_obj_set_pos(led_bomba,  x, y);
    lv_led_off(led_bomba);
    lv_obj_t * label5;
    label5 = lv_label_create(parent, NULL);
    //lv_obj_align(label5, NULL, LV_ALIGN_CENTER, 40, -40);
    y = y + 55;
    x = x - 35;
    lv_obj_set_pos(label5,  x, y);
    lv_label_set_text(label5, "Bomba:  00:00"); 
}



void ui_init() {
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_obj_t *tabview;
    tabview = lv_tabview_create(lv_scr_act(), NULL);
    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Cantina");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Campo");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Log");
    /*Add content to the tabs*/
    tab_cantina_create(tab1);
    tab_campo_create(tab2);
    tab_log_create(tab3);

    wifi_label = lv_label_create(tabview, NULL);
    lv_obj_align(wifi_label,NULL,LV_ALIGN_IN_TOP_RIGHT, 0, 6);
    lv_label_set_text(wifi_label, LV_SYMBOL_WIFI);
    lv_label_set_recolor(wifi_label, true);


    
    //lv_ex_btn_1();
   /*static const char * btns[] ={"Sim", "Cancelar", ""};

    lv_obj_t * mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
    lv_msgbox_set_text(mbox1, "Tem certeza?");
    lv_msgbox_add_btns(mbox1, btns);
    lv_obj_set_width(mbox1, 200);
    lv_obj_set_event_cb(mbox1, event_handler);
    lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0); */ /*Align to the corner*/
    
    xSemaphoreGive(xGuiSemaphore);
}

void app_main()
{   
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(80);
    Core2ForAWS_LED_Enable(1);

    xMaxNoiseSemaphore = xSemaphoreCreateMutex();
    
    ui_init();
    initialise_wifi();

    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 4096*2, NULL, 5, NULL, 1);
    Core2ForAWS_Port_PinMode(PORT_B_ADC_PIN, ADC);
    xTaskCreatePinnedToCore(&ReadSensors, "readsensor", 1024*4, NULL, 6, NULL, 1);
}



