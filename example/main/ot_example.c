#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <opentherm.h>
#include <esp_log.h>
#include <esp_err.h>

#define GPIO_OT_IN 14
#define GPIO_OT_OUT 12
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "ot-example";

volatile float dhwTemp = 0;
volatile float chTemp = 0;
volatile bool fault = false;
static int targetDHWTemp = 41;
static int targetCHTemp = 60;

static void ot_handler(void *arg);
static void IRAM_ATTR ot_processResponseCallback(unsigned long response, OpenThermResponseStatus_t responseStatus);

static void init_hw(void)
{
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ot_init(GPIO_OT_IN, GPIO_OT_OUT, false, ot_handler, ot_processResponseCallback);
}

static void IRAM_ATTR ot_handler(void *arg)
{
    ot_handleInterrupt();
}

static void IRAM_ATTR ot_processResponseCallback(unsigned long response, OpenThermResponseStatus_t responseStatus)
{
    // normally you shouldn't call printf in an interrupt handler, it will crash something.
    // ESP_LOGI(TAG, "Response from processResponseCallback!");
    // ESP_LOGI(TAG, "var response From CB: %lu", response);
    // ESP_LOGI(TAG, "var responseStatus from CB: %i", (int)responseStatus);
}

void otControl(void *pvParameter)
{
    while (1)
    {
        unsigned long status = ot_setBoilerStatus(false, true, false, false, false, true, false);

        OpenThermResponseStatus_t responseStatus = ot_getLastResponseStatus();
        if (responseStatus == OT_SUCCESS)
        {
            ESP_LOGI(TAG, "Central Heating: %s", ot_isCentralHeatingActive(status) ? "ON" : "OFF");
            ESP_LOGI(TAG, "Hot Water: %s", ot_isHotWaterActive(status) ? "ON" : "OFF");
            ESP_LOGI(TAG, "Flame: %s", ot_isFlameOn(status) ? "ON" : "OFF");
            fault = ot_isFault(status);
            ESP_LOGI(TAG, "Fault: %s", fault ? "YES" : "NO");
            if (fault)
            {
                ot_reset();
            }
            ot_setBoilerTemperature(targetCHTemp);
            ESP_LOGI(TAG, "Set CH Temp to: %i", targetCHTemp);
            ot_setDHWTemperature(targetDHWTemp);
            ESP_LOGI(TAG, "Set DHW Temp to: %i", targetDHWTemp);
            dhwTemp = ot_getDHWTemperature();
            ESP_LOGI(TAG, "DHW Temp: %.1f", dhwTemp);
            chTemp = ot_getBoilerTemperature();
            ESP_LOGI(TAG, "CH Temp: %.1f", chTemp);
            unsigned long slaveProductVersion = ot_getSlaveProductVersion();
            ESP_LOGI(TAG, "Slave Version: %08lX", slaveProductVersion);
            float slaveOTVersion = ot_getSlaveOTVersion();
            ESP_LOGI(TAG, "Slave OT Version: %.1f", slaveOTVersion);
        }
        else if (responseStatus == OT_TIMEOUT)
        {
            ESP_LOGE(TAG, "OT Communication Timeout");
        }
        else if (responseStatus == OT_INVALID)
        {
            ESP_LOGE(TAG, "OT Communication Invalid");
        }
        else if (responseStatus == OT_NONE)
        {
            ESP_LOGE(TAG, "OpenTherm not initialized");
        }

        if (fault)
        {
            ESP_LOGE(TAG, "Fault Code: %i", ot_getFault());
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    printf("OpenTherm library example!");
    init_hw();

    xTaskCreate(otControl, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
    vTaskSuspend(NULL);
}