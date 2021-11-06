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
    ESP_LOGI(TAG, "Response from processResponseCallback!\n");
    ESP_LOGI(TAG, "var response From CB: %lu\n", response);
    ESP_LOGI(TAG, "var responseStatus from CB: %i\n", (int)responseStatus);
}

void otControl(void *pvParameter)
{
    while (1)
    {
        unsigned long status = ot_setBoilerStatus(false, true);

        OpenThermResponseStatus_t responseStatus = ot_getLastResponseStatus();
        if (responseStatus == OT_SUCCESS)
        {
            ESP_LOGI(TAG, "Central Heating: %s\n", ot_isCentralHeatingActive(status) ? "ON" : "OFF");
            ESP_LOGI(TAG, "Hot Water: %s\n", ot_isHotWaterActive(status) ? "ON" : "OFF");
            ESP_LOGI(TAG, "Flame: %s\n", ot_isFlameOn(status) ? "ON" : "OFF");
            fault = ot_isFault(status);
            ESP_LOGI(TAG, "Fault: %s\n", fault ? "YES" : "NO");
            ESP_LOGI(TAG, "Set CH Temp to: %i\n", ot_setBoilerTemperature(targetCHTemp));
            ESP_LOGI(TAG, "Set DHW Temp to: %i\n", ot_setDHWTemperature(targetDHWTemp));
            dhwTemp = ot_getDHWTemperature();
            ESP_LOGI(TAG, "DHW Temp: %.1f\n", dhwTemp);
        }
        else if (responseStatus == OT_TIMEOUT)
        {
            ESP_LOGE(TAG, "OT Communication Timeout\n");
        }
        else if (responseStatus == OT_INVALID)
        {
            ESP_LOGE(TAG, "OT Communication Invalid\n");
        }
        else if (responseStatus == OT_NONE)
        {
            ESP_LOGE(TAG, "OpenTherm not initialized\n");
        }

        if (fault)
        {
            ESP_LOGE(TAG, "Fault Code: %i\n", ot_getFault());
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    printf("OpenTherm library example!\n");
    init_hw();

    xTaskCreate(otControl, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
    vTaskSuspend(NULL);
}