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
    printf("Response from processResponseCallback!\n");
    printf("var response From CB: %lu\n", response);
    printf("var responseStatus from CB: %i\n", (int)responseStatus);
}

void otControl(void *pvParameter)
{
    while (1)
    {
        unsigned long status = ot_setBoilerStatus(true, true);

        OpenThermResponseStatus_t responseStatus = ot_getLastResponseStatus();
        if (responseStatus == OT_SUCCESS)
        {
            printf("Central Heating: %s\n", ot_isCentralHeatingActive(status) ? "ON" : "OFF");
            printf("Hot Water: %s\n", ot_isHotWaterActive(status) ? "ON" : "OFF");
            fault = ot_isFault(status);
            printf("Flame: %s\n", ot_isFlameOn(status) ? "ON" : "OFF");
            printf("Fault: %s\n", fault ? "YES" : "NO");
        }
        else if (responseStatus == OT_TIMEOUT)
        {
            printf("Timeout\n");
        }
        else if (responseStatus == OT_INVALID)
        {
            printf("Invalid\n");
        }
        else if (responseStatus == OT_NONE)
        {
            printf("OpenTherm not initialized\n");
        }
        if (ot_setDHWTemperature(41))
        {
            printf("Set DHW Temp!\n");
        };
        if (fault)
        {
            printf("Fault Code: %i\n", ot_getFault());
        }
        dhwTemp = ot_getDHWTemperature();
        printf("DHW Temp: %.1f\n", dhwTemp);
        ot_setBoilerTemperature(80);
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