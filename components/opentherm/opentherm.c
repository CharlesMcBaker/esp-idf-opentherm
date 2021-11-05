#include "opentherm.h"
#include <rom/ets_sys.h>

#define HIGH 1
#define LOW 0
#ifndef bit
#define bit(b) (1UL << (b))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)

#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
#ifndef HELPER_TARGET_IS_ESP32
#define HELPER_TARGET_IS_ESP32 (1)
#define HELPER_TARGET_IS_ESP8266 (0)
#endif
#elif defined(CONFIG_IDF_TARGET_ESP8266)
#ifndef HELPER_TARET_IS_ESP8266
#define HELPER_TARGET_IS_ESP32 (0)
#define HELPER_TARGET_IS_ESP8266 (1)
#endif
#else
#error BUG: cannot determine the target
#endif

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL()
#endif

static void __empty()
{
    // Empty
}
void yield(void) __attribute__((weak, alias("__empty")));

esp_err_t ot_init(gpio_num_t pin_in, gpio_num_t pin_out, bool isSlave, gpio_isr_t handleInterruptCallback, void (*processResponseCallback)(unsigned long, OpenThermResponseStatus_t))
{
    // Initialize the GPIO
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_in);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin_out);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    _ot_pin_in = pin_in;
    _ot_pin_out = pin_out;
    _otIsSlave = isSlave;

    if (handleInterruptCallback != NULL)
    {
        _otHandleInterruptCallback = &handleInterruptCallback;
        gpio_isr_handler_add(pin_in, *_otHandleInterruptCallback, NULL);
    }
    ot_activateBoiler();
    _otStatus = OT_READY;
    _otProcessResponseCallback = processResponseCallback;
    return ESP_OK;
}

bool IRAM_ATTR ot_isReady()
{
    return _otStatus == OT_READY;
}

int IRAM_ATTR ot_readState()
{
    return gpio_get_level(_ot_pin_in);
}

void ot_activateBoiler()
{
    gpio_set_level(_ot_pin_out, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void sendBit(bool high)
{
    gpio_set_level(_ot_pin_out, !high);
    ets_delay_us(500);
    gpio_set_level(_ot_pin_out, high);
    ets_delay_us(500);
}

bool ot_sendRequestAsync(unsigned long request)
{
    bool ready = ot_isReady();
    if (!ready)
        return false;
    PORT_ENTER_CRITICAL;
    _otStatus = OT_REQUEST_SENDING;
    _otResponse = 0;
    _otResponseStatus = OT_NONE;

    sendBit(HIGH);
    for (int i = 31; i >= 0; i--)
    {
        sendBit(bitRead(request, i));
    }
    sendBit(HIGH);
    // setIdleState();
    gpio_set_level(_ot_pin_out, HIGH);

    _otStatus = OT_RESPONSE_WAITING;
    _otResponseTimeStamp = esp_timer_get_time();
    PORT_EXIT_CRITICAL;
    return true;
}

unsigned long ot_sendRequest(unsigned long request)
{
    if (!ot_sendRequestAsync(request))
        return 0;
    while (!ot_isReady())
    {
        ot_process();
        yield();
    }

    return _otResponse;
}

bool ot_sendResponse(unsigned long request)
{
    _otStatus = OT_REQUEST_SENDING;
    _otResponse = 0;
    _otResponseStatus = OT_NONE;

    sendBit(HIGH);
    for (int i = 31; i >= 0; i--)
    {
        sendBit(bitRead(request, i));
    }
    sendBit(HIGH);
    gpio_set_level(_ot_pin_out, HIGH);
    _otStatus = OT_READY;
    return true;
}

OpenThermResponseStatus_t ot_getLastResponseStatus()
{
    return _otResponseStatus;
}

void IRAM_ATTR ot_handleInterrupt()
{
    if (ot_isReady())
    {
        if (_otIsSlave && ot_readState() == HIGH)
        {
            _otStatus = OT_RESPONSE_WAITING;
        }
        else
        {
            return;
        }
    }

    int64_t newTs = esp_timer_get_time();
    if (_otStatus == OT_RESPONSE_WAITING)
    {
        if (ot_readState() == HIGH)
        {
            _otStatus = OT_RESPONSE_START_BIT;
            _otResponseTimeStamp = newTs;
        }
        else
        {
            _otStatus = OT_INVALID;
            _otResponseTimeStamp = newTs;
        }
    }
    else if (_otStatus == OT_RESPONSE_START_BIT)
    {
        if ((newTs - _otResponseTimeStamp < 750) && ot_readState() == LOW)
        {
            _otStatus = OT_RESPONSE_RECEIVING;
            _otResponseTimeStamp = newTs;
            _otResponseBitIndex = 0;
        }
        else
        {
            _otStatus = OT_RESPONSE_INVALID;
            _otResponseTimeStamp = newTs;
        }
    }
    else if (_otStatus == OT_RESPONSE_RECEIVING)
    {
        if ((newTs - _otResponseTimeStamp) > 750)
        {
            if (_otResponseBitIndex < 32)
            {
                _otResponse = (_otResponse << 1) | !ot_readState();
                _otResponseTimeStamp = newTs;
                _otResponseBitIndex++;
            }
            else
            {
                _otStatus = OT_RESPONSE_READY;
                _otResponseTimeStamp = newTs;
            }
        }
    }
}

void ot_process()
{
    PORT_ENTER_CRITICAL;
    OpenThermStatus_t st = _otStatus;
    int64_t ts = _otResponseTimeStamp;

    if (st == OT_READY)
        return;
    int64_t newTs = esp_timer_get_time();
    if (st != OT_NOT_INITIALIZED && (newTs - ts) > 1000000)
    {
        _otStatus = OT_READY;
        _otResponseStatus = OT_TIMEOUT;
        if (_otProcessResponseCallback != NULL)
            _otProcessResponseCallback(_otResponse, _otResponseStatus);
    }
    else if (st == OT_RESPONSE_INVALID)
    {
        _otStatus = OT_DELAY;
        _otResponseStatus = OT_INVALID;
        if (_otProcessResponseCallback != NULL)
            _otProcessResponseCallback(_otResponse, _otResponseStatus);
    }
    else if (st == OT_RESPONSE_READY)
    {
        _otStatus = OT_DELAY;
        _otResponseStatus = (_otIsSlave ? ot_isValidRequest(_otResponse) : ot_isValidResponse(_otResponse)) ? OT_SUCCESS : OT_INVALID;
        if (_otProcessResponseCallback != NULL)
            _otProcessResponseCallback(_otResponse, _otResponseStatus);
    }
    else if (st == OT_DELAY)
    {
        if ((newTs - ts) > 100000)
        {
            _otStatus = OT_READY;
        }
    }
    PORT_EXIT_CRITICAL;
}

bool parity(unsigned long frame)
{
    byte p = 0;
    while (frame > 0)
    {
        if (frame & 1)
            p++;
        frame = frame >> 1;
    }
    return (p & 1);
}

OpenThermMessageID_t ot_getDataId(unsigned long frame)
{
    return (OpenThermMessageID_t)((frame >> 16) & 0xFF);
}

unsigned long ot_buildRequest(OpenThermMessageType_t type, OpenThermMessageID_t id, unsigned int data)
{
    unsigned long request = data;
    if (type == OT_WRITE_DATA)
    {
        request |= 1ul << 28;
    }
    request |= ((unsigned long)id) << 16;
    if (parity(request))
        request |= (1ul << 31);
    return request;
}

unsigned long ot_buildResponse(OpenThermMessageType_t type, OpenThermMessageID_t id, unsigned int data)
{
    unsigned long response = data;
    response |= 1ul << 28;
    response |= ((unsigned long)id) << 16;
    if (parity(response))
        response |= 1ul << 31;
    return response;
}

bool ot_isValidResponse(unsigned long response)
{
    if (parity(response))
        return false;
    byte msgType = (response << 1) >> 29;
    return msgType == OT_READ_ACK || msgType == OT_WRITE_ACK;
}

bool ot_isValidRequest(unsigned long request)
{
    if (parity(request))
        return false;
    byte msgType = (request << 1) >> 29;
    return msgType == OT_READ_DATA || msgType == OT_WRITE_DATA;
}

void ot_end()
{
    if (_otHandleInterruptCallback != NULL)
        gpio_isr_handler_remove(_ot_pin_in);
}

bool ot_isFault(unsigned long response)
{
    return response & 0x1;
}

bool ot_isCentralHeatingActive(unsigned long response)
{
    return response & 0x2;
}

bool ot_isHotWaterActive(unsigned long response)
{
    return response & 0x4;
}

bool ot_isFlameOn(unsigned long response)
{
    return response & 0x08;
}

bool ot_isDiagnostic(unsigned long response)
{
    return response & 0x40;
}

uint16_t ot_getUInt(const unsigned long response)
{
    const uint16_t u88 = response & 0xffff;
    return u88;
}

float ot_getFloat(const unsigned long response)
{
    const uint16_t u88 = ot_getUInt(response);
    const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
    return f;
}

unsigned int ot_temperatureToData(float temperature)
{
    if (temperature < 0)
        temperature = 0;
    if (temperature > 100)
        temperature = 100;
    unsigned int data = (unsigned int)(temperature * 256);
    return data;
}

unsigned long buildGetBoilerTemperatureRequest()
{
    return ot_buildRequest(OT_READ_DATA, otTboiler, 0);
}

unsigned long buildSetBoilerTemperatureRequest(float temperature)
{
    unsigned int data = ot_temperatureToData(temperature);
    return ot_buildRequest(OT_WRITE_DATA, otTSet, data);
}

unsigned long buildSetBoilerStatusRequest(bool enableCH, bool enableDHW)
{
    unsigned int data = enableCH | enableDHW << 1;
    data <<= 8;
    return ot_buildRequest(OT_READ_DATA, otStatus, data);
}

unsigned long ot_setBoilerStatus(bool enableCH, bool enableDHW)
{
    return ot_sendRequest(buildSetBoilerStatusRequest(enableCH, enableDHW));
}

bool ot_setBoilerTemperature(float temperature)
{
    unsigned long response = ot_sendRequest(buildSetBoilerTemperatureRequest(temperature));
    return ot_isValidResponse(response);
}

float ot_getBoilerTemperature()
{
    unsigned long response = ot_sendRequest(buildGetBoilerTemperatureRequest());
    return ot_isValidResponse(response) ? ot_getFloat(response) : 0;
}

float ot_getReturnTemperature()
{
    unsigned long response = ot_sendRequest(ot_buildRequest(OT_READ_DATA, otTret, 0));
    return ot_isValidResponse(response) ? ot_getFloat(response) : 0;
}

bool ot_setDHWTemperature(float temperature)
{
    unsigned int data = ot_temperatureToData(temperature);
    unsigned long response = ot_sendRequest(ot_buildRequest(OT_WRITE_DATA, otTdhwSet, data));
    return ot_isValidResponse(response);
}

float ot_getDHWTemperature()
{
    unsigned long response = ot_sendRequest(ot_buildRequest(OT_READ_DATA, otTdhw, 0));
    return ot_isValidResponse(response) ? ot_getFloat(response) : 1;
}

float ot_getModulation()
{
    unsigned long response = ot_sendRequest(ot_buildRequest(OT_READ_DATA, otRelModLevel, 0));
    return ot_isValidResponse(response) ? ot_getFloat(response) : 0;
}

float ot_getPressure()
{
    unsigned long response = ot_sendRequest(ot_buildRequest(OT_READ_DATA, otCHPressure, 0));
    return ot_isValidResponse(response) ? ot_getFloat(response) : 0;
}

unsigned int ot_getFault()
{
    return ((ot_sendRequest(ot_buildRequest(OT_READ_DATA, otASFflags, 0)) >> 8) & 0xff);
}