#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
typedef enum OpenThermResponseStatus
{
    OT_NONE,
    OT_SUCCESS,
    OT_INVALID,
    OT_TIMEOUT
} OpenThermResponseStatus_t;

typedef enum OpenThermMessageType
{
    /*  Master to Slave */
    OT_READ_DATA = 0b000,
    OT_WRITE_DATA = 0b001,
    OT_INVALID_DATA = 0b010,
    OT_RESERVED = 0b011,
    /* Slave to Master */
    OT_READ_ACK = 0b100,
    OT_WRITE_ACK = 0b101,
    OT_DATA_INVALID = 0b110,
    OT_UNKNOWN_DATA_ID = 0b111
} OpenThermMessageType_t;

typedef enum OpenThermMessageID
{
    otStatus,                       // flag8 / flag8  Master and Slave Status flags.
    otTSet,                         // f8.8  Control setpoint  ie CH  water temperature setpoint (°C)
    otMConfigMMemberIDcode,         // flag8 / u8  Master Configuration Flags /  Master MemberID Code
    otSConfigSMemberIDcode,         // flag8 / u8  Slave Configuration Flags /  Slave MemberID Code
    otCommand,                      // u8 / u8  Remote Command
    otASFflags,                     // / OEM-fault-code  flag8 / u8  Application-specific fault flags and OEM fault code
    otRBPflags,                     // flag8 / flag8  Remote boiler parameter transfer-enable & read/write flags
    otCoolingControl,               // f8.8  Cooling control signal (%)
    otTsetCH2,                      // f8.8  Control setpoint for 2e CH circuit (°C)
    otTrOverride,                   // f8.8  Remote override room setpoint
    otTSP,                          // u8 / u8  Number of Transparent-Slave-Parameters supported by slave
    otTSPindexTSPvalue,             // u8 / u8  Index number / Value of referred-to transparent slave parameter.
    otFHBsize,                      // u8 / u8  Size of Fault-History-Buffer supported by slave
    otFHBindexFHBvalue,             // u8 / u8  Index number / Value of referred-to fault-history buffer entry.
    otMaxRelModLevelSetting,        // f8.8  Maximum relative modulation level setting (%)
    otMaxCapacityMinModLevel,       // u8 / u8  Maximum boiler capacity (kW) / Minimum boiler modulation level(%)
    otTrSet,                        // f8.8  Room Setpoint (°C)
    otRelModLevel,                  // f8.8  Relative Modulation Level (%)
    otCHPressure,                   // f8.8  Water pressure in CH circuit  (bar)
    otDHWFlowRate,                  // f8.8  Water flow rate in DHW circuit. (litres/minute)
    otDayTime,                      // special / u8  Day of Week and Time of Day
    otDate,                         // u8 / u8  Calendar date
    otYear,                         // u16  Calendar year
    otTrSetCH2,                     // f8.8  Room Setpoint for 2nd CH circuit (°C)
    otTr,                           // f8.8  Room temperature (°C)
    otTboiler,                      // f8.8  Boiler flow water temperature (°C)
    otTdhw,                         // f8.8  DHW temperature (°C)
    otToutside,                     // f8.8  Outside temperature (°C)
    otTret,                         // f8.8  Return water temperature (°C)
    otTstorage,                     // f8.8  Solar storage temperature (°C)
    otTcollector,                   // f8.8  Solar collector temperature (°C)
    otTflowCH2,                     // f8.8  Flow water temperature CH2 circuit (°C)
    otTdhw2,                        // f8.8  Domestic hot water temperature 2 (°C)
    otTexhaust,                     // s16  Boiler exhaust temperature (°C)
    otFanSpeed = 35,                // u16  Fan speed(rpm)
    otTdhwSetUBTdhwSetLB = 48,      // s8 / s8  DHW setpoint upper & lower bounds for adjustment  (°C)
    otMaxTSetUBMaxTSetLB,           // s8 / s8  Max CH water setpoint upper & lower bounds for adjustment  (°C)
    otHcratioUBHcratioLB,           // s8 / s8  OTC heat curve ratio upper & lower bounds for adjustment
    otTdhwSet = 56,                 // f8.8  DHW setpoint (°C)    (Remote parameter 1)
    otMaxTSet,                      // f8.8  Max CH water setpoint (°C)  (Remote parameters 2)
    otHcratio,                      // f8.8  OTC heat curve ratio (°C)  (Remote parameter 3)
    otRemoteOverrideFunction = 100, // flag8 / -  Function of manual and program changes in master and remote room setpoint.
    otOEMDiagnosticCode = 115,      // u16  OEM-specific diagnostic/service code
    otBurnerStarts,                 // u16  Number of starts burner
    otCHPumpStarts,                 // u16  Number of starts CH pump
    otDHWPumpValveStarts,           // u16  Number of starts DHW pump/valve
    otDHWBurnerStarts,              // u16  Number of starts burner during DHW mode
    otBurnerOperationHours,         // u16  Number of hours that burner is in operation (i.e. flame on)
    otCHPumpOperationHours,         // u16  Number of hours that CH pump has been running
    otDHWPumpValveOperationHours,   // u16  Number of hours that DHW pump has been running or DHW valve has been opened
    otDHWBurnerOperationHours,      // u16  Number of hours that burner is in operation during DHW mode
    otOpenThermVersionMaster,       // f8.8  The implemented version of the OpenTherm Protocol Specification in the master.
    otOpenThermVersionSlave,        // f8.8  The implemented version of the OpenTherm Protocol Specification in the slave.
    otMasterVersion,                // u8 / u8  Master product version number and type
    otSlaveVersion,                 // u8 / u8  Slave product version number and type
} OpenThermMessageID_t;

typedef enum OpenThermStatus
{
    OT_NOT_INITIALIZED,
    OT_READY,
    OT_DELAY,
    OT_REQUEST_SENDING,
    OT_RESPONSE_WAITING,
    OT_RESPONSE_START_BIT,
    OT_RESPONSE_RECEIVING,
    OT_RESPONSE_READY,
    OT_RESPONSE_INVALID
} OpenThermStatus_t;

typedef uint8_t byte;

gpio_num_t _ot_pin_in;
gpio_num_t _ot_pin_out;
gpio_isr_t *_otHandleInterruptCallback;
void (*_otProcessResponseCallback)(unsigned long, OpenThermResponseStatus_t);
bool _otIsSlave;
volatile OpenThermStatus_t _otStatus;
volatile unsigned long _otResponse;
volatile OpenThermResponseStatus_t _otResponseStatus;
int64_t _otResponseTimeStamp;
volatile byte _otResponseBitIndex;

esp_err_t ot_init(gpio_num_t pin_in, gpio_num_t pin_out, bool isSlave, gpio_isr_t handleInterruptCallback, void (*processResponseCallback)(unsigned long, OpenThermResponseStatus_t));
bool IRAM_ATTR ot_isReady();
int IRAM_ATTR ot_readState();
void ot_activateBoiler();
bool ot_sendRequestAsync(unsigned long request);
unsigned long ot_sendRequest(unsigned long request);
bool ot_sendResponse(unsigned long request);
OpenThermResponseStatus_t ot_getLastResponseStatus();
void IRAM_ATTR ot_handleInterrupt();
void ot_process();
OpenThermMessageID_t ot_getDataId(unsigned long frame);
unsigned long ot_buildRequest(OpenThermMessageType_t type, OpenThermMessageID_t id, unsigned int data);
unsigned long ot_buildResponse(OpenThermMessageType_t type, OpenThermMessageID_t id, unsigned int data);
bool ot_isValidResponse(unsigned long response);
bool ot_isValidRequest(unsigned long request);
void ot_end();
bool ot_isFault(unsigned long response);
bool ot_isCentralHeatingActive(unsigned long response);
bool ot_isHotWaterActive(unsigned long response);
bool ot_isFlameOn(unsigned long response);
bool ot_isDiagnostic(unsigned long response);
uint16_t ot_getUInt(const unsigned long response);
float ot_getFloat(const unsigned long response);
unsigned int ot_temperatureToData(float temperature);
unsigned long ot_setBoilerStatus(bool enableCH, bool enableDHW, bool enableCooling, bool enableOTC, bool enableCH2, bool enableSummerMode, bool dhwBlock);
bool ot_setBoilerTemperature(float temperature);
float ot_getBoilerTemperature();
float ot_getReturnTemperature();
bool ot_setDHWTemperature(float temperature);
float ot_getDHWTemperature();
float ot_getModulation();
float ot_getPressure();
unsigned int ot_getFault();
unsigned long ot_reset();
unsigned long ot_getSlaveProductVersion();
float ot_getSlaveOTVersion();
unsigned long ot_getSlaveConfiguration();
OpenThermMessageType_t ot_getMessageType(unsigned long message);
OpenThermMessageID_t ot_getDataID(unsigned long frame);
const char *ot_statusToString(OpenThermResponseStatus_t status);
const char *ot_messageTypeToString(OpenThermMessageType_t message_type);