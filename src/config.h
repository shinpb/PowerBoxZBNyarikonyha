#pragma once
// External Libraries
#include "ina3221.h"
#include "ezbee/zha.h"
#include "dht.h"
// Drivers
#include "driver/gpio.h"
#include "driver/uart.h"
// Internal (protocol constants only: PZEM_ADDR_DEFAULT; no ESP-IDF dependency)
#include "pzem_proto.h"
// Sys
#include "esp_log.h"
// STDlib
#include <stdint.h>

// ******** Configuration ********
#define n_INA 5
#define n_DHT 2
#define n_I2C 2
#define n_UART 1
#define n_PZEM 1
// Reporting thresholds of one DC channel (raw ZCL units follow the endpoint divisors, see zcl_values.h)
struct DcReportThr {
    float Volt;   // V
    float Cur;    // A
    float Pwr;    // W
};

struct AppConfig {
    // Device & Identification
    struct {
        // ZCL Strings MUST be length-prefixed.
        const char Manufacturer[11] = "\x09" "Espressif";
        const char Model[24] = "\x16" "PowerBox-DC15-AC1-DHT2";   // 22 characters (length byte 0x16); the Z2M zigbeeModel must be identical
        uint8_t PowerSource = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
        // ZCL Basic version attributes (uint8_t, application-defined, free to choose)
        uint8_t AppVersion   = 1;   // ApplicationVersion (0x0001)
        uint8_t StackVersion = 2;   // StackVersion (0x0002)
        uint8_t HwVersion    = 1;   // HWVersion (0x0003)
        //BOOT_PIN GPIO_NUM_9
        const gpio_num_t ButtonPin = GPIO_NUM_9;
        const uint16_t ButtonHoldMs = 3000;    // hold time for the Zigbee factory reset
        const uint16_t ButtonPollMs = 50;      // button polling period
        const uint16_t ResetRebootDelayMs = 2000; // delay between the factory reset and the restart
        const uint8_t  ZclVersion = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;   // ZCL Basic ZCLVersion attribute (library default, 0x08)
    } Device;

    // FreeRTOS tasks (stack in bytes, priority). Stack sizes are reviewed in phase 7 (see the heartbeat "stack_free" values).
    struct TaskDef { const uint32_t Stack; const uint8_t Prio; };
    struct {
        const TaskDef ZigbeeMain      = { 8192,  5 };   // "Zigbee_main"
        const TaskDef ReportDefaults  = { 4096,  3 };   // "Rep_defaults" (one-shot)
        const TaskDef MeasDht         = { 4096,  1 };   // "Task_DHT" (DHT + board temperature)
        const TaskDef MeasDc          = { 4096,  1 };   // "Task_DC"
        const TaskDef MeasPzem        = { 4096,  1 };   // "Task_PZEM" (Modbus polling of the PZEM modules)
        const TaskDef Heartbeat       = { 4096,  1 };   // "Task_HB"
        const TaskDef Button          = { 2048,  1 };   // "Task_Button"
    } Tasks;

    // Measurement Intervals
    struct {
        const int DhtMs = 3000;
        const int DcMs = 1000;
        const int PzemMs = 1000;   // PZEM poll period (the module refreshes its registers a few times per second)
    } Measurement;

    // Serial & Logging
    struct {
        const uint8_t EnabledMeasurements = 0;
        const uint8_t ReportUpdates = 0;     // 1 = one log line per successfully written default reporting record (52 lines at a factory-new start)
        const uint8_t InaDiag = 0;           // 1 = run the INA3221 register dump (ina_diag()) at boot (bring-up aid, ~25 lines, delays the Zigbee start)
        const uint16_t InaFailLogEvery = 60;     // a failing INA measurement is logged at the first failure and then every N-th cycle
        const uint16_t DhtFailLogEvery = 20;      // a failing DHT read is logged at the first failure of a streak and then every N-th
        const uint8_t PzemDiag = 0;          // 1 = run the PZEM register dump (pzem_diag()) at boot (bring-up aid: address, alarm threshold, raw registers)
        const uint8_t PzemRaw = 0;           // 1 = hex dump of every PZEM request/response frame (very verbose, bring-up aid)
        const uint16_t PzemFailLogEvery = 20;     // a failing PZEM poll is logged at the first failure of a streak and then every N-th
        // Library log levels (the firmware logs the failures itself, the libraries repeat them per retry): ESP_LOG_NONE = silent,
        // ESP_LOG_WARN = library default behaviour (e.g. ~12 lines per failed I2C access).
        const esp_log_level_t I2cdevLibLevel = ESP_LOG_NONE;   // tag "i2cdev" (esp-idf-lib)
        const esp_log_level_t DhtLibLevel    = ESP_LOG_NONE;   // tag "dht" (esp-idf-lib)
        const uint16_t SetAttrFailLogEvery = 200; // a failing attribute write is logged at the first failure and then every N-th
        // Topics: every log line belongs to one topic (tag). Use the LOGx_<TOPIC>() macros below (end of file).
        const char* TagSetup = "Setup";
        const char* TagInitSensor = "InitSensor";
        const char* TagInitBus = "InitBus";
        const char* TagZigbee = "ZigBee";
        const char* TagMeasurement = "Measurement";
        const char* TagReporting = "Reporting";
        const char* TagHealth = "Health";
        const char* TagHeartbeat = "Heartbeat";
        const char* TagInaDiag = "INADiag";
        const char* TagPzemDiag = "PZEMDiag";
    } Logging;

    // Error indication & fault handling (see health.h)
    // Pins: GPIO number, a NEGATIVE value (-1, GPIO_NUM_NC) disables that indication (no pin is used).
    // Pin level: unit state 0 (OK) -> low, anything else -> high. All pins start low.
    struct {
        const int GlobalPin = 18;   // high if ANY unit is not OK (GPIO18 free, see pinout)
        const int ZigbeePin = -1;
        const int BusPin    = -1;
        const int SensorPin = -1;
        const uint8_t  FailThreshold   = 3;    // consecutive failed reads before the error flag is set
        const uint8_t  RecoverThreshold = 3;   // consecutive good reads before the error flag is cleared (hysteresis; while recovering the value stays invalid)
        const uint16_t RetryS          = 30;   // re-init / probe period of a failed INA or PZEM module (s)
        const uint8_t  AbortOnInitFail = 0;    // 1 = development: abort on a sensor init failure (old behavior)
        const uint16_t HeartbeatS      = 30;   // heartbeat log period (s), 0 = off (see heartbeat.h)
        const uint8_t  HeartbeatLongEvery = 5; // every N-th heartbeat also prints the extended diagnostic lines, 0 = never
    } Health;

    // Zigbee ZCL Reporting Rules = FIRMWARE DEFAULTS (D3/N4)
    // Written once, on a factory-new start BEFORE the first join (a task, ~0.5 s per record, ~26 s for all 52 + 6 per PZEM
    // records; the steering starts after it). The coordinator (Z2M: configure on pairing, Reconfigure) overwrites
    // them afterwards, and a plain reboot never touches them. Enabled = 0: that category is left entirely to the
    // coordinator (its records stay at the stack default until the coordinator configures them).
    // Keep the values equal to the converter's configure() (D4): temperature 5/60 s 0.5 C, humidity 2 %,
    // DC 2/30 s with the per channel thresholds of Dc.Thresholds, fault 0/3600 s.
    struct {
        const uint8_t Enabled = 1;          // global switch, 0 = no firmware default reporting at all

        struct {                            // board temperature (EP 1)
            const uint8_t  Enabled = 1;
            const uint16_t MaxInterval = 60;
            const uint16_t MinInterval = 5;
            const float ThresholdTemp = 0.5f;   // C
        } Board;

        struct {                            // DHT temperature and humidity (EP 2, 3)
            const uint8_t  Enabled = 1;
            const uint16_t MaxInterval = 60;
            const uint16_t MinInterval = 5;
            const float ThresholdTemp = 0.5f;   // C
            const float ThresholdHum = 2.0f;    // %
        } Dht;

        struct {                            // 15 x (V, I, P) DC channels, 45 records
            const uint8_t  Enabled = 1;
            const uint16_t MaxInterval = 30;
            const uint16_t MinInterval = 2;
            // Per channel thresholds {V, A, W}, indexed like Ina.Shunts. The current threshold is kept at >= ~20 LSB of the
            // channel's shunt (LSB = 40 uV / R) and about 1 % of its expected peak (shunt_fuse.txt); INA5 = batteries/charger (10 mOhm, 10 A).
            const DcReportThr Thresholds[n_INA][INA3221_BUS_NUMBER] = {
                {{0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}},   // INA1: ESP32, RPi (25 mOhm), switch
                {{0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}},   // INA2: POE uplink, KAM 0, KAM 1
                {{0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}},   // INA3: spare
                {{0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}, {0.06f, 0.03f, 0.30f}},   // INA4: spare
                {{0.06f, 0.10f, 1.00f}, {0.06f, 0.10f, 1.00f}, {0.06f, 0.10f, 1.00f}},   // INA5: BTR 0, BTR 1, CHARGER (10 mOhm)
            };
        } Dc;

        struct {                            // PZEM-004T AC meters (Config.Zigbee.Pzem): 6 records per module (V, I, P, f, PF, E)
            const uint8_t  Enabled = 1;
            const uint16_t MaxInterval = 60;
            const uint16_t MinInterval = 5;
            // Reportable changes in physical units (raw units follow the ZCL_AC_DIV_* divisors, zcl_values.h)
            const float   ThresholdVolt   = 1.0f;     // V
            const float   ThresholdCur    = 0.05f;    // A
            const float   ThresholdPwr    = 2.0f;     // W
            const float   ThresholdFreq   = 0.2f;     // Hz
            const uint8_t ThresholdPf     = 5;        // percent points
            const float   ThresholdEnergy = 0.010f;   // kWh (10 Wh)
        } Pzem;

        struct {                            // fault indication: Binary Input (EP 1) + 0xFC11 fault mask
            const uint8_t  Enabled = 1;
            const uint16_t MaxInterval = 3600;
            const uint16_t MinInterval = 0;
        } Fault;
    } Reporting;

    // I2C Bus Configuration
    struct {
        const uint8_t Count = n_I2C;
        struct BusDef {
            i2c_port_t port;
            struct PinDef {
                gpio_num_t sda;
                gpio_num_t scl;
            } pin;
            const unsigned long Speed;
        } bus[n_I2C] = {
            { I2C_NUM_0,    { GPIO_NUM_23, GPIO_NUM_22 }, 100000 },
            { LP_I2C_NUM_0, { GPIO_NUM_6,  GPIO_NUM_7  }, 100000 }
        };
    } I2C;

    // UART Bus Configuration (same structure as the I2C buses: the number of buses, per bus the port and the pins)
    // The pins go through the GPIO matrix, so any free GPIO works. NOT GPIO16/17 (UART0 default pins: serial download /
    // boot log, a connected module can disturb flashing), not the strapping pins (4, 5, 8, 9, 15), not USB (12, 13).
    // The port must not be the console UART (checked at the init, log "InitBus"). The pins are also checked against
    // the I2C, DHT, button and error indication pins. GPIO2/3: free on the ESP32-C6 SuperMini (check the pinout file).
    struct {
        const uint8_t Count = n_UART;
        struct BusDef {
            uart_port_t port;
            struct PinDef {
                gpio_num_t tx;
                gpio_num_t rx;
            } pin;
            const uint32_t Baud;          // 8N1; PZEM-004T v3: 9600
            const uint16_t RxBufBytes;    // driver RX ring buffer (> the hardware FIFO of 128 bytes)
        } bus[n_UART] = {
            { UART_NUM_1, { GPIO_NUM_2, GPIO_NUM_3 }, 9600, 256 }
        };
    } UART;

    // Physical Sensor Settings
    struct {
        struct {
            const uint8_t Count = 2;
            const float MinTemp = -40.0f; // °C
            const float MaxTemp = 80.0f;  // °C
            const float MinHum = 0.0f;    // %
            const float MaxHum = 100.0f;  // %
            const dht_sensor_type_t Type = DHT_TYPE_AM2301;
            const gpio_num_t Pins[n_DHT] = {
                 GPIO_NUM_20, 
                 GPIO_NUM_19
            };
        } Dht;
        
        struct {
            const uint8_t Count = n_INA;
            const uint8_t CountBus0 = 4;       // modules 0..CountBus0-1 on I2C bus 0, the rest on bus 1
            const uint8_t BaseAddr = 0x40;     // I2C address of the first module on each bus (A0/A1 strapping: +0, +1, ...)
            // Measurement mode (hardware averaging and conversion times, see SHUNT_CALC.md 8). Per channel the shunt
            // integrates ShuntCt x Avg, the bus ADC BusCt x Avg; one cycle of a module = 3 x (shunt + bus) time and
            // has to stay well below Measurement.DcMs (checked and logged at the init: "INA mode"). The chip default is
            // Avg 1, 1.1 ms; the library starts from 0 (Avg 1, 140 us: the noisiest setting).
            const ina3221_avg_t Avg     = INA3221_AVG_16;
            const ina3221_ct_t  ShuntCt = INA3221_CT_4156;
            const ina3221_ct_t  BusCt   = INA3221_CT_332;
            // Shunt resistance in Ohm per channel (whole mOhm only: the library takes integer mOhm). Range = 0.1638 V / R.
            // Plan and rationale: hwdocs/shunt_fuse.txt, SHUNT_CALC.md (D19).
            const float Shunts[n_INA][INA3221_BUS_NUMBER] = {
                {0.1f,   0.025f, 0.1f},     // INA1: ESP32 (5 V), RPi (5 V, 2x0.05 parallel, 6.55 A), Mikrotik switch
                {0.1f,   0.1f,   0.1f},     // INA2: POE uplink, KAM 0, KAM 1
                {0.1f,   0.1f,   0.1f},     // INA3: spare
                {0.1f,   0.1f,   0.1f},     // INA4: spare
                {0.01f,  0.02f,  0.01f},    // INA5: BTR 0 (2x0.02), BTR 1 (TEMPORARY 1x0.02, 8.2 A), CHARGER (2x0.02); 16.4 A / 8.2 A / 16.4 A
            };
        } Ina;

        struct {
            const uint8_t Count = n_PZEM;
            struct ModDef {
                const uint8_t bus;         // index of Config.UART.bus[]
                const uint8_t addr;        // Modbus-RTU slave address (PZEM_ADDR_DEFAULT = 0x01 factory default, unique per bus)
            };
            // Modules on the UART buses. Several modules can share a bus (addresses must differ: set them one by one,
            // module by module, with the other modules disconnected). Zigbee endpoints: Config.Zigbee.Pzem.
            const ModDef module[n_PZEM] = {
                { 0, PZEM_ADDR_DEFAULT },     // PZEM 0 (AC mains side of the charger / PSU)
            };
            const uint16_t TimeoutMs = 300;   // wait for the first byte of the answer (the module answers in ~30-80 ms)
        } Pzem;

        struct {
            const int MinTemp = -10;   // °C, a tsens mérési tartomány alja
            const int MaxTemp = 80;    // °C, a tsens mérési tartomány teteje
        } Board;
    } Sensor;

    // Zigbee Stack & Endpoint Mappings
    struct {
        const uint8_t Disabled = 0;
        // Channels (11..26): the device scans the primary channel first, then the secondary mask (bit n = channel n).
        const uint8_t  PrimaryChannel       = 13;
        const uint32_t SecondaryChannelMask = 0x07FFF800;   // all channels 11..26
        const uint32_t PrimaryChannelMask   = (1U << PrimaryChannel);
        const char *StoragePartition = "nvs";  // NVS partition of the Zigbee stack (see partitions.csv)
        const uint16_t RetryInitMs  = 1000;    // BDB initialization retry after a failed start
        const uint16_t RetrySteerMs = 1000;    // steering retry after a failed join
        const uint16_t LeaveResetMs = 2000;    // after a leave WITHOUT rejoin (e.g. removed in Z2M): delay before the factory reset + restart (same as the button)
        const uint16_t RejoinPollMs = 1000;    // after a leave WITH rejoin: period of the "joined again?" check
        const uint8_t MaxChildren = 10;        // router: max. directly connected end devices
        const uint8_t BindTableSize = 32;      // APS binding table source size (the coordinator binds ~20 clusters, default 16 is too small)
        struct {
            const uint8_t Endpoint = 0x01;
        } Board;

        struct {
            const uint8_t Endpoints[n_DHT] = { 
                0x02, 
                0x03
            };
        } Dht;
        
        struct {
            const uint8_t Endpoints[n_INA][INA3221_BUS_NUMBER] = {
                {0x10, 0x11, 0x12},
                {0x13, 0x14, 0x15},
                {0x16, 0x17, 0x18},
                {0x19, 0x1A, 0x1B},
                {0x20, 0x21, 0x22},
            };
        } Ina;

        struct {
            // Electrical Measurement (AC: voltage, current, active power, frequency, power factor) + Metering (energy)
            const uint8_t Endpoints[n_PZEM] = { 0x30 };
        } Pzem;
    } Zigbee;
};
extern const AppConfig Config;

// The fault bit map has room for: 7 DHT (bits 1..7), 5 INA (8..12), 3 PZEM (13..15) sensor flags and 3 I2C (bits 1..3),
// 3 UART (bits 5..7) bus flags (health.h). A larger count needs a new bit map (health.h, converter FAULT_BITS).
static_assert(n_DHT <= 7, "health.h: HF_SENS_DHT(i) has bits 1..7");
static_assert(n_INA <= 5, "health.h: HF_SENS_INA(i) has bits 8..12");
static_assert(n_PZEM <= 3, "health.h: HF_SENS_PZEM(i) has bits 13..15");
static_assert(n_I2C <= 3, "health.h: HF_BUS_I2C(b) has bits 1..3");
static_assert(n_UART <= 3, "health.h: HF_BUS_UART(b) has bits 5..7");

// ******** Log macros per topic ********
// LOG<level>_<TOPIC>(format, ...) = ESP_LOG<level>(Config.Logging.Tag<Topic>, format, ...). The tag text comes from
// Config.Logging, so a topic can be renamed in one place.
#define LOGE_SETUP(...)        ESP_LOGE(Config.Logging.TagSetup, __VA_ARGS__)
#define LOGW_SETUP(...)        ESP_LOGW(Config.Logging.TagSetup, __VA_ARGS__)
#define LOGI_SETUP(...)        ESP_LOGI(Config.Logging.TagSetup, __VA_ARGS__)
#define LOGD_SETUP(...)        ESP_LOGD(Config.Logging.TagSetup, __VA_ARGS__)
#define LOGE_INIT_SENSOR(...)  ESP_LOGE(Config.Logging.TagInitSensor, __VA_ARGS__)
#define LOGW_INIT_SENSOR(...)  ESP_LOGW(Config.Logging.TagInitSensor, __VA_ARGS__)
#define LOGI_INIT_SENSOR(...)  ESP_LOGI(Config.Logging.TagInitSensor, __VA_ARGS__)
#define LOGD_INIT_SENSOR(...)  ESP_LOGD(Config.Logging.TagInitSensor, __VA_ARGS__)
#define LOGE_INIT_BUS(...)     ESP_LOGE(Config.Logging.TagInitBus, __VA_ARGS__)
#define LOGW_INIT_BUS(...)     ESP_LOGW(Config.Logging.TagInitBus, __VA_ARGS__)
#define LOGI_INIT_BUS(...)     ESP_LOGI(Config.Logging.TagInitBus, __VA_ARGS__)
#define LOGD_INIT_BUS(...)     ESP_LOGD(Config.Logging.TagInitBus, __VA_ARGS__)
#define LOGE_ZB(...)           ESP_LOGE(Config.Logging.TagZigbee, __VA_ARGS__)
#define LOGW_ZB(...)           ESP_LOGW(Config.Logging.TagZigbee, __VA_ARGS__)
#define LOGI_ZB(...)           ESP_LOGI(Config.Logging.TagZigbee, __VA_ARGS__)
#define LOGD_ZB(...)           ESP_LOGD(Config.Logging.TagZigbee, __VA_ARGS__)
#define LOGE_MEAS(...)         ESP_LOGE(Config.Logging.TagMeasurement, __VA_ARGS__)
#define LOGW_MEAS(...)         ESP_LOGW(Config.Logging.TagMeasurement, __VA_ARGS__)
#define LOGI_MEAS(...)         ESP_LOGI(Config.Logging.TagMeasurement, __VA_ARGS__)
#define LOGD_MEAS(...)         ESP_LOGD(Config.Logging.TagMeasurement, __VA_ARGS__)
#define LOGE_REP(...)          ESP_LOGE(Config.Logging.TagReporting, __VA_ARGS__)
#define LOGW_REP(...)          ESP_LOGW(Config.Logging.TagReporting, __VA_ARGS__)
#define LOGI_REP(...)          ESP_LOGI(Config.Logging.TagReporting, __VA_ARGS__)
#define LOGD_REP(...)          ESP_LOGD(Config.Logging.TagReporting, __VA_ARGS__)
#define LOGE_HEALTH(...)       ESP_LOGE(Config.Logging.TagHealth, __VA_ARGS__)
#define LOGW_HEALTH(...)       ESP_LOGW(Config.Logging.TagHealth, __VA_ARGS__)
#define LOGI_HEALTH(...)       ESP_LOGI(Config.Logging.TagHealth, __VA_ARGS__)
#define LOGD_HEALTH(...)       ESP_LOGD(Config.Logging.TagHealth, __VA_ARGS__)
#define LOGE_HB(...)           ESP_LOGE(Config.Logging.TagHeartbeat, __VA_ARGS__)
#define LOGW_HB(...)           ESP_LOGW(Config.Logging.TagHeartbeat, __VA_ARGS__)
#define LOGI_HB(...)           ESP_LOGI(Config.Logging.TagHeartbeat, __VA_ARGS__)
#define LOGD_HB(...)           ESP_LOGD(Config.Logging.TagHeartbeat, __VA_ARGS__)
#define LOGE_INADIAG(...)      ESP_LOGE(Config.Logging.TagInaDiag, __VA_ARGS__)
#define LOGW_INADIAG(...)      ESP_LOGW(Config.Logging.TagInaDiag, __VA_ARGS__)
#define LOGI_INADIAG(...)      ESP_LOGI(Config.Logging.TagInaDiag, __VA_ARGS__)
#define LOGD_INADIAG(...)      ESP_LOGD(Config.Logging.TagInaDiag, __VA_ARGS__)
#define LOGE_PZEMDIAG(...)     ESP_LOGE(Config.Logging.TagPzemDiag, __VA_ARGS__)
#define LOGW_PZEMDIAG(...)     ESP_LOGW(Config.Logging.TagPzemDiag, __VA_ARGS__)
#define LOGI_PZEMDIAG(...)     ESP_LOGI(Config.Logging.TagPzemDiag, __VA_ARGS__)
#define LOGD_PZEMDIAG(...)     ESP_LOGD(Config.Logging.TagPzemDiag, __VA_ARGS__)
