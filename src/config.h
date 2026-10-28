#pragma once
// External Libraries
#include "ina3221.h"
#include "ezbee/zha.h"
#include "dht.h"
// Drivers
#include "driver/gpio.h"
// STDlib
#include <stdint.h>

// ******** Configuration ********
#define n_INA 5
#define n_DHT 2
#define n_I2C 2
struct AppConfig {
    // Device & Identification
    struct {
        // ZCL Strings MUST be length-prefixed.
        const char Manufacturer[11] = "\x09" "Espressif";
        const char Model[21] = "\x13" "PowerBox-INA15-DHT2";
        uint8_t PowerSource = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
        // ZCL Basic version attributes (uint8_t, application-defined, free to choose)
        uint8_t AppVersion   = 1;   // ApplicationVersion (0x0001)
        uint8_t StackVersion = 2;   // StackVersion (0x0002)
        uint8_t HwVersion    = 1;   // HWVersion (0x0003)
        //BOOT_PIN GPIO_NUM_9
        const gpio_num_t ButtonPin = GPIO_NUM_9;
    } Device;

    // Measurement Intervals
    struct {
        const int DhtMs = 3000;
        const int DcMs = 1000;
    } Measurement;

    // Serial & Logging
    struct {
        const uint8_t EnabledMeasurements = 0;
        const uint8_t EnabledReports = 1;
        const char* TagSetup = "Setup";
        const char* TagInitSensor = "InitSensor";
        const char* TagInitBus = "InitBus";
        const char* TagZigbee = "ZigBee";
        const char* TagMeasurement = "Measurement";
        const char* TagReporting = "Reporting";
    } Logging;

    // Zigbee ZCL Reporting Rules
    struct {
        struct {
            const uint16_t MaxInterval = 60;
            const uint16_t MinInterval = 5;
            const float ThresholdTemp = 0.5f;
            const float ThresholdHum = 2.0f;
        } Dht;
        
        struct {
            const uint16_t MaxInterval = 60;
            const uint16_t MinInterval = 5;
            const float ThresholdVolt = 0.06f;
            const float ThresholdCur = 0.03f;
            const float ThresholdPwr = 0.05f;
        } Dc;
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
            const uint8_t CountBus0 = 4;
            const float Shunts[n_INA][INA3221_BUS_NUMBER] = {
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
            };
        } Ina;

        struct {
            const int MinTemp = -10;   // °C, a tsens mérési tartomány alja
            const int MaxTemp = 80;    // °C, a tsens mérési tartomány teteje
        } Board;
    } Sensor;

    // Zigbee Stack & Endpoint Mappings
    struct {
        const uint8_t Disabled = 0;
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
    } Zigbee;
};
extern const AppConfig Config;
