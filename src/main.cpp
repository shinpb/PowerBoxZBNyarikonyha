// ******** Libraries config ********
// Sys
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"     // esp_restart()
#include "esp_timer.h"      // esp_timer_get_time()
#include "i2cdev.h"
// Drivers
#include "driver/gpio.h"    // gpio_config, gpio_get_level
#include "driver/i2c.h"
// STDLib
#include <stdio.h>
#include <math.h>           // fabs, isnan
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "nvs_flash.h"
#include "dht.h"
#include "ina3221.h"



// ******** Local Functions ********
void setup();
void loop();
void init_native_i2c_buses();
void init_sensors_dht();
void init_sensors_ina();
void init_zb_stack();
void init_zb_ep_basic_cluster(ezb_af_ep_desc_t * cluster_list);
void init_zb_ep_dht(ezb_af_device_desc_t ep_list);
void init_zb_ep_dc(ezb_af_device_desc_t ep_list);
void init_zb_config_reporting();
bool ezb_app_signal_handler(const ezb_app_signal_t *signal_struct);
void init_zb_network_up();
void zigbee_mainloop_task(void *pvParameters);
void dht_measurement_task(void* pvParameters);
void dc_measurement_task(void* pvParameters);
void button_task(void*);
// ******** Compatibility Functions ********
uint64_t millis();



// ******** Configuration ********
struct AppConfig {
    // Device & Identification
    struct {
        const char* Manufacturer = "Espressif";
        const char* Model = "PowerBox-INA15-DHT2";
        uint8_t PowerSource = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
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
            const uint16_t MaxInterval = 30;
            const uint16_t MinInterval = 2;
            const float ThresholdVolt = 0.06f;
            const float ThresholdCur = 0.03f;
            const float ThresholdPwr = 0.05f;
        } Dc;
    } Reporting;

    // I2C Bus Configuration
    struct {
        const uint8_t Count = 2;
        struct BusDef {
            i2c_port_t port;
            struct PinDef {
                gpio_num_t sda;
                gpio_num_t scl;
            } pin;
            const unsigned long Speed;
        } bus[2] = {
            { I2C_NUM_0,    { GPIO_NUM_23, GPIO_NUM_22 }, 100000 },
            { LP_I2C_NUM_0, { GPIO_NUM_6,  GPIO_NUM_7  }, 100000 }
        };
    } I2C;

    // Physical Sensor Settings
    struct {
        struct {
            const uint8_t Count = 2;
            const gpio_num_t Pins[2] = { GPIO_NUM_20, GPIO_NUM_19 };
        } Dht;
        
        struct {
            const uint8_t Count = 5;
            const uint8_t CountBus0 = 4;
            const float MaxBusVoltage_V = 26.0f;
            const float MaxshuntVoltage_mV = 163.84f;
            const float Shunts[5][INA3221_BUS_NUMBER] = {
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
                {0.1f, 0.1f, 0.1f},
            };
        } Ina;
    } Sensor;

    // Zigbee Stack & Endpoint Mappings
    struct {
        const uint8_t Disabled = 0;

        struct {
            const uint8_t Endpoints[2] = { 0x01, 0x02 };
        } Dht;
        
        struct {
            const uint8_t Endpoints[5][INA3221_BUS_NUMBER] = {
                {0x10, 0x11, 0x12},
                {0x13, 0x14, 0x15},
                {0x16, 0x17, 0x18},
                {0x19, 0x1A, 0x1B},
                {0x20, 0x21, 0x22},
            };
        } Ina;
    } Zigbee;
};
static const AppConfig Config;



// ******** Global variables ********
//ZB Global flag
static volatile bool is_zigbee_connected = false;
ina3221_t ina3221_arr[5]; //Config.Sensor.Ina.Count





// ESP-IDF main entry point
extern "C" void app_main() {
    // 1. Initialize Arduino core peripherals and drivers
    // initArduino();

    // 2. Call standard Arduino setup
    setup();

    // 3. Main loop execution
    while (true) {
        loop();
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to prevent Watchdog Timer (WDT) triggers
    }
}

void setup() {
    ESP_LOGI(Config.Logging.TagSetup, "=== ZIGBEE MEMORY LIMITS (PRE-COMPILED CORE) ===");

#ifdef CONFIG_ZB_CONFIG_MAX_EP_NUMBER
    ESP_LOGI(Config.Logging.TagSetup, "MAX_EP_NUMBER: %d", CONFIG_ZB_CONFIG_MAX_EP_NUMBER);
#else
    ESP_LOGI(Config.Logging.TagSetup, "CONFIG_ZB_CONFIG_MAX_EP_NUMBER: NOT DEFINED in sdkconfig.h (Using build defaults)");
#endif

#ifdef CONFIG_ZB_CONFIG_BINDING_TABLE_SIZE
    ESP_LOGI(Config.Logging.TagSetup, "BINDING_TABLE_SIZE: %d", CONFIG_ZB_CONFIG_BINDING_TABLE_SIZE);
#else
    ESP_LOGI(Config.Logging.TagSetup, "ONFIG_ZB_CONFIG_BINDING_TABLE_SIZE: NOT DEFINED in sdkconfig.h (Using build defaults)");
#endif

    ESP_LOGI(Config.Logging.TagSetup, "================================================");

    ESP_LOGI(Config.Logging.TagInitSensor, "Init DHT sensors");
    init_sensors_dht();

    ESP_LOGI(Config.Logging.TagInitBus, "Init I2C buses");
    init_native_i2c_buses();

    ESP_LOGI(Config.Logging.TagInitSensor, "Init INA sensors");
    init_sensors_ina();

    if(!Config.Zigbee.Disabled){
        //pinMode(button, INPUT_PULLUP);
        gpio_config_t btn_conf = {};
        btn_conf.pin_bit_mask = (1ULL << Config.Device.ButtonPin);
        btn_conf.mode = GPIO_MODE_INPUT;
        btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        btn_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&btn_conf);

        ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee Stack");
        init_zb_stack();

        xTaskCreate(
            button_task, 
            "Task_Button",  
            2048,
            NULL, 
            1,
            NULL
        );
    }

    xTaskCreate(
        dht_measurement_task, 
        "Task_DHT",     
        4096,            
        NULL,            
        1,
        NULL             
    );

    xTaskCreate(
        dc_measurement_task,  
        "Task_DC",      
        4096,            
        NULL,            
        1,
        NULL             
    );


    ESP_LOGI(Config.Logging.TagSetup, "Setup done");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void init_native_i2c_buses() {
    ESP_ERROR_CHECK(i2cdev_init());
    
    for(int i=0; i<2; i++) {
      ESP_LOGI(Config.Logging.TagInitBus, "Init I2C %d", i);
      i2c_config_t conf = {};
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = Config.I2C.bus[i].pin.sda;
      conf.scl_io_num = Config.I2C.bus[i].pin.scl;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = Config.I2C.bus[i].Speed;

      ESP_ERROR_CHECK(i2c_param_config(Config.I2C.bus[i].port, &conf));
      ESP_ERROR_CHECK(i2c_driver_install(Config.I2C.bus[i].port, conf.mode, 0, 0, 0));

//      ESP_ERROR_CHECK(i2cdev_init());
    }
}

void init_sensors_dht() {
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        gpio_set_pull_mode(Config.Sensor.Dht.Pins[idx], GPIO_PULLUP_ONLY);
    }
}

void init_sensors_ina() {
  for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
    for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
        ina3221_arr[idx].shunt[channel] = 
            (uint16_t)(Config.Sensor.Ina.Shunts[idx][channel] * 1000); // Ohm -> mOhm
    }
    // ~/.arduino15/packages/esp32/hardware/esp32/3.3.11/variants/esp32c6/pins_arduino.h
    uint8_t bus_idx = (idx < Config.Sensor.Ina.CountBus0) ? (0) : (1);
    uint8_t i2c_addr = 0x40 + (
        (idx < Config.Sensor.Ina.CountBus0) ? 
        (idx) : 
        (idx - Config.Sensor.Ina.CountBus0)
    );

    ESP_ERROR_CHECK(
        ina3221_init_desc(
            &ina3221_arr[idx],
            i2c_addr,
            Config.I2C.bus[bus_idx].port,
            Config.I2C.bus[bus_idx].pin.sda,
            Config.I2C.bus[bus_idx].pin.scl
        )
    );
    ESP_ERROR_CHECK(
        ina3221_set_options(
            &ina3221_arr[idx],
            true,   // continuous mode
            true,   // bus measurement enabled
            true    // shunt measurement enabled
        )
    );
  }
}

void init_zb_basic_cluster(ezb_af_ep_desc_t cluster_list) {
    ezb_zcl_cluster_desc_t basic_desc = ezb_zcl_basic_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);

    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)Config.Device.Manufacturer);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)Config.Device.Model);

    uint8_t power_source = Config.Device.PowerSource;
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);

    ezb_af_endpoint_add_cluster_desc(cluster_list, basic_desc);
}


void init_zb_ep_dht(ezb_af_device_desc_t ep_list) {
    // Each DHT sensor
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];
        // Culster list
        ezb_af_ep_desc_t cluster_list = ezb_af_get_ep_desc(ep);

        // Append basic cluster to endpoint 0x01
        if (ep == 0x01) {
            init_zb_basic_cluster(cluster_list);
        }

        // Temperature cluster
        ezb_zcl_temperature_measurement_cluster_config_t temp_cfg = {
            .measured_value = (int16_t)0x8000, // 0x8000 = Érvénytelen/Nincs még mérés (ZCL standard)
        };
        ezb_zcl_cluster_desc_t temp_cluster = ezb_zcl_temperature_measurement_create_cluster_desc(&temp_cfg, EZB_ZCL_CLUSTER_SERVER);
        const int16_t dht_t_min = (int16_t)-4000;       // -40.00 °C
        const int16_t dht_t_max = (int16_t)8000;        //  80.00 °C
        ezb_zcl_temperature_measurement_cluster_desc_add_attr(temp_cluster, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE_ID, (void*)&dht_t_min);
        ezb_zcl_temperature_measurement_cluster_desc_add_attr(temp_cluster, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MAX_MEASURED_VALUE_ID, (void*)&dht_t_max);
        ezb_af_endpoint_add_cluster_desc(cluster_list, temp_cluster);

        // Humidity cluster
        ezb_zcl_rel_humidity_measurement_cluster_config_t hum_cfg = {
            .measured_value = 0xFFFF, // 0xFFFF = Érvénytelen/Nincs még mérés
        };
        ezb_zcl_cluster_desc_t hum_cluster = ezb_zcl_rel_humidity_measurement_create_cluster_desc(&hum_cfg, EZB_ZCL_CLUSTER_SERVER);
        const int16_t dht_h_min = (int16_t)0;       //   0.00 %
        const int16_t dht_h_max = (int16_t)10000;   // 100.00 %
        ezb_zcl_temperature_measurement_cluster_desc_add_attr(hum_cluster, EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_ID, (void*)&dht_t_min);
        ezb_zcl_temperature_measurement_cluster_desc_add_attr(hum_cluster, EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_ID, (void*)&dht_t_max);
        ezb_af_endpoint_add_cluster_desc(cluster_list, hum_cluster);

        ezb_af_device_add_endpoint_desc(ep_list, cluster_list);
        //, ep, EZB_AF_HA_PROFILE_ID, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
    }
}

void init_zb_ep_dc(ezb_af_device_desc_t ep_list) {
    // Each Modul sensor
    for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
        // Each channel
        for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
            
            // ZB Endpiont ID
            uint8_t ep = Config.Zigbee.Ina.Endpoints[idx][channel];
            ESP_LOGI(Config.Logging.TagZigbee, "Create EP: DC %d/%d (EP 0x%X)", idx, channel, ep);
            
            // Culster list
            ezb_af_ep_desc_t cluster_list = ezb_af_get_ep_desc(ep);

            // Append basic cluster to endpoint 0x01
            if(ep == 0x01) {
                init_zb_basic_cluster(cluster_list);
            }

            // Electrical measurement cluster
            ezb_zcl_cluster_desc_t electrical_cluster = ezb_zcl_electrical_measurement_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);

            // Constants
            const float r_shunt = Config.Sensor.Ina.Shunts[idx][channel];
            const int16_t dc_v_max = static_cast<int16_t>(Config.Sensor.Ina.MaxBusVoltage_V * 1000.0f);
            const int16_t dc_v_min = 0;
            const float   max_i_ma = Config.Sensor.Ina.MaxshuntVoltage_mV / r_shunt;
            const int16_t dc_i_max = static_cast<int16_t>(max_i_ma);
            const int16_t dc_i_min = static_cast<int16_t>(-max_i_ma);
            const float   max_p_mw = Config.Sensor.Ina.MaxBusVoltage_V * max_i_ma;
            const int16_t dc_p_max = static_cast<int16_t>(max_p_mw / 10.0f);
            const int16_t dc_p_min = static_cast<int16_t>(-max_p_mw / 10.0f);
            const uint16_t mult_v = 1, div_v = 1000;
            const uint16_t mult_i = 1, div_i = 1000;
            const uint16_t mult_p = 1, div_p = 100;
            int16_t default_val = 0;

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &default_val);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MIN_VALUE_ID, (void*)&dc_v_min);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MAX_VALUE_ID, (void*)&dc_v_max);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MULTIPLIER_ID, (void*)&mult_v);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_DIVISOR_ID, (void*)&div_v);

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &default_val);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MIN_VALUE_ID, (void*)&dc_i_min);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MAX_VALUE_ID, (void*)&dc_i_max);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MULTIPLIER_ID, (void*)&mult_i);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_DIVISOR_ID, (void*)&div_i);

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &default_val);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MIN_VALUE_ID, (void*)&dc_p_min);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MAX_VALUE_ID, (void*)&dc_p_max);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MULTIPLIER_ID, (void*)&mult_p);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, (void*)&div_p);

            ezb_af_endpoint_add_cluster_desc(cluster_list, electrical_cluster);
            ezb_af_device_add_endpoint_desc(ep_list, cluster_list);
            //, EZB_AF_HA_PROFILE_ID, EZB_ZHA_METER_INTERFACE_DEVICE_ID);
        }
    }
}

void init_zb_config_reporting() {
    // --- DHT SZENZOROK JELENTÉSI SZABÁLYAI ---
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        //ZB Endpoint ID
        uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];
        
        ezb_zcl_reporting_info_t temp_rep_info = ezb_zcl_reporting_info_find(
            ep,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_CLUSTER_SERVER,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            0 // manuf_code (0 = EZB_ZCL_NOT_MANUFACTURER_SPECIFIC)
        );

        if (temp_rep_info != EZB_ZCL_INVALID_REPORTING_INFO) {
            ezb_zcl_attr_variable_t temp_delta = { .s16 = (int16_t)(Config.Reporting.Dht.ThresholdTemp * 100) };
            ezb_zcl_reporting_info_update(
                temp_rep_info,
                Config.Reporting.Dht.MinInterval,
                Config.Reporting.Dht.MaxInterval,
                &temp_delta
            );
        }

        ezb_zcl_reporting_info_t hum_rep_info = ezb_zcl_reporting_info_find(
            ep,
            EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            EZB_ZCL_CLUSTER_SERVER,
            EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID,
            0 // manuf_code
        );

        if (hum_rep_info != EZB_ZCL_INVALID_REPORTING_INFO) {
            ezb_zcl_attr_variable_t hum_delta = { .u16 = (uint16_t)(Config.Reporting.Dht.ThresholdHum * 100) };
            ezb_zcl_reporting_info_update(
                hum_rep_info,
                Config.Reporting.Dht.MinInterval,
                Config.Reporting.Dht.MaxInterval,
                &hum_delta
            );
        }
    }

    // --- INA3221 SZENZOROK (DC MÉRÉS) JELENTÉSI SZABÁLYAI ---
    for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
        for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
            uint8_t ep = Config.Zigbee.Ina.Endpoints[idx][channel];

            ezb_zcl_reporting_info_t v_rep_info = ezb_zcl_reporting_info_find(
                ep,
                EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                EZB_ZCL_CLUSTER_SERVER,
                EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID,
                0 // manuf_code
            );

            if (v_rep_info != EZB_ZCL_INVALID_REPORTING_INFO) {
                ezb_zcl_attr_variable_t v_delta = { .s16 = (int16_t)(Config.Reporting.Dc.ThresholdVolt * 1000)};

                ezb_zcl_reporting_info_update(
                    v_rep_info,
                    Config.Reporting.Dc.MinInterval,
                    Config.Reporting.Dc.MaxInterval,
                    &v_delta
                );
            }

            ezb_zcl_reporting_info_t i_rep_info = ezb_zcl_reporting_info_find(
                ep,
                EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                EZB_ZCL_CLUSTER_SERVER,
                EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,
                0 // manuf_code
            );

            if (i_rep_info != EZB_ZCL_INVALID_REPORTING_INFO) {
                ezb_zcl_attr_variable_t i_delta = { .s16 = (int16_t)(Config.Reporting.Dc.ThresholdCur * 1000)};
                ezb_zcl_reporting_info_update(
                    i_rep_info,
                    Config.Reporting.Dc.MinInterval,
                    Config.Reporting.Dc.MaxInterval,
                    &i_delta
                );
            }

            // Teljesítmény (szorzó: 100)
            ezb_zcl_reporting_info_t p_rep_info = ezb_zcl_reporting_info_find(
                ep,
                EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                EZB_ZCL_CLUSTER_SERVER,
                EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID,
                0 // manuf_code
            );

            if (p_rep_info != EZB_ZCL_INVALID_REPORTING_INFO) {
                ezb_zcl_attr_variable_t p_delta = { .s16 = (int16_t)(Config.Reporting.Dc.ThresholdPwr * 100) };

                ezb_zcl_reporting_info_update(
                    p_rep_info,
                    Config.Reporting.Dc.MinInterval,
                    Config.Reporting.Dc.MaxInterval,
                    &p_delta
                );
            }
        }
    }
}

bool ezb_app_signal_handler(const ezb_app_signal_t *signal_struct) {
    ezb_app_signal_type_t sig_type = ezb_app_signal_get_type(signal_struct);

    switch (sig_type) {
        case EZB_ZDO_SIGNAL_SKIP_STARTUP:
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            break;
        case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
            // Retrieve parameters using the ezbee API
            const ezb_bdb_signal_simple_params_t *params = 
                (const ezb_bdb_signal_simple_params_t *)ezb_app_signal_get_params(signal_struct);
            
            // Check status against EZB_BDB_STATUS_SUCCESS
            if (params && params->status == EZB_BDB_STATUS_SUCCESS) {
                is_zigbee_connected = true;
            } else {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
        }
        case EZB_ZDO_SIGNAL_LEAVE:
            is_zigbee_connected = false;
            break;
        default:
            break;
    }
    
    // Return true if handled, false otherwise
    return false; 
}

void init_zb_stack() {
    ESP_ERROR_CHECK(nvs_flash_init_partition("zb_storage"));
    
    // 1. Initialize the ezbee core
    ESP_ERROR_CHECK(ezb_core_init());
    
    // 2. Set primary channels. 0x07FFF800 represents all channels from 11 to 26
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(0x07FFF800));

    ezb_mem_config_t mem_config = {};
    mem_config.buffer_pool_size = 200;
    ezb_config_memory(&mem_config);

    ezb_af_device_desc_t ep_list = ezb_af_create_device_desc();

    ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee DHT endpoints");
    init_zb_ep_dht(ep_list);

    ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee DC endpoints");
    init_zb_ep_dc(ep_list);

    ESP_LOGI(Config.Logging.TagZigbee, "Registering Endpoints");
    ezb_af_device_desc_register(ep_list);

    ESP_LOGI(Config.Logging.TagZigbee, "Configuring Zigbee Reporting Rules");
    init_zb_config_reporting();

    // 3. Register the application signal handler using the ezbee API
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(ezb_app_signal_handler));

    ESP_LOGI(Config.Logging.TagZigbee, "Start Zigbee network");
    init_zb_network_up(); 
}

void zigbee_mainloop_task(void *pvParameters) {
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
    vTaskDelete(NULL);
}

void init_zb_network_up() {
    ESP_LOGI(Config.Logging.TagZigbee, "Starting Zigbee stack execution...");
    
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    
    xTaskCreate(zigbee_mainloop_task, "Zigbee_main", 4096, NULL, 5, NULL);

    ESP_LOGI(Config.Logging.TagZigbee, "Waiting for network connection...");
    while (!is_zigbee_connected) {
        ESP_LOGI(Config.Logging.TagZigbee, ".");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(Config.Logging.TagZigbee, "Network is UP and running!");
}

void dht_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DhtMs);

    while (true) {
        for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
            float temperature = 0.0;
            float humidity = 0.0;
            if (dht_read_float_data(DHT_TYPE_AM2301, Config.Sensor.Dht.Pins[idx], &humidity, &temperature) == ESP_OK) {
                if(Config.Logging.EnabledMeasurements) {
                  ESP_LOGI(Config.Logging.TagMeasurement, "DHT %d: %.2f°C %.0f%%", idx, temperature, humidity);
                }

                int16_t measured_temp = (int16_t)(temperature * 100);
                uint16_t measured_hum = (uint16_t)(humidity * 100);
                uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];

                if (is_zigbee_connected) {
                    esp_zigbee_lock_acquire(portMAX_DELAY);
                    ezb_zcl_set_attr_value(
                        ep, 
                        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                        EZB_ZCL_CLUSTER_SERVER, 
                        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
                        0, // manuf_code
                        &measured_temp,
                        false
                    );
                    ezb_zcl_set_attr_value(
                        ep,
                        EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                        EZB_ZCL_CLUSTER_SERVER, 
                        EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID,
                        0,
                        &measured_hum,
                        false
                    );
                    esp_zigbee_lock_release();
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void dc_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DcMs); // 1 másodperces precíz mérés

    while (true) {
        for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
            for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
                float voltage_V = 0.0;
                ezb_err_t err_v = ina3221_get_bus_voltage(
                    &ina3221_arr[idx],
                    (ina3221_channel_t)channel,
                    &voltage_V
                );
                
                float shuntVoltage_mV = 0.0f;
                float current_mA = 0.0; 
                ezb_err_t err_i = ina3221_get_shunt_value(
                    &ina3221_arr[idx],
                    (ina3221_channel_t)channel,
                    &shuntVoltage_mV,
                    &current_mA
                );
                
                if ((err_v == ESP_OK) && (err_i == ESP_OK)) { // measurement success
                    //const float r_shunt = Config.Sensor.Ina.Shunts[idx][channel];
                    //current_mA = shuntVoltage_mV / r_shunt;
                    float power_mW = voltage_V * current_mA;

                    if(Config.Logging.EnabledMeasurements) {
                        ESP_LOGI(Config.Logging.TagMeasurement, "DC %d/%d: %.4fV %7.1fmA", idx, channel, voltage_V, current_mA);
                    }

                    int16_t measured_v = (int16_t)(voltage_V * 1000.0f);
                    int16_t measured_i = (int16_t)(current_mA);
                    int16_t measured_p = (int16_t)(power_mW / 10.0); //cW because of int16

                    if (is_zigbee_connected) {
                        esp_zigbee_lock_acquire(portMAX_DELAY);
                        
                        uint8_t ep_id = Config.Zigbee.Ina.Endpoints[idx][channel];
                        
                        // Feszültség
                        ezb_zcl_set_attr_value(
                            ep_id, 
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_CLUSTER_SERVER, 
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID,
                            0, // manuf_code (0 for standard ZCL attributes)
                            &measured_v,
                            false
                        );
                        
                        // Áram
                        ezb_zcl_set_attr_value(
                            ep_id,
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_CLUSTER_SERVER, 
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,
                            0,
                            &measured_i,
                            false
                        );
                        
                        // Teljesítmény
                        ezb_zcl_set_attr_value(
                            ep_id,
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_CLUSTER_SERVER, 
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID,
                            0,
                            &measured_p,
                            false
                        );
                        
                        esp_zigbee_lock_release();
                    }
                } else { // Measurement failed
                    ESP_LOGW(Config.Logging.TagMeasurement, "Measure DC %d/%d failed: %d, %d",
                      idx, channel,
                      err_v, err_i );
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void button_task(void *pvParameters) {
    uint64_t pressStartTime = 0;
    bool isPressed = false;
    
    while (true) {
        if (gpio_get_level(Config.Device.ButtonPin) == 0) {
            if (!isPressed) {
                isPressed = true;
                pressStartTime = millis(); 
            } else if ((millis() - pressStartTime) > 3000) {
                ESP_LOGW(Config.Logging.TagZigbee, "Resetting Zigbee to factory and rebooting in 1s.");
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_zigbee_lock_acquire(portMAX_DELAY);
                esp_zigbee_factory_reset();
                esp_zigbee_lock_release();
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        } else {
            isPressed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

uint64_t millis() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}