#include <Arduino.h>
#include "Zigbee.h"
#include <Wire.h>
#include "DHT.h"
#include "Adafruit_INA3221.h"

// ******** Global config ********

// ID
static const char* const Manufacturer = "Espressif";
static const char* const Model = "PowerBox-INA15-DHT2";
// ZB
static const uint8_t ZigBeeDisabled = 0;
// Measurement
static const int MEASURE_DHT_MS = 3000;
static const int MEASURE_DC_MS = 1000;
// Button
static const uint8_t button = BOOT_PIN;
// Serial
static const uint8_t MeasurementsToSerial = 0;
static const uint8_t ReportsToSerial = 1;



// ******** Sensor config ********

// DHT
static const uint8_t dht_n = 2;
DHT dht_arr[dht_n] = {
    { 20, DHT22 },
    { 19, DHT22 }
};
//INA
static const uint8_t ina3221_n = 5;
Adafruit_INA3221 ina3221_arr[ ina3221_n ];
static const float ina3221_shunt[ ina3221_n * 3 ] = {
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
};



// ******** ZigBee endpoint config ********

// DHT
ZigbeeTempSensor zbTempSensors[ dht_n ] =  {
	{0x01}, {0x02}
};
// INA
constexpr float INA3221_MAX_BUS_VOLTAGE_V = 26.0f;     // Max busz feszültség: 26V
constexpr float INA3221_MAX_SHUNT_VOLTAGE_MV = 163.84f; // Max sönt feszültség: 163.84mV
ZigbeeElectricalMeasurement zbElectricalMeasurements[ ina3221_n * 3 ] = {
	{0x10}, {0x11}, {0x12},
	{0x13}, {0x14}, {0x15},
	{0x16}, {0x17}, {0x18},
	{0x19}, {0x1A}, {0x1B},
	{0x20}, {0x21}, {0x22},
};



// ******** Reporting config ********

// Global
const unsigned long MAX_REPORT_INTERVAL = 60000;
const unsigned long MIN_REPORT_INTERVAL = 5000;
// DHT
static const float REPORT_THRESHOLD_DHT_TEMP = 0.5; // °C
static const float REPORT_THRESHOLD_DHT_HUM = 2.0; // %
// INA
static const float REPORT_THRESHOLD_DC_VOLT = 0.05; // V 
static const float REPORT_THRESHOLD_DC_CUR = 0.01; // A
//ReportState 
struct SensorState {
    float lastVal1 = -1000;
    float lastVal2 = -1000;
    unsigned long lastReportTime = 0;
};
SensorState dhtStates[dht_n];
SensorState inaStates[ina3221_n * 3];



// Forward declarations
void setup();
void loop();
void init_sensors_dht();
void init_sensors_ina();
void init_zb_dht();
void init_zb_dc();
void zb_network_up();
void dht_measurement_task(void*);
void dc_measurement_task(void*);
void button_task(void*);

// ESP-IDF main entry point
/*extern "C" void app_main() {
    // 1. Initialize Arduino core peripherals and drivers
    initArduino();

    // 2. Call standard Arduino setup
    setup();

    // 3. Main loop execution
    while (true) {
        loop();
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to prevent Watchdog Timer (WDT) triggers
    }
}*/

void setup() {
  Serial.begin(9600);
  Serial.printf("\n\n");
  
  Serial.println("=== ZIGBEE MEMORY LIMITS (PRE-COMPILED CORE) ===");

#ifdef CONFIG_ZB_CONFIG_MAX_EP_NUMBER
    Serial.printf("MAX_EP_NUMBER: %d\n", CONFIG_ZB_CONFIG_MAX_EP_NUMBER);
#else
    Serial.println("CONFIG_ZB_CONFIG_MAX_EP_NUMBER: NOT DEFINED in sdkconfig.h (Using build defaults)");
#endif

#ifdef CONFIG_ZB_CONFIG_BINDING_TABLE_SIZE
    Serial.printf("BINDING_TABLE_SIZE: %d\n", CONFIG_ZB_CONFIG_BINDING_TABLE_SIZE);
#else
    Serial.println("CONFIG_ZB_CONFIG_BINDING_TABLE_SIZE: NOT DEFINED in sdkconfig.h (Using build defaults)");
#endif

  Serial.println("================================================");

  Serial.printf("Init DHT sensors\n");
  init_sensors_dht();
  
  Serial.printf("Init INA sensors\n");
  init_sensors_ina();

  if(!ZigBeeDisabled){
    pinMode(button, INPUT_PULLUP);

    Serial.printf("Init Zigbee DHT endpoints\n");
    init_zb_dht();

    Serial.printf("Init Zigbee DC endpoints\n");
    init_zb_dc();

    Serial.printf("Start Zigbee network\n");
    zb_network_up();
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

  xTaskCreate(
      button_task, 
      "Task_Button",  
      2048,
      NULL, 
      1,
      NULL
  );

  Serial.printf("Setup done\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void init_sensors_dht() {
  for (int idx = 0; idx < dht_n; idx++) {
    Serial.printf("\tInit DHT %d\n", idx);
    dht_arr[idx].begin();
  }
}

void init_sensors_ina() {
  for (int idx = 0; idx < ina3221_n; idx++) {
    // ~/.arduino15/packages/esp32/hardware/esp32/3.3.11/variants/esp32c6/pins_arduino.h
    //SDA = 23;
    //SCL = 22;
    //SDA1 = 6;
    //SCL1 = 7;
    //SCL / SDA: Wire(22, 23), Wire1(7, 6)
    TwoWire* I2C_Bus = (idx < 4) ? (&Wire) : (&Wire1);
    uint8_t i2c_addr = 0x40 + (idx % 4);
    Adafruit_INA3221& ina3221 = ina3221_arr[idx];
    if (!ina3221.begin(i2c_addr, I2C_Bus)) {
      Serial.printf("\tFailed to find INA3221 chip %d (Bus %d 0x%X)\n",
        idx, I2C_Bus->getBusNum(), i2c_addr);
      delay(1000);
      ESP.restart();
    }
      Serial.printf("\tFound INA3221 chip %d (Bus %d 0x%X)\n",
        idx, I2C_Bus->getBusNum(), i2c_addr);

    ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);

    for (uint8_t channel = 0; channel < 3; channel++) {
      const float r_shunt = ina3221_shunt[idx * 3 + channel];
      ina3221.setShuntResistance(channel, r_shunt);
    }
  }
}

void init_zb_dht() {
  for (int idx = 0; idx < dht_n; idx++) {
    ZigbeeTempSensor& zbTempSensor = zbTempSensors[idx];
    Serial.printf("\tInit DHT %d (EP 0x%X)\n", idx, zbTempSensor.getEndpoint());

    zbTempSensor.setManufacturerAndModel(Manufacturer, Model);

    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor.setMinMaxValue(-40, 80);

    // Optional: Set default (initial) value for the temperature sensor to 10.0°C to match the minimum temperature measurement value
    //zbTempSensor.setDefaultValue(10.0);

    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTempSensor.setTolerance(REPORT_THRESHOLD_DHT_TEMP);

    // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
    // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_UNKNOWN);

    // Add humidity cluster to the temperature sensor device with min, max, tolerance and default values
    zbTempSensor.addHumiditySensor(0, 100, REPORT_THRESHOLD_DHT_HUM, 0.0);

    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(&zbTempSensor);
  }
}

void init_zb_dc() {
  for (int idx = 0; idx < ina3221_n; idx++) {
    for (uint8_t channel = 0; channel < 3; channel++) {
      uint8_t flat_idx = idx * 3 + channel;
      const float r_shunt = ina3221_shunt[flat_idx];
      ZigbeeElectricalMeasurement& zbElectricalMeasurement = zbElectricalMeasurements[flat_idx];
      zbElectricalMeasurement.setManufacturerAndModel(Manufacturer, Model);
      Serial.printf("\tInit DC %d (EP 0x%X)\n", flat_idx, zbElectricalMeasurement.getEndpoint());
      zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
      zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT);
      zbElectricalMeasurement.addDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_POWER);
      const int16_t dc_v_max = static_cast<int16_t>(INA3221_MAX_BUS_VOLTAGE_V * 1000.0f);
      const int16_t dc_v_min = 0;
      zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, dc_v_min, dc_v_max);
      const float   max_i_ma = INA3221_MAX_SHUNT_VOLTAGE_MV / r_shunt;
      const int16_t dc_i_max = static_cast<int16_t>(max_i_ma);
      const int16_t dc_i_min = static_cast<int16_t>(-max_i_ma);
      zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, dc_i_min, dc_i_max);
      const float   max_p_mw = INA3221_MAX_BUS_VOLTAGE_V * max_i_ma; // V * mA = mW
      const int16_t dc_p_max = static_cast<int16_t>(max_p_mw / 10.0f);
      const int16_t dc_p_min = static_cast<int16_t>(-max_p_mw / 10.0f);
      zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, dc_p_min, dc_p_max);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, 1, 1000);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 1, 1000);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 1, 100);

      Zigbee.addEndpoint(&zbElectricalMeasurement);
    }
  }
}

void zb_network_up() {
  if (!Zigbee.begin()) {
    Serial.println("\tZigbee failed to start!");
    delay(1000);
    Serial.println("\tRebooting...");
    ESP.restart();
  } else {
    Serial.println("\tZigbee started successfully!");
  }
  Serial.println("\tConnecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\tConnected");
}

void dht_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MEASURE_DHT_MS);

    unsigned long startMillis = millis();
    for (int idx = 0; idx < dht_n; idx++) {
        dhtStates[idx].lastReportTime = startMillis - MAX_REPORT_INTERVAL;
    }

    while (true) {
        unsigned long currentMillis = millis();
        for (int idx = 0; idx < dht_n; idx++) {
            float humidity = dht_arr[idx].readHumidity();
            float temperature = dht_arr[idx].readTemperature();
            
            if (!isnan(humidity) && !isnan(temperature)) {
                bool tempChanged = fabs(temperature - dhtStates[idx].lastVal1) > REPORT_THRESHOLD_DHT_TEMP;
                bool humChanged = fabs(humidity - dhtStates[idx].lastVal2) > REPORT_THRESHOLD_DHT_HUM;
                
                if(MeasurementsToSerial) {
                  Serial.printf("DHT %d: %.2f°C %.0f%%\n", idx, temperature, humidity);
                }

                unsigned long elapsed = currentMillis - dhtStates[idx].lastReportTime;
                bool timeToReport = elapsed >= MAX_REPORT_INTERVAL;
                bool canReport = elapsed >= MIN_REPORT_INTERVAL;

                if ((tempChanged || humChanged || timeToReport) && canReport) {
                    if (!ZigBeeDisabled) {
                      zbTempSensors[idx].setTemperature(temperature);
                      zbTempSensors[idx].setHumidity(humidity);
                      esp_err_t err_t = zbTempSensors[idx].reportTemperature();
                      esp_err_t err_h = zbTempSensors[idx].reportHumidity();
                      if(ReportsToSerial) {
                        Serial.printf( "Report DHT %d: EP 0x%X: %d, %d\n",
                          idx, zbTempSensors[idx].getEndpoint(),
                          err_t, err_h );
                      }
                    }
                    dhtStates[idx].lastVal1 = temperature;
                    dhtStates[idx].lastVal2 = humidity;
                    dhtStates[idx].lastReportTime = currentMillis;
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void dc_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MEASURE_DC_MS); // 1 másodperces precíz mérés

    unsigned long startMillis = millis();
    for (int idx = 0; idx < ina3221_n * 3; idx++) {
        inaStates[idx].lastReportTime = startMillis - MAX_REPORT_INTERVAL;
    }

    while (true) {
        unsigned long currentMillis = millis();
        for (int idx = 0; idx < ina3221_n; idx++) {
            for (uint8_t channel = 0; channel < 3; channel++) {
                int flat_idx = (idx * 3) + channel;
                
                float voltageV = ina3221_arr[idx].getBusVoltage(channel);
                float currentA = ina3221_arr[idx].getCurrentAmps(channel);
                float powerW = voltageV * currentA;
               
                if(MeasurementsToSerial) {
                  Serial.printf("DC %d/%d: %.4fV %.4fA\n", idx, channel, voltageV, currentA);
                }

                bool vChanged = fabs(voltageV - inaStates[flat_idx].lastVal1) > REPORT_THRESHOLD_DC_VOLT;
                bool iChanged = fabs(currentA - inaStates[flat_idx].lastVal2) > REPORT_THRESHOLD_DC_CUR;
                
                unsigned long elapsed = currentMillis - inaStates[flat_idx].lastReportTime;
                bool timeToReport = elapsed >= MAX_REPORT_INTERVAL;
                bool canReport = elapsed >= MIN_REPORT_INTERVAL;

                if ((vChanged || iChanged || timeToReport) && canReport) {
                    if (!ZigBeeDisabled) {
                        zbElectricalMeasurements[flat_idx].setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, voltageV * 1000);
                        zbElectricalMeasurements[flat_idx].setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, currentA * 1000);
                        zbElectricalMeasurements[flat_idx].setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, powerW * 100);
                        
                        esp_err_t err_v = zbElectricalMeasurements[flat_idx].reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
                        esp_err_t err_c = zbElectricalMeasurements[flat_idx].reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT);
                        esp_err_t err_p = zbElectricalMeasurements[flat_idx].reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_POWER);
                        vTaskDelay(pdMS_TO_TICKS(50));
                        if(ReportsToSerial) {
                          Serial.printf( "Report DC %d/%d: EP 0x%X: %d, %d, %d\n",
                            idx, channel, zbElectricalMeasurements[flat_idx].getEndpoint(),
                            err_v, err_c, err_p );
                        }
                    }
                    inaStates[flat_idx].lastVal1 = voltageV;
                    inaStates[flat_idx].lastVal2 = currentA;
                    inaStates[flat_idx].lastReportTime = currentMillis;
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void button_task(void *pvParameters) {
    unsigned long pressStartTime = 0;
    bool isPressed = false;
    
    while (true) {
        if (digitalRead(button) == LOW) {
            if (!isPressed) {
                isPressed = true;
                pressStartTime = millis(); 
            } else if ((millis() - pressStartTime) > 3000) {
                Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
                vTaskDelay(pdMS_TO_TICKS(500));
                Zigbee.factoryReset();
                vTaskDelay(pdMS_TO_TICKS(500));
                ESP.restart();
            }
        } else {
            isPressed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}
