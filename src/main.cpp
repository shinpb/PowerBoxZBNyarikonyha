#include <Arduino.h>
#include "Zigbee.h"
#include <Wire.h>
#include "DHT.h"
#include "Adafruit_INA3221.h"

#define SerialPrint 1

static const uint8_t dht_n = 2;
DHT dht_arr[dht_n] = {
    { 20, DHT22 },
    { 19, DHT22 }
};
ZigbeeTempSensor zbTempSensors[ dht_n ] =  {
	{0x30}, {0x31}
};

static const uint8_t ina3221_n = 5;
Adafruit_INA3221 ina3221_arr[ ina3221_n ];
static const float ina3221_shunt[ ina3221_n * 3 ] = {
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
	0.1, 0.1, 0.1,
};
constexpr float INA3221_MAX_BUS_VOLTAGE_V = 26.0f;     // Max busz feszültség: 26V
constexpr float INA3221_MAX_SHUNT_VOLTAGE_MV = 163.84f; // Max sönt feszültség: 163.84mV
ZigbeeElectricalMeasurement zbElectricalMeasurements[ ina3221_n * 3 ] = {
	{0x10}, {0x11}, {0x12},
	{0x13}, {0x14}, {0x15},
	{0x16}, {0x17}, {0x18},
	{0x19}, {0x1A}, {0x1B},
	{0x20}, {0x21}, {0x22},
};

uint8_t button = BOOT_PIN;

// Forward declarations
void setup();
void loop();
void init_sensors_dht();
void init_sensors_ina();
void init_zb_dht();
void init_zb_dc();
void zb_network_up();
void measure_dht();
void measure_dc();

// ESP-IDF main entry point
extern "C" void app_main() {
    // 1. Initialize Arduino core peripherals and drivers
    initArduino();

    // 2. Call standard Arduino setup
    setup();

    // 3. Main loop execution
    while (true) {
        loop();
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to prevent Watchdog Timer (WDT) triggers
    }
}

void setup() {
  Serial.begin(9600);
  Serial.printf("\n\n\n\n\n");

  Serial.printf("Init DHT sensors\n");
  init_sensors_dht();
  
  Serial.printf("Init INA sensors\n");
  init_sensors_ina();

  Serial.printf("Init Zigbee DHT endpoints\n");
  init_zb_dht();

  Serial.printf("Init Zigbee DC endpoints\n");
  init_zb_dc();

  Serial.printf("Start Zigbee network\n");
  zb_network_up();

  Serial.printf("Setup done\n");
}


void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  measure_dht();
  
  measure_dc();
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

    // Optional: set Zigbee device name and model
    zbTempSensor.setManufacturerAndModel("Espressif", "DHT22");

    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor.setMinMaxValue(-40, 80);

    // Optional: Set default (initial) value for the temperature sensor to 10.0°C to match the minimum temperature measurement value
    //zbTempSensor.setDefaultValue(10.0);

    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTempSensor.setTolerance(1);

    // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
    // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_UNKNOWN);

    // Add humidity cluster to the temperature sensor device with min, max, tolerance and default values
    zbTempSensor.addHumiditySensor(0, 100, 1, 0.0);

    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(&zbTempSensor);
  }
}

void init_zb_dc() {
  for (int idx = 0; idx < ina3221_n; idx++) {
    for (uint8_t channel = 0; channel < 3; channel++) {
      const float r_shunt = ina3221_shunt[idx * 3 + channel];
      ZigbeeElectricalMeasurement& zbElectricalMeasurement = zbElectricalMeasurements[idx * 3 + channel];
      Serial.printf("\tInit DC %d (EP 0x%X)\n", idx * 3 + channel, zbElectricalMeasurement.getEndpoint());
      zbElectricalMeasurement.setManufacturerAndModel("Espressif", "ZigbeeElectricalMeasurementDC");
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
      const int16_t dc_p_max = static_cast<int16_t>(max_p_mw);
      const int16_t dc_p_min = static_cast<int16_t>(-max_p_mw);
      zbElectricalMeasurement.setDCMinMaxValue(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, dc_p_min, dc_p_max);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, 1, 1000);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, 1, 1000);
      zbElectricalMeasurement.setDCMultiplierDivisor(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, 1, 1000);

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

void measure_dht() {
  for (int idx = 0; idx < dht_n; idx++) {
    DHT& dht=dht_arr[idx];
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      Serial.printf("Failed to read from DHT %d!", idx);
      continue;
    }

    ZigbeeTempSensor& zbTempSensor = zbTempSensors[idx];
    zbTempSensor.setTemperature(temperature);
    zbTempSensor.setHumidity(humidity);
    zbTempSensor.reportTemperature();
  }
}

void measure_dc() {
  for (int idx = 0; idx < ina3221_n; idx++) {
    Adafruit_INA3221& ina3221 = ina3221_arr[idx];
    for (uint8_t channel = 0; channel < 3; channel++) {
      float voltageV = ina3221.getBusVoltage(channel);
      float currentA = ina3221.getCurrentAmps(channel);
      ZigbeeElectricalMeasurement& zbElectricalMeasurement = zbElectricalMeasurements[idx * 3 + channel];
      zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE, voltageV * 1000);
      zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT, currentA * 1000);
      zbElectricalMeasurement.setDCMeasurement(ZIGBEE_DC_MEASUREMENT_TYPE_POWER, voltageV * currentA * 1000);
      zbElectricalMeasurement.reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_VOLTAGE);
      zbElectricalMeasurement.reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_CURRENT);
      zbElectricalMeasurement.reportDC(ZIGBEE_DC_MEASUREMENT_TYPE_POWER);
    }
  }
}
