// Zigbee2MQTT external converter: PowerBox-INA15-DHT2
//
// Install: copy to <zigbee2mqtt>/data/external_converters/powerbox_ina15_dht2.mjs
//          restart Zigbee2MQTT, then remove + re-pair the device (or use "Reconfigure").
//
// Endpoint map MUST match config.h:
//   EP 0x01                  -> Board Temperature; Basic cluster
//   EP 0x02, 0x03            -> DHT #1, #2 (temperature + humidity)
//   EP 0x10..0x12            -> INA3221 #1 CH1..3
//   EP 0x13..0x15            -> INA3221 #2 CH1..3
//   EP 0x16..0x18            -> INA3221 #3 CH1..3
//   EP 0x19, 0x1A, 0x1B      -> INA3221 #4 CH1..3
//   EP 0x20..0x22            -> INA3221 #5 CH1..3
//
// DC units on the device side (haElectricalMeasurement, DC attributes):
//   dcVoltage 0x0100  int16  mV   (divisor 1000)  -> V
//   dcCurrent 0x0103  int16  mA   (divisor 1000)  -> A
//   dcPower   0x0106  int16  cW   (divisor 100)   -> W  (1 raw unit = 10 mW)

import * as m from 'zigbee-herdsman-converters/lib/modernExtend';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';
import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as utils from 'zigbee-herdsman-converters/lib/utils';

const e = exposes.presets;

// ---------------------------------------------------------------- endpoint map
const BOARD_ENDPOINT = {board: 1};
const DHT_ENDPOINTS = {dht1: 2, dht2: 3};

const INA_EPS = [
    [0x10, 0x11, 0x12],
    [0x13, 0x14, 0x15],
    [0x16, 0x17, 0x18],
    [0x19, 0x1a, 0x1b],
    [0x20, 0x21, 0x22],
];

// names like ina1ch1 .. ina5ch3  ->  properties voltage_ina1ch1, current_ina1ch1, power_ina1ch1
const DC_ENDPOINTS = {};
INA_EPS.forEach((channels, mod) => {
    channels.forEach((ep, ch) => {
        DC_ENDPOINTS[`ina${mod + 1}ch${ch + 1}`] = ep;
    });
});

// ---------------------------------------------------------------- DC attributes
const ATTR_DC_VOLTAGE = 0x0100;
const ATTR_DC_CURRENT = 0x0103;
const ATTR_DC_POWER = 0x0106;
const TYPE_INT16 = 0x29;

// ---------------------------------------------------------------- fromZigbee
const fzDcMeasurement = {
    cluster: 'haElectricalMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        // Herdsman may deliver known attributes by name, unknown ones by numeric ID.
        const get = (name, id) => (msg.data[name] !== undefined ? msg.data[name] : msg.data[id]);
        const round = (value, digits) => Math.round(value * 10 ** digits) / 10 ** digits;

        const payload = {};
        const v = get('dcVoltage', ATTR_DC_VOLTAGE);
        const i = get('dcCurrent', ATTR_DC_CURRENT);
        const p = get('dcPower', ATTR_DC_POWER);

        if (v !== undefined) payload[utils.postfixWithEndpointName('voltage', msg, model, meta)] = round(v / 1000, 3);
        if (i !== undefined) payload[utils.postfixWithEndpointName('current', msg, model, meta)] = round(i / 1000, 3);
        if (p !== undefined) payload[utils.postfixWithEndpointName('power', msg, model, meta)] = round(p / 100, 2);

        return Object.keys(payload).length ? payload : undefined;
    },
};

// ---------------------------------------------------------------- exposes
const dcExposes = [];
for (const name of Object.keys(DC_ENDPOINTS)) {
    dcExposes.push(
        e.voltage().withEndpoint(name),
        e.current().withEndpoint(name),
        e.power().withEndpoint(name),
    );
}

// ---------------------------------------------------------------- definition
const definition = {
    zigbeeModel: ['PowerBox-INA15-DHT2'],
    model: 'PowerBox-INA15-DHT2',
    vendor: 'DIY',
    description: '5x INA3221 (15 DC channels: V/I/P) + 2x DHT temperature/humidity (ESP32-C6)',

    extend: [
        m.deviceEndpoints({endpoints: {...BOARD_ENDPOINT, ...DHT_ENDPOINTS, ...DC_ENDPOINTS}}),
        // min/max in seconds, change in raw ZCL units (0.01 degC / 0.01 %RH)
        m.temperature({endpointNames: ['board', ...Object.keys(DHT_ENDPOINTS)], reporting: {min: 5, max: 60, change: 50}}),
        m.humidity({endpointNames: Object.keys(DHT_ENDPOINTS), reporting: {min: 5, max: 60, change: 200}}),
    ],

    fromZigbee: [fzDcMeasurement],
    toZigbee: [],
    exposes: dcExposes,

    // bind + reporting for the 15 DC endpoints (DHT is configured by the modernExtend above)
    configure: async (device, coordinatorEndpoint) => {
        const failures = [];
        for (const [name, id] of Object.entries(DC_ENDPOINTS)) {
            const endpoint = device.getEndpoint(id);
            if (!endpoint) {
                failures.push(`${name}: endpoint ${id} not found`);
                continue;
            }
            try {
                await reporting.bind(endpoint, coordinatorEndpoint, ['haElectricalMeasurement']);
                await endpoint.configureReporting('haElectricalMeasurement', [
                    {
                        attribute: {ID: ATTR_DC_VOLTAGE, type: TYPE_INT16},
                        minimumReportInterval: 2,
                        maximumReportInterval: 30,
                        reportableChange: 60, // 60 mV
                    },
                    {
                        attribute: {ID: ATTR_DC_CURRENT, type: TYPE_INT16},
                        minimumReportInterval: 2,
                        maximumReportInterval: 30,
                        reportableChange: 30, // 30 mA
                    },
                    {
                        attribute: {ID: ATTR_DC_POWER, type: TYPE_INT16},
                        minimumReportInterval: 2,
                        maximumReportInterval: 30,
                        reportableChange: 5, // 50 mW
                    },
                ]);
            } catch (error) {
                failures.push(`${name} (EP ${id}): ${error.message}`);
            }
        }
        if (failures.length > 0) {
            throw new Error(`DC reporting setup failed: ${failures.join(' | ')}`);
        }
    },
};

export default definition;
