// Zigbee2MQTT external converter: PowerBox-INA15-DHT2
//
// Install: copy to <zigbee2mqtt>/data/external_converters/powerbox_ina15_dht2.mjs
//          restart Zigbee2MQTT, then remove + re-pair the device (or use "Reconfigure").
//
// Endpoint map and reporting values MUST match the firmware (config.h: Config.Reporting and the endpoint tables;
// the firmware is the authoritative source, this file is a copy, D4):
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
//
// Unavailable sensor: the device writes the ZCL "invalid" value into the attribute
//   temperature 0x8000 (-32768), humidity 0xFFFF, DC int16 0x8000 (-32768)
// and this converter publishes null for it (Home Assistant: "unknown"), instead of -327.68 / 655.35 / -32.768.

import {Zcl} from 'zigbee-herdsman';
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

// ---------------------------------------------------------------- reporting (mirror of Config.Reporting)
// min/max in seconds, change in raw ZCL units. The firmware writes the same values as factory defaults, so a
// Reconfigure from Z2M does not change anything unless one of these constants is edited.
const REPORTING = {
    board: {min: 5, max: 60, change: 50}, // 0.5 degC
    dhtTemperature: {min: 5, max: 60, change: 50}, // 0.5 degC
    dhtHumidity: {min: 5, max: 60, change: 200}, // 2 %RH
    dc: {min: 2, max: 30}, // thresholds per channel: DC_THRESHOLDS
    fault: {min: 0, max: 3600, change: 1},
};

// DC thresholds per channel in raw units (mV, mA, cW = 10 mW), mirror of Config.Reporting.Dc.Thresholds.
// ina1..ina4: 60 mV / 30 mA / 300 mW; ina5 (batteries, charger; 10 mOhm shunt): 60 mV / 100 mA / 1 W.
const DC_THRESHOLDS = {};
for (const name of Object.keys(DC_ENDPOINTS)) {
    DC_THRESHOLDS[name] = name.startsWith('ina5') ? {voltage: 60, current: 100, power: 100} : {voltage: 60, current: 30, power: 30};
}

// ---------------------------------------------------------------- DC attributes
const ATTR_DC_VOLTAGE = 0x0100;
const ATTR_DC_CURRENT = 0x0103;
const ATTR_DC_POWER = 0x0106;
const TYPE_INT16 = 0x29;

// ZCL invalid measurement values (must match zcl_values.h)
const INVALID_INT16 = -32768; // 0x8000: temperature, DC voltage/current/power
const INVALID_UINT16 = 0xffff; // humidity
const isInvalid = (raw, invalidValue) =>
    raw === null || (typeof raw === 'number' && (Number.isNaN(raw) || raw === invalidValue));

// ---------------------------------------------------------------- fromZigbee
const fzDcMeasurement = {
    cluster: 'haElectricalMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        // Herdsman may deliver known attributes by name, unknown ones by numeric ID.
        const get = (name, id) => (msg.data[name] !== undefined ? msg.data[name] : msg.data[id]);
        // invalid raw value -> null; otherwise scaled and rounded
        const scale = (raw, divisor, digits) =>
            isInvalid(raw, INVALID_INT16) ? null : Math.round((raw / divisor) * 10 ** digits) / 10 ** digits;

        const payload = {};
        const v = get('dcVoltage', ATTR_DC_VOLTAGE);
        const i = get('dcCurrent', ATTR_DC_CURRENT);
        const p = get('dcPower', ATTR_DC_POWER);

        if (v !== undefined) payload[utils.postfixWithEndpointName('voltage', msg, model, meta)] = scale(v, 1000, 3);
        if (i !== undefined) payload[utils.postfixWithEndpointName('current', msg, model, meta)] = scale(i, 1000, 3);
        if (p !== undefined) payload[utils.postfixWithEndpointName('power', msg, model, meta)] = scale(p, 100, 2);

        return Object.keys(payload).length ? payload : undefined;
    },
};

// ---------------------------------------------------------------- device fault indication (EP 0x01)
// Standard Binary Input (genBinaryInput, presentValue = any fault) plus a manufacturer-specific
// cluster with the fault mask (bitmap32). Must match health.h (HF_SENS_*, HF_BUS_* << 16) and zcl_values.h:
//   bits 0..15  sensor faults: 0 board, 1..2 DHT1..2, 8..12 INA1..5
//   bits 16..23 bus faults:    16 I2C init, 17 I2C bus 0, 18 I2C bus 1
const DIAG_CLUSTER = 'powerboxDiag';
const DIAG_CLUSTER_ID = 0xfc11;
const DIAG_ATTR_FAULT_MASK = 0x0000;
const TYPE_BITMAP32 = 0x1b;

const FAULT_BITS = [
    [0, 'board'], [1, 'dht1'], [2, 'dht2'],
    [8, 'ina1'], [9, 'ina2'], [10, 'ina3'], [11, 'ina4'], [12, 'ina5'],
    [16, 'i2c_init'], [17, 'i2c_bus0'], [18, 'i2c_bus1'],
];
const faultText = (mask) => {
    const names = FAULT_BITS.filter(([bit]) => (mask >>> bit) & 1).map(([, name]) => name);
    return names.length ? names.join(', ') : 'ok';
};

const fzFault = {
    cluster: 'genBinaryInput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.presentValue === undefined) return undefined;
        return {fault: Boolean(msg.data.presentValue)};
    },
};

const convertFaultMask = (model, msg, publish, options, meta) => {
    const raw = msg.data.faultMask !== undefined ? msg.data.faultMask : msg.data[DIAG_ATTR_FAULT_MASK];
    if (raw === undefined || raw === null || Number.isNaN(raw)) return undefined;
    return {fault_mask: raw, fault_detail: faultText(raw)};
};

const fzFaultMask = {
    cluster: DIAG_CLUSTER,
    type: ['attributeReport', 'readResponse'],
    convert: convertFaultMask,
};

// Refresh button / "get": reads the fault state and the mask from the board endpoint
const tzFault = {
    key: ['fault', 'fault_detail'],
    convertGet: async (entity, key, meta) => {
        const board = meta.device.getEndpoint(BOARD_ENDPOINT.board);
        await board.read('genBinaryInput', ['presentValue']);
        await board.read(DIAG_CLUSTER, ['faultMask']);
    },
};

// ---------------------------------------------------------------- temperature / humidity
// The modernExtend converters would publish the invalid raw values as numbers (-327.68 / 655.35).
// Wrap their fromZigbee converters: an invalid measuredValue is published as null.
const nullOnInvalid = (extend, cluster, property, invalidValue) => {
    extend.fromZigbee = extend.fromZigbee.map((converter) => {
        if (converter.cluster !== cluster) return converter;
        return {
            ...converter,
            convert: (model, msg, publish, options, meta) => {
                if (msg.data.measuredValue !== undefined && isInvalid(msg.data.measuredValue, invalidValue)) {
                    return {[utils.postfixWithEndpointName(property, msg, model, meta)]: null};
                }
                return converter.convert(model, msg, publish, options, meta);
            },
        };
    });
    return extend;
};

// The board and the DHT temperature share the endpoint list of one extend, so their reporting must be equal.
const temperatureExtend = nullOnInvalid(
    m.temperature({
        endpointNames: ['board', ...Object.keys(DHT_ENDPOINTS)],
        reporting: {min: REPORTING.dhtTemperature.min, max: REPORTING.dhtTemperature.max, change: REPORTING.dhtTemperature.change},
    }),
    'msTemperatureMeasurement', 'temperature', INVALID_INT16,
);
const humidityExtend = nullOnInvalid(
    m.humidity({
        endpointNames: Object.keys(DHT_ENDPOINTS),
        reporting: {min: REPORTING.dhtHumidity.min, max: REPORTING.dhtHumidity.max, change: REPORTING.dhtHumidity.change},
    }),
    'msRelativeHumidity', 'humidity', INVALID_UINT16,
);

// ---------------------------------------------------------------- exposes
const dcExposes = [
    exposes.binary('fault', exposes.access.STATE_GET, true, false)
        .withDescription('Any sensor or bus fault is active')
        .withCategory('diagnostic'),
    exposes.text('fault_detail', exposes.access.STATE_GET)
        .withDescription('Faulty units (board, dhtN, inaN, i2c_*) or "ok"')
        .withCategory('diagnostic'),
];
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
        m.deviceAddCustomCluster(DIAG_CLUSTER, {
            ID: DIAG_CLUSTER_ID,
            name: DIAG_CLUSTER,
            attributes: {
                faultMask: {name: 'faultMask', ID: DIAG_ATTR_FAULT_MASK, type: Zcl.DataType.BITMAP32},
            },
            commands: {},
            commandsResponse: {},
        }),
        temperatureExtend,
        humidityExtend,
    ],

    fromZigbee: [fzDcMeasurement, fzFault, fzFaultMask],
    toZigbee: [tzFault],
    exposes: dcExposes,

    // bind + reporting for the 15 DC endpoints (DHT is configured by the modernExtend above)
    configure: async (device, coordinatorEndpoint) => {
        const failures = [];

        // fault indication on the board endpoint: bind, report on change (max 1 h), read the initial state
        const board = device.getEndpoint(BOARD_ENDPOINT.board);
        if (!board) {
            failures.push(`board: endpoint ${BOARD_ENDPOINT.board} not found`);
        } else {
            try {
                await reporting.bind(board, coordinatorEndpoint, ['genBinaryInput', DIAG_CLUSTER]);
                await board.configureReporting('genBinaryInput', [
                    {
                        attribute: 'presentValue',
                        minimumReportInterval: REPORTING.fault.min,
                        maximumReportInterval: REPORTING.fault.max,
                        reportableChange: REPORTING.fault.change,
                    },
                ]);
                await board.configureReporting(DIAG_CLUSTER, [
                    {
                        attribute: {ID: DIAG_ATTR_FAULT_MASK, type: TYPE_BITMAP32},
                        minimumReportInterval: REPORTING.fault.min,
                        maximumReportInterval: REPORTING.fault.max,
                        reportableChange: REPORTING.fault.change,
                    },
                ]);
                await board.read('genBinaryInput', ['presentValue']);
                await board.read(DIAG_CLUSTER, [DIAG_ATTR_FAULT_MASK]);
            } catch (error) {
                failures.push(`fault indication (EP ${BOARD_ENDPOINT.board}): ${error.message}`);
            }
        }

        // 15 DC endpoints: every endpoint is tried (one failure does not stop the others), the errors are
        // collected per endpoint and reported together at the end.
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
                        minimumReportInterval: REPORTING.dc.min,
                        maximumReportInterval: REPORTING.dc.max,
                        reportableChange: DC_THRESHOLDS[name].voltage,
                    },
                    {
                        attribute: {ID: ATTR_DC_CURRENT, type: TYPE_INT16},
                        minimumReportInterval: REPORTING.dc.min,
                        maximumReportInterval: REPORTING.dc.max,
                        reportableChange: DC_THRESHOLDS[name].current,
                    },
                    {
                        attribute: {ID: ATTR_DC_POWER, type: TYPE_INT16},
                        minimumReportInterval: REPORTING.dc.min,
                        maximumReportInterval: REPORTING.dc.max,
                        reportableChange: DC_THRESHOLDS[name].power,
                    },
                ]);
            } catch (error) {
                failures.push(`${name} (EP ${id}): ${error.message}`);
            }
        }
        if (failures.length > 0) {
            throw new Error(
                `Reporting setup failed on ${failures.length} of ${Object.keys(DC_ENDPOINTS).length + 1} endpoint groups ` +
                    `(Reconfigure repeats it): ${failures.join(' | ')}`,
            );
        }
    },
};

export default definition;
