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
//   EP 0x30                  -> PZEM-004T #1 (AC mains meter: V, I, P, f, PF, energy); further modules EP 0x31, 0x32
//
// DC units on the device side (haElectricalMeasurement, DC attributes):
//   dcVoltage 0x0100  int16  mV   (divisor 1000)  -> V
//   dcCurrent 0x0103  int16  mA   (divisor 1000)  -> A
//   dcPower   0x0106  int16  cW   (divisor 100)   -> W  (1 raw unit = 10 mW)
//
// AC units on the PZEM endpoints (haElectricalMeasurement AC attributes + seMetering), mirror of zcl_values.h:
//   rmsVoltage 0x0505 uint16 0.1 V (divisor 10)   rmsCurrent 0x0508 uint16 10 mA (divisor 100)
//   activePower 0x050B int16 1 W                  acFrequency 0x0300 uint16 0.1 Hz (divisor 10)
//   powerFactor 0x0510 int8 percent               currentSummDelivered 0x0000 uint48 Wh (-> kWh, divisor 1000)
//
// Unavailable sensor: the device writes the ZCL "invalid" value into the attribute
//   temperature 0x8000 (-32768), humidity 0xFFFF, DC int16 0x8000 (-32768),
//   AC uint16 0xFFFF, AC power 0x8000, power factor 0x80, energy 0xFFFFFFFFFFFF
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

// PZEM modules: properties voltage_pzem1, current_pzem1, power_pzem1, ac_frequency_pzem1, power_factor_pzem1, energy_pzem1
const PZEM_ENDPOINTS = {pzem1: 0x30};

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
    // PZEM, raw units: 0.1 V, 10 mA, 1 W, 0.1 Hz, 1 %, 1 Wh (mirror of Config.Reporting.Pzem: 1 V, 0.05 A, 2 W, 0.2 Hz, 5 %, 0.010 kWh)
    pzem: {min: 5, max: 60, voltage: 10, current: 5, power: 2, frequency: 2, powerFactor: 5, energy: 10},
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
const TYPE_UINT16 = 0x21;
const TYPE_INT8 = 0x28;
const TYPE_UINT48 = 0x25;

// AC attributes of the PZEM endpoints
const ATTR_AC_FREQUENCY = 0x0300;
const ATTR_RMS_VOLTAGE = 0x0505;
const ATTR_RMS_CURRENT = 0x0508;
const ATTR_ACTIVE_POWER = 0x050b;
const ATTR_POWER_FACTOR = 0x0510;
const ATTR_CURRENT_SUMM_DELIVERED = 0x0000; // seMetering

// ZCL invalid measurement values (must match zcl_values.h)
const INVALID_INT16 = -32768; // 0x8000: temperature, DC voltage/current/power
const INVALID_UINT16 = 0xffff; // humidity, AC voltage/current/frequency
const INVALID_INT8 = -128; // 0x80: power factor
const INVALID_UINT48 = 0xffffffffffff; // energy
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

// ---------------------------------------------------------------- PZEM AC measurement + energy
const fzAcMeasurement = {
    cluster: 'haElectricalMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const get = (name, id) => (msg.data[name] !== undefined ? msg.data[name] : msg.data[id]);
        const scale = (raw, divisor, digits, invalidValue) =>
            isInvalid(raw, invalidValue) ? null : Math.round((raw / divisor) * 10 ** digits) / 10 ** digits;
        const name = (property) => utils.postfixWithEndpointName(property, msg, model, meta);

        const payload = {};
        const v = get('rmsVoltage', ATTR_RMS_VOLTAGE);
        const i = get('rmsCurrent', ATTR_RMS_CURRENT);
        const p = get('activePower', ATTR_ACTIVE_POWER);
        const f = get('acFrequency', ATTR_AC_FREQUENCY);
        const pf = get('powerFactor', ATTR_POWER_FACTOR);

        if (v !== undefined) payload[name('voltage')] = scale(v, 10, 1, INVALID_UINT16);
        if (i !== undefined) payload[name('current')] = scale(i, 100, 2, INVALID_UINT16);
        if (p !== undefined) payload[name('power')] = scale(p, 1, 0, INVALID_INT16);
        if (f !== undefined) payload[name('ac_frequency')] = scale(f, 10, 1, INVALID_UINT16);
        if (pf !== undefined) payload[name('power_factor')] = scale(pf, 100, 2, INVALID_INT8);

        return Object.keys(payload).length ? payload : undefined;
    },
};

const fzEnergy = {
    cluster: 'seMetering',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        let raw = msg.data.currentSummDelivered !== undefined ? msg.data.currentSummDelivered : msg.data[ATTR_CURRENT_SUMM_DELIVERED];
        if (raw === undefined) return undefined;
        // uint48 may arrive as number, bigint or [high32, low32]
        if (Array.isArray(raw)) raw = raw[0] * 2 ** 32 + raw[1];
        else if (typeof raw === 'bigint') raw = Number(raw);
        const property = utils.postfixWithEndpointName('energy', msg, model, meta);
        if (isInvalid(raw, INVALID_UINT48)) return {[property]: null};
        return {[property]: Math.round(raw) / 1000}; // Wh -> kWh
    },
};

// ---------------------------------------------------------------- device fault indication (EP 0x01)
// Standard Binary Input (genBinaryInput, presentValue = any fault) plus a manufacturer-specific
// cluster with the fault mask (bitmap32). Must match health.h (HF_SENS_*, HF_BUS_* << 16) and zcl_values.h:
//   bits 0..15  sensor faults: 0 board, 1..2 DHT1..2, 8..12 INA1..5, 13..15 PZEM1..3
//   bits 16..23 bus faults:    16 I2C init, 17..19 I2C bus 0..2, 20 UART init, 21..23 UART bus 0..2
//   (a PZEM fault also appears when the mains is off: the meter is powered from the mains side)
const DIAG_CLUSTER = 'powerboxDiag';
const DIAG_CLUSTER_ID = 0xfc11;
const DIAG_ATTR_FAULT_MASK = 0x0000;
const TYPE_BITMAP32 = 0x1b;

const FAULT_BITS = [
    [0, 'board'], [1, 'dht1'], [2, 'dht2'],
    [8, 'ina1'], [9, 'ina2'], [10, 'ina3'], [11, 'ina4'], [12, 'ina5'],
    [13, 'pzem1'], [14, 'pzem2'], [15, 'pzem3'],
    [16, 'i2c_init'], [17, 'i2c_bus0'], [18, 'i2c_bus1'], [19, 'i2c_bus2'],
    [20, 'uart_init'], [21, 'uart_bus0'], [22, 'uart_bus1'], [23, 'uart_bus2'],
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
        .withDescription('Faulty units (board, dhtN, inaN, pzemN, i2c_*, uart_*) or "ok"')
        .withCategory('diagnostic'),
];
for (const name of Object.keys(DC_ENDPOINTS)) {
    dcExposes.push(
        e.voltage().withEndpoint(name),
        e.current().withEndpoint(name),
        e.power().withEndpoint(name),
    );
}

for (const name of Object.keys(PZEM_ENDPOINTS)) {
    dcExposes.push(
        e.voltage().withEndpoint(name),
        e.current().withEndpoint(name),
        e.power().withEndpoint(name),
        exposes.numeric('ac_frequency', exposes.access.STATE).withUnit('Hz').withDescription('Mains frequency').withEndpoint(name),
        exposes.numeric('power_factor', exposes.access.STATE).withDescription('Power factor (0..1)').withEndpoint(name),
        exposes.numeric('energy', exposes.access.STATE).withUnit('kWh').withDescription('Energy counter of the PZEM module').withEndpoint(name),
    );
}

// ---------------------------------------------------------------- definition
const definition = {
    zigbeeModel: ['PowerBox-INA15-DHT2'],
    model: 'PowerBox-INA15-DHT2',
    vendor: 'DIY',
    description: '5x INA3221 (15 DC channels: V/I/P) + 2x DHT temperature/humidity + PZEM-004T AC meter (ESP32-C6)',

    extend: [
        m.deviceEndpoints({endpoints: {...BOARD_ENDPOINT, ...DHT_ENDPOINTS, ...DC_ENDPOINTS, ...PZEM_ENDPOINTS}}),
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

    fromZigbee: [fzDcMeasurement, fzAcMeasurement, fzEnergy, fzFault, fzFaultMask],
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

        // PZEM endpoints: AC measurement + energy (same pattern, errors collected)
        const rp = REPORTING.pzem;
        const rep = (attribute, change) => ({
            attribute,
            minimumReportInterval: rp.min,
            maximumReportInterval: rp.max,
            reportableChange: change,
        });
        for (const [name, id] of Object.entries(PZEM_ENDPOINTS)) {
            const endpoint = device.getEndpoint(id);
            if (!endpoint) {
                failures.push(`${name}: endpoint ${id} not found`);
                continue;
            }
            try {
                await reporting.bind(endpoint, coordinatorEndpoint, ['haElectricalMeasurement', 'seMetering']);
            } catch (error) {
                failures.push(`${name} (EP ${id}) bind: ${error.message}`);
                continue;
            }
            try {
                await endpoint.configureReporting('haElectricalMeasurement', [
                    rep({ID: ATTR_RMS_VOLTAGE, type: TYPE_UINT16}, rp.voltage),
                    rep({ID: ATTR_RMS_CURRENT, type: TYPE_UINT16}, rp.current),
                    rep({ID: ATTR_ACTIVE_POWER, type: TYPE_INT16}, rp.power),
                    rep({ID: ATTR_AC_FREQUENCY, type: TYPE_UINT16}, rp.frequency),
                    rep({ID: ATTR_POWER_FACTOR, type: TYPE_INT8}, rp.powerFactor),
                ]);
            } catch (error) {
                failures.push(`${name} (EP ${id}) electrical measurement: ${error.message}`);
            }
            try {
                await endpoint.configureReporting('seMetering', [rep({ID: ATTR_CURRENT_SUMM_DELIVERED, type: TYPE_UINT48}, rp.energy)]);
            } catch (error) {
                failures.push(`${name} (EP ${id}) metering: ${error.message}`);
            }
        }
        if (failures.length > 0) {
            throw new Error(
                `Reporting setup failed on ${failures.length} of ${Object.keys(DC_ENDPOINTS).length + Object.keys(PZEM_ENDPOINTS).length + 1} endpoint groups ` +
                    `(Reconfigure repeats it): ${failures.join(' | ')}`,
            );
        }
    },
};

export default definition;
