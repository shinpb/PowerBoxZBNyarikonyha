import exposes from 'zigbee-herdsman-converters/lib/exposes';
import fz from 'zigbee-herdsman-converters/converters/fromZigbee';
import reporting from 'zigbee-herdsman-converters/lib/reporting';

const e = exposes.presets || exposes;
const haExposes = [];

for (let i = 1; i <= 2; i++) {
    const ep = `dht${i}`;
    haExposes.push(e.temperature().withEndpoint(ep));
    haExposes.push(e.humidity().withEndpoint(ep));
}

for (let i = 1; i <= 15; i++) {
    const ep = `dc${i}`;
    haExposes.push(e.voltage().withEndpoint(ep));
    haExposes.push(e.current().withEndpoint(ep));
    haExposes.push(e.power().withEndpoint(ep));
}

const fz_custom_dc = {
    cluster: 'haElectricalMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const payload = {};
        
        // Nyers értékek kinyerése string és numerikus formátumokból is
        const rawV = msg.data.dcVoltage ?? msg.data['256'] ?? msg.data['0x0100'] ?? msg.data[256];
        const rawI = msg.data.dcCurrent ?? msg.data['512'] ?? msg.data['0x0200'] ?? msg.data[512];
        const rawP = msg.data.dcPower   ?? msg.data['768'] ?? msg.data['0x0300'] ?? msg.data[768];

        // Végpont azonosító visszakeresése a névhez (pl. 16 -> 'dc1')
        const endpoints = model.endpoint(meta.device);
        const epName = Object.keys(endpoints).find(key => endpoints[key] === msg.endpoint.ID);

        // Csak akkor csatoljuk az adatokat, ha sikeres volt a végpont azonosítása
        if (epName) {
            if (rawV !== undefined) payload[`voltage_${epName}`] = rawV / 1000.0;
            if (rawI !== undefined) payload[`current_${epName}`] = rawI / 1000.0;
            if (rawP !== undefined) payload[`power_${epName}`] = rawP / 100.0;
        }

        return payload; // Helyes formátum visszadása: { "voltage_dc1": 12.3 }
    },
};

const converter = {
    zigbeeModel: ['PowerBox-INA15-DHT2'],
    model: 'PowerBox-INA15-DHT2',
    vendor: 'Espressif',
    description: '15-channel DC monitor with 2 DHT sensors',
    
    meta: { multiEndpoint: true },
    
    endpoint: (device) => {
        return {
            'dht1': 1, 'dht2': 2,
            'dc1': 16, 'dc2': 17, 'dc3': 18,
            'dc4': 19, 'dc5': 20, 'dc6': 21,
            'dc7': 22, 'dc8': 23, 'dc9': 24,
            'dc10': 25, 'dc11': 26, 'dc12': 27,
            'dc13': 32, 'dc14': 33, 'dc15': 34,
        };
    },

    fromZigbee: [fz.temperature, fz.humidity, fz_custom_dc],
    toZigbee: [], 
    exposes: haExposes,

    configure: async (device, coordinatorEndpoint) => {
        // 1. DHT Endpoints (EP 1, EP 2)
        for (const epId of [1, 2]) {
            const ep = device.getEndpoint(epId);
            if (ep) {
                try {
                    await reporting.bind(ep, coordinatorEndpoint, ['msTemperatureMeasurement', 'msRelativeHumidity']);
                } catch (e) {
                    // Ignore individual endpoint timeout to keep loop running
                }
                await new Promise((r) => setTimeout(r, 300));
            }
        }

        // 2. DC Measurement Endpoints (EP 16-27, 32-34)
        const dcEndpoints = [16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 32, 33, 34];
        for (const epId of dcEndpoints) {
            const ep = device.getEndpoint(epId);
            if (ep) {
                try {
                    await reporting.bind(ep, coordinatorEndpoint, ['haElectricalMeasurement']);
                } catch (e) {
                    // Ignore individual endpoint timeout
                }
                await new Promise((r) => setTimeout(r, 300));
            }
        }
    },
};

export default converter;