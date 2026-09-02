import exposes from 'zigbee-herdsman-converters/lib/exposes';
import fz from 'zigbee-herdsman-converters/converters/fromZigbee';

const e = exposes.presets || exposes;
const haExposes = [];

// DHT szenzorok (Hőmérséklet, Páratartalom) dinamikus generálása
for (let i = 1; i <= 2; i++) {
    const ep = `dht_${i}`;
    haExposes.push(e.temperature().withEndpoint(ep));
    haExposes.push(e.humidity().withEndpoint(ep));
}

// DC szenzorok (Feszültség, Áram, Teljesítmény) dinamikus generálása
for (let i = 1; i <= 15; i++) {
    const ep = `dc_${i}`;
    haExposes.push(e.voltage().withEndpoint(ep));
    haExposes.push(e.current().withEndpoint(ep));
    haExposes.push(e.power().withEndpoint(ep));
}

const converter = {
    zigbeeModel: ['PowerBox-INA15-DHT2'],
    model: 'PowerBox-INA15-DHT2',
    vendor: 'Espressif',
    description: '15-channel DC monitor with 2 DHT sensors',
    
    meta: { multiEndpoint: true },
    
    endpoint: (device) => {
        return {
            'dht_1': 0x01,
            'dht_2': 0x02,
            
            'dc_1': 0x10, 'dc_2': 0x11, 'dc_3': 0x12,
            'dc_4': 0x13, 'dc_5': 0x14, 'dc_6': 0x15,
            'dc_7': 0x16, 'dc_8': 0x17, 'dc_9': 0x18,
            'dc_10': 0x19, 'dc_11': 0x1A, 'dc_12': 0x1B,
            'dc_13': 0x20, 'dc_14': 0x21, 'dc_15': 0x22,
        };
    },

    fromZigbee: [fz.temperature, fz.humidity, fz.electrical_measurement],
    toZigbee: [], 
    exposes: haExposes,
};

export default converter;