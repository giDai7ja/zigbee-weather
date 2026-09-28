const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;

// Кастомный обработчик для давления с высокой точностью
const fz_local = {
    rlyeh_station_pressure: {
        cluster: 'msPressureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.hasOwnProperty('measuredValue')) {
                // Устройство отправляет значение в единицах 10 Па (чтобы влезть в int16).
                // Например, 10132 означает 101320 Па = 1013.2 гПа.
                // Делим на 10, чтобы получить гПа (hPa) с точностью до 0.1.
                const pressure_hpa = msg.data['measuredValue'] / 10.0;
                return {pressure: pressure_hpa};
            }
        },
    }
};

const definition = {
    zigbeeModel: ['RlyehStation'],
    model: 'R\'lyeh Station',
    vendor: 'Cthulhu',
    description: 'High Precision Zigbee Weather Sensor from R\'lyeh 🐙 (AHT20 + BMP280)',
    fromZigbee: [
        fz.temperature,                       // ZCL отдает 0.01 °C, Z2M это отлично понимает
        fz.humidity,                          // ZCL отдает 0.01 %, Z2M это отлично понимает
        fz_local.rlyeh_station_pressure,      // Наш высокоточный обработчик давления
        fz.battery                            // ZCL отдает и процент, и напряжение
    ],
    toZigbee: [],
    exposes: [
        e.temperature().withPrecision(2),     // Принудительная точность 0.01 °C
        e.humidity().withPrecision(2),        // Принудительная точность 0.01 %
        e.pressure().withPrecision(1),        // Принудительная точность 0.1 гПа (hPa)
        e.battery(),
        e.battery_voltage(),                  // Напряжение батареи в мВ
        e.linkquality()
    ],
};

module.exports = definition;