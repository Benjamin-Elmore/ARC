#pragma once

#include "RS485Utils.h"
#include "SensorUtils.h"

namespace SensorOffsetConfig {

    template <std::size_t N>
    void readTemperature(Sensor<N>& sensor, LockGuard& lock) {
        constexpr uint8_t temperatureRegister = 0x00;
        uint8_t temperatureBytes[2]{};
        const esp_err_t readStatus = sensor.readDeviceRegister(
            temperatureRegister, temperatureBytes, sizeof(temperatureBytes), lock);

        if (readStatus != ESP_OK) {
            return;
        }

        const uint16_t highByte = temperatureBytes[0];
        const uint16_t lowByte = temperatureBytes[1];
        const uint16_t rawTemperature = (highByte << 8) | lowByte;
        sensor.setReadings({rawTemperature, 0, 0});
    }

    template <std::size_t N>
    void addTemperatureOffsets(Sensor<N>& sensor) {
        constexpr uint8_t temperatureOffsetRegister = 0x07;
        constexpr uint16_t defaultTemperatureOffset = 0;
        constexpr uint8_t temperaturePacketSection = 0;
        constexpr uint8_t temperatureComponent = 0;
        constexpr auto registerLayout = SensorUtils::OffsetLayout::REG_16_BIT_HIGH_FIRST;

        sensor.appendOffset(
            "offsetTemperature", defaultTemperatureOffset, temperatureOffsetRegister,
            temperaturePacketSection, temperatureComponent, registerLayout);
    }

    template <std::size_t N>
    void addOpticalOffsets(Sensor<N>& sensor) {
        constexpr auto layout = SensorUtils::OffsetLayout::REG_16_BIT_LOW_FIRST;
        constexpr uint8_t positionSection = 0;

        // OTOS position offset registers for X, Y, and heading.
        sensor.appendOffset("offsetX", 0, 0x10, positionSection, 0, layout);
        sensor.appendOffset("offsetY", 0, 0x12, positionSection, 1, layout);
        sensor.appendOffset("offsetH", 0, 0x14, positionSection, 2, layout);
    }


    // Register preset offsets once, before starting the sensor.
    template <std::size_t N>
    void addOffsets(Sensor<N>& sensor, SensorUtils::SensorConfig type) {

        if (sensor.getSensorType() != type) {
            // Change the type, but don't add offsets, this prevents a potential
            // recursion issue by recalling addOffsets
            sensor.setSensorTypeWoOffset(type);
        }

        switch (type) {
            case SensorUtils::SensorConfig::TEMPERATURE:
                addTemperatureOffsets(sensor);
                break;

            case SensorUtils::SensorConfig::OPTICAL:
                addOpticalOffsets(sensor);
                break;

            case SensorUtils::SensorConfig::NONE:
            case SensorUtils::SensorConfig::COLOR:
            case SensorUtils::SensorConfig::ENCODER:
                break;
        }
    }

}
