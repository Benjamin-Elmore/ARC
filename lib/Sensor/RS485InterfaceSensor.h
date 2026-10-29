#pragma once

#include <string_view>
#include <cstdint>
#include "esp_err.h"
#include "Vec3.h"

class RS485InterfaceSensor {
    public:
        virtual ~RS485InterfaceSensor() = default;

        virtual std::string_view getName() = 0;
        virtual uint8_t getID() = 0;

        virtual esp_err_t setOffsets(Vec3<uint16_t> o) = 0;
        virtual esp_err_t registerSensor() = 0;
        virtual Vec3<uint16_t> getReadings() = 0;
};
