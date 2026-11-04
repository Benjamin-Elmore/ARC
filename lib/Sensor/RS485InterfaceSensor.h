#pragma once

#include <string_view>
#include <cstdint>

#include "esp_err.h"
#include "Vec3.h"
#include "SensorUtils.h"
#include "RS485Utils.h"

class RS485InterfaceSensor {
    public:
        virtual ~RS485InterfaceSensor() = default;

        virtual std::string_view getName() = 0;
        virtual uint8_t getID() = 0;

        virtual esp_err_t setOffsets(Vec3<uint16_t> o) = 0;
        virtual esp_err_t registerSensor() = 0;
        virtual bool getReadings(Vec3<uint16_t>& readings) = 0;
        virtual bool setSensorType(SensorUtils::SensorConfig preconfig) = 0;

        virtual bool setAppendedOffsets(std::array<Vec3<uint16_t>, MAX_OFFSET_VECTOR_COUNT> offsetArr) = 0;
};
