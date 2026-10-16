#pragma once

#include <string_view>

class RS485InterfaceSensor {
    public:
        virtual ~RS485InterfaceSensor() = default;

        virtual std::string_view getName() = 0;
        virtual uint8_t getID() = 0;
        virtual void setOffsets() = 0;
        virtual void registerSensor() = 0;
};