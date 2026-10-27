#pragma once

#include "RS485Comm.h"
#include "RS485InterfaceSensor.h"

namespace RS485ARSENAL {
    inline RS485Comm* BaseRS485Instance = nullptr;

    inline void createRS485Instance() {
        struct SharedInstance {
            RS485Comm comm;
            ~SharedInstance() { BaseRS485Instance = nullptr; }
        };

        static SharedInstance instance;
        BaseRS485Instance = &instance.comm;

    }

    inline esp_err_t addSensor(RS485InterfaceSensor* sensor) {
        if (!BaseRS485Instance) {
            RS485ARSENAL::createRS485Instance();
        }

        return BaseRS485Instance->addSensortoRegistry(sensor);
    }

    inline void removeSensor(RS485InterfaceSensor* sensor) {
        if (BaseRS485Instance) {
            BaseRS485Instance->removeSensorFromRegistry(sensor);
        }
    }

    inline esp_err_t startRS485ARSENAL() {
        if (!BaseRS485Instance) {
            return ESP_ERR_INVALID_STATE;
        }

        return BaseRS485Instance->start();
    }
}
