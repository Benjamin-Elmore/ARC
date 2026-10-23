#pragma once

#include "RS485Comm.h"
#include "RS485InterfaceSensor.h"

namespace RS485ARSENAL {
    RS485Comm* BaseRS485Instance = nullptr;

    void startRS485Instance() {
        BaseRS485Instance = new RS485Comm();
    }

    esp_err_t addSensor(RS485InterfaceSensor* sensor) {
        if (RS485ARSENAL::BaseRS485Instance == nullptr) {
            RS485ARSENAL::startRS485Instance();
        }

        return BaseRS485Instance->addSensortoRegistry(sensor);
    }

    esp_err_t startRS485ARSENAL() {
        if (BaseRS485Instance == nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        return BaseRS485Instance->start();
    }
}