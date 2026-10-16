#pragma once

#include "RS485Comm.h"
#include "RS485InterfaceSensor.h"

namespace RS485ARSENAL {
    RS485Comm* BaseRS485Instance;

    void startRS485Instance() {
        BaseRS485Instance = new RS485Comm();
    }

    void addSensor(RS485InterfaceSensor* sensor) {
        BaseRS485Instance->addSensortoRegistry(sensor);
    }
}