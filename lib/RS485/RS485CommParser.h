#pragma once

#include "RS485Comm.h"
#include "SensorUtils.h"

inline bool RS485Comm::parseRead() {

    const size_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (uint8_t i = 0; i < idCount; i++) {
        // Loop through and make a validation check against sensorIDs and what
        // RS485Comm has
        if (_sensorIDS[i] == 0){
            continue;
        }

        if (!(_registryMap.contains(_sensorIDS[i]))) {
            // A sensor ID was requested that does not exist
            return false;
        }
    }

    return true;
}

inline bool RS485Comm::parseResetStart() {

    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (int i = 0; i < idCount; i++) {
        if (_sensorIDS[i] == 0) {
            continue;
        }

        // Identify the numerical value for the sensor type
        uint16_t sensorType = RS485Utils::convertU16InArr(_packet, INDEX_SENSOR_DATA + 6 * i);

        if (this->_registryMap.contains(_sensorIDS[i])) {
            // Set the sensor type for the ARC sensor
            bool result = (_registryMap.at(i))->setSensorType(static_cast<SensorUtils::SensorConfig>(sensorType));
            
            if (!result) {
                // Return an error state
                return false;
            }
        }
    }
    return true;
}

inline bool RS485Comm::parseResetContinue() {

    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (int i = 0; i < idCount; i++) {
        if (_sensorIDS[i] == 0) {
            continue;
        }

        const size_t base = INDEX_SENSOR_DATA + 6 * i;

        if (this->_registryMap.contains(_sensorIDS[i])) {
            Vec3<uint16_t> values{
                RS485Utils::convertU16InArr(_packet, base),
                RS485Utils::convertU16InArr(_packet, base + 2),
                RS485Utils::convertU16InArr(_packet, base + 4)
            };

            if (_currentOffsetSection > MAX_OFFSET_VECTOR_COUNT) {
                return false;
            }

            // Set the pending offsets within the array
            _pendingOffsets[i][_currentOffsetSection] = values;
        }
    }

    // Move on to the next set of offset writing
    _currentOffsetSection++;

    return true;
}

inline bool RS485Comm::parseResetEnd() {

    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (int i = 0; i < idCount; i++) {
        if (_sensorIDS[i] == 0) {
            continue;
        }

        const size_t base = INDEX_SENSOR_DATA + 6 * i;

        if (this->_registryMap.contains(_sensorIDS[i])) {
            Vec3<uint16_t> values{
                RS485Utils::convertU16InArr(_packet, base),
                RS485Utils::convertU16InArr(_packet, base + 2),
                RS485Utils::convertU16InArr(_packet, base + 4)
            };

            if (_currentOffsetSection > MAX_OFFSET_VECTOR_COUNT) {
                // We overshot our maximum
                return false;
            }

            // Set the pending offsets within the array
            _pendingOffsets[i][_currentOffsetSection] = values;

            std::array<Vec3<uint16_t>, MAX_OFFSET_VECTOR_COUNT> offsetSet{};

            for (int j = 0; j < MAX_OFFSET_VECTOR_COUNT; j++){
                offsetSet[j] = *_pendingOffsets[i][j];
            }

            // We have reached the end of the offsets for this sensor, give it over to
            // the sensor
            this->_registryMap.at(i)->setOffsets(offsetSet);
        }
    }

    return true;
}

inline bool RS485Comm::parseReboot() {

    return false;
}
