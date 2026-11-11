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
    // Validate every requested slot before changing any sensor.
    if (!validateRequestedSensorData(2)) {
        return false;
    }

    // Reset the offset details in the case that a previous reboot leaves unused data
    _currentOffsetSection = 0;
    _pendingOffsets = {};

    // Set the ID type, then get offsets
    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (int i = 0; i < idCount; i++) {
        if (_sensorIDS[i] == 0) {
            continue;
        }

        // Identify the numerical value for the sensor type
        uint16_t sensorType = RS485Utils::convertU16InArr(_packet, INDEX_SENSOR_DATA + 6 * i);

        if (this->_registryMap.contains(_sensorIDS[i])) {
            // Set the sensor type for the ARC sensor
            bool result = (_registryMap.at(_sensorIDS[i]))->setSensorType(static_cast<SensorUtils::SensorConfig>(sensorType));
            
            if (!result) {
                // Return an error state
                return false;
            }
        }
    }
    return true;
}

inline bool RS485Comm::parseResetContinue() {
    if (_currentOffsetSection >= MAX_OFFSET_VECTOR_COUNT ||
        !validateRequestedSensorData(6)) {
        return false;
    }

    // Keep building the offsets
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

            // Set the pending offsets within the array
            _pendingOffsets[_sensorIDS[i]][_currentOffsetSection] = values;
        }
    }

    // Move on to the next set of offset writing
    _currentOffsetSection++;

    return true;
}

inline bool RS485Comm::parseResetEnd() {
    // Wrap up the reset and set the offsets given
    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    if (_currentOffsetSection >= MAX_OFFSET_VECTOR_COUNT ||
        !validateRequestedSensorData(6)) {
        return false;
    }
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

            if (_currentOffsetSection >= MAX_OFFSET_VECTOR_COUNT) {
                // We overshot our maximum
                return false;
            }

            // Set the pending offsets within the array
            _pendingOffsets[_sensorIDS[i]][_currentOffsetSection] = values;

            std::array<Vec3<uint16_t>, MAX_OFFSET_VECTOR_COUNT> offsetSet{};

            for (int j = 0; j < MAX_OFFSET_VECTOR_COUNT; j++){
                offsetSet[j] = _pendingOffsets[_sensorIDS[i]][j].value_or(Vec3<uint16_t>{
                    SensorUtils::UNUSED_OFFSETS, SensorUtils::UNUSED_OFFSETS,
                    SensorUtils::UNUSED_OFFSETS});
            }

            // We have reached the end of the offsets for this sensor, give it over to
            // the sensor
            const esp_err_t writeStatus = this->_registryMap.at(_sensorIDS[i])->setOffsets(offsetSet);
            if (writeStatus != ESP_OK) {
                printf("Offset write failed: sensor ID %u: %s\n", _sensorIDS[i], esp_err_to_name(writeStatus));
                return false;
            }
            printf("Offsets applied on I2C bus: sensor ID %u, first value 0x%04X\n",
                   _sensorIDS[i], values.x);
        }
    }

    _currentOffsetSection = 0;
    _pendingOffsets = {};
    return true;
}

inline bool RS485Comm::parseReboot() {
    // Reboot the sensors registered to the comm
    const uint16_t idCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

    for (int i = 0; i < idCount; i++) {
        if (_sensorIDS[i] == 0) {
            continue;
        }

        if (this->_registryMap.contains(_sensorIDS[i])) {
            esp_err_t rebootStatus = this->_registryMap.at(_sensorIDS[i])->reboot();

            if (rebootStatus != ESP_OK) {
                return false;
            }
        }       
    }

    return true;
}
