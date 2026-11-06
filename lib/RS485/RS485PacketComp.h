#pragma once

#include "RS485Comm.h"
#include "RS485Utils.h"

inline void RS485Comm::addTimeData(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]) {
    // TIME OF SENDING AND TIME SINCE RECEIVE
    const uint32_t timeNow = esp_timer_get_time();

    const uint32_t timeSinceReceive = static_cast<uint32_t>(timeNow - _receivedTime);
    const uint32_t timeSending = static_cast<uint32_t>(timeNow);

    RS485Utils::writeU32ToArr(packet, INDEX_TIME_SINCE_LAST_RECEIVE, timeSinceReceive);
    RS485Utils::writeU32ToArr(packet, INDEX_TIME_SENT, timeSending);
}

inline void RS485Comm::addSensorIDs(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]) {
    // SENSOR IDS
    // Sensor IDS will always be even since 2 * MAX_SENSOR IDS is always even
    for (int i = 0; i < SENSOR_ID_SECTION_BYTECOUNT; i++) {
        // Get even and odd id
        uint8_t first = _registryMap.contains(_sensorIDS[2 * i]) ? _sensorIDS[2 * i] : 0;
        uint8_t second = _registryMap.contains(_sensorIDS[2 * i + 1]) ? _sensorIDS[2 * i + 1] : 0;

        // Place as a compacted byte at target index
        packet[INDEX_SENSOR_IDS + i] = static_cast<uint8_t>((first << 4) | (second & 0xF));
    }

}

inline void RS485Comm::addPacketBacker(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD], uint8_t length) {
    // FIRST ESCAPE SEQUENCE
    // Index found by taking the starting index where the length is measured, and adding the length invariant
    // we found.
    RS485Utils::writeU32ToArr(
        packet,
        INDEX_TIME_SINCE_LAST_RECEIVE + length,
        BINARY_ESCAPE_SEQUENCE
    );

    // CHECKSUM
    RS485Utils::writeU32ToArr(
        packet,
        INDEX_TIME_SINCE_LAST_RECEIVE + length + 4,
        this->calculateChecksum(packet, length)
    );

    // SECOND ESCAPE SEQUENCE
    RS485Utils::writeU32ToArr(
        packet,
        INDEX_TIME_SINCE_LAST_RECEIVE + length + 8,
        BINARY_ESCAPE_SEQUENCE
    );
}

inline size_t RS485Comm::sendPkt(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD], const uint8_t packetHeader) {
    //NOTE: We wait to add the length and header until the end with the full packet
    
    // TIME
    this->addTimeData(packet);

    // SENSOR IDs
    this->addSensorIDs(packet);

    // DATA
    // Each compiler returns the body length, or zero when no response is available.
    size_t packetLengthInvariant{PACKET_LENGTH_MIN};
    switch (static_cast<RS485Utils::PacketType>(packetHeader)) {
        case RS485Utils::PacketType::Read:
            if (!parseRead()) { return 0; }
            packetLengthInvariant = compileReadData(packet);
            break;

        case RS485Utils::PacketType::ResetStart:
        case RS485Utils::PacketType::ResetContinue:
        case RS485Utils::PacketType::ResetEnd:
        case RS485Utils::PacketType::Failure:
            break;

        case RS485Utils::PacketType::Reboot:
            if (!parseReboot()) { return 0; }
            packetLengthInvariant = compileRebootData(packet);
            break;

        default:
            return 0;
    }

    if (packetLengthInvariant == 0) {
        return 0;
    }

    // LENGTH
    // The invariant is now equal to the minimum length + our variable data
    // We can now write the length in the packet
    RS485Utils::writeU16ToArr(packet, INDEX_PACKET_LENGTH, packetLengthInvariant);
    
    // Packet Backer adds escape sequence, checksum, escape sequence
    this->addPacketBacker(packet, packetLengthInvariant);

    // Return the size of the packet
    return NON_LENGTH_BYTECOUNT + packetLengthInvariant + 3 * ESCAPE_SEQUENCE_BYTECOUNT;
}

inline size_t RS485Comm::compileReadData(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]) {
    uint8_t packetLengthInvariant{PACKET_LENGTH_MIN};

    for (int i = 0; i < sizeof(_sensorIDS) / sizeof(_sensorIDS[0]); i++) {
        // Each sensor portion uses 6 positions in the packet, so for each iteration
        // (6 * i + INDEX_SENSOR_DATA) gives you the first position of where to place the first
        // byte.

        if (_sensorIDS[i] == 0) {
            continue;
        }

        packetLengthInvariant = PACKET_LENGTH_MIN + 6 * (i + 1);

        if (_sensorIDS[i] == 0xFF) {
            // There was an error with the sensors, and we are passing through
            // Error data
            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i, 0xFF);
            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 2, 0xFF);
            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 4, 0xFF);
            continue;
        }

        if(_registryMap.contains(_sensorIDS[i])) {
            Vec3<uint16_t> reading{}; 
            const bool validResult = (_registryMap.at(_sensorIDS[i]))->getReadings(reading);

            if (!validResult) {
                reading.x = 0xFFFF;
                reading.y = 0xFFFF;
                reading.z = 0xFFFF;
            }

            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i, reading.x);
            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 2, reading.y);
            RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 4, reading.z);
        }
    }
    return packetLengthInvariant;
}

inline size_t RS485Comm::compileRebootData(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]) {
    // TODO: Define the response format for this packet type.
    return 0;
}
