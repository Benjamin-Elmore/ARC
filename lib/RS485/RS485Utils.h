#pragma once
#include <cstddef>
#include <cstdint>

namespace RS485Utils {

    enum ParseState {
        WAITING_FOR_HEADER,
        LENGTH,
        BODY,
        CHECKSUM,
        FIRST_ESCAPE_SEQUENCE,
        SECOND_ESCAPE_SEQUENCE
    };

    template <size_t S>
    uint32_t convertU32InArr(const uint8_t (&packet)[S], const uint8_t startingIndex){
        uint32_t converted{0};
        for (int i = 0; i < 4; i++) {
            // Starting L to R, add each 8 bit value into converted.
            // 8 * (3-1) is 24, 16, 8, 0 for each iter
            converted |= (packet[startingIndex + i] << (8 * (3 - i)));
        }

        return converted;
    }

    template <size_t S>
    void writeU16ToArr(uint8_t (&packet)[S], const uint8_t startingIndex, const uint16_t& value) {
        for (int i = 0; i < 2; i++) {
            // Write the high byte first, followed by the low byte.
            packet[startingIndex + i] =
                static_cast<uint8_t>((value >> (8 * (1 - i))) & 0xFFu);
        }
    }

    template <size_t S>
    void writeU32ToArr(uint8_t (&packet)[S], const uint8_t startingIndex, const uint32_t& value) {
        for (int i = 0; i < 4; i++) {
            // Shift by 24, 16, 8, then 0 bits.
            // Keep the lowest 8 bits and write them into the packet.
            packet[startingIndex + i] =
                static_cast<uint8_t>((value >> (8 * (3 - i))) & 0xFFu);
        }
    }
};
