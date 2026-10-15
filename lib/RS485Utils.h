#pragma once

namespace RS485Utils {
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
};