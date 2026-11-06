#pragma once
#include <cstddef>
#include <cstdint>
#include <array>
#include <algorithm>

// UART CONFIGURATIONS
#define RS485_COMM_UART UART_NUM_1
#ifndef UART_RTS_PIN
#define UART_RTS_PIN    14      // TRANSMIT/RECIEVE ENABLE
#endif
#ifndef UART_TX_PIN
#define UART_TX_PIN     17      // TRANSMIT PIN
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN     18      // RECEIVE PIN
#endif
#define BAUD_RATE       115200  // BITS PER SECOND

// COMM DEFAULT VALUES
#define DEFAULT_COMM_PRIORITY           1
#define DEFAULT_COMM_NAME               "RS485Comm"
#define DEFAULT_COMM_TASK_STACK_SIZE    4096
#define DEFAULT_COMM_TASK_CORE          1
#define DEFAULT_COMM_RX_BUFF            128
#define DEFAULT_MAX_PAYLOAD             128
#define DEFAULT_MAX_LENGTH              121

// BINARY HEADERS
#define BINARY_COMM_READ            0b100000    // READING DATA
// #define BINARY_COMM_UPDATE          0b010000    // UPDATE VALUES
#define BINARY_COMM_RESET_START     0b111111    // START RESET
#define BINARY_COMM_RESET_CONT      0b111110    // CONTINUE RESET
#define BINARY_COMM_RESET_END       0b101010    // END RESET
#define BINARY_COMM_REBOOT          0b010101    // REBOOT ARSENAL
#define BINARY_COMM_FAILURE         0b000000    // ARSENAL FAILURE
#define BINARY_ESCAPE_SEQUENCE      0xDEADBEEF  // ESCAPE SEQUENCE

// SIZE OF SECTIONS
#define NON_LENGTH_BYTECOUNT                   3           // # OF BYTES BEFORE LENGTH
#define PACKET_LENGTH_BYTECOUNT                4           // PACKET LENGTH BYTECOUNT
#define SENSOR_TIME_SECTION_BYTECOUNT_SUM      4           // BYTECOUNT OF TWO TIME SECTIONS
#define SENSOR_ID_SECTION_BYTECOUNT            8           // SENSOR ID BYTECOUNT
#define ESCAPE_SEQUENCE_BYTECOUNT              4           // ESCAPE SEQUENCE BYTECOUNT

// CONSISTENT FIRST INDICES PACKET SECTIONS
#define INDEX_PACKET_LENGTH                 1           // PACKET LENGTH (Time, Time, ID, Data)
#define INDEX_TIME_SINCE_LAST_RECEIVE       3           // TIME SINCE LAST RECEIVE
#define INDEX_TIME_SENT                     7           // TIME OF SENDING
#define INDEX_SENSOR_IDS                    11          // SENSOR IDs
#define INDEX_SENSOR_DATA                   19          // SENSOR_DATA

// MAX AND MIN FOR PACKET LENGTH
#define PACKET_LENGTH_MIN           16      // MIN
#define PACKET_LENGTH_MAX           113     // MAX

// PARSER DATA
#define MAX_OFFSET_VECTOR_COUNT 3

namespace RS485Utils {

    enum class PacketType : std::uint8_t {
        Read          = BINARY_COMM_READ,
        ResetStart    = BINARY_COMM_RESET_START,
        ResetContinue = BINARY_COMM_RESET_CONT,
        ResetEnd      = BINARY_COMM_RESET_END,
        Reboot        = BINARY_COMM_REBOOT,
        Failure       = BINARY_COMM_FAILURE
    };

    inline bool isValidHeader(std::uint8_t header) {
        switch (static_cast<PacketType>(header)) {
            case PacketType::Read:
            case PacketType::ResetStart:
            case PacketType::ResetContinue:
            case PacketType::ResetEnd:
            case PacketType::Reboot:
            case PacketType::Failure:
                return true;

            default:
                return false;
        }
    }

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
    void writeU32ToArr(uint8_t (&packet)[S], const uint8_t startingIndex, const uint32_t& value) {
        for (int i = 0; i < 4; i++) {
            // Shift by 24, 16, 8, then 0 bits.
            // Keep the lowest 8 bits and write them into the packet.
            packet[startingIndex + i] =
                static_cast<uint8_t>((value >> (8 * (3 - i))) & 0xFFu);
        }
    }

    template <size_t S>
    uint16_t convertU16InArr(const uint8_t (&packet)[S], const size_t startingIndex) {
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(packet[startingIndex]) << 8)
            | static_cast<uint16_t>(packet[startingIndex + 1])
        );
    }

    template <size_t S>
    void writeU16ToArr(uint8_t (&packet)[S], const uint8_t startingIndex, const uint16_t& value) {
        for (int i = 0; i < 2; i++) {
            // Write the high byte first, followed by the low byte.
            packet[startingIndex + i] =
                static_cast<uint8_t>((value >> (8 * (1 - i))) & 0xFFu);
        }
    }
};
