#pragma once

#include <unordered_map>
#include <string_view>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include "RS485Utils.h"
#include "RS485InterfaceSensor.h"

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
#define BINARY_COMM_UPDATE          0b010000    // UPDATE VALUES
#define BINARY_COMM_RESET_START     0b111111    // START RESET
#define BINARY_COMM_RESET_CONT      0b111110    // CONTINUE RESET
#define BINARY_COMM_RESET_END       0b101010    // END RESET
#define BINARY_COMM_REBOOT          0b010101    // REBOOT ARSENAL
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

class RS485Comm{
public:
    // CONSTRUCTOR:
    RS485Comm(){
        
        // Configuration object for uart_param_config
        uart_config_t configUart = {
            .baud_rate = BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT
        };

        // Run config with this comm as UART_NUM_1
        this->_status = uart_param_config(RS485_COMM_UART, &configUart);

        this->_status = uart_set_pin(
            RS485_COMM_UART,
            UART_TX_PIN,
            UART_RX_PIN,
            UART_RTS_PIN,
            UART_PIN_NO_CHANGE
        );

        // Set up drivers for UART
        this->_status = uart_driver_install(
            RS485_COMM_UART,
            2048,
            0,
            0,
            nullptr,
            0
        );

        // Set the comm mode that the ESP32 is using
        this->_status = uart_set_mode(
            RS485_COMM_UART,
            UART_MODE_RS485_HALF_DUPLEX
        );
    }

    esp_err_t addSensortoRegistry(RS485InterfaceSensor* sensor) {
        if (sensor->getID() == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        _registryMap.emplace(sensor->getID(), sensor);
        return ESP_OK;
    }

    esp_err_t start() {
        if (this->_taskObject != nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        if (this->_status != ESP_OK) {
            return this->_status;
        }

        this->createRTOSTask();
        return this->_status;
    }

    // Checker for the RS485 comms
    bool checkCommStatus(){ return (this->_status == ESP_OK); }

private:
    // CORE FIELDS
    esp_err_t _status{ESP_ERR_NOT_FINISHED};
    TaskHandle_t _taskObject = nullptr;
    const char* _name{DEFAULT_COMM_NAME};
    const uint8_t _priority{DEFAULT_COMM_PRIORITY};
    const uint8_t _core{DEFAULT_COMM_TASK_CORE};

    //Registered sensors in the instance
    std::unordered_map<uint8_t, RS485InterfaceSensor*> _registryMap = {};

    // READ BUFFER
    uint8_t _lastReadBytes[DEFAULT_COMM_RX_BUFF]{};

    // PACKET PARSING STATES, reset with resetParsing()
    bool _isFirstRead{true};
    RS485Utils::ParseState _currentPacketState{RS485Utils::ParseState::WAITING_FOR_HEADER};
    size_t _currentPacketIndex{};
    uint16_t _dataLastIndex{};
    bool _validPacket{false};

    // PACKET DATA, persistent for a complete packet
    uint8_t _packet[DEFAULT_MAX_PAYLOAD]{};
    uint16_t _packetLength{};
    uint32_t _timeSinceLastReceive{};
    uint32_t _timeOfSending{};
    uint8_t _sensorIDS[2 * SENSOR_ID_SECTION_BYTECOUNT]{};

    uint64_t _receivedTime{};

    void compileSentPacket(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]){
        packet[0] = (BINARY_COMM_READ << 2) | 0b0010;

        //NOTE: We wait to add the length until the end with the full packet
        
        // TIME OF SENDING AND TIME SINCE RECEIVE
        const uint32_t timeNow = esp_timer_get_time();

        const uint32_t timeSinceReceive = static_cast<uint32_t>(timeNow - _receivedTime);
        const uint32_t timeSending = static_cast<uint32_t>(timeNow);

        RS485Utils::writeU32ToArr(packet, INDEX_TIME_SINCE_LAST_RECEIVE, timeSinceReceive);
        RS485Utils::writeU32ToArr(packet, INDEX_TIME_SENT, timeSending);

        // SENSOR IDS
        // Sensor IDS will always be even since 2 * MAX_SENSOR IDS is always even
        for (int i = 0; i < SENSOR_ID_SECTION_BYTECOUNT; i++) {
            // Get even and odd id
            uint8_t first = _registryMap.contains(_sensorIDS[2 * i]) ? _sensorIDS[2 * i] : 0;
            uint8_t second = _registryMap.contains(_sensorIDS[2 * i + 1]) ? _sensorIDS[2 * i + 1] : 0;

            // Place as a compacted byte at target index
            packet[INDEX_SENSOR_IDS + i] = static_cast<uint8_t>((first << 4) | (second & 0xF));
        }

        // DATA
        // The index we need will need to be reduced based on how many sensors we are sending.
        // We include the data here.

        // Loop invariant that controls the length based on the size
        // (invariant += 6) each time we add data to the variable data section
        uint8_t packetLengthInvariant{PACKET_LENGTH_MIN};

        for (int i = 0; i < sizeof(_sensorIDS) / sizeof(_sensorIDS[0]); i++) {
            // Each sensor portion uses 6 positions in the packet, so for each iteration
            // (6 * i + INDEX_SENSOR_DATA) gives you the first position of where to place the first
            // byte.

            if (_sensorIDS[i] == 0) {
                continue;
            }

            packetLengthInvariant = PACKET_LENGTH_MIN + 6 * (i + 1);

            if(_registryMap.contains(_sensorIDS[i])) {
                Vec3<uint16_t> reading = (_registryMap.at(_sensorIDS[i]))->getReadings();
                RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i, reading.x);
                RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 2, reading.y);
                RS485Utils::writeU16ToArr(packet, INDEX_SENSOR_DATA + 6 * i + 4, reading.z);
            } else {
                
            }
        }

        // LENGTH
        // The invariant is now equal to the minimum length + our variable data
        // We can now write the length in the packet
        RS485Utils::writeU16ToArr(packet, INDEX_PACKET_LENGTH, packetLengthInvariant);
        
        // FIRST ESCAPE SEQUENCE
        // Index found by taking the starting index where the length is measured, and adding the length invariant
        // we found.
        RS485Utils::writeU32ToArr(
            packet,
            INDEX_TIME_SINCE_LAST_RECEIVE + packetLengthInvariant,
            BINARY_ESCAPE_SEQUENCE
        );

        // CHECKSUM
        RS485Utils::writeU32ToArr(
            packet,
            INDEX_TIME_SINCE_LAST_RECEIVE + packetLengthInvariant + 4,
            this->calculateChecksum(packet, packetLengthInvariant)
        );

        // SECOND ESCAPE SEQUENCE
        RS485Utils::writeU32ToArr(
            packet,
            INDEX_TIME_SINCE_LAST_RECEIVE + packetLengthInvariant + 8,
            BINARY_ESCAPE_SEQUENCE
        );
    }

    void sendData(){
        // Send a responding data packet based on a validated packet received
        // from the V5 

        uint8_t sendingPacket[DEFAULT_MAX_PAYLOAD]{};

        this->compileSentPacket(sendingPacket);

        int bytesWritten = uart_write_bytes(
            UART_NUM_1,
            sendingPacket,
            sizeof(sendingPacket)
        );

        if (bytesWritten != static_cast<int>(sizeof(sendingPacket))) {
            this->_status = ESP_ERR_INVALID_STATE;
        }

        return;
    }

    void calculateSensorIDs(){
        // For loop to add sensor IDs to the array field with the IDs

        for (int i = 0; i < SENSOR_ID_SECTION_BYTECOUNT; i++){
            // currentByte is at the first index of the ID portion + i
            uint8_t currentByte = _packet[INDEX_SENSOR_IDS + i];

            if (currentByte != 0) {
                // Byte contains data
                // Written every two bytes, so first byte is even byte
                // second byte is odd byte
                
                // Left side of byte, right shifted to be 0000[][][][]
                _sensorIDS[2*i] = (currentByte >> 4);
                // Right side of the byte, compared to 00001111, to make it
                // also be 0000[][][][]
                _sensorIDS[2*i + 1] = (currentByte & 0xF);
            } else {
                // Complete fewer ops if we know its empty
                _sensorIDS[2*i] = 0;
                _sensorIDS[2*i + 1] = 0;
            }
        }
    }

    void extractData(){
        // Extract data from our checked packet

        // Time Data
        this->_timeSinceLastReceive = RS485Utils::convertU32InArr(_packet, INDEX_TIME_SINCE_LAST_RECEIVE);
        this->_timeOfSending        = RS485Utils::convertU32InArr(_packet, INDEX_TIME_SENT);
        
        this->calculateSensorIDs();

        if (this->_isFirstRead) {
            // Loop through the entire set of sensors
            for (int i = 0; i < sizeof(_sensorIDS) / sizeof(_sensorIDS[0]); i++) {
                // Check if the map contains the sensor ID
                if (_registryMap.contains(_sensorIDS[i])) {
                    //Add the offsets to a vector and pass it to set offsets

                    Vec3<uint16_t> offsets;
                    offsets.x = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + (3 * i)]);
                    offsets.y = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + (3 * i) + 1]);
                    offsets.z = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + (3 * i) + 2]);

                    (_registryMap.at(_sensorIDS[i]))->setOffsets(offsets);
                }
            }

            // Finished the offset setting for the first read.
            _isFirstRead = false;
        }
    }

    template <size_t S>
    uint32_t calculateChecksum(const uint8_t (&packet)[S], const uint16_t packetLength) {
        uint32_t sum{};
        for (int i = INDEX_TIME_SINCE_LAST_RECEIVE; i < INDEX_TIME_SINCE_LAST_RECEIVE + packetLength; i++) {
            sum += packet[i];
        }

        return sum % 256;
    }

    bool validateChecksum(){
        // Validate the checksum given in the packet.
        // Checked by modulo 256 

        // First checksum byte = First index for times since last + _packetLength + 4 bytes (esc sequence)
        uint16_t firstChecksumByte = static_cast<uint16_t>(INDEX_TIME_SINCE_LAST_RECEIVE + _packetLength + ESCAPE_SEQUENCE_BYTECOUNT);
        uint32_t checksum{RS485Utils::convertU32InArr(_packet, firstChecksumByte)};

        // Add each item of the packet together, up to right before our checksum
        uint32_t sum = calculateChecksum(_packet, _packetLength);

        return (sum) == (checksum);
    }

    bool invalidPacketLength(){
        // Over maximum, or length does not make it to sensor data index w/o
        // Header and length section
        return (
            this->_packetLength > PACKET_LENGTH_MAX ||
            this->_packetLength < PACKET_LENGTH_MIN
            );
    }

    void resetParsing(){
        _currentPacketIndex = 0;
        _currentPacketState = RS485Utils::ParseState::WAITING_FOR_HEADER;
        _dataLastIndex = 0;
    }

    void parseByte(uint8_t byte){
        switch (_currentPacketState)
        {
        case RS485Utils::ParseState::WAITING_FOR_HEADER:
            if ((byte >> 2) == BINARY_COMM_READ) {
                this->_packet[_currentPacketIndex] = byte;

                this->_currentPacketIndex = 1; // Next _currentPacketIndex
                _currentPacketState = RS485Utils::ParseState::LENGTH;
                return; // This bit is done
            } else {
                _validPacket = false;
            }
            break;
        
        case RS485Utils::ParseState::LENGTH:
            // Add the byte
            this->_packet[_currentPacketIndex] = byte;

            if (_currentPacketIndex == 2) {
                // New packet length
                _packetLength = static_cast<uint16_t>(_packet[1] << 8) |
                                static_cast<uint16_t>(_packet[2]);
                _dataLastIndex = NON_LENGTH_BYTECOUNT + _packetLength - 1;

                if (this->invalidPacketLength()) {
                    this->resetParsing();
                    break;
                } else {
                    this->_currentPacketState = RS485Utils::ParseState::BODY;
                    _currentPacketIndex++;
                    break;
                }
            }
            
            // Increment the current packet index
            _currentPacketIndex++;

            break;
        
        case RS485Utils::ParseState::BODY:
            // Check if we are still within the index
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == (this->_dataLastIndex)) {
                // This is the last bit, move on to the next part of the packet
                this->_currentPacketState = RS485Utils::ParseState::FIRST_ESCAPE_SEQUENCE;
            }
            
            // Increment the index, the escape sequence is next
            _currentPacketIndex++;
            break;
        
        case RS485Utils::ParseState::FIRST_ESCAPE_SEQUENCE:
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == NON_LENGTH_BYTECOUNT + _packetLength + 3) {
                // We are on the last byte
                if (RS485Utils::convertU32InArr(this->_packet, _currentPacketIndex - 3) == BINARY_ESCAPE_SEQUENCE) {
                    // Packet is correct
                    this->_currentPacketState = RS485Utils::ParseState::CHECKSUM;
                    _currentPacketIndex++;
                } else {
                    this->resetParsing();
                }
            } else {
                _currentPacketIndex++;
            }

            break;

        case RS485Utils::ParseState::CHECKSUM:
            // Get the 4 bits for the checksum
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == _dataLastIndex + 8) {
                // We are on the last checksum value
                // We will use the packet to validate the checksum
               if (this->validateChecksum()) {
                    // If the checksum is correct, move on to check the escape sequence
                    this->_currentPacketState = RS485Utils::ParseState::SECOND_ESCAPE_SEQUENCE;
                    _currentPacketIndex++;
                } else {
                    // The checksum failed, we need to reset for the next packet
                    this->resetParsing();
                }
            } else {
                // Increment the index, not done with the checksum yet
                _currentPacketIndex++;
            }
            break;
        case RS485Utils::ParseState::SECOND_ESCAPE_SEQUENCE:
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == _dataLastIndex + 12) {
                
                if (RS485Utils::convertU32InArr(this->_packet, _currentPacketIndex - 3) == BINARY_ESCAPE_SEQUENCE) {
                    // Check the packet, starting from four bits back

                    // Pull the data and add it
                    this->extractData();
                    this->_validPacket = true;
                    _receivedTime = esp_timer_get_time();
                    // Reset the parser for a clean slate
                } else {
                    _validPacket = false;
                }

                this->resetParsing();
            } else {
                // Not at the desired last index for the checksum, continue
                _currentPacketIndex++;
            }
            break;
        default:
            this->resetParsing();
            break;
        }
    }

    void getBytesLoop() {
        for(;;) {
            int received = uart_read_bytes(
                RS485_COMM_UART,
                this->_lastReadBytes,
                sizeof(_lastReadBytes),
                pdMS_TO_TICKS(100)
            );

            if (received > 0) {
                // UART read bytes, and we will parse them and send the requested data
                for (int i = 0; i < received; i++) {
                    this->parseByte(this->_lastReadBytes[i]);
                    if (this->_validPacket) {
                        this->_validPacket = false;
                        this->sendData();
                    }
                }
            }
        }
    }

    void createRTOSTask() {
        // Create the RTOS task for transceiver listening.

        BaseType_t status = xTaskCreatePinnedToCore (
            this->_taskEntry,                       //Task Loop
            this->_name,                            //Loop Name
            DEFAULT_COMM_TASK_STACK_SIZE,           //Stack size
            this,                                   //Ptr to task object
            this->_priority,                        //Priority
            &(this->_taskObject),                   //taskHandle_t in RS485Comm
            this->_core                             //Core
        );

        if (status != pdTRUE) {
            // Creating the RTOS task failed
            this->_status = ESP_ERR_INVALID_STATE;
        }
    }

    // WRAPPER ON TASK LOOP for RTOS Task
    static void _taskEntry(void* ptr) {
       reinterpret_cast<RS485Comm*>(ptr)->getBytesLoop();
    }
};