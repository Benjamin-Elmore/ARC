#pragma once

#include <unordered_map>
#include <optional>
#include <string_view>
#include <cstdio>
#include <cinttypes>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "RS485Utils.h"
#include "RS485InterfaceSensor.h"

#define RX_IDLE_THRESHOLD 4

class RS485Comm{
public:
    // CONSTRUCTOR:
    RS485Comm(int txPin = UART_TX_PIN, int rxPin = UART_RX_PIN, int enablePin = UART_RTS_PIN)
        : _txPin(txPin), _rxPin(rxPin), _enablePin(enablePin) {
        
        // Configuration object for uart_param_config
        uart_config_t configUart{};
        configUart.baud_rate = BAUD_RATE;
        configUart.data_bits = UART_DATA_8_BITS;
        configUart.parity = UART_PARITY_DISABLE;
        configUart.stop_bits = UART_STOP_BITS_1;
        configUart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        configUart.source_clk = UART_SCLK_DEFAULT;

        // Run config with this comm as UART_NUM_1
        this->_status = uart_param_config(RS485_COMM_UART, &configUart);

        if (!checkCommStatus()) { return; }

        this->_status = uart_set_pin(
            RS485_COMM_UART,
            _txPin,
            _rxPin,
            _enablePin,
            UART_PIN_NO_CHANGE
        );

        if (!checkCommStatus()) { return; }

        // Set up drivers for UART
        this->_status = uart_driver_install(
            RS485_COMM_UART,
            2048,
            0,
            0,
            &_uartEvents,   // HANDLE FOR UART
            0
        );

        if (!checkCommStatus()) { return; }

        this->_status = uart_set_rx_timeout(RS485_COMM_UART, RX_IDLE_THRESHOLD);
        if (!checkCommStatus()) { return; }
        uart_set_always_rx_timeout(RS485_COMM_UART, true);

        // Set the comm mode that the ESP32 is using
        this->_status = uart_set_mode(
            RS485_COMM_UART,
            UART_MODE_RS485_HALF_DUPLEX
        );
    }

    esp_err_t addSensortoRegistry(RS485InterfaceSensor* sensor) {
        // Included to prevent registration add/remove once RTOS task is started
        if (_taskObject != nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        if (sensor == nullptr) { return ESP_ERR_INVALID_ARG; }
        const uint8_t id = sensor->getID();
        if (id == 0 || id > 15) {
            return ESP_ERR_INVALID_ARG;
        }

        const bool inserted = _registryMap.emplace(id, sensor).second;
        return inserted ? ESP_OK : ESP_ERR_INVALID_ARG;
    }

    esp_err_t removeSensorFromRegistry(RS485InterfaceSensor* sensor) {
        // Included to prevent registration add/remove once RTOS task is started
        if (_taskObject != nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        if (sensor == nullptr) { return ESP_ERR_INVALID_ARG; }
        const uint8_t id = sensor->getID();
        if (id == 0 || id > 15) {
            return ESP_ERR_INVALID_ARG;
        }

        const auto it = _registryMap.find(id);

        if (it == _registryMap.end() || it->second != sensor) {
            return ESP_ERR_INVALID_ARG;
        }

        _registryMap.erase(it);
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

    void setSerialMonitoring(bool on) {
        _serialMonitoring = on;
        return;
    }

    void serialStatus() {
        const char* stateName = "UNKNOWN";
        switch (_currentPacketState) {
        case RS485Utils::ParseState::WAITING_FOR_HEADER: stateName = "WAITING_FOR_HEADER"; break;
        case RS485Utils::ParseState::LENGTH: stateName = "LENGTH"; break;
        case RS485Utils::ParseState::BODY: stateName = "BODY"; break;
        case RS485Utils::ParseState::FIRST_ESCAPE_SEQUENCE: stateName = "FIRST_ESCAPE_SEQUENCE"; break;
        case RS485Utils::ParseState::CHECKSUM: stateName = "CHECKSUM"; break;
        case RS485Utils::ParseState::SECOND_ESCAPE_SEQUENCE: stateName = "SECOND_ESCAPE_SEQUENCE"; break;
        default: break;
        }

        const auto printBytes = [](const char* label, const uint8_t* bytes, size_t count) {
            printf("%s (%zu bytes, hex):\n", label, count);
            for (size_t i = 0; i < count; ++i) {
                if (i % 16 == 0) { printf("  %03zu: ", i); }
                printf("%02X%s", static_cast<unsigned int>(bytes[i]),
                       (i % 16 == 15 || i + 1 == count) ? "\n" : " ");
            }
        };

        printf("\n=== SERIAL STATUS ===\n");
        printf("_status: %s (%" PRId32 ")\n", esp_err_to_name(_status), static_cast<int32_t>(_status));
        printf("_validPacket: %s\n", _validPacket ? "true" : "false");
        printf("_currentPacketState: %s (%d)\n", stateName, static_cast<int>(_currentPacketState));
        printf("_packetLength: %u body bytes\n", static_cast<unsigned int>(_packetLength));
        printf("_timeSinceLastReceive: %" PRIu32 "\n", _timeSinceLastReceive);
        printf("_timeOfSending: %" PRIu32 "\n", _timeOfSending);

        // Full buffers are shown; unused positions may contain zeros or older bytes.
        printBytes("_lastReadBytes [full buffer]", _lastReadBytes, sizeof(_lastReadBytes));
        printBytes("_packet [full buffer]", _packet, sizeof(_packet));
        printf("_sensorIDS (slot order):");
        for (const uint8_t id : _sensorIDS) {
            printf(" %u", static_cast<unsigned int>(id));
        }
        printf("\n=====================\n");
    }

private:
    // CORE FIELDS
    const int _txPin;
    const int _rxPin;
    const int _enablePin;
    esp_err_t _status{ESP_ERR_NOT_FINISHED};
    TaskHandle_t _taskObject = nullptr;
    const char* _name{DEFAULT_COMM_NAME};
    const uint8_t _priority{DEFAULT_COMM_PRIORITY};
    const uint8_t _core{DEFAULT_COMM_TASK_CORE};
    QueueHandle_t _uartEvents{};

    //Registered sensors in the instance
    std::unordered_map<uint8_t, RS485InterfaceSensor*> _registryMap = {};

    // READ BUFFER
    uint8_t _lastReadBytes[DEFAULT_COMM_RX_BUFF]{};

    // PACKET PARSING STATES, reset with resetParsing()
    RS485Utils::ParseState _currentPacketState{RS485Utils::ParseState::WAITING_FOR_HEADER};
    size_t _currentPacketIndex{};
    uint16_t _dataLastIndex{};
    bool _validPacket{false};
    bool _discardUntilIdle{false};
    size_t _frameSize{};

    // PACKET DATA, persistent for a complete packet
    uint8_t _packet[DEFAULT_MAX_PAYLOAD]{};
    uint8_t _packetHeader{};
    uint16_t _packetLength{};
    uint32_t _timeSinceLastReceive{};
    uint32_t _timeOfSending{};
    uint8_t _sensorIDS[2 * SENSOR_ID_SECTION_BYTECOUNT]{};

    // SETUP HOLDER FOR RESET, nested array for double access, used only on a reset
    uint8_t _currentOffsetSection{};
    std::array<std::array<std::optional<Vec3<uint16_t>>, MAX_OFFSET_VECTOR_COUNT>, 16> _pendingOffsets{};

    uint64_t _receivedTime{};

    bool _serialMonitoring{false};

    // PARSER FUNCTIONS
    bool validateRequestedSensorData(size_t bytesPerSlot) const {
        const size_t bodyEnd = NON_LENGTH_BYTECOUNT + size_t(_packetLength);
        const size_t slotCount = sizeof(_sensorIDS) / sizeof(_sensorIDS[0]);

        for (size_t slot = 0; slot < slotCount; ++slot) {
            const uint8_t id = _sensorIDS[slot];
            if (id == 0) {
                continue;
            }

            const size_t base = INDEX_SENSOR_DATA + 6 * slot;
            // Validate against the body boundary, not the full backing buffer.
            if (!_registryMap.contains(id) || base > bodyEnd ||
                bytesPerSlot > bodyEnd - base) {
                return false;
            }
        }

        return true;
    }

    bool parseRead();
    bool parseResetStart();
    bool parseResetContinue();
    bool parseResetEnd();
    bool parseReboot();
    bool parseFailure();

    void addTimeData(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]);
    void addSensorIDs(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]);
    void addPacketBacker(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD], uint8_t length);

    // PACKET COMPILER FUNCTIONS
    size_t sendPkt(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD], const uint8_t packetHeader);
    size_t compileReadData(uint8_t (&packet)[DEFAULT_MAX_PAYLOAD]);

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

    bool handleResponse(const bool failedRead = false){
        // Extract data from our checked packet and send a response

        // Packet Header, determining what packet we received
        // Its shifted by 2 since the last 2 bits are irrelevant for determination
        // 6 bits needed for id, so shift by 2
        this->_packetHeader = (_packet[0] >> 2);

        // Time Data
        this->_timeSinceLastReceive = RS485Utils::convertU32InArr(_packet, INDEX_TIME_SINCE_LAST_RECEIVE);
        this->_timeOfSending        = RS485Utils::convertU32InArr(_packet, INDEX_TIME_SENT);
        

        if (!failedRead){
            this->calculateSensorIDs();
        } else {
            this->setFaultyIDS();
        }

        uint8_t packet[DEFAULT_MAX_PAYLOAD]{};
        const size_t packetSize = sendPkt(packet, _packetHeader);
        if (packetSize == 0) {
            return false;
        }

        const int bytesWritten = uart_write_bytes(RS485_COMM_UART, packet, packetSize);
        if (bytesWritten != static_cast<int>(packetSize)) {
            _status = ESP_ERR_INVALID_STATE;
            return false;
        }

        return true;
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
        _frameSize = 0;
        _discardUntilIdle = false;
    }

    void setFailedPacketState() {
        setFaultyIDS();

    }

    void setFaultyIDS() {
        // Set IDS to 0xFF so that sensor read runs
        const size_t IdSize = 2 * SENSOR_ID_SECTION_BYTECOUNT;

        for (size_t i = 0; i < IdSize; i++) {
            _sensorIDS[i] = 0xFF;
        }
    }

    void parseByte(uint8_t byte){

        if (_discardUntilIdle) {
            return;
        }

        if (_currentPacketIndex >= sizeof(_packet)) {
            _validPacket = false;
            resetParsing();
            return;
        }

        switch (_currentPacketState)
        {
        case RS485Utils::ParseState::WAITING_FOR_HEADER:

            _validPacket = false;

            if (RS485Utils::isValidHeader(byte >> 2)) {
                this->_packet[_currentPacketIndex] = byte;
                this->_packetHeader = byte >> 2;

                this->_currentPacketIndex = 1; // Next _currentPacketIndex
                _currentPacketState = RS485Utils::ParseState::LENGTH;
            } else {
                _discardUntilIdle = true;
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
                _frameSize = NON_LENGTH_BYTECOUNT + size_t(_packetLength);
                _frameSize += 3 * ESCAPE_SEQUENCE_BYTECOUNT;

                if (_frameSize > sizeof(_packet) || invalidPacketLength()) {
                    _validPacket = false;
                    _discardUntilIdle = true;
                    break;
                }

                this->_currentPacketState = RS485Utils::ParseState::BODY;
            }
            
            // Increment the current packet index
            _currentPacketIndex++;

            break;
        
        case RS485Utils::ParseState::BODY:
            // Check if we are still within the index
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex >= (this->_dataLastIndex)) {
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
                    _validPacket = false;
                    _discardUntilIdle = true;
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
                    _validPacket = false;
                    _discardUntilIdle = true;
                }
            } else {
                // Increment the index, not done with the checksum yet
                _currentPacketIndex++;
            }
            break;
        case RS485Utils::ParseState::SECOND_ESCAPE_SEQUENCE:
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == _dataLastIndex + 12) {
                
                _validPacket = RS485Utils::convertU32InArr(this->_packet, _currentPacketIndex - 3) == BINARY_ESCAPE_SEQUENCE;
                
                // Reset the parser for a clean slate
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

            uart_event_t uartEvent{};
            if (xQueueReceive(_uartEvents, &uartEvent, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            
            if (uartEvent.type == UART_DATA) {
                size_t remaining = uartEvent.size;

                while (remaining > 0) {
                    int received = uart_read_bytes(
                        RS485_COMM_UART,
                        this->_lastReadBytes,
                        sizeof(_lastReadBytes),
                        pdMS_TO_TICKS(100)
                    );

                    if (received <= 0) {
                        _validPacket = false;
                        _discardUntilIdle = true;
                        break;
                    }

                    printf("UART received: %d\n", received);

                    for (size_t i = 0; i < received; i++) {
                        parseByte(_lastReadBytes[i]);
                    }

                    remaining -= static_cast<size_t>(received);
                }

                if (uartEvent.timeout_flag) {
                    if (_validPacket) {
                        _receivedTime = esp_timer_get_time();
                        
                        // Automatically handle the response, if it fails, give it a failed read
                        if(
                            !handleResponse() && 
                            static_cast<RS485Utils::PacketType>(_packetHeader) == RS485Utils::PacketType::Read
                        ) {
                            // Handle the response for a completely failed read
                            handleResponse(true);
                        }
                    } else if (static_cast<RS485Utils::PacketType>(_packetHeader) == RS485Utils::PacketType::Read) {
                        // We have a read, but not a valid packet
                        handleResponse(true);
                    }
                    
                    // Reset the parsing otherwise
                    resetParsing();
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

#include "RS485CommParser.h"
#include "RS485PacketComp.h"
