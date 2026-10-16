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
#define UART_RTS_PIN    17      // TRANSMIT/RECIEVE ENABLE
#endif
#ifndef UART_TX_PIN
#define UART_TX_PIN     17      // TRANSMIT PIN
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN     18      // RECEIVE PIN
#endif
#define BAUD_RATE       11520   //BITS PER SECOND

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
#define SENSOR_TIME_SECTION_LENGTH_SUM      4           // LENGTH OF TWO TIME SECTIONS
#define SENSOR_ID_SECTION_LENGTH            8           // SENSOR ID LENGTH

// CONSISTENT FIRST INDICES PACKET SECTIONS
#define INDEX_TIME_SINCE_LAST_RECEIVE       3           // TIME SINCE LAST RECEIVE
#define INDEX_TIME_SENT                     7           // TIME OF SENDING
#define INDEX_SENSOR_IDS                    11          // SENSOR IDs
#define INDEX_SENSOR_DATA                   19          // SENSOR_DATA

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

        if (this->checkCommStatus()) {
            this->createRTOSTask();
        }
    }

    void addSensortoRegistry(RS485InterfaceSensor* sensor) {
        _registryMap.emplace(sensor->getID(), sensor);
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

    // PACKET DATA, persistent for a complete packet
    uint8_t _packet[DEFAULT_MAX_PAYLOAD]{};
    uint16_t _packetLength{};
    uint32_t _timeSinceLastReceive{};
    uint32_t _timeOfSending{};
    uint8_t _sensorIDS[2 * SENSOR_ID_SECTION_LENGTH]{};

    void calculateSensorIDs(){
        // For loop to add sensor IDs to the array field with the IDs

        for (int i = 0; i < SENSOR_ID_SECTION_LENGTH; i++){
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
            //TODO: Add offset inputting and writing in the registry
        }
    }

    bool validateChecksum(){
        // Validate the checksum given in the packet.
        // Checked by modulo 256 

        uint32_t sum{};
        uint16_t firstChecksumByte{sizeof(_packet) / sizeof(_packet[0]) - 4};
        uint32_t checksum{RS485Utils::convertU32InArr(_packet, firstChecksumByte)};

        // Add each item of the packet together, up to right before our checksum
        for (int i = 0; i < firstChecksumByte - 4; i++) {
            sum += _packet[i];
        }
        return (sum % 256) == (checksum);
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
            }
            break;
        
        case RS485Utils::ParseState::LENGTH:
            if (_currentPacketIndex == 2){
                // Compare our byte with the
                this->_packetLength =
                    static_cast<uint16_t>(this->_packet[1] << 8) |
                    static_cast<uint16_t>(byte);

                if (this->_packetLength > DEFAULT_MAX_LENGTH) {
                    //Reset the values if the length is too long. There is something wrong
                    this->resetParsing();
                    return;
                } else {
                    // This is the correct length for the packet,
                    // save for later
                    this->_packetLength = this->_packetLength;

                    // This is the last index of the data packet. This is the top bound
                    this->_dataLastIndex = 3 + this->_packetLength - SENSOR_TIME_SECTION_LENGTH_SUM;

                    //Change the state to the body
                    this->_currentPacketState = RS485Utils::ParseState::BODY;
                }
            }
            
            // Add the byte
            this->_packet[_currentPacketIndex++] = byte;

            break;
        
        case RS485Utils::ParseState::BODY:
            // Check if we are still within the index
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex >= (this->_dataLastIndex)) {
                // This is the last bit, move on to the next part of the packet
                this->_currentPacketState = RS485Utils::ParseState::CHECKSUM;
            }

            _currentPacketIndex++;
            // Add the last value
            break;
        
        case RS485Utils::ParseState::CHECKSUM:
            // Get the 4 bits for the checksum
            this->_packet[_currentPacketIndex] = byte;
            if (_currentPacketIndex == _dataLastIndex + 4) {
                // We are on the last checksum value
                // We will use the packet to validate the checksum
               if (this->validateChecksum()) {
                    this->extractData();
                }
                this->resetParsing();
            } else {
                // Increment the index, not done with the checksum yet
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