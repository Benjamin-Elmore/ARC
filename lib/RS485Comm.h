#pragma once

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

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

// BINARY HEADERS
#define BINARY_COMM_READ            0b100000    // READING DATA
#define BINARY_COMM_UPDATE          0b010000    // UPDATE VALUES
#define BINARY_COMM_RESET_START     0b111111    // START RESET
#define BINARY_COMM_RESET_CONT      0b111110    // CONTINUE RESET
#define BINARY_COMM_RESET_END       0b101010    // END RESET
#define BINARY_COMM_REBOOT          0b010101    // REBOOT ARSENAL

#define SENSOR_ID_SECTION_LENGTH    8           // LENGTH OF SENSOR ID PACKET

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

    // Checker for the RS485 comms
    bool checkCommStatus(){ return (this->_status == ESP_OK); }

private:
    esp_err_t _status{ESP_ERR_NOT_FINISHED};
    TaskHandle_t _taskObject = nullptr;

    const char* _name{DEFAULT_COMM_NAME};
    const uint8_t _priority{DEFAULT_COMM_PRIORITY};
    const uint8_t _core{DEFAULT_COMM_TASK_CORE};

    size_t _currentBit{};
    uint8_t _lastReadBytes[DEFAULT_COMM_RX_BUFF]{};

    // + 3 due to the header and length bytes
    uint8_t _packet[DEFAULT_MAX_PAYLOAD + 3]{};
    uint16_t _packetLength{};
    uint32_t _timeSinceLastReceive{};
    uint32_t _timeOfSending{};

    void parseByte(uint8_t byte) {
        // Parses the bytes sent through from the RTOS read task and formulates
        // them into a packet if they have the header, the correct length, data, and
        // the checksum

        // The currentBit will not kick off the rest of the read unless we get the header
        // byte

        // HEADER (pos 0, 1 byte, 8 bits)
        if (this->_currentBit == 0) {

            // Compare a Bitwise AND comparison on the 6 bits on the left
            // to determine what we are working with
            // Lshift twice for the comparison.
            // If we are reading, move the packet over

            if ((byte >> 2) == BINARY_COMM_READ) {
                this->_packet[_currentBit] = byte;

                _currentBit++; // Next _currentBit
                return; // This bit is done
            }
        }
        // LENGTH (pos 1-2, 2 bytes, 16 bits)
        else if (_currentBit == 1 || _currentBit == 2) {
            // These are length values, so we will add them if we're on it

            if (_currentBit == 2) {
                // Check if the combined 16 bits of length are larger than the allowed payload

                uint16_t length =
                    static_cast<uint16_t>(this->_packet[1] << 8) |
                    static_cast<uint16_t>(byte);

                if (length > DEFAULT_MAX_PAYLOAD) {
                    //Reset the values if the length is too long. There is something wrong
                    this->_currentBit = 0;
                    return;
                } else {
                    // This is the correct length for the packet,
                    // save for later
                    this->_packetLength = length;
                }
            }

            this->_packet[_currentBit] = byte;
            _currentBit++;
        }
        // TIME SINCE LAST RECEIVE (pos 3-6, 4 bytes, 32 bits)
        else if (_currentBit >= 3 && _currentBit <= 6) {

            this->_packet[_currentBit] = byte;

            // Last Bit of the time
            if (_currentBit == 6){
                uint32_t timeSinceLast = 
                    ((uint32_t)_packet[3] << 24) |
                    ((uint32_t)_packet[4] << 16) |
                    ((uint32_t)_packet[5] << 8) |
                    ((uint32_t)_packet[6]);

                this->_timeSinceLastReceive = timeSinceLast;
            }

            _currentBit++;
        }
        // TIME OF SEND (pos 3-6, 4 bytes, 32 bits)
        else if (_currentBit >= 7 && _currentBit <= 10) {

            this->_packet[_currentBit] = byte;

            // Last Bit of the time
            if (_currentBit == 10){
                uint32_t timeOfSending = 
                    ((uint32_t)_packet[7] << 24) |
                    ((uint32_t)_packet[8] << 16) |
                    ((uint32_t)_packet[9] << 8) |
                    ((uint32_t)_packet[10]);

                this->_timeOfSending = timeOfSending;
            }

            _currentBit++;
        }
        // SensorID's (pos 11-16, 8 bytes, 64 bits)
        else if (_currentBit >= 11 && _currentBit <= 16) {
            
        }
    }

    void getBytesLoop() {
        uint8_t rxBuffer[DEFAULT_COMM_RX_BUFF]{};

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
            &(this->_taskObject),                   //taskHandle_t in SensorParent
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