// BASE INCLUDES
#pragma once
#include <stdio.h>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <string_view>
#include <list>
#include <string>
#include <atomic>
#include <cstdio>
#include <cinttypes>

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// OUTSIDE IMPORTS
#include "SensorUtils.h"
#include "LockGuard.h"
#include "Vec3.h"
// #include "OpticalPreconfig.h" // Deferred until optical preset work resumes.
#include "I2CBus.h"
#include "RS485Comm.h"
#include "RS485InterfaceSensor.h"
#include "RS485Instance.h"

// MAXIMUM VALUES PER SENSOR
#ifndef SENSOR_MAX_FILTERS
#define SENSOR_MAX_FILTERS 4  // Maximum filter callbacks stored per sensor.
#endif
#ifndef SENSOR_MAX_METHODS
#define SENSOR_MAX_METHODS 6  // Maximum named command callbacks per sensor.
#endif
#ifndef SENSOR_MAX_ON_SETUP
#define SENSOR_MAX_ON_SETUP 4  // Maximum named command callbacks per sensor.
#endif
#ifndef SENSOR_MAX_ON_LOOP
#define SENSOR_MAX_ON_LOOP 4  // Maximum named command callbacks per sensor.
#endif
#ifndef SENSOR_USER_SLOTS
#define SENSOR_USER_SLOTS  4  // Number of per-sensor int scratch values.
#endif
#ifndef TASK_STACK_SIZE
#define TASK_STACK_SIZE 2048
#endif
#ifndef DEFAULT_PRIORITY
#define DEFAULT_PRIORITY 2
#endif
#ifndef LOCK_FAIL_DELAY
#define LOCK_FAIL_DELAY 50
#endif



enum FunctionReferences {
    ON_SETUP,
    ON_LOOP
};

enum SensorPreconfig {
    NONE,
    OPTICAL,
    COLOR,
    ENCODER
};
template <size_t N>
class Sensor : public RS485InterfaceSensor {
    public:
        using FunctionCallback = SensorUtils::FunctionCallback<N>;

        // CONSTRUCTOR:
        // Establish the fields for the sensor to be accessed later
        // If the Sensor is utilized through the mux, then provide the channel that
        // the mux exists on.
        Sensor (
                const char* name,                                                               // NAME of the Sensor
                I2CBus<N>& sensorCommBus,                                                       // BUS CLASS for Sensor
                uint8_t deviceAddrI2C,                                                          // I2C ADDRESS
                uint8_t deviceID,                                                               // DEVICE ID
                bool printReadings = false,                                                     // PRINT READINGS
                SensorPreconfig sensorType = SensorPreconfig::NONE,                             // SENSOR PRECONFIG
                uint8_t muxChannel = SensorUtils::UNUSED_MUX,                                   // MUX CHANNEL
                uint8_t priority = DEFAULT_PRIORITY,                                            // SENSOR TASK PRIORITY
                std::initializer_list <FunctionCallback> appendedOnSetup = {},                  // STARTUP FUNCTIONS
                std::initializer_list<FunctionCallback> appendedOnLoop = {},                    // LOOP FUNCTIONS
                std::initializer_list<SensorUtils::AppendedMethod<N>> appendedMethods = {},     // APPENDED METHODS
                std::initializer_list<SensorUtils::AppendedVariable> appendedVariables = {},    // APPENDED VARIABLES
                FunctionCallback sensorReadFunction = NULL,                                     // READ FUNCTION
                uint8_t offsetMemReg = SensorUtils::UNUSED_OFFSET_REG                           // MEM REG FOR OFFSETS
            )
            : _name(name == nullptr ? "" : name),
              _outputReadings(printReadings),
              _sensorID(deviceID),
              _offsetMemReg(offsetMemReg),
              _devAddrI2C(deviceAddrI2C),
              _sensorCommBusI2C(sensorCommBus),
              _muxChannel(muxChannel),
              _priority(priority)
        {

            if (sensorType != SensorPreconfig::NONE){
                //TODO: Add preconfig function for reader
            }
            // SETUP AND LOOP SETUP
            // the FuncHead fields hold the head of the linked list
            if (appendedOnSetup.size() != 0){
                // Appended functions are in the init list
                
                if (appendedOnSetup.size() > SENSOR_MAX_ON_SETUP) {
                    // Too many sensors in the setup
                    this->_status = SensorUtils::SENSOR_ERR_ARG;
                } else {
                    for (const FunctionCallback func : appendedOnSetup) {
                        appendSetupFunc(func);
                    }
                }
            }
            if (appendedOnLoop.size() != 0){
                // Appended functions are in the init list

                if (appendedOnLoop.size() > SENSOR_MAX_ON_LOOP) {
                    // Too many sensors in the loop
                    this->_status = SensorUtils::SENSOR_ERR_ARG;
                } else {
                    for (const FunctionCallback func : appendedOnLoop) {
                        appendLoopFunc(func);
                    }
                }
            }
            
            // APPENDED METHODS SETUP
            for (const SensorUtils::AppendedMethod<N>& method : appendedMethods) {
                this->appendMethod(method.methodName, method.method);
            }
            for (const SensorUtils::AppendedVariable& variable : appendedVariables) {
                this->appendVariable(variable.variableName, variable.variable);
            }

            if (sensorReadFunction != NULL) {
                this->_readFunction = sensorReadFunction;
            } else{
                this->_status = SensorUtils::SENSOR_ERR_NO_READER;
            }

            // Throw error with a problem for any of the key arguments
            if (_name.empty() || _sensorID == 0 || _sensorID > 15) {
                _status = SensorUtils::SENSOR_ERR_ARG;
            }
        }

        // DESTRUCTOR:
        // Default for the sensor
        ~Sensor() {
            // If needed, end the RTOS task when the sensor is destroyed
            // Set the taskHandle_t to not point to anything
            if (this->_taskObject != NULL) {
                vTaskDelete(this->_taskObject);
                this->_taskObject = NULL;
            }
        }

        // NON COPY-ABILITY for the sensor class
        // This prevents misues of copied instances
        Sensor(const Sensor&) = delete;
        Sensor& operator=(const Sensor&) = delete;

        SensorUtils::sensor_status_t setup() {
            // Setup the sensor, and start the RTOS task
            // When a child class inherits this class, the super
            // of setup() must be called.

            if(this->_setupRan) {
                this->_status = SensorUtils::SENSOR_ERR_FUNC_CALL;
                return SensorUtils::SENSOR_ERR_FUNC_CALL;
            }

            if (_status != SensorUtils::SENSOR_ERR_NOT_READY &&
                _status != SensorUtils::SENSOR_OK) {
                return _status;
            }

            // A failed bus setup has no mutex to wait for.
            if (_sensorCommBusI2C.getMutex() == nullptr) {
                _status = SensorUtils::SENSOR_ERR_IO;
                return _status;
            }

            for (;;) {
                LockGuard lock(_sensorCommBusI2C.getMutex());
                if (!lock.isMutexLocked()) {
                    vTaskDelay(pdMS_TO_TICKS(LOCK_FAIL_DELAY));
                    continue;
                }

                const esp_err_t busStatus = _sensorCommBusI2C.addSensorToBus(
                    getName(), _muxChannel, _devAddrI2C
                );
                if (busStatus != ESP_OK) {
                    _status = SensorUtils::SENSOR_ERR_IO;
                    return _status;
                }

                const esp_err_t muxStatus = _sensorCommBusI2C.selectMuxChannel(getName());
                if (muxStatus != ESP_OK) {
                    _sensorCommBusI2C.removeSensorFromBus(this->getName());
                    _status = SensorUtils::SENSOR_ERR_IO;

                    return _status;
                }

                executeFunctionList(_setupFuncs);

                // Check the status of the sensor after running custom
                // setup functions
                if (_status != SensorUtils::SENSOR_ERR_NOT_READY &&
                    _status != SensorUtils::SENSOR_OK) {
                    return _status;
                }

                break; // Release the bus lock before creating the task.
            }

            // Register Sensor on the RS485Comm
            const esp_err_t regStatus = this->registerSensor();

            if (regStatus != ESP_OK) {
                this->_status = SensorUtils::SENSOR_ERR_IO;
                this->fullStopFAIL();
                return SensorUtils::SENSOR_ERR_IO;
            }

            // The sensor has been registered for RS485
            this->_status = SensorUtils::SENSOR_OK;

            // Create the RTOS Task and store it in taskObject
            BaseType_t taskStatus = xTaskCreatePinnedToCore
            (
                _taskEntry,             //Task Loop
                this->_name.c_str(),    //Loop Name
                TASK_STACK_SIZE,        //Stack size
                this,                   //Ptr to task object
                this->_priority,        //Priority
                &(this->_taskObject),   //taskHandle_t in SensorParent
                this->_core             //Core
            );

            if (taskStatus != pdPASS) {
                this->_status = SensorUtils::SENSOR_ERR_FAULT;
                this->fullStopFAIL();
                return SensorUtils::SENSOR_ERR_FAULT;
            }

            this->_setupRan = true;

            return SensorUtils::SENSOR_OK;
        }

        void requestStop() {
            _stopRequested.store(true);
        }

        void appendMethod(std::string_view functionName, FunctionCallback function) {
            // Append a method to the hash table

            if (function == nullptr) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return;
            }

            if (_appendedMethods.contains(functionName)) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return;
            }

            if (this->_appendedMethods.size() >= SENSOR_MAX_METHODS) {
                this->_status = SensorUtils::SENSOR_ERR_FULL;
                return;
            }
            
            this->_appendedMethods.emplace(functionName, function);
        }

        void appendVariable(std::string_view variableName, uint16_t variable) {
            // Append a variable to the hash table

            if (_appendedVariables.contains(variableName)) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return;
            }

            if (this->_appendedVariables.size() >= SENSOR_USER_SLOTS) {
                this->_status = SensorUtils::SENSOR_ERR_FULL;
                return;
            }
            
            this->_appendedVariables.emplace(variableName, variable);
        }

        void executeAppendedMethod(char* methodName) {
            if (this->_appendedMethods.contains(methodName)) {
                this->_appendedMethods.at(methodName)(*this);
            } else {
                this->_status = SensorUtils::SENSOR_ERR_NOT_FOUND;
            }
        }

        uint16_t getAppendedVariable(char* variableName) {
            if (this->_appendedVariables.contains(variableName)){
                return this->_appendedVariables.at(variableName);
            } else {
                this->_status = SensorUtils::SENSOR_ERR_NOT_FOUND;
                // Return a sentinel value
                return SensorUtils::FAULTY_VARIABLE;
            }
        }

        void setAppendedVariable(std::string_view variableName, uint16_t data) {
            if (this->_appendedVariables.contains(variableName)) {
                this->_appendedVariables.at(variableName) = data;
            } else {
                this->_status = SensorUtils::SENSOR_ERR_ARG;
            }
        }

        void appendSetupFunc(FunctionCallback function) {
            if (function == nullptr) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return;
            }

            if (this->_setupFuncs.size() >= SENSOR_MAX_ON_SETUP) {
                // Too many sensor in the setup
                this->_status = SensorUtils::SENSOR_ERR_FULL;
            } else {
                this->_setupFuncs.push_back(function);
            }
        }
        void appendLoopFunc(FunctionCallback function) {
            if (function == nullptr) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return;
            }

            if (this->_loopFuncs.size() >= SENSOR_MAX_ON_LOOP) {
                // Too many sensors in the loop
                this->_status = SensorUtils::SENSOR_ERR_FULL;
            } else {
                this->_loopFuncs.push_back(function);
            }
        }

        // GETTERS

        uint8_t getAddressI2C() { return this->_devAddrI2C; }

        std::string_view getName() override { return this->_name; }

        uint8_t getMuxI2C() { return this->_muxChannel; }

        uint8_t getID() override { return this->_sensorID; }

        void printReadings() {
            printf("%s READINGS", std::string(this->_name).c_str());
            printf("X: %d", this->_reading.x);
            printf("Y: %d", this->_reading.y);
            printf("H: %d", this->_reading.z);

            return;
        }

        // WRAPPERS FOR I2CBus
        esp_err_t readDeviceRegister(const uint8_t regAddr, uint8_t *data, const size_t length) {
            esp_err_t status = this->_sensorCommBusI2C.readRegister(this->_name, regAddr, data, length);
            this->confirmStatus(status);
            return status;
        }
        
        esp_err_t writeDeviceRegister(uint8_t regAddr, uint8_t data) {
            esp_err_t status = this->_sensorCommBusI2C.writeRegister(this->_name, regAddr, data);
            this->confirmStatus(status);
            return status;
        }

        // OFFSET SETTER
        esp_err_t setOffsets(Vec3<uint16_t> o) override {
            this->_offsets.x = o.x;
            this->_offsets.y = o.y;
            this->_offsets.z = o.z;

            // Change the memory registry for the sensor offsets
            // Return the success
            return this->setSensorOffsets();
        }

        void setReadings(const Vec3<uint16_t>& reading) {
            portENTER_CRITICAL(&_readingMux);
            _reading = reading;
            portEXIT_CRITICAL(&_readingMux);
        }

        Vec3<uint16_t> getReadings() override {
            portENTER_CRITICAL(&_readingMux);
            Vec3<uint16_t> reading = {this->_reading.x, this->_reading.y, this->_reading.z};
            portEXIT_CRITICAL(&_readingMux);
            return reading;
        }

        esp_err_t registerSensor() override {
            return RS485ARSENAL::addSensor(this);
        }

    protected:
        // Meant to be interfaced with in child classes

        // TASK OBJECT
        TaskHandle_t _taskObject = nullptr;

        // Constructor-defined variables
        std::string _name;
        bool _outputReadings{};

        FunctionCallback _readFunction = NULL;
        
        // CRITICAL DATA:
        Vec3<uint16_t> _offsets{};
        uint8_t _sensorID{};
        uint8_t _offsetMemReg{};

        Vec3<uint16_t> _reading{};

        //OPTIONAL MEMBERS:
        uint8_t _devAddrI2C = SensorUtils::ADDR_NOT_INCLUDED;

        // SETUP AND LOOP FUNCTIONS, stored in linked list
        // Pointers to the head of each function list:
        std::list<FunctionCallback> _setupFuncs{};
        std::list<FunctionCallback> _loopFuncs{};

        //APPENDED METHODS AND VARIABLES, stored in hash tables
        std::unordered_map<std::string_view, FunctionCallback> _appendedMethods = {};
        std::unordered_map<std::string_view, uint16_t> _appendedVariables = {};

        void fullStopFAIL() {
            if (_taskObject != nullptr) {
                vTaskDelete(_taskObject);
                _taskObject = nullptr;
            }
        }

        void confirmStatus(esp_err_t status) {
            // Check the status of the mux, and if it's not ESP_OK,
            // change the bus's status.

            if (status != ESP_OK &&
                (_status == SensorUtils::SENSOR_OK || _status == SensorUtils::SENSOR_ERR_NOT_READY)) {
                this->_status = SensorUtils::SENSOR_ERR_IO;
            }
        }

        esp_err_t setSensorOffsets() {

            if (_offsetMemReg == SensorUtils::UNUSED_OFFSET_REG) {
                _status = SensorUtils::SENSOR_ERR_ARG;
                return _status;
            }

            if (
                _offsets.x != SensorUtils::UNUSED_OFFSETS &&
                _offsets.y != SensorUtils::UNUSED_OFFSETS &&
                _offsets.z != SensorUtils::UNUSED_OFFSETS
            ) {
                if (_sensorCommBusI2C.getMutex() == nullptr) {
                    _status = SensorUtils::SENSOR_ERR_IO;
                    return _status;
                }

                for(;;) {
                    // Run through this loop to wait for the mutex
                    LockGuard lock(this->_sensorCommBusI2C.getMutex());

                    if (lock.isMutexLocked()) {
                        // Select the mux channel
                        const esp_err_t muxStatus = _sensorCommBusI2C.selectMuxChannel(this->getName());

                        confirmStatus(muxStatus);

                        if (muxStatus != ESP_OK) {
                            return muxStatus;
                        }

                        // Create the write buffer for the offsets
                        uint8_t offsetWriteBuf[6]{};
                        SensorUtils::convertOffsetsToArr(offsetWriteBuf, _offsets);

                        // Write the buffer
                        esp_err_t status = _sensorCommBusI2C.writeArrayRegister(
                            this->getName(),
                            _offsetMemReg,
                            offsetWriteBuf
                        );
                        
                        confirmStatus(status);
                        return _status;
                    }

                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            return ESP_OK;
        }

        void executeFunctionList(std::list<FunctionCallback>& funcList) {
            // Execute all of the functions in the linked list

            for (const FunctionCallback func : funcList){
                // Run each function with a reference to the Sensor
                func(*this);
            }
        }

        // TASK LOOP utilized within the actual RTOS task
        void taskLoop() {
            // Loop on RTOS TASK
            while (!_stopRequested.load()) {
                if (this->_readFunction == NULL) {
                    // The reader function does not exist, the delay needs to go in place
                    // This task is useless without the critical read function

                    vTaskDelay(pdMS_TO_TICKS(500));
                    this->_status = SensorUtils::SENSOR_ERR_NO_READER;
                }

                {
                    // Lock the bus mutex with a Lockguard, if it is not currently available
                    // we will run a delay on the mutex
                    LockGuard loopLock = LockGuard(this->_sensorCommBusI2C.getMutex());
                    if (!loopLock.isMutexLocked()) {
                        // Run a delay on the RTOS task, the mutex is not
                        // currently available
                        vTaskDelay(pdMS_TO_TICKS(LOCK_FAIL_DELAY));

                    } else if (this->_readFunction != NULL) {
                        // Run the preconfigured read function
                        // The if statement is for safety
                        const esp_err_t muxStatus = _sensorCommBusI2C.selectMuxChannel(this->getName());

                        if (muxStatus != ESP_OK){
                            this->confirmStatus(muxStatus);
                        } else {
                            this->_readFunction(*this);
                        }
                    }
                }

                //Execute Loop Functions outside of mutex lock
                if (!this->_loopFuncs.empty()){
                    this->executeFunctionList(this->_loopFuncs);
                }

                if (_outputReadings) {
                    this->printReadings();
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // Exits when the stop is requested
            vTaskDelete(nullptr);
        }

    private:
        //STATUS FIELDS
        I2CBus<N>& _sensorCommBusI2C;
        uint8_t _muxChannel;

        uint8_t _core{1};
        uint8_t _priority{DEFAULT_PRIORITY};

        bool _exceededMaximums{false};
        bool _setupRan{false};
        // _status is atomic as the RS485Comm task has a route to change this from
        // its task loop
        std::atomic<SensorUtils::sensor_status_t> _status = {SensorUtils::SENSOR_ERR_NOT_READY};
        portMUX_TYPE _readingMux = portMUX_INITIALIZER_UNLOCKED;
        std::atomic<bool> _stopRequested{false};

        // WRAPPER ON TASK LOOP for RTOS Task
        static void _taskEntry(void* ptr) {
            reinterpret_cast<Sensor*>(ptr)->taskLoop();
        }
};
