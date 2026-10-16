// BASE INCLUDES
#pragma once
#include <stdio.h>
#include <functional>
#include <unordered_map>
#include <string_view>
#include <list>

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// OUTSIDE IMPORTS
#include "SensorUtils.h"
#include "LockGuard.h"
#include "Vec3.h"
#include "OpticalPreconfig.h"
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
class Sensor : RS485InterfaceSensor {
    public:
        using FunctionCallback = SensorUtils::FunctionCallback<N>;

        // CONSTRUCTOR:
        // Establish the fields for the sensor to be accessed later
        // If the Sensor is utilized through the mux, then provide the channel that
        // the mux exists on.
        Sensor (
                char* name,                                                                     // NAME of the Sensor
                I2CBus<N>& sensorCommBus,                                                       // BUS CLASS for Sensor
                uint8_t deviceAddrI2C,                                                          // I2C ADDRESS
                uint8_t deviceNumber,                                                           // DEVICE NUMBER
                SensorPreconfig sensorType = SensorPreconfig::NONE,                             // SENSOR PRECONFIG
                uint8_t muxChannel = SensorUtils::UNUSED_MUX,                                   // MUX CHANNEL
                uint8_t priority = DEFAULT_PRIORITY,                                            // SENSOR TASK PRIORITY
                std::initializer_list <FunctionCallback> appendedOnSetup = {},                  // STARTUP FUNCTIONS
                std::initializer_list<FunctionCallback> appendedOnLoop = {},                    // LOOP FUNCTIONS
                std::initializer_list<SensorUtils::AppendedMethod<N>> appendedMethods = {},     // APPENDED METHODS
                std::initializer_list<SensorUtils::AppendedVariable> appendedVariables = {},    // APPENDED VARIABLES
                FunctionCallback sensorReadFunction = NULL                                      // READ FUNCTION
            )
            : _name(name), _devAddrI2C(deviceAddrI2C), _sensorCommBusI2C(sensorCommBus), _muxChannel(muxChannel), _priority(priority),
            _sensorID(deviceNumber)
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
                        this->_setupFuncs.push_back(func);
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
                        this->_loopFuncs.push_back(func);
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
            
            RS485ARSENAL::addSensor(this);
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

        void setup() {
            // Setup the sensor, and start the RTOS task
            // When a child class inherits this class, the super
            // of setup() must be called.

            if(this->_setupRan) {
                this->_status = SensorUtils::SENSOR_ERR_FUNC_CALL;
                return;
            }

            // Add the sensor to the I2C bus with its address identifier
            this->_sensorCommBusI2C.addSensorClassToBus(
                this->getName(),
                this->_muxChannel,
                this->_devAddrI2C
            );

            // Execute the functions in the "on startup" list
            // Only completed if there are functions in the list
            if (!this->_setupFuncs.empty()){
                this->executeFunctionList(this->_setupFuncs);
            }

            // Create the RTOS Task and store it in taskObject
            BaseType_t status = xTaskCreatePinnedToCore
            (
                _taskEntry,             //Task Loop
                this->_name,            //Loop Name
                TASK_STACK_SIZE,        //Stack size
                this,                   //Ptr to task object
                this->_priority,        //Priority
                &(this->_taskObject),   //taskHandle_t in SensorParent
                this->_core             //Core
            );

            if (status != pdFALSE) {
                this->_status = SensorUtils::SENSOR_OK;
                this->_setupRan = true;
            } else {
                this->_status = SensorUtils::SENSOR_ERR_FAULT;
            }
        }

        void appendMethod(std::string_view functionName, FunctionCallback function) {
            // Append a method to the hash table

            if (this->_appendedMethods.size() >= SENSOR_MAX_METHODS) {
                this->_status = SensorUtils::SENSOR_ERR_FULL;
            }
            
            this->_appendedMethods.emplace(functionName, function);
        }

        void appendVariable(std::string_view variableName, uint8_t variable) {
            // Append a variable to the hash table

            if (this->_appendedVariables.size() >= SENSOR_USER_SLOTS) {
                this->_status = SensorUtils::SENSOR_ERR_FULL;
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

        uint8_t getAppendedVariable(char* variableName) {
            if (this->_appendedVariables.contains(variableName)){
                return this->_appendedVariables.at(variableName);
            } else {
                this->_status = SensorUtils::SENSOR_ERR_NOT_FOUND;
                // Return a sentinel value
                return SensorUtils::FAULTY_VARIABLE;
            }
        }

        void setAppendedVariable(std::string_view variableName, uint8_t data) {
            if (this->_appendedVariables.contains(variableName)) {
                this->_appendedVariables.at(variableName) = data;
            } else {
                this->_status = SensorUtils::SENSOR_ERR_ARG;
            }
        }

        void appendSetupFunc(FunctionCallback function) {
            if (this->_setupFuncs.size() >= SENSOR_MAX_ON_SETUP) {
                // Too many sensor in the setup
                this->_status = SensorUtils::SENSOR_ERR_FULL;
            } else {
                this->_setupFuncs.push_back(function);
            }
        }
        void appendLoopFunc(FunctionCallback function) {
            if (this->_loopFuncs.size() > SENSOR_MAX_ON_LOOP) {
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

        uint8_t getID() { return this->_sensorID; }

        // WRAPPERS FOR I2CBus
        esp_err_t readDeviceRegister(const uint8_t regAddr, uint8_t *data, const size_t length) {
            return this->_sensorCommBusI2C.readRegister(this->_name, regAddr, data, length);
        }
        
        esp_err_t writeDeviceRegister(uint8_t regAddr, uint8_t data) {
            return this->_sensorCommBusI2C.writeRegister(this->_name, regAddr, data);
        }

        // OFFSET SETTER
        void setOffsets(Vec3<uint16_t> o) override {
            this->_offsets.x = o.x;
            this->_offsets.y = o.y;
            this->_offsets.z = o.z;
        }

    protected:
        // Meant to be interfaced with in child classes

        // TASK OBJECT
        TaskHandle_t _taskObject = nullptr;

        // Constructor-defined variables
        char* _name;

        FunctionCallback _readFunction = NULL;

        // CRITICAL DATA:
        Vec3<uint16_t> _offsets{};
        uint8_t _sensorID{};

        //OPTIONAL MEMBERS:
        uint8_t _devAddrI2C = SensorUtils::ADDR_NOT_INCLUDED;

        // SETUP AND LOOP FUNCTIONS, stored in linked list
        // Pointers to the head of each function list:
        std::list<FunctionCallback> _setupFuncs{};
        std::list<FunctionCallback> _loopFuncs{};

        //APPENDED METHODS AND VARIABLES, stored in hash tables
        std::unordered_map<std::string_view, FunctionCallback> _appendedMethods = {};
        std::unordered_map<std::string_view, uint16_t> _appendedVariables = {};

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
            for (;;) {
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
                        this->_readFunction(*this);
                    }
                }

                //Execute Loop Functions outside of mutex lock
                if (!this->_loopFuncs.empty()){
                    this->executeFunctionList(this->_loopFuncs);
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

    private:
        //STATUS FIELDS
        I2CBus<N>& _sensorCommBusI2C;
        uint8_t _muxChannel;

        uint8_t _core{1};
        uint8_t _priority{DEFAULT_PRIORITY};

        bool _exceededMaximums{false};
        bool _setupRan{false};
        SensorUtils::sensor_status_t _status {SensorUtils::SENSOR_ERR_NOT_READY};

        // WRAPPER ON TASK LOOP for RTOS Task
        static void _taskEntry(void* ptr) {
            reinterpret_cast<Sensor*>(ptr)->taskLoop();
        }
};
