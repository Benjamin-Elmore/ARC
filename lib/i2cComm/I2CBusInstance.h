#pragma once

#include "I2CBus.h"
#include "ARC_Sensor.h"
#include "LockGuard.h"

#define I2C_INSTANCE_MUX_SCL GPIO_NUM_9         // MULTIPLEXER BUS SCL
#define I2C_INSTANCE_MUX_SDA GPIO_NUM_8         // MULTIPLEXER BUS SDA
#define I2C_INSTANCE_MUX_RESET GPIO_NUM_7       // MULTIPLEXER BUS RESET
#define I2C_INSTANCE_DIRECT_SCL GPIO_NUM_6      // DIRECT BUS SCL
#define I2C_INSTANCE_DIRECT_SDA GPIO_NUM_5      // DIRECT BUS SDA

namespace I2CARSENAL {
    inline I2CBus<2>* multiplexerBus = nullptr;
    inline I2CBus<0>* directBus = nullptr;

    enum Bus {
        MULTIPLEXER_BUS,
        DIRECT_BUS
    };

    struct I2CSharedInstance {
        I2CBus<2> multiplexerBus;
        I2CBus<0> directBus;

        I2CSharedInstance(
            gpio_num_t muxGpioSCL,
            gpio_num_t muxGpioSDA,
            gpio_num_t muxGpioReset,
            gpio_num_t directGpioSCL,
            gpio_num_t directGpioSDA,
            std::array<uint8_t, 2> muxAddrs
        )
            : multiplexerBus(
                muxGpioSCL,         // SCL
                muxGpioSDA,         // SDA
                I2C_NUM_0,          // I2C PORT
                muxGpioReset,       // RESET PIN
                muxAddrs            // MUX ADDRESSES
            ),
            directBus(  // Mux reset and addresses not needed.
                directGpioSCL,      // SCL
                directGpioSDA,      // SDA
                I2C_NUM_1           // I2C PORT
            ) {}
    };

    inline void createI2CInstances() {
        static I2CSharedInstance instance(
            I2C_INSTANCE_MUX_SCL,
            I2C_INSTANCE_MUX_SDA,
            I2C_INSTANCE_MUX_RESET,
            I2C_INSTANCE_DIRECT_SCL,
            I2C_INSTANCE_DIRECT_SDA,
            {0x70, 0x71}
        );

        multiplexerBus = &instance.multiplexerBus;
        directBus = &instance.directBus;   
    }

    template <size_t N>
    inline esp_err_t addSensorToBus(Bus targetBus, Sensor<N>& sensor, LockGuard& lock) {
        if (multiplexerBus == nullptr || directBus == nullptr) {
            createI2CInstances();
        }
        
        I2CBus<N>* operatingBus = nullptr;

        switch (targetBus) {
            case MULTIPLEXER_BUS:
                if constexpr (N == 2) {
                    operatingBus = multiplexerBus;
                }
            break;
            case DIRECT_BUS:
                if constexpr (N == 0){
                    operatingBus = directBus;
                }
            break;
            default:
            break;
        }

        if (operatingBus == nullptr) {
            // There was an error, this ensures we escape before dereferencing
            // a null ptr.
            return ESP_ERR_INVALID_ARG;
        }

        if (!lock.isMutexLocked()) {
            return ESP_ERR_INVALID_STATE;
        }

        return operatingBus->addSensorToBus(
            sensor.getName(),
            sensor.getMuxI2C(),
            sensor.getAddressI2C(),
            lock
        );
    }

    template <size_t N>
    inline esp_err_t removeSensorFromBus(Bus targetBus, Sensor<N>& sensor) {
        if (multiplexerBus == nullptr || directBus == nullptr) {
            createI2CInstances();
        }

        I2CBus<N>* operatingBus = nullptr;

        switch (targetBus) {
            case MULTIPLEXER_BUS:
                if constexpr (N == 2) {
                    operatingBus = multiplexerBus;
                }
            break;
            case DIRECT_BUS:
                if constexpr (N == 0){
                    operatingBus = directBus;
                }
            break;
            default:
            break;
        }

        if (operatingBus == nullptr) {
            // There was an error, this ensures we escape before dereferencing
            // a null ptr.
            return ESP_ERR_INVALID_ARG;
        }

        LockGuard lock(operatingBus->getMutex());
        if (!lock.isMutexLocked()) {
            return ESP_ERR_INVALID_STATE;
        }

        return operatingBus->removeSensorFromBus(sensor.getName(), lock);
    }
}
