#include "ARC_Sensor.h"
#include "I2CBusInstance.h"
#include "i2cUtils.h"
#include "esp_log.h"

namespace {
    constexpr const char* TAG = "ARC";
    constexpr uint8_t OTOS_ADDRESS = 0x17;
    constexpr uint8_t OTOS_SENSOR_ID = 1;
    constexpr uint8_t OTOS_MUX_CHANNEL = 0;
    constexpr uint8_t OTOS_POSITION_REGISTER = 0x20;
    constexpr uint8_t OTOS_OFFSET_REGISTER = 0x10;

    void readOtosPosition(Sensor<2>& sensor) {
        uint8_t data[6]{};
        if (sensor.readDeviceRegister(OTOS_POSITION_REGISTER, data, sizeof(data)) != ESP_OK) {
            return;
        }

        // Preserve the signed OTOS register bits in the existing uint16_t wire format.
        sensor.setReadings({
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[0])),
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[2])),
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[4]))
        });
    }
}

extern "C" void app_main(void)
{
    I2CARSENAL::createI2CInstances();
    RS485ARSENAL::createRS485Instance();

    ESP_ERROR_CHECK(I2CARSENAL::multiplexerBus->getStatus());

    // Firmware-lifetime instance; register it before starting the RS485 receiver.
    // Readings are raw OTOS register values (x, y, heading), without unit conversion.
    static Sensor<2> baseSensor(
        "OTOS",
        *I2CARSENAL::multiplexerBus,
        OTOS_ADDRESS,
        OTOS_SENSOR_ID,
        SensorPreconfig::NONE,
        OTOS_MUX_CHANNEL,
        DEFAULT_PRIORITY,
        {}, {}, {}, {},
        readOtosPosition,
        OTOS_OFFSET_REGISTER
    );

    const SensorUtils::sensor_status_t sensorStatus = baseSensor.setup();
    if (sensorStatus != SensorUtils::SENSOR_OK) {
        ESP_LOGE(TAG, "OTOS setup failed: %s", SensorUtils::sensor_status_str(sensorStatus));
        return;
    }

    ESP_ERROR_CHECK(RS485ARSENAL::startRS485ARSENAL());
}
