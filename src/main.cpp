#include "ARC_Sensor.h"
#include "I2CBus.h"
#include "i2cUtils.h"
#include "esp_log.h"

namespace {
    constexpr const char* TAG = "ARC";
    constexpr uint8_t OTOS_ADDRESS = 0x17;
    constexpr uint8_t OTOS_SENSOR_ID = 1;
    constexpr uint8_t OTOS_MUX_CHANNEL = 0;
    constexpr uint8_t OTOS_POSITION_REGISTER = 0x20;
    constexpr uint8_t OTOS_OFFSET_REGISTER = 0x10;

    void readOtosPosition(Sensor<1>& sensor) {
        uint8_t data[6]{};
        if (sensor.readDeviceRegister(OTOS_POSITION_REGISTER, data, sizeof(data)) != ESP_OK) {
            return;
        }
        // OTOS registers are little-endian; preserve signed bits in the wire values.
        sensor.setReadings({
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[0])),
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[2])),
            static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[4]))
        });
    }
}

extern "C" void app_main(void)
{
    // static RS485Comm comm25(2, 3, 4);
    // comm25.setSerialMonitoring(true);
    // if (!comm25.checkCommStatus()) {
    //     ESP_LOGE(TAG, "RS485 UART setup failed");
    //     comm25.serialStatus();
    //     return;
    // }

    static I2CBus<1> bus25(GPIO_NUM_6, GPIO_NUM_7, I2C_NUM_1, GPIO_NUM_NC, {0x70});
    ESP_ERROR_CHECK(bus25.getStatus());

    // Sensor::setup() registers through this pointer; use the existing UART instance.
    // RS485ARSENAL::BaseRS485Instance = &comm25;
    static Sensor<1> opticalSensor(
        "OTOS", bus25, OTOS_ADDRESS, OTOS_SENSOR_ID, true,
        SensorPreconfig::NONE, OTOS_MUX_CHANNEL, DEFAULT_PRIORITY,
        {}, {}, {}, {}, readOtosPosition, OTOS_OFFSET_REGISTER
    );
    const auto sensorStatus = opticalSensor.setup();
    if (sensorStatus != SensorUtils::SENSOR_OK) {
        ESP_LOGE(TAG, "OTOS setup failed: %s", SensorUtils::sensor_status_str(sensorStatus));
        return;
    }

    // Register the sensor before the receiver task makes the registry immutable.
    // const esp_err_t status = comm25.start();
    // if (status != ESP_OK) {
    //     ESP_LOGE(TAG, "RS485 start failed: %s", esp_err_to_name(status));
    //     comm25.serialStatus();
    // }
}
