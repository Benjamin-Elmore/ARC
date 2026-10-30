#include "ARC_Sensor.h"
#include "I2CBus.h"
#include "i2cUtils.h"
#include "esp_log.h"
#include "I2CBusInstance.h"

namespace {
    constexpr const char* TAG = "ARC";
    constexpr uint8_t OTOS_ADDRESS = 0x17;
    constexpr uint8_t OTOS_SENSOR_ID = 1;
    constexpr uint8_t OTOS_MUX_CHANNEL = 3;
    constexpr uint8_t OTOS_POSITION_REGISTER = 0x20;
    constexpr uint8_t OTOS_OFFSET_REGISTER = 0x10;

    void readOtosPosition(Sensor<1>& sensor, LockGuard& lock) {
        uint8_t data[6]{};
        if (sensor.readDeviceRegister(OTOS_POSITION_REGISTER, data, sizeof(data), lock) != ESP_OK) {
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
    ESP_LOGI(TAG, "Starting ARC; sensor serial output enabled");
    static RS485Comm comm25(2, 3, 4);
    // comm25.setSerialMonitoring(true);
    if (!comm25.checkCommStatus()) {
        ESP_LOGE(TAG, "RS485 UART setup failed");
        comm25.serialStatus();
        return;
    }

    ESP_LOGI(TAG, "Initializing I2C: SCL=6 SDA=7 mux=0x70");
    static I2CBus<1> bus25(GPIO_NUM_6, GPIO_NUM_7, I2C_NUM_1, GPIO_NUM_NC, {0x70});
    ESP_ERROR_CHECK(bus25.getStatus());

    // Sensor::setup() registers through this pointer; use the existing UART instance.
    RS485ARSENAL::BaseRS485Instance = &comm25;
    static Sensor<1> opticalSensor(
        "OTOS", bus25, OTOS_ADDRESS, OTOS_SENSOR_ID, true,
        SensorPreconfig::NONE, OTOS_MUX_CHANNEL, DEFAULT_PRIORITY,
        {}, {}, {}, {}, readOtosPosition, OTOS_OFFSET_REGISTER
    );
    ESP_LOGI(TAG, "Setting up OTOS: address=0x%02X mux channel=%u",
             static_cast<unsigned int>(OTOS_ADDRESS),
             static_cast<unsigned int>(OTOS_MUX_CHANNEL));
    const auto sensorStatus = opticalSensor.setup();
    if (sensorStatus != SensorUtils::SENSOR_OK) {
        ESP_LOGE(TAG, "OTOS setup failed: %s", SensorUtils::sensor_status_str(sensorStatus));
        return;
    }

    ESP_LOGI(TAG, "OTOS task started; readings will print to the console");

    // Register the sensor before the receiver task makes the registry immutable.
    const esp_err_t status = comm25.start();
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "RS485 start failed: %s", esp_err_to_name(status));
        comm25.serialStatus();
    }
}
