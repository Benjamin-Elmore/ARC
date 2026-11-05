#include "RS485Comm.h"
#include "Logging/SdLogger.h"
#include "esp_log.h"
#include "ARC_Sensor.h"

namespace {
    constexpr const char* TAG = "ARC";
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ARSENAL OTOS hardware test: TX=2 RX=3 ENABLE=4, 115200 baud");
    static RS485Comm comm25(2, 3, 4);
    comm25.setSerialMonitoring(true);
    if (!comm25.checkCommStatus()) {
        ESP_LOGE(TAG, "RS485 UART setup failed");
        comm25.serialStatus();
        return;
    }

    // Direct OTOS connection: SCL GPIO9, SDA GPIO8, address 0x17.
    static I2CBus<0> offsetBus(GPIO_NUM_9, GPIO_NUM_8, I2C_NUM_0);
    static Sensor<0> offsetSensor(
        "otos", offsetBus, 0x17, 1, true,
        SensorUtils::SensorConfig::NONE, SensorUtils::UNUSED_MUX,
        DEFAULT_PRIORITY, {}, {}, {}, {},
        {
            {"offset-x", 0, 0x10, 0, 0, SensorUtils::OffsetLayout::REG_16_BIT_LOW_FIRST},
            {"offset-y", 0, 0x12, 0, 1, SensorUtils::OffsetLayout::REG_16_BIT_LOW_FIRST},
            {"offset-h", 0, 0x14, 0, 2, SensorUtils::OffsetLayout::REG_16_BIT_LOW_FIRST}
        },
        +[](Sensor<0>& sensor, LockGuard& lock) {
            uint8_t data[6]{};
            if (sensor.readDeviceRegister(0x20, data, sizeof(data), lock) != ESP_OK) {
                return;
            }
            const auto u16 = [](const uint8_t* bytes) {
                return static_cast<uint16_t>(bytes[0] | (uint16_t(bytes[1]) << 8));
            };
            // Preserve raw signed-position bit patterns for the uint16_t wire format.
            sensor.setReadings({u16(data), u16(data + 2), u16(data + 4)});
        }, 0x10
    );
    RS485ARSENAL::BaseRS485Instance = &comm25;
    if (offsetSensor.setup() != SensorUtils::SENSOR_OK) {
        ESP_LOGE(TAG, "OTOS setup failed");
        return;
    }
    const esp_err_t status = comm25.start();
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "RS485 start failed: %s", esp_err_to_name(status));
        comm25.serialStatus();
        return;
    }
    ESP_LOGI(TAG, "ARSENAL receiver started; OTOS sensor ID 1 registered");

#ifdef ARC_ENABLE_SD_LOGGING
    SDLogging::startSdLogger();
#endif
}

