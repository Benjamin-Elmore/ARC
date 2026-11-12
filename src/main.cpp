#include "RS485Instance.h"
#include "I2CBusInstance.h"
#include "esp_log.h"

namespace {
    constexpr const char* TAG = "ARC";
    constexpr uint8_t TMP117_ADDRESS = 0x48;
    constexpr uint8_t TEMP1_ID = 2;
    constexpr uint8_t TEMP2_ID = 3;
    constexpr uint8_t TEMP3_ID = 4;
    constexpr uint8_t TEMP1_MUX_CHANNEL = 3;   // 0x70, channel 3
    constexpr uint8_t TEMP2_MUX_CHANNEL = 4;   // 0x70, channel 4
    constexpr uint8_t TEMP3_MUX_CHANNEL = 11;  // 0x71, channel 3
}

extern "C" void app_main()
{
    RS485ARSENAL::createRS485Instance();
    RS485ARSENAL::BaseRS485Instance->setSerialMonitoring(true);

    I2CARSENAL::createI2CInstances();
    auto& sensorBus = *I2CARSENAL::multiplexerBus;
    constexpr auto temperature = SensorUtils::SensorConfig::TEMPERATURE;
    static Sensor<2> temp1(
        "temp1", sensorBus, TMP117_ADDRESS, TEMP1_ID, false, temperature, TEMP1_MUX_CHANNEL);
    static Sensor<2> temp2(
        "temp2", sensorBus, TMP117_ADDRESS, TEMP2_ID, false, temperature, TEMP2_MUX_CHANNEL);
    static Sensor<2> temp3(
        "temp3", sensorBus, TMP117_ADDRESS, TEMP3_ID, false, temperature, TEMP3_MUX_CHANNEL);

    for (auto* sensor : {&temp1, &temp2, &temp3}) {
        const auto status = sensor->setup();
        if (status != SensorUtils::SENSOR_OK) {
            ESP_LOGE(TAG, "%s setup failed: %s", sensor->getName().data(),
                     SensorUtils::sensor_status_str(status));
            continue;
        }
        ESP_LOGI(TAG, "%s registered as RS485 sensor ID %u", sensor->getName().data(),
                 static_cast<unsigned>(sensor->getID()));
    }

    ESP_ERROR_CHECK(RS485ARSENAL::startRS485ARSENAL());
    ESP_LOGI(TAG, "Arsenal RS485 listening on UART%d at %d baud (TX=%d RX=%d RTS=%d)",
             RS485_COMM_UART, BAUD_RATE, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN);
}
