#include <array>
#include <cstdint>

#include "RS485Comm.h"
#include "ARC_Sensor.h"
#include "Logging/SdLogger.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "RS485TransceiverTest.h"

namespace {
    constexpr const char* TAG = "ARC";
    constexpr const char* TEMPERATURE_TAG = "TEMPERATURE";
    constexpr int RS485_TX_GPIO = 17;
    constexpr int RS485_RX_GPIO = 18;
    constexpr int RS485_ENABLE_GPIO = 14;
    constexpr std::array<uint8_t, 2> MUX_ADDRESSES{0x70, 0x71};
    constexpr uint8_t TMP117_ADDRESS = 0x48;
    constexpr uint8_t TEMP1_SENSOR_ID = 2;
    constexpr uint8_t TEMP2_SENSOR_ID = 3;
    constexpr uint8_t TEMP3_SENSOR_ID = 4;
    constexpr uint8_t TEMP1_MUX_CHANNEL = 3;
    constexpr uint8_t TEMP2_MUX_CHANNEL = 4;
    constexpr uint8_t TEMP3_MUX_CHANNEL = 11;
    constexpr uint32_t TEMPERATURE_REPORT_INTERVAL_MS = 1000;
    constexpr uint32_t TEMPERATURE_DIAGNOSTIC_STACK_BYTES = 8192;
}

extern "C" void app_main(void)
{
    constexpr bool runRawTransceiverTest = true;
    if (runRawTransceiverTest) {
        runRS485TransceiverTest();
        return;
    }

    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set(TEMPERATURE_TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "========== ARSENAL TMP117 HARDWARE TEST ==========");
    ESP_LOGI(TAG, "Starting Arsenal temperature test: TX=%d RX=%d ENABLE=%d, 115200 baud",
             RS485_TX_GPIO, RS485_RX_GPIO, RS485_ENABLE_GPIO);

    ESP_LOGI(TAG, "Creating RS485 UART and enabling packet diagnostics");
    static RS485Comm arsenalComm(RS485_TX_GPIO, RS485_RX_GPIO, RS485_ENABLE_GPIO);
    arsenalComm.setSerialMonitoring(true);
    if (!arsenalComm.checkCommStatus()) {
        ESP_LOGE(TAG, "RS485 UART setup failed");
        arsenalComm.serialStatus();
        return;
    }
    RS485ARSENAL::BaseRS485Instance = &arsenalComm;
    ESP_LOGI(TAG, "RS485 UART ready; ARSENAL shared instance assigned");
    arsenalComm.serialStatus();

    ESP_LOGI(TAG, "Creating I2C bus 0: SCL=GPIO9 SDA=GPIO8 RESET=GPIO7; muxes U2=0x70 U7=0x71");
    static I2CBus<2> sensorBus(
        GPIO_NUM_9, GPIO_NUM_8, I2C_NUM_0, GPIO_NUM_7, MUX_ADDRESSES);
    if (sensorBus.getMutex() == nullptr) {
        ESP_LOGE(TAG, "I2C bus initialization failed; stopping temperature test");
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized; creating three TMP117 sensors at address 0x48");
    ESP_LOGI(TAG, "TMP117 preset: read register=0x00, offset register=0x07, MSB first, offset default=0");

    static Sensor<2> temperature1(
        "temp1", sensorBus, TMP117_ADDRESS, TEMP1_SENSOR_ID, false,
        SensorUtils::SensorConfig::TEMPERATURE, TEMP1_MUX_CHANNEL);
    static Sensor<2> temperature2(
        "temp2", sensorBus, TMP117_ADDRESS, TEMP2_SENSOR_ID, false,
        SensorUtils::SensorConfig::TEMPERATURE, TEMP2_MUX_CHANNEL);
    static Sensor<2> temperature3(
        "temp3", sensorBus, TMP117_ADDRESS, TEMP3_SENSOR_ID, false,
        SensorUtils::SensorConfig::TEMPERATURE, TEMP3_MUX_CHANNEL);

    static const std::array<Sensor<2>*, 3> temperatureSensors{
        &temperature1, &temperature2, &temperature3};

    for (Sensor<2>* sensor : temperatureSensors) {
        ESP_LOGI(TAG, "Setting up %s: RS485 ID=%u mux=0x%02X physical channel=%u combined channel=%u",
                 sensor->getName().data(), static_cast<unsigned>(sensor->getID()),
                 static_cast<unsigned>(MUX_ADDRESSES[sensor->getMuxI2C() / 8]),
                 static_cast<unsigned>(sensor->getMuxI2C() % 8), static_cast<unsigned>(sensor->getMuxI2C()));
        const SensorUtils::sensor_status_t setupStatus = sensor->setup();
        if (setupStatus != SensorUtils::SENSOR_OK) {
            ESP_LOGE(TAG, "%s setup failed: %s", sensor->getName().data(),
                     SensorUtils::sensor_status_str(setupStatus));
            return;
        }
        ESP_LOGI(TAG, "%s ready: RS485 ID %u, I2C address 0x%02X, mux channel %u",
                 sensor->getName().data(), static_cast<unsigned>(sensor->getID()),
                 static_cast<unsigned>(TMP117_ADDRESS),
                 static_cast<unsigned>(sensor->getMuxI2C()));
        ESP_LOGI(TAG, "%s registered with ARSENAL RS485; sensor polling task started",
                 sensor->getName().data());

        LockGuard busLock(sensorBus.getMutex());
        if (!busLock.isMutexLocked()) {
            ESP_LOGW(TAG, "%s register diagnostics skipped: bus lock unavailable", sensor->getName().data());
            continue;
        }
        const esp_err_t muxStatus = sensorBus.selectMuxChannel(sensor->getName(), busLock);
        if (muxStatus != ESP_OK) {
            ESP_LOGE(TAG, "%s diagnostic mux selection failed: %s",
                     sensor->getName().data(), esp_err_to_name(muxStatus));
            continue;
        }
        constexpr std::array<uint8_t, 3> diagnosticRegisters{0x0F, 0x01, 0x07};
        constexpr std::array<const char*, 3> registerNames{"device ID", "configuration", "temperature offset"};
        for (size_t index = 0; index < diagnosticRegisters.size(); ++index) {
            uint8_t registerBytes[2]{};
            const esp_err_t readStatus = sensor->readDeviceRegister(
                diagnosticRegisters[index], registerBytes, sizeof(registerBytes), busLock);
            if (readStatus == ESP_OK) {
                const unsigned registerValue = (unsigned(registerBytes[0]) << 8) | registerBytes[1];
                ESP_LOGI(TAG, "%s %s [0x%02X] = 0x%04X", sensor->getName().data(),
                         registerNames[index], static_cast<unsigned>(diagnosticRegisters[index]), registerValue);
                if (index == 0 && registerValue != 0x0117) {
                    ESP_LOGW(TAG, "%s unexpected device ID; expected TMP117 ID 0x0117", sensor->getName().data());
                }
            } else {
                ESP_LOGE(TAG, "%s %s read failed: %s", sensor->getName().data(),
                         registerNames[index], esp_err_to_name(readStatus));
            }
        }
    }

    arsenalComm.serialStatus();
    ESP_LOGI(TAG, "Starting RS485 receiver task; query temperature sensor IDs 2, 3, 4");
    const esp_err_t startStatus = arsenalComm.start();
    if (startStatus != ESP_OK) {
        ESP_LOGE(TAG, "RS485 start failed: %s", esp_err_to_name(startStatus));
        arsenalComm.serialStatus();
        return;
    }

    ESP_LOGI(TAG, "Arsenal temperature test ready; Celsius = int16_t(first value) / 128.0");


#ifdef ARC_ENABLE_SD_LOGGING
    ESP_LOGI(TAG, "Starting SD logger");
    SDLogging::startSdLogger();
#else
    ESP_LOGI(TAG, "SD logging disabled in this build");
#endif
    const BaseType_t diagnosticTaskStatus = xTaskCreate(
        +[](void*) {
            TickType_t lastReportTick = xTaskGetTickCount();
            for (;;) {
                ESP_LOGI(TEMPERATURE_TAG, "uptime=%llu ms diagnostic_stack_min_free=%u bytes",
                         static_cast<unsigned long long>(esp_timer_get_time() / 1000),
                         static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
                for (Sensor<2>* sensor : temperatureSensors) {
                    Sensor<2>& temperatureSensor = *sensor;
                    Vec3<uint16_t> reading{};
                    if (temperatureSensor.getReadings(reading)) {
                        const float temperatureC = static_cast<int16_t>(reading.x) / 128.0f;
                        ESP_LOGI(TEMPERATURE_TAG, "%s ID=%u mux=0x%02X channel=%u: %.4f C raw=0x%04X signed=%d packet={%u,%u,%u}",
                                 temperatureSensor.getName().data(), static_cast<unsigned>(temperatureSensor.getID()),
                                 static_cast<unsigned>(MUX_ADDRESSES[temperatureSensor.getMuxI2C() / 8]),
                                 static_cast<unsigned>(temperatureSensor.getMuxI2C() % 8), temperatureC,
                                 static_cast<unsigned>(reading.x), static_cast<int>(static_cast<int16_t>(reading.x)),
                                 static_cast<unsigned>(reading.x), static_cast<unsigned>(reading.y),
                                 static_cast<unsigned>(reading.z));
                    } else {
                        ESP_LOGW(TEMPERATURE_TAG, "%s ID=%u: no valid temperature reading",
                                 temperatureSensor.getName().data(),
                                 static_cast<unsigned>(temperatureSensor.getID()));
                    }

                }
                vTaskDelayUntil(&lastReportTick, pdMS_TO_TICKS(TEMPERATURE_REPORT_INTERVAL_MS));
            }
        }, "temperature_diagnostics", TEMPERATURE_DIAGNOSTIC_STACK_BYTES, nullptr, 1, nullptr);
    if (diagnosticTaskStatus != pdPASS) {
        ESP_LOGE(TAG, "Could not start temperature diagnostic task");
    } else {
        ESP_LOGI(TAG, "Temperature diagnostics started: interval=%lu ms stack=%lu bytes tag=%s",
                 static_cast<unsigned long>(TEMPERATURE_REPORT_INTERVAL_MS),
                 static_cast<unsigned long>(TEMPERATURE_DIAGNOSTIC_STACK_BYTES), TEMPERATURE_TAG);
    }

}
