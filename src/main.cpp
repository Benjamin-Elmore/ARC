#include "RS485Comm.h"
#include "esp_log.h"

namespace {
    constexpr const char* TAG = "ARC";
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting RS485-only test: TX=2 RX=3 ENABLE=4, 115200 baud");
    static RS485Comm comm25(2, 3, 4);
    comm25.setSerialMonitoring(true);
    if (!comm25.checkCommStatus()) {
        ESP_LOGE(TAG, "RS485 UART setup failed");
        comm25.serialStatus();
        return;
    }

    // Leave the registry empty to exercise requests/replies without I2C hardware.
    const esp_err_t status = comm25.start();
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "RS485 start failed: %s", esp_err_to_name(status));
        comm25.serialStatus();
        return;
    }
    ESP_LOGI(TAG, "RS485 receiver started; waiting for V5 requests (no sensors registered)");
}
