#pragma once
#include <cstring>
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

inline void runRS485TransceiverTest() {
    constexpr uart_port_t port = UART_NUM_1;
    constexpr gpio_num_t enablePin = GPIO_NUM_14;
    constexpr const char* tag = "RS485_TEST";
    ESP_LOGI(tag, "RAW TEST: TX=17 RX=18 DE=/RE=14 baud=115200; packet parser bypassed");
    gpio_config_t enableConfig{};
    enableConfig.pin_bit_mask = 1ULL << enablePin;
    enableConfig.mode = GPIO_MODE_INPUT_OUTPUT;
    ESP_ERROR_CHECK(gpio_set_level(enablePin, 0));
    ESP_ERROR_CHECK(gpio_config(&enableConfig));
    uart_config_t config{};
    config.baud_rate = 115200;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(port, &config));
    ESP_ERROR_CHECK(uart_set_pin(port, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, 2048, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_set_mode(port, UART_MODE_UART));

    ESP_ERROR_CHECK(uart_set_loop_back(port, true));
    ESP_ERROR_CHECK(uart_flush_input(port));
    const uint8_t pattern[]{0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0xA8, 0x80};
    const int loopWritten = uart_write_bytes(port, pattern, sizeof(pattern));
    ESP_ERROR_CHECK(uart_wait_tx_done(port, pdMS_TO_TICKS(100)));
    uint8_t loopReceived[sizeof(pattern)]{};
    const int loopRead = uart_read_bytes(port, loopReceived, sizeof(loopReceived), pdMS_TO_TICKS(200));
    const bool loopPassed = loopWritten == sizeof(pattern) && loopRead == sizeof(pattern) &&
                            std::memcmp(pattern, loopReceived, sizeof(pattern)) == 0;
    ESP_LOGI(tag, "INTERNAL UART LOOPBACK %s: written=%d received=%d; external RS485 driver held OFF",
             loopPassed ? "PASS" : "FAIL", loopWritten, loopRead);
    ESP_ERROR_CHECK(uart_set_loop_back(port, false));
    ESP_ERROR_CHECK(uart_flush_input(port));
    ESP_LOGI(tag, "LISTENING: DE=/RE forced LOW; any bytes will be printed and echoed unchanged");
    ESP_LOGI(tag, "Echo is a wiring test, not an ARSENAL protocol acknowledgment");
    volatile uint32_t rxEdges = 0;
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_set_intr_type(GPIO_NUM_18, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_18, +[](void* context) {
        volatile uint32_t* edges = static_cast<volatile uint32_t*>(context);
        *edges = *edges + 1;
    }, const_cast<uint32_t*>(&rxEdges)));
    uint32_t totalReceived = 0;
    int64_t lastReport = esp_timer_get_time();
    int64_t lastBeacon = esp_timer_get_time();
    uint32_t beaconSequence = 0;
    for (;;) {
        uint8_t received[128]{};
        const int count = uart_read_bytes(port, received, sizeof(received), pdMS_TO_TICKS(100));
        if (count > 0) {
            totalReceived += count;
            ESP_LOGI(tag, "RAW RX %d bytes; total=%lu", count, static_cast<unsigned long>(totalReceived));
            ESP_LOG_BUFFER_HEX_LEVEL(tag, received, count, ESP_LOG_INFO);
            ESP_ERROR_CHECK(gpio_set_level(enablePin, 1));
            esp_rom_delay_us(10);
            const int written = uart_write_bytes(port, received, count);
            const esp_err_t txStatus = uart_wait_tx_done(port, pdMS_TO_TICKS(200));
            ESP_ERROR_CHECK(gpio_set_level(enablePin, 0));
            ESP_LOGI(tag, "RAW ECHO TX accepted=%d/%d completed=%s; receiver re-enabled", written, count,
                     esp_err_to_name(txStatus));
        }
        if (count == 0 && esp_timer_get_time() - lastBeacon >= 2000000) {
            lastBeacon = esp_timer_get_time();
            char beacon[64]{};
            const int length = snprintf(beacon, sizeof(beacon), "ARSENAL RS485 TEST seq=%lu\r\n",
                                        static_cast<unsigned long>(++beaconSequence));
            ESP_ERROR_CHECK(gpio_set_level(enablePin, 1));
            esp_rom_delay_us(10);
            const int written = uart_write_bytes(port, beacon, length);
            const esp_err_t txStatus = uart_wait_tx_done(port, pdMS_TO_TICKS(200));
            ESP_ERROR_CHECK(gpio_set_level(enablePin, 0));
            ESP_LOGI(tag, "BEACON seq=%lu UART accepted=%d/%d completed=%s",
                     static_cast<unsigned long>(beaconSequence), written, length, esp_err_to_name(txStatus));
        }
        if (esp_timer_get_time() - lastReport >= 1000000) {
            lastReport = esp_timer_get_time();
            ESP_LOGI(tag, "LISTEN loopback=%s totalRX=%lu RXedges=%lu DE=/RE=%d RXpin=%d stackMinFree=%u bytes",
                     loopPassed ? "PASS" : "FAIL", static_cast<unsigned long>(totalReceived),
                     static_cast<unsigned long>(rxEdges), gpio_get_level(enablePin),
                     gpio_get_level(GPIO_NUM_18), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
        }
    }
}
