#pragma once

#ifdef ARC_ENABLE_SD_LOGGING

#if !defined(ARC_SD_CS_GPIO) || !defined(ARC_SD_MOSI_GPIO) || \
    !defined(ARC_SD_SCLK_GPIO) || !defined(ARC_SD_MISO_GPIO)
#error "Define all four ARC_SD_*_GPIO flags for the wired SD card."
#endif

#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace SDLogging {
    inline constexpr char TAG[] = "SD_LOG";
    inline constexpr char MOUNT_PATH[] = "/sdcard";
    inline constexpr char LOG_PATH[] = "/sdcard/runtime.csv";
    inline constexpr char CSV_HEADER[] = "uptime_ms,free_heap_bytes,min_free_heap_bytes\n";
    inline constexpr TickType_t SAMPLE_INTERVAL = pdMS_TO_TICKS(5000);

    inline void logSamples() {
        FILE* file = std::fopen(LOG_PATH, "a");
        if (file == nullptr) {
            ESP_LOGE(TAG, "Open %s failed: %s", LOG_PATH, std::strerror(errno));
            return;
        }

        // Add the CSV header when the log is new or empty.
        const int fd = fileno(file);
        struct stat fileInfo{};
        if (fstat(fd, &fileInfo) != 0) {
            ESP_LOGE(TAG, "Could not read CSV file size: %s", std::strerror(errno));
        } else if (fileInfo.st_size == 0 && std::fputs(CSV_HEADER, file) == EOF) {
            ESP_LOGE(TAG, "CSV header write failed: %s", std::strerror(errno));
        } else {
            ESP_LOGI(TAG, "Logging to %s every 5 seconds", LOG_PATH);
            TickType_t lastWake = xTaskGetTickCount();

            while (true) {
                // Add sensor and bus snapshots here. Use a new log file if the columns change.
                const int64_t uptimeMs = esp_timer_get_time() / 1000;
                const uint32_t freeHeap = esp_get_free_heap_size();
                const uint32_t minFreeHeap = esp_get_minimum_free_heap_size();

                const int written = std::fprintf(
                    file,
                    "%" PRId64 ",%" PRIu32 ",%" PRIu32 "\n",
                    uptimeMs,
                    freeHeap,
                    minFreeHeap
                );
                if (written < 0) {
                    ESP_LOGE(TAG, "CSV row write failed: %s", std::strerror(errno));
                    break;
                }

                // Flush the file buffer before syncing the row to the card.
                if (std::fflush(file) != 0 || fsync(fd) != 0) {
                    ESP_LOGE(TAG, "CSV sync failed: %s", std::strerror(errno));
                    break;
                }

                vTaskDelayUntil(&lastWake, SAMPLE_INTERVAL);
            }
        }

        if (std::fclose(file) != 0) {
            ESP_LOGE(TAG, "CSV close failed: %s", std::strerror(errno));
        }
    }

    inline void logToCard() {
        // ESP-IDF 5.3's default macro leaves its trailing optional members zero-initialized.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
        const sdmmc_host_t host = SDSPI_HOST_DEFAULT();
#pragma GCC diagnostic pop
        const auto bus = static_cast<spi_host_device_t>(host.slot);

        // Set up the SPI bus for the SD card.
        spi_bus_config_t busConfig{};
        busConfig.mosi_io_num = ARC_SD_MOSI_GPIO;
        busConfig.miso_io_num = ARC_SD_MISO_GPIO;
        busConfig.sclk_io_num = ARC_SD_SCLK_GPIO;
        busConfig.quadwp_io_num = -1;
        busConfig.quadhd_io_num = -1;
        busConfig.data4_io_num = -1;
        busConfig.data5_io_num = -1;
        busConfig.data6_io_num = -1;
        busConfig.data7_io_num = -1;

        const esp_err_t busStatus = spi_bus_initialize(
            bus,
            &busConfig,
            SPI_DMA_CH_AUTO
        );
        if (busStatus != ESP_OK) {
            ESP_LOGE(TAG, "SPI setup failed: %s", esp_err_to_name(busStatus));
            return;
        }

        // Mount the card with its existing filesystem.
        sdspi_device_config_t deviceConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
        deviceConfig.host_id = bus;
        deviceConfig.gpio_cs = static_cast<gpio_num_t>(ARC_SD_CS_GPIO);

        esp_vfs_fat_sdmmc_mount_config_t mountConfig{};
        mountConfig.max_files = 1;
        mountConfig.format_if_mount_failed = false;

        sdmmc_card_t* card = nullptr;
        const esp_err_t mountStatus = esp_vfs_fat_sdspi_mount(
            MOUNT_PATH,
            &host,
            &deviceConfig,
            &mountConfig,
            &card
        );

        if (mountStatus == ESP_OK) {
            logSamples();
            const esp_err_t unmountStatus = esp_vfs_fat_sdcard_unmount(MOUNT_PATH, card);
            if (unmountStatus != ESP_OK) {
                ESP_LOGE(TAG, "SD unmount failed: %s", esp_err_to_name(unmountStatus));
            }
        } else {
            ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(mountStatus));
        }

        const esp_err_t freeStatus = spi_bus_free(bus);
        if (freeStatus != ESP_OK) {
            ESP_LOGE(TAG, "SPI release failed: %s", esp_err_to_name(freeStatus));
        }
    }

    inline void sdLoggerTask(void*) {
        logToCard();
        vTaskDelete(nullptr);
    }

    inline void startSdLogger() {
        const BaseType_t result = xTaskCreate(
            sdLoggerTask,
            "sd_logger",
            4096,
            nullptr,
            tskIDLE_PRIORITY + 1,
            nullptr
        );
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Could not create SD logger task");
        }
    }

} // namespace SDLogging

#endif
