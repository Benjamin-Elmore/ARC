#pragma once
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <vector>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0, ESP_ERR_INVALID_ARG = 0x102, ESP_FAIL = -1;
using gpio_num_t = int;
using i2c_port_t = int;
constexpr int I2C_CLK_SRC_DEFAULT = 0, I2C_ADDR_BIT_LEN_7 = 7, GPIO_NUM_7 = 7;
struct FakeBus {};
struct FakeDevice {};
using i2c_master_bus_handle_t = FakeBus*;
using i2c_master_dev_handle_t = FakeDevice*;
struct i2c_master_bus_config_t {
    int i2c_port, sda_io_num, scl_io_num, clk_source;
    int glitch_ignore_cnt, intr_priority, trans_queue_depth;
    struct { bool enable_internal_pullup; } flags;
};
struct i2c_device_config_t {
    int dev_addr_length;
    std::uint16_t device_address;
    std::uint32_t scl_speed_hz;
};
namespace fake_i2c {
inline FakeBus bus;
inline FakeDevice devices[32]{};
inline esp_err_t bus_result = ESP_OK, add_result = ESP_OK, transfer_result = ESP_OK;
inline int transfer_calls = 0, timeout = 0;
inline i2c_master_dev_handle_t last_device = nullptr;
inline i2c_master_bus_config_t bus_config{};
inline std::vector<i2c_device_config_t> added;
inline std::vector<std::uint8_t> sent, response;
inline void reset() {
    bus_result = add_result = transfer_result = ESP_OK;
    transfer_calls = timeout = 0;
    last_device = nullptr;
    bus_config = {};
    added.clear(); sent.clear(); response.clear();
}
}
inline esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* config,
                                  i2c_master_bus_handle_t* handle) {
    fake_i2c::bus_config = *config;
    *handle = &fake_i2c::bus;
    return fake_i2c::bus_result;
}
inline esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t,
        const i2c_device_config_t* config, i2c_master_dev_handle_t* handle) {
    if (fake_i2c::add_result != ESP_OK) return fake_i2c::add_result;
    *handle = &fake_i2c::devices[fake_i2c::added.size()];
    fake_i2c::added.push_back(*config);
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
        const std::uint8_t* data, std::size_t size, int timeout) {
    ++fake_i2c::transfer_calls;
    fake_i2c::last_device = device;
    fake_i2c::sent.assign(data, data + size);
    fake_i2c::timeout = timeout;
    return fake_i2c::transfer_result;
}
inline esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
        const std::uint8_t* data, std::size_t size, std::uint8_t* output,
        std::size_t output_size, int timeout) {
    const auto status = i2c_master_transmit(device, data, size, timeout);
    if (status == ESP_OK) std::copy_n(fake_i2c::response.begin(),
        std::min(output_size, fake_i2c::response.size()), output);
    return status;
}
