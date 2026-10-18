#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include "I2CBus.h"
#include "Vec3.h"
#include "RS485Utils.h"
#include "SensorUtils.h"
#include "i2cUtils.h"

// Checks remain active in Release builds, unlike assert().
#define CHECK(...) do { if (!(__VA_ARGS__)) throw std::runtime_error( \
    "line " + std::to_string(__LINE__) + ": " #__VA_ARGS__); } while (false)

void check_near(float actual, float expected, float tolerance = 1e-5f) {
    CHECK(std::isfinite(actual));
    CHECK(std::fabs(actual - expected) <= tolerance);
}
// Verify component-wise integer arithmetic, scalar multiplication/division,
// negation, and comparison using vectors with both positive and negative values.
void vector_arithmetic() {
    const Vec3i a{6, -4, 12}, b{2, 2, 3};
    CHECK(a + b == Vec3i{8, -2, 15});
    CHECK(a - b == Vec3i{4, -6, 9});
    CHECK(a * b == Vec3i{12, -8, 36});
    CHECK(a / b == Vec3i{3, -2, 4});
    CHECK(a * 2 == Vec3i{12, -8, 24});
    CHECK(a * 2 == 2 * a);
    CHECK(a / 2 == Vec3i{3, -2, 6});
    CHECK(-a == Vec3i{-6, 4, -12});
    CHECK(a != b);
}
// Verify known lengths and distances, dot products, cross-product orientation,
// unit-vector normalization, and angles between perpendicular/opposite vectors.
void vector_geometry() {
    const Vec3f a{3, 4, 0};
    check_near(length(a), 5);
    check_near(lengthSq(a), 25);
    check_near(distance(a, Vec3f{3, 4, 12}), 12);
    CHECK(cross(Vec3i::unitX(), Vec3i::unitY()) == Vec3i::unitZ());
    CHECK(cross(Vec3i::unitY(), Vec3i::unitX()) == -Vec3i::unitZ());
    CHECK(dot(Vec3i{1, 2, 3}, Vec3i{4, -5, 6}) == 12);
    CHECK(nearlyEqual(normalize(a), Vec3f{0.6f, 0.8f, 0}));
    check_near(angle(Vec3f::unitX(), Vec3f::unitY()), std::acos(-1.0f) / 2);
    check_near(angle(Vec3f::unitX(), -Vec3f::unitX()), std::acos(-1.0f));
}
// Verify zero and near-zero normalization (including a custom threshold),
// projection onto zero, zero-vector angles, and length clamping of zero.
void vector_degenerate_inputs() {
    CHECK(normalize(Vec3f::zero()) == Vec3f::zero());
    CHECK(normalize(Vec3f{1e-8f, 0, 0}) == Vec3f::zero());
    CHECK(normalize(Vec3f{0.01f, 0, 0}, 0.02f) == Vec3f::zero());
    CHECK(project(Vec3f::one(), Vec3f::zero()) == Vec3f::zero());
    check_near(angle(Vec3f::zero(), Vec3f::one()), 0);
    CHECK(clampLength(Vec3f::zero(), 2.0f) == Vec3f::zero());
}
// Verify projection onto a non-unit axis and the perpendicular remainder,
// reflection about a unit normal, and direction-preserving length clamping.
void vector_projection_and_reflection() {
    const Vec3f v{3, -4, 2}, axis{2, 0, 0};
    CHECK(project(v, axis) == Vec3f{3, 0, 0});
    CHECK(reject(v, axis) == Vec3f{0, -4, 2});
    CHECK(project(v, axis) + reject(v, axis) == v);
    CHECK(reflect(v, Vec3f::unitY()) == Vec3f{3, 4, 2});
    CHECK(nearlyEqual(clampLength(Vec3f{3, 4, 0}, 2.0f), Vec3f{1.2f, 1.6f, 0}));
    CHECK(clampLength(v, 10.0f) == v);
}
// Verify interpolation endpoints/midpoint and extrapolation, component bounds,
// absolute values, and acceptance/rejection of differences within/beyond tolerance.
void vector_interpolation_and_bounds() {
    const Vec3f a{0, 2, -4}, b{4, 6, 0};
    CHECK(lerp(a, b, 0.0f) == a);
    CHECK(lerp(a, b, 1.0f) == b);
    CHECK(lerp(a, b, 0.5f) == Vec3f{2, 4, -2});
    CHECK(lerp(a, b, 2.0f) == Vec3f{8, 10, 4});
    CHECK(clamp(Vec3i{-3, 2, 9}, Vec3i::zero(), Vec3i{4, 4, 4}) == Vec3i{0, 2, 4});
    CHECK(abs(Vec3i{-3, 0, 9}) == Vec3i{3, 0, 9});
    CHECK(nearlyEqual(a, a + Vec3f{1e-6f, 0, 0}));
    CHECK(!nearlyEqual(a, a + Vec3f{0, 0, 0.1f}));
}
// Verify copyable vector storage and floating-point integer-vector lengths,
// indexed access/clamping, truncating casts, and the Z-up direction convention.
void vector_types_and_access() {
    static_assert(std::is_trivially_copyable_v<Vec3f>);
    static_assert(std::is_same_v<decltype(length(Vec3i{})), float>);
    Vec3i v{};
    v[0] = 1; v[1] = 2; v[2] = 3;
    const Vec3i copy = v;
    CHECK(copy[0] == 1 && copy[1] == 2 && copy[2] == 3);
    CHECK(copy[99] == 3); // Documented clamping behavior.
    CHECK(vec3_cast<int>(Vec3f{1.9f, -2.9f, 0}) == Vec3i{1, -2, 0});
    CHECK(Vec3i::up() == Vec3i::unitZ());
    CHECK(Vec3i::forward() == Vec3i::unitX());
    CHECK(Vec3i::right() == Vec3i{0, -1, 0});
}
// Verify big-endian U32 decoding at nonzero offsets and the last valid position,
// including all-one bytes, a set sign-position bit, and an all-zero value.
void decode_u32_big_endian() {
    const uint8_t bytes[]{0xAA, 0x12, 0x34, 0x56, 0x78, 0xFF, 0xFF, 0xFF, 0xFF};
    CHECK(RS485Utils::convertU32InArr(bytes, 1) == 0x12345678u);
    CHECK(RS485Utils::convertU32InArr(bytes, 5) == UINT32_MAX);
    const uint8_t high_bit[]{0x80, 0, 0, 0}, zero[]{0, 0, 0, 0};
    CHECK(RS485Utils::convertU32InArr(high_bit, 0) == 0x80000000u);
    CHECK(RS485Utils::convertU32InArr(zero, 0) == 0);
}
// Verify little-endian signed I16 decoding for zero, a positive value, both
// signed limits, and negative two's-complement values (-1 and -2).
void decode_i16_little_endian() {
    const struct { uint8_t bytes[2]; int16_t expected; } cases[]{
        {{0, 0}, 0}, {{0x34, 0x12}, 0x1234}, {{0xFF, 0x7F}, INT16_MAX},
        {{0, 0x80}, INT16_MIN}, {{0xFF, 0xFF}, -1}, {{0xFE, 0xFF}, -2}};
    for (const auto& item : cases) CHECK(i2cUtils::convertTo16Bits(item.bytes) == item.expected);
}
// Verify that every defined sensor status maps to its expected diagnostic text.
void sensor_status_messages() {
    using namespace SensorUtils;
    const struct { sensor_status_t status; std::string_view expected; } cases[]{
        {SENSOR_OK, "ok"}, {SENSOR_ERR_NOT_READY, "not_ready"}, {SENSOR_ERR_IO, "io"},
        {SENSOR_ERR_TIMEOUT, "timeout"}, {SENSOR_ERR_RANGE, "range"},
        {SENSOR_ERR_NO_READER, "no_reader"}, {SENSOR_ERR_DISABLED, "disabled"},
        {SENSOR_ERR_FAULT, "fault"}, {SENSOR_ERR_ARG, "bad_arg"},
        {SENSOR_ERR_NOT_FOUND, "not_found"}, {SENSOR_ERR_FULL, "full"},
        {SENSOR_ERR_FUNC_CALL, "call_fail"}, {SENSOR_ERR_TASK, "task_fail"}};
    for (const auto& item : cases) CHECK(sensor_status_str(item.status) == item.expected);
}
// Verify that an acquired mutex is held until scope exit, then released once
// using the same handle; also verify that LockGuard cannot be copied.
void mutex_released_on_scope_exit() {
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    {
        LockGuard guard(mutex);
        CHECK(guard.isMutexLocked());
        CHECK(fake_rtos::taken == mutex);
        CHECK(fake_rtos::wait_ticks == portMAX_DELAY);
        CHECK(fake_rtos::give_calls == 0);
    }
    CHECK(fake_rtos::give_calls == 1 && fake_rtos::given == mutex);
    static_assert(!std::is_copy_constructible_v<LockGuard>);
    static_assert(!std::is_copy_assignable_v<LockGuard>);
}
// Verify that a null mutex leaves the guard unlocked and makes no RTOS calls.
void mutex_null_is_ignored() {
    SemaphoreHandle_t mutex = nullptr;
    { LockGuard guard(mutex); CHECK(!guard.isMutexLocked()); }
    CHECK(fake_rtos::take_calls == 0 && fake_rtos::give_calls == 0);
}
// Verify that failed acquisition leaves the guard unlocked and never releases
// a mutex it did not acquire.
void mutex_failed_take_is_not_released() {
    fake_rtos::take_result = pdFALSE;
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    { LockGuard guard(mutex); CHECK(!guard.isMutexLocked()); }
    CHECK(fake_rtos::take_calls == 1 && fake_rtos::give_calls == 0);
}
// Verify that stack unwinding releases an acquired mutex exactly once when
// an exception exits the guarded scope.
void mutex_released_during_exception() {
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    try { LockGuard guard(mutex); throw std::runtime_error("test"); }
    catch (const std::runtime_error&) {}
    CHECK(fake_rtos::give_calls == 1);
}
// Verify successful bus setup uses the requested pins, creates a mutex, and
// registers both multiplexers with their requested I2C addresses.
void bus_initialization() {
    I2CBus<2> bus(9, 8, 0, {0x70, 0x71});
    CHECK(bus.getStatus() == ESP_OK);
    CHECK(bus.getMutex() != nullptr);
    CHECK(fake_i2c::bus_config.scl_io_num == 9 && fake_i2c::bus_config.sda_io_num == 8);
    CHECK(fake_i2c::added.size() == 2);
    CHECK(fake_i2c::added[0].device_address == 0x70);
    CHECK(fake_i2c::added[1].device_address == 0x71);
}
// Verify that a bus-creation failure remains the reported status after later
// error and success reports, preserving the original failure for diagnosis.
void bus_preserves_first_setup_error() {
    fake_i2c::bus_result = ESP_FAIL;
    I2CBus<0> bus(9, 8, 0);
    bus.confirmStatus(ESP_ERR_INVALID_ARG);
    bus.confirmStatus(ESP_OK);
    CHECK(bus.getStatus() == ESP_FAIL);
}
// Verify that reusing a sensor name returns an argument error without adding
// another device or replacing the original device address.
void bus_rejects_duplicate_sensor() {
    I2CBus<0> bus(9, 8, 0);
    CHECK(bus.addSensorClassToBus("optical", 0xFF, 0x17) == ESP_OK);
    CHECK(bus.addSensorClassToBus("optical", 0xFF, 0x18) == ESP_ERR_INVALID_ARG);
    CHECK(fake_i2c::added.size() == 1);
    CHECK(fake_i2c::added[0].device_address == 0x17);
}
// Verify that reads, writes, array writes, and mux selection for an unknown
// sensor return an argument error without performing any I2C transfer.
void bus_unknown_sensor_does_not_transfer() {
    I2CBus<1> bus(9, 8, 0, {0x70});
    uint8_t data[2]{};
    const uint8_t payload[]{1, 2};
    CHECK(bus.readRegister("missing", 0x20, data, 2) == ESP_ERR_INVALID_ARG);
    CHECK(bus.writeRegister("missing", 0x20, 1) == ESP_ERR_INVALID_ARG);
    CHECK(bus.writeArrayRegister("missing", 0x20, payload) == ESP_ERR_INVALID_ARG);
    CHECK(bus.selectMuxChannel("missing") == ESP_ERR_INVALID_ARG);
    CHECK(fake_i2c::transfer_calls == 0);
}
// Verify that a register read targets the registered device, sends the register
// address, copies the response into the caller's buffer, and uses timeout 1000.
void bus_read_register() {
    I2CBus<0> bus(9, 8, 0);
    CHECK(bus.addSensorClassToBus("optical", 0xFF, 0x17) == ESP_OK);
    fake_i2c::response = {0x34, 0x12};
    uint8_t data[2]{};
    CHECK(bus.readRegister("optical", 0x20, data, 2) == ESP_OK);
    CHECK(fake_i2c::sent == std::vector<uint8_t>{0x20});
    CHECK(fake_i2c::last_device == &fake_i2c::devices[0]);
    CHECK(data[0] == 0x34 && data[1] == 0x12);
    CHECK(fake_i2c::timeout == 1000);
}
// Verify that single-byte and array writes target the registered device and
// prefix the payload with the register address while preserving byte order.
void bus_write_register_and_array() {
    I2CBus<0> bus(9, 8, 0);
    CHECK(bus.addSensorClassToBus("optical", 0xFF, 0x17) == ESP_OK);
    CHECK(bus.writeRegister("optical", 0x10, 0xAB) == ESP_OK);
    CHECK(fake_i2c::sent == std::vector<uint8_t>{0x10, 0xAB});
    const uint8_t payload[]{0, 0xFF, 0x12, 0x34};
    CHECK(bus.writeArrayRegister("optical", 0x20, payload) == ESP_OK);
    CHECK(fake_i2c::sent == std::vector<uint8_t>{0x20, 0, 0xFF, 0x12, 0x34});
    CHECK(fake_i2c::last_device == &fake_i2c::devices[0]);
}
// Verify that read, single-byte write, and array-write failures reach the caller;
// a failed read must leave the caller's existing buffer contents intact.
void bus_propagates_transfer_errors() {
    I2CBus<0> bus(9, 8, 0);
    CHECK(bus.addSensorClassToBus("optical", 0xFF, 0x17) == ESP_OK);
    fake_i2c::transfer_result = ESP_FAIL;
    uint8_t data[]{0xAA};
    CHECK(bus.readRegister("optical", 0x20, data, 1) == ESP_FAIL);
    CHECK(data[0] == 0xAA);
    CHECK(bus.writeRegister("optical", 0x20, 1) == ESP_FAIL);
    CHECK(bus.writeArrayRegister("optical", 0x20, data) == ESP_FAIL);
}
// Verify that global channels 0 and 15 select the first/second mux respectively
// and transmit the one-hot masks for local channels 0 and 7.
void bus_selects_mux_and_channel_boundaries() {
    I2CBus<2> bus(9, 8, 0, {0x70, 0x71});
    CHECK(bus.addSensorClassToBus("first", 0, 0x17) == ESP_OK);
    CHECK(bus.addSensorClassToBus("last", 15, 0x17) == ESP_OK);
    CHECK(bus.selectMuxChannel("first") == ESP_OK);
    CHECK(fake_i2c::sent == std::vector<uint8_t>{1});
    CHECK(fake_i2c::last_device == &fake_i2c::devices[0]);
    CHECK(bus.selectMuxChannel("last") == ESP_OK);
    CHECK(fake_i2c::sent == std::vector<uint8_t>{0x80});
    CHECK(fake_i2c::last_device == &fake_i2c::devices[1]);
}
// Verify that a failed mux-selection transfer is returned to the caller and
// also recorded as the bus error status.
void bus_mux_failure_updates_status() {
    I2CBus<1> bus(9, 8, 0, {0x70});
    CHECK(bus.addSensorClassToBus("optical", 3, 0x17) == ESP_OK);
    fake_i2c::transfer_result = ESP_FAIL;
    CHECK(bus.selectMuxChannel("optical") == ESP_FAIL);
    CHECK(bus.getStatus() == ESP_FAIL);
}
int main() {
    const struct { const char* name; void (*run)(); } tests[]{
#define TEST(name) {#name, name}
        TEST(vector_arithmetic), TEST(vector_geometry), TEST(vector_degenerate_inputs),
        TEST(vector_projection_and_reflection), TEST(vector_interpolation_and_bounds),
        TEST(vector_types_and_access), TEST(decode_u32_big_endian), TEST(decode_i16_little_endian),
        TEST(sensor_status_messages), TEST(mutex_released_on_scope_exit),
        TEST(mutex_null_is_ignored), TEST(mutex_failed_take_is_not_released),
        TEST(mutex_released_during_exception), TEST(bus_initialization),
        TEST(bus_preserves_first_setup_error), TEST(bus_rejects_duplicate_sensor),
        TEST(bus_unknown_sensor_does_not_transfer), TEST(bus_read_register),
        TEST(bus_write_register_and_array), TEST(bus_propagates_transfer_errors),
        TEST(bus_selects_mux_and_channel_boundaries), TEST(bus_mux_failure_updates_status)
#undef TEST
    };
    int failed = 0;
    for (const auto& test : tests) {
        fake_rtos::reset(); fake_i2c::reset();
        try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        }
    }
    std::cout << std::size(tests) - failed << '/' << std::size(tests) << " tests passed\n";
    return failed == 0 ? 0 : 1;
}
