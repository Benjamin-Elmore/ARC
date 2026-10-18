#pragma once
#include <cstdint>
using BaseType_t = int;
using TickType_t = std::uint32_t;
struct FakeSemaphore {};
using SemaphoreHandle_t = FakeSemaphore*;
constexpr BaseType_t pdFALSE = 0;
constexpr BaseType_t pdTRUE = 1;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
namespace fake_rtos {
inline FakeSemaphore semaphore;
inline BaseType_t take_result = pdTRUE;
inline int take_calls = 0, give_calls = 0;
inline SemaphoreHandle_t taken = nullptr, given = nullptr;
inline TickType_t wait_ticks = 0;
inline void reset() {
    take_result = pdTRUE;
    take_calls = give_calls = 0;
    taken = given = nullptr;
    wait_ticks = 0;
}
}
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return &fake_rtos::semaphore; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t ticks) {
    ++fake_rtos::take_calls;
    fake_rtos::taken = handle;
    fake_rtos::wait_ticks = ticks;
    return fake_rtos::take_result;
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) {
    ++fake_rtos::give_calls;
    fake_rtos::given = handle;
    return pdTRUE;
}
