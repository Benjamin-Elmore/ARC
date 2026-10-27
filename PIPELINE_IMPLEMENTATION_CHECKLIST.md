# ARC sensor pipeline fix checklist

Updated: October 26, 2026. Scope: current local `CPPDev` working tree in
`C:\Doc\CBU_Robotics\ARC-Testing`.

**29 tracked items, reviewed individually below.** This is a manual implementation
checklist. Update this Markdown as we work; Benjamin applies source changes.
Do not treat the historical snippets as already applied or as standalone patches.
The **Current status and next change** block in each item takes precedence over
its historical issue excerpt and proposal.

Automated tests: **removed at Benjamin's request**. References to the old test suite below are historical; do not restore it or add test-only APIs.

Status meanings:
- **Complete in current source:** the listed defect is corrected; this does not imply hardware validation.
- **Partial:** some of the change exists, but the item still has remaining work.
- **Open:** the required behavior is absent or still incorrect.

Current verification, performed before this documentation update:
- `pio run -e seeed_xiao_esp32s3`: PASS, with ignored PlatformIO build flags,
  missing UART member initializers, and an 8 MB board / 2 MB SDK flash warning.
- Rebuilt and ran `arc_unit_tests`: 22/22 PASS.
- Rebuilding `arc_pipeline_tests`: FAIL. It targets APIs from the reverted
  implementation (`update`, `setReadings`, `processReceivedByte`, the old
  `startRS485Instance` name, and a namespaced communicator). It also lacks a direct
  `<string>` include for `std::to_string`. An old executable is not current evidence.
- The firmware build does not instantiate the optical path or construct sensors
  in `app_main`; build success does not verify that pipeline.
- No hardware validation was performed in this review.

Work order:
1. Items 9-10: direct-device routing and disconnecting other muxes.
2. Items 7-8 and 11: initialization/error handling and registry ownership.
3. Items 12-20: optical reader, publication, and safe sensor setup.
4. Items 4 and 22-24: shared instance, lifetime, UART errors, and registry rules.
5. Items 21 and 25-27: offsets and packets after checking the V5 contract.
6. Items 6 and 28-29: build configuration and application integration.

Next manual step: item 9 together with item 10. Review the combined routing code
before applying it, then verify a direct sensor with no mux, switching between two
muxes, switching to a direct device, and a failed disconnect preventing a read.
The caller must retain the bus mutex through selection and the device transfer.

The historical proposals below are retained for reference. They describe an earlier
reverted implementation; adapt names and interfaces to the current source. In
particular, the current initializer is `createRS485Instance()`, and moving the
communicator class into a namespace is optional if all callers remain consistent.
Confirm V5 request/reply layouts and byte order before applying items 25-27.


## Latest validation — October 26, 2026

Follow-up result: firmware build PASS using the isolated PlatformIO build directory.
The missing timer include, direct-device routing order, and constructor/include
corrections are present. Item 18 now has a shared critical-section snapshot API.
No source changes, desktop tests, or firmware upload were made by this validation.

UART member-initializer and flash-size mismatch warnings remain. app_main still
constructs no sensors, so this build does not instantiate every templated method
or validate physical I2C, OTOS, RS485, or concurrent reading behavior. Existing
startup/status concurrency and rollback work in items 15/17/22 remains open.

Log: C:\Doc\CBU_Robotics\ARC-Testing\.pio\validation-progress.log.
## Sensor lifetime decision — October 26, 2026

Sensors and their I2C buses live for the entire firmware run. Runtime sensor
replacement, dynamic destruction, and cooperative stop/join infrastructure are
deferred. Complete configuration and registration before worker creation, and
start RS485 reception only after all sensors are ready. Startup failure handling
remains required; permanent lifetime does not make failed initialization successful.
## Resume here — October 26, 2026

Stopped after reviewing item 15. Benjamin moved RS485 registration before task
creation. Next: move the SENSOR_OK assignment before task creation, then add
startup registration rollback for mux/callback/registration/task-creation failures.
The exact branch cleanup requirements are recorded in item 15.

Sensors and their buses live for the entire firmware run. Runtime shutdown and
worker-exit handshakes remain deferred. Do not recreate the removed generated
tests. Give concise numbered Issue / Fix / Validation steps; Benjamin normally
applies changes, and Codex edits source only when specifically requested.

Last passing firmware build: after item 19. Later item 20/21 guards and item 22a
helper were inspected/applied without a subsequent firmware build. The latest
item 15 review did not run a build. Physical I2C/mux/OTOS/V5 behavior remains
unverified. app_main still constructs no sensors.
## 1. Missing semicolon

**Current status and next change**

**Complete in current source.** `taskLoop()` now terminates `confirmStatus(muxStatus);` correctly. The ESP32 build passes this location. No further edit for this item.

**Historical issue excerpt — compare with current status above**

The error-status call is missing its semicolon.

```cpp
this->confirmStatus(muxStatus)
```

**Historical proposal — adapt before applying**

`lib/Sensor/ARC_Sensor.h` → `taskLoop()`. **Attempted:** terminate the call.

```cpp
this->confirmStatus(muxStatus);
```

## 2. Private interface inheritance

**Current status and next change**

**Complete in current source.** `Sensor` publicly inherits `RS485InterfaceSensor`. Preserve that public conversion when changing registration.

**Historical issue excerpt — compare with current status above**

Private inheritance prevents conversion to the registry interface.

```cpp
template <size_t N>
class Sensor : RS485InterfaceSensor {
    // Existing class body.
};
```

**Historical proposal — adapt before applying**

`lib/Sensor/ARC_Sensor.h`. **Attempted:** allow registry pointer conversion.

```cpp
template <size_t N>
class Sensor : public RS485InterfaceSensor {
    // Existing class body.
};
```

## 3. Communicator namespace

**Current status and next change**

**Open in the regression suite; design choice in production.** The current communicator class is global, and current production callers use that global type consistently. The regression suite expects `RS485ARSENAL::RS485Comm`. Choose one interface and align tests/callers; moving the class is not inherently required to fix production behavior.

**Historical issue excerpt — compare with current status above**

The class is global while the shared instance lives in RS485ARSENAL.

```cpp
// RS485Comm.h: global class.
class RS485Comm {
    // Existing class body.
};
// RS485Instance.h:
namespace RS485ARSENAL {
    RS485Comm* BaseRS485Instance = nullptr;
}
```

**Historical proposal — adapt before applying**

`lib/RS485/RS485Comm.h`. **Attempted:** wrap the class; keep existing callers valid.

```cpp
namespace RS485ARSENAL {
class RS485Comm {
    // Existing class body.
};
}
using RS485Comm = RS485ARSENAL::RS485Comm;
```

## 4. Shared instance: duplicate definitions and repeated allocation

**Current status and next change**

**Complete in current source.** Benjamin added `inline` to `BaseRS485Instance` and `startRS485ARSENAL()`; both additions were checked in the current file. Static instance reuse was already present. A fresh two-translation-unit build has not been run for this change.

**Historical issue excerpt — compare with current status above**

Header definitions are duplicated across translation units, and repeated initialization allocates another communicator.

```cpp
namespace RS485ARSENAL {
    RS485Comm* BaseRS485Instance = nullptr;
    void startRS485Instance() {
        BaseRS485Instance = new RS485Comm();
    }
}
```

**Historical proposal — adapt before applying**

`lib/RS485/RS485Instance.h`. **Attempted:** replace the header definitions below.
Call initialization serially from application startup.

```cpp
namespace RS485ARSENAL {
inline RS485Comm* BaseRS485Instance = nullptr;

inline void startRS485Instance() {
    struct SharedInstance {
        RS485Comm comm;
        ~SharedInstance() { BaseRS485Instance = nullptr; }
    };
    static SharedInstance instance;
    BaseRS485Instance = &instance.comm;
}

inline esp_err_t addSensor(RS485InterfaceSensor* sensor) {
    if (!BaseRS485Instance) { startRS485Instance(); }
    return BaseRS485Instance->addSensortoRegistry(sensor);
}
inline void removeSensor(RS485InterfaceSensor* sensor) {
    if (BaseRS485Instance) {
        BaseRS485Instance->removeSensorFromRegistry(sensor); // Item 24.
    }
}
inline esp_err_t startRS485ARSENAL() {
    if (!BaseRS485Instance) { return ESP_ERR_INVALID_STATE; }
    return BaseRS485Instance->start();
}
}
```

## 5. Missing direct includes

**Current status and next change**

**Source-verified; firmware build passes.** esp_timer.h is restored in RS485Comm.h, and ARC_Sensor.h directly includes <string>. The identified direct-include corrections are present. No desktop tests were recreated.

**Historical issue excerpt — compare with current status above**

Headers rely on other files to include task, semaphore, error, and integer declarations first.

```cpp
// LockGuard.h currently includes:
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// RS485Comm.h uses TaskHandle_t without directly including task.h.
// RS485InterfaceSensor.h uses esp_err_t without including esp_err.h.
// RS485Utils.h uses size_t/uint32_t without including their headers.
```

**Historical proposal — adapt before applying**

**Attempted:** add each include to its owning header.

```cpp
// lib/LockGuard.h
#include "freertos/semphr.h"

// lib/RS485/RS485Comm.h
#include "freertos/task.h"
#include "LockGuard.h"
#include <atomic>

// lib/Sensor/RS485InterfaceSensor.h
#include <cstdint>
#include "esp_err.h"

// lib/RS485/RS485Utils.h
#include <cstddef>
#include <cstdint>

// lib/Sensor/ARC_Sensor.h
#include <string>
#include <atomic>
```

## 6. Build flags in the wrong section

**Current status and next change**

**Applied; firmware build passes.** Environment flags and application-scoped stack flags are present; ignored-option warnings are gone. The validation build succeeds. Flash-size mismatch and incomplete UART configuration initializer warnings remain separate items.

**Historical issue excerpt — compare with current status above**

Build options are under the global section, and gnu17 is not the intended C++ standard flag.

```ini
[platformio]
core_dir = C:/pio-user
build_unflags =
    -std=gnu99
build_flags =
    -fstack-usage
    -Wstack-usage=2048
    -std=gnu17
```

**Historical proposal — adapt before applying**

`platformio.ini`, `src/CMakeLists.txt`. **Attempted:** move flags into the
environment, use C++20, and scope stack analysis to application sources.

```ini
[env:seeed_xiao_esp32s3]
platform = espressif32
board = seeed_xiao_esp32s3
framework = espidf
build_flags =
    -std=gnu++20

[platformio]
core_dir = C:/pio-user
```

```cmake
# Add esp_timer to the existing idf_component_register(REQUIRES ...) list.
# After idf_component_register(...):
target_compile_options(${COMPONENT_LIB} PRIVATE -fstack-usage -Wstack-usage=2048)
```

## 7. Continue-after-failure bus initialization

**Current status and next change**

**Partial; source-verified.** confirmStatus now returns failure for repeated failures, and setup refuses retries after a stored initialization error. `_i2cPort` still uses uint8_t instead of the SDK i2c_port_t. Runtime failure/retry behavior has not been exercised.

**Historical issue excerpt — compare with current status above**

Initialization records errors but continues with potentially invalid handles.

```cpp
confirmStatus(i2c_new_master_bus(&i2cMasterConfig, &_busHandle));
// Existing mux loop continues even after the call above fails:
confirmStatus(i2c_master_bus_add_device(
    _busHandle, &muxConfig, &_muxHandles.at(i)));
// Mutex allocation is not checked:
_busMutex = xSemaphoreCreateMutex();
```

**Historical proposal — adapt before applying**

`lib/i2cComm/I2CBus.h` → `setupI2CBus()`. **Attempted:** stop on each error and
only mark the bus ready after creating the mutex.

```cpp
// Member:
bool _initialized = false;

// Start of setupI2CBus():
if (_busHandle != nullptr) { return; }

// Replace the master-bus call:
const esp_err_t busStatus = i2c_new_master_bus(&i2cMasterConfig, &_busHandle);
confirmStatus(busStatus);
if (busStatus != ESP_OK) { return; }

// Inside the existing mux loop:
const esp_err_t muxStatus = i2c_master_bus_add_device(
    _busHandle, &muxConfig, &_muxHandles.at(i));
confirmStatus(muxStatus);
if (muxStatus != ESP_OK) { return; }

// After the mux loop:
_busMutex = xSemaphoreCreateMutex();
if (!_busMutex) { confirmStatus(ESP_ERR_NO_MEM); return; }
_initialized = true;
```

## 8. Invalid channels and failed adds polluting the registry

**Current status and next change**

**Applied; source-verified.** Uninitialized registration returns INVALID_STATE; duplicate name and address/channel validation remain, and insertion occurs only after successful driver registration. No hardware behavior was exercised.

**Historical issue excerpt — compare with current status above**

The registry entry is inserted even when device creation failed; channel/address checks are missing.

```cpp
esp_err_t addConfigStatus = i2c_master_bus_add_device(
    _busHandle, &sensorConfig, &newSensorRegistry.sensorHandle);
_registryMap.emplace(name, newSensorRegistry);
return addConfigStatus;
```

**Historical proposal — adapt before applying**

`I2CBus.h`. **Attempted:** validate before adding; insert only after success.

```cpp
// At the start of addSensorClassToBus():
if (!_initialized) { return ESP_ERR_INVALID_STATE; }
if (name.empty() || i2cAddress == 0 || i2cAddress > 0x7F ||
    (muxChannel != 0xFF && muxChannel / MUX_CHANNEL_CT >= N)) {
    return ESP_ERR_INVALID_ARG;
}

// Replace unconditional insertion after the existing driver call:
if (addConfigStatus == ESP_OK) {
    _registryMap.emplace(name, newSensorRegistry);
}
return addConfigStatus;

// Member: preserve the SDK's port type.
const i2c_port_t _i2cPort{};
```

**Attempted mux-address check:** run before creating the controller.

```cpp
for (size_t i = 0; i < N; ++i) {
    if (_muxAddresses[i] == 0 || _muxAddresses[i] > 0x7F) {
        confirmStatus(ESP_ERR_INVALID_ARG); return;
    }
    for (size_t j = 0; j < i; ++j) {
        if (_muxAddresses[i] == _muxAddresses[j]) {
            confirmStatus(ESP_ERR_INVALID_ARG); return;
        }
    }
}
```

## 9. Direct devices indexing a nonexistent mux

**Current status and next change**

**Applied for direct-device indexing; source-verified.** Direct devices return before accessing mux index 31. The return currently occurs too early to disconnect other muxes; that remaining behavior is tracked in item 10.

**Historical issue excerpt — compare with current status above**

UNUSED_MUX (0xFF) becomes mux index 31 and is indexed before checking the direct-device case.

```cpp
// In addSensorClassToBus():
SensorRegistry newSensorRegistry{
    static_cast<uint8_t>(muxChannel / MUX_CHANNEL_CT),
    static_cast<uint8_t>(muxChannel % MUX_CHANNEL_CT)
};
// In selectMuxChannel():
i2c_master_dev_handle_t muxHandle = _muxHandles.at(targetSensor.muxNumber);
```

**Historical proposal — adapt before applying**

`I2CBus.h` → `selectMuxChannel()`. **Attempted:** recognize the unused-mux
sentinel before indexing. Use together with item 10.

```cpp
// targetSensor is the existing registry entry.
const bool direct = targetSensor.muxNumber == 0xFF / MUX_CHANNEL_CT;
if (!direct && targetSensor.muxNumber >= N) {
    return ESP_ERR_INVALID_ARG;
}
// Disable other muxes here (item 10).
return direct ? ESP_OK : setMux(targetSensor.muxNumber,
    static_cast<uint8_t>(1u << targetSensor.muxChannel));
```

## 10. Previous mux remains connected

**Current status and next change**

**Applied; source-verified.** The direct-device return now follows the disconnect loop, so all muxes are disconnected before using a direct route. Muxed routes disconnect other muxes before enabling the target and return on a failed disconnect. Firmware build passes; physical mux transfers remain unverified.

**Historical issue excerpt — compare with current status above**

Only the target mux is changed; another mux can keep its previous channel connected.

```cpp
uint8_t controlRegister = (1 << targetSensor.muxChannel);
esp_err_t ret = i2c_master_transmit(
    _muxHandles.at(targetSensor.muxNumber), &controlRegister, 1, -1);
```

**Historical proposal — adapt before applying**

`I2CBus.h`. **Attempted:** disable other muxes before enabling the target.
Call under the same bus lock as the device transaction.

```cpp
// Before the return in item 9:
for (size_t i = 0; i < N; ++i) {
    if (!direct && i == targetSensor.muxNumber) { continue; }
    const esp_err_t status = setMux(i, 0);
    if (status != ESP_OK) { return status; }
}

// New private helper; initialize every cached mask to 0xFF during setup.
esp_err_t setMux(size_t index, uint8_t mask) {
    if (_muxCurrentChannels[index] == mask) { return ESP_OK; }
    const esp_err_t status = i2c_master_transmit(_muxHandles[index], &mask, 1, 1000);
    if (status == ESP_OK) { _muxCurrentChannels[index] = mask; }
    else { _muxCurrentChannels[index] = 0xFF; confirmStatus(status); }
    return status;
}
```

A direct device cannot share an address with a selected downstream device on
that same controller; disabling other muxes cannot disconnect the direct device.

## 11. Bus ownership and dangling registry keys

**Current status and next change**

**Applied; source-verified.** Registry keys own std::string storage, the direct include is present, and all name-based lookups/insertions convert keys. Cleanup/copy prevention remain. Coordinated sensor/bus lifetime remains item 22. Runtime lifetime behavior was not exercised.

**Historical issue excerpt — compare with current status above**

The owning bus has no cleanup/copy policy, and registry keys borrow caller storage.

```cpp
~I2CBus() = default;
std::unordered_map<std::string_view, SensorRegistry> _registryMap{};
// Copy construction/assignment are not explicitly disabled.
```

**Historical proposal — adapt before applying**

`I2CBus.h`. **Attempted:** own names, forbid handle copies, and free owned resources.
Destroy sensors before their bus.

```cpp
#include <string>

I2CBus(const I2CBus&) = delete;
I2CBus& operator=(const I2CBus&) = delete;
std::unordered_map<std::string, SensorRegistry> _registryMap{};

// Update EVERY existing find/contains/at lookup to use std::string(name),
// or std::string(sensorName) in selectMuxChannel().

~I2CBus() {
    for (const auto& item : _registryMap) {
        i2c_master_bus_rm_device(item.second.sensorHandle);
    }
    for (auto handle : _muxHandles) {
        if (handle) { i2c_master_bus_rm_device(handle); }
    }
    if (_busHandle) { i2c_del_master_bus(_busHandle); }
    if (_busMutex) { vSemaphoreDelete(_busMutex); }
}
```

## 12. OPTICAL preset does not install a reader

**Current status and next change**

**Open.** The preset branch is still a TODO, and `OpticalPreconfig.h` is excluded from `ARC_Sensor.h`. Selecting `OPTICAL` installs no reader. Wire a compatible optical reader, choose explicit unsupported-preset behavior, and default the optical offset register only for that preset. Keep custom reader override behavior intentional. Verify constructing and setting up an optical sensor actually instantiates this path.

**Historical issue excerpt — compare with current status above**

The preset branch is empty, so selecting OPTICAL does not provide a reader.

```cpp
if (sensorType != SensorPreconfig::NONE) {
    // TODO: Add preconfig function for reader
}
if (sensorReadFunction != NULL) {
    _readFunction = sensorReadFunction;
} else {
    _status = SensorUtils::SENSOR_ERR_NO_READER;
}
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`, constructor. **Attempted:** replace both preset and reader-selection
blocks. Add `bool _offsetsLittleEndian = false;` as a member.

```cpp
if (sensorType == OPTICAL) {
    _readFunction = [](Sensor& sensor) {
        sensor.confirmStatus(OpticalPreconfig::getOpticalPosition(sensor));
    };
    if (_offsetMemReg == SensorUtils::UNUSED_OFFSET_REG) {
        _offsetMemReg = OTOS_REG_OFFSETS;
    }
    _offsetsLittleEndian = true;
} else if (sensorType != NONE) {
    _status = SensorUtils::SENSOR_ERR_ARG; // COLOR/ENCODER not implemented.
}
if (sensorReadFunction) { _readFunction = sensorReadFunction; }
else if (!_readFunction) { _status = SensorUtils::SENSOR_ERR_NO_READER; }
```

## 13. Optical helper cannot instantiate or publish readings

**Current status and next change**

**Open.** The optical helper takes noncopyable `Sensor<M>` by value and calls a nonexistent one-argument setter. Change to a reference, check read status, decode all three 16-bit values, and publish through item 18. The offset helper omits the registered sensor name in its bus call. Fix that wrapper/signature too. Resolve the header cycle with forward declarations and include ordering; verify the actual helper instantiates, not just its containing header.

**Historical issue excerpt — compare with current status above**

The optical reader copies a non-copyable Sensor and calls a nonexistent one-argument setter.

```cpp
// Current getOpticalPosition parameter:
Sensor<M> sensor
// Current publication statements:
sensor.setAppendedVariable("x") = static_cast<uint16_t>(i2cUtils::convertTo16Bits(&opticalData[0]));
sensor.setAppendedVariable("y") = static_cast<uint16_t>(i2cUtils::convertTo16Bits(&opticalData[2]));
sensor.setAppendedVariable("h") = static_cast<uint16_t>(i2cUtils::convertTo16Bits(&opticalData[4]));
// Current offset call omits the registered sensor name:
return commBus.writeArrayRegister<6>(OTOS_REG_OFFSETS, offsetWrite);
```

**Historical proposal — adapt before applying**

`OpticalPreconfig.h`. **Attempted:** remove its `#include "ARC_Sensor.h"`,
retain `SensorUtils.h`'s forward declaration, and replace the reader.

```cpp
template <size_t M>
inline esp_err_t getOpticalPosition(Sensor<M>& sensor) {
    uint8_t data[6]{};
    const esp_err_t status = sensor.readDeviceRegister(OTOS_REG_POS, data, sizeof(data));
    if (status != ESP_OK) { return status; }
    sensor.setReadings({ // Item 18.
        static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[0])),
        static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[2])),
        static_cast<uint16_t>(i2cUtils::convertTo16Bits(&data[4]))
    });
    return ESP_OK;
}
```

**Attempted offset-helper change:** keep its existing six-byte buffer, change the
first parameter and return call. Add the Sensor wrapper shown below.

```cpp
// OpticalPreconfig::setOffSets first parameter:
Sensor<M>& sensor
// Return call, using the existing offsetWrite[6]:
return sensor.writeDeviceArrayRegister(OTOS_REG_OFFSETS, offsetWrite);

// New public Sensor method (caller already owns bus lock):
template <size_t S>
esp_err_t writeDeviceArrayRegister(uint8_t reg, const uint8_t (&data)[S]) {
    const esp_err_t status = _sensorCommBusI2C.writeArrayRegister(getName(), reg, data);
    confirmStatus(status);
    return status;
}
```

## 14. Sensor name lifetime and invalid IDs

**Current status and next change**

**Applied; source-verified.** Constructor accepts const char*, owns the name with null handling, validates empty names and IDs 1-15, and supplies c_str() to task creation. The direct <string> include is present. Firmware build passes but app_main does not instantiate sensors.

**Historical issue excerpt — compare with current status above**

Names are stored as mutable borrowed pointers, and IDs are not validated in the Sensor constructor.

```cpp
// Constructor parameter:
char* name
// Stored member:
char* _name;
// Constructor initializer:
_name(name)
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`. **Attempted:** accept string literals, own the name, and validate it.

```cpp
// Constructor parameter:
const char* name
// Member:
std::string _name;
// Constructor initializer:
_name(name == nullptr ? "" : name)
// Constructor validation:
if (_name.empty() || _sensorID == 0 || _sensorID > 15) {
    _status = SensorUtils::SENSOR_ERR_ARG;
}
// xTaskCreatePinnedToCore task-name argument:
_name.c_str()
```

## 15. Setup runs despite invalid configuration and leaks failed registrations

**Current status and next change**

**Partial; latest source review saved for resumption.** RS485 registration now occurs BEFORE task creation. The remaining success assignment `_status = SENSOR_OK` is still AFTER xTaskCreatePinnedToCore; move it immediately before task creation so worker errors are not overwritten. Keep `_setupRan = true` after successful task creation. Startup rollback remains missing: remove I2C registration under its mutex after mux/callback/RS485-registration failure; remove both registrations after task-creation failure. Check removal outcomes. fullStopFAIL only deletes a task and does not unregister. This review was source-only; no fresh build was run for the latest ordering edit.

**Historical issue excerpt — compare with current status above**

The task is created before registration completes, setup failures leave bus state behind, and final success overwrites status.

```cpp
// The task has already been created at this point:
const esp_err_t regStatus = this->registerSensor();
if (regStatus != ESP_OK) {
    _status = SensorUtils::SENSOR_ERR_IO;
    fullStopFAIL();
    return SensorUtils::SENSOR_ERR_IO;
}
_status = SensorUtils::SENSOR_OK;
_setupRan = true;
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h` → `setup()`. **Attempted:** validate first, gate the task until
registration succeeds, and roll back failures. Helpers/state are in item 22.

```cpp
// At setup entry, after the existing repeat-setup check:
if (_status != SensorUtils::SENSOR_OK) { return _status.load(); }

// After the EXISTING xTaskCreatePinnedToCore(...) call:
if (taskStatus != pdPASS) {
    _status = SensorUtils::SENSOR_ERR_FAULT;
    fullStopFAIL(); cleanupBus();
    return SensorUtils::SENSOR_ERR_FAULT;
}
const esp_err_t regStatus = RS485ARSENAL::addSensor(this);
if (regStatus != ESP_OK) {
    _status = SensorUtils::SENSOR_ERR_IO;
    fullStopFAIL(); cleanupBus();
    return SensorUtils::SENSOR_ERR_IO;
}
_setupRan = true;
return SensorUtils::SENSOR_OK;
```

Remove the unconditional final `_status = SENSOR_OK`. In public `registerSensor()`,
the attempted guard was:

```cpp
if (!_setupRan) { return ESP_ERR_INVALID_STATE; }
return RS485ARSENAL::addSensor(this);
```

## 16. Setup callbacks run without selecting/locking their bus

**Current status and next change**

**Applied; source-verified.** Setup locks the I2C bus through device registration, mux selection, and callbacks, then releases it before task creation. A null mutex fails immediately; failed lock attempts delay/retry. LockGuard itself blocks on a busy mutex using portMAX_DELAY, so the outer loop does not impose a finite retry interval. Hardware scheduling was not exercised.

**Historical issue excerpt — compare with current status above**

Setup callbacks execute without acquiring the bus mutex or selecting their mux channel.

```cpp
// setup(), immediately after I2C registration:
if (!_setupFuncs.empty()) {
    executeFunctionList(_setupFuncs);
}
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h` → `setup()`. **Attempted:** scope this lock around I2C registration,
mux selection, and setup callbacks; release it before task/RS485 registration.

```cpp
{
    LockGuard lock(_sensorCommBusI2C.getMutex());
    if (!lock.isMutexLocked()) {
        _status = SensorUtils::SENSOR_ERR_IO; return _status.load();
    }
    // Keep existing addSensorClassToBus() call and failure check here.
    _busRegistered = true;
    const esp_err_t status = _sensorCommBusI2C.selectMuxChannel(getName());
    if (status != ESP_OK) {
        _status = SensorUtils::SENSOR_ERR_IO; rollbackBus(); return _status.load();
    }
    executeFunctionList(_setupFuncs);
    if (_status != SensorUtils::SENSOR_OK) { rollbackBus(); return _status.load(); }
}
```

Readers also select the mux under the bus lock (item 18). Setup/read callbacks use
raw wrappers; they must not call `update()` or `setOffsets()` and re-lock that mutex.

## 17. Setup errors are missed or overwritten

**Current status and next change**

**Partial; source-verified.** Sensor confirmStatus accepts NOT_READY and OK, read/write wrappers record errors, and setup checks status after the callback list. These requested changes are present. Later callbacks still execute after an earlier error; worker/setup status access is unsynchronized, and final setup success can overwrite worker failure. No hardware callback failure was injected.

**Historical issue excerpt — compare with current status above**

Errors during NOT_READY setup are ignored; wrappers do not record errors themselves.

```cpp
SensorUtils::sensor_status_t _status{SensorUtils::SENSOR_ERR_NOT_READY};
void confirmStatus(esp_err_t status) {
    if (status != ESP_OK && _status == SensorUtils::SENSOR_OK) {
        _status = SensorUtils::SENSOR_ERR_IO;
    }
}
// Current readDeviceRegister() body:
return _sensorCommBusI2C.readRegister(_name, regAddr, data, length);
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`. **Attempted:** initialize configuration status before validation,
use atomic status, expose inspection/reporting, and report wrapper failures.

```cpp
// Member (replace existing _status declaration):
std::atomic<SensorUtils::sensor_status_t> _status{SensorUtils::SENSOR_ERR_NOT_READY};
// First constructor-body statement before preset/argument validation:
_status = SensorUtils::SENSOR_OK;

// Public methods:
SensorUtils::sensor_status_t getStatus() const { return _status.load(); }
void reportIOStatus(esp_err_t status) { confirmStatus(status); }

// readDeviceRegister() body; apply same pattern to writeDeviceRegister():
const esp_err_t status = _sensorCommBusI2C.readRegister(_name, regAddr, data, length);
confirmStatus(status);
return status;
```

The attempt retained the existing first-error behavior in `confirmStatus()`.
`RS485Comm::_status` was changed to `std::atomic<esp_err_t>` as well.

## 18. Missing publication API and unsynchronized readings

**Current status and next change**

**Snapshot API applied; source-verified.** setReadings and getReadings protect the complete vector copy with the same initialized portMUX_TYPE critical section. I2C stays outside these methods. Readers must publish through setReadings only after successful reads; optical reader integration remains deferred in items 12-13. Direct protected _reading access would bypass the lock. Firmware build passes; concurrent runtime publication remains unverified.

**Historical issue excerpt — compare with current status above**

RS485 copies readings while the sensor task can update them, without a synchronized publication path.

```cpp
Vec3<uint16_t> getReadings() override {
    Vec3<uint16_t> reading = {_reading.x, _reading.y, _reading.z};
    return reading;
}
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`. **Attempted:** synchronize snapshots and extract one read cycle.
Allocate/check `_readingMutex` in the constructor; delete it after stopping the task.

```cpp
SemaphoreHandle_t _readingMutex = nullptr; // Member.
// Constructor body:
_readingMutex = xSemaphoreCreateMutex();
if (!_readingMutex) { _status = SensorUtils::SENSOR_ERR_FAULT; }

// Public:
void setReadings(const Vec3<uint16_t>& reading) {
    LockGuard lock(_readingMutex);
    if (lock.isMutexLocked()) { _reading = reading; }
}
Vec3<uint16_t> getReadings() override {
    LockGuard lock(_readingMutex);
    return lock.isMutexLocked() ? _reading : Vec3<uint16_t>{};
}
SensorUtils::sensor_status_t update() {
    if (!_setupRan) { return SensorUtils::SENSOR_ERR_NOT_READY; }
    LockGuard lock(_sensorCommBusI2C.getMutex());
    if (!lock.isMutexLocked()) { _status = SensorUtils::SENSOR_ERR_IO; return _status.load(); }
    const esp_err_t status = _sensorCommBusI2C.selectMuxChannel(getName());
    if (status != ESP_OK) { confirmStatus(status); return _status.load(); }
    _readFunction(*this);
    return _status.load();
}
```

Replace the read-lock block in `taskLoop()` with `update()`. Keep loop callbacks outside
that lock. All readers must publish through the synchronized path.

## 19. Appended variables lose their upper byte

**Current status and next change**

**Applied; firmware build passes.** appendVariable/setAppendedVariable now accept uint16_t and getAppendedVariable returns uint16_t, matching map and initializer storage. Bodies were retained. The existing failure sentinel remains ambiguous with valid values; a separate success/result interface is not part of this change. Runtime round-trip behavior was not exercised; app_main instantiates no sensors.

**Historical issue excerpt — compare with current status above**

The map stores uint16_t, but the accessor parameters and return type are uint8_t.

```cpp
std::unordered_map<std::string_view, uint16_t> _appendedVariables{};
// Current method signatures:
void appendVariable(std::string_view variableName, uint8_t variable);
uint8_t getAppendedVariable(char* variableName);
void setAppendedVariable(std::string_view variableName, uint8_t data);
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`. **Attempted:** change these signatures; keep existing method bodies.

```cpp
void appendVariable(std::string_view variableName, uint16_t variable);
uint16_t getAppendedVariable(const char* variableName);
void setAppendedVariable(std::string_view variableName, uint16_t data);
void executeAppendedMethod(const char* methodName);
```

## 20. Capacity checks still insert; null callbacks can execute

**Current status and next change**

**Applied; source-verified, runtime validation deferred.** Full method/variable containers return before insertion; setup/loop limits use >=; method/setup/loop append functions reject null callbacks; method/variable duplicate names are rejected. Constructor setup/loop lists now call the guarded append methods. These checks do not resolve unrelated callback-name storage lifetime or shared-state synchronization.

**Historical issue excerpt — compare with current status above**

Full registries still insert entries, and the loop check permits one extra callback.

```cpp
if (_appendedMethods.size() >= SENSOR_MAX_METHODS) {
    _status = SensorUtils::SENSOR_ERR_FULL;
}
_appendedMethods.emplace(functionName, function);
// appendLoopFunc() currently checks:
if (_loopFuncs.size() > SENSOR_MAX_ON_LOOP) {
    _status = SensorUtils::SENSOR_ERR_FULL;
}
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h`. **Attempted:** reject before inserting. Apply the same pattern to
variables and setup/loop callbacks; validate constructor lists via those append methods.

```cpp
// At appendMethod() entry:
if (_setupRan || functionName.empty() || !function) {
    _status = SensorUtils::SENSOR_ERR_ARG; return;
}
if (_appendedMethods.size() >= SENSOR_MAX_METHODS) {
    _status = SensorUtils::SENSOR_ERR_FULL; return;
}
_appendedMethods.emplace(functionName, function);

// appendLoopFunc(): change > to >=.
if (_loopFuncs.size() >= SENSOR_MAX_ON_LOOP) {
    _status = SensorUtils::SENSOR_ERR_FULL;
} else { _loopFuncs.push_back(function); }
```

Also add the `_setupRan || !function` rejection to both callback append methods.
The attempt did not implement a new duplicate-name policy; choose one if needed.

## 21. Invalid offset-register writes and inconsistent byte order

**Current status and next change**

**Partial; guards applied, validation deferred.** Unused offset registers are rejected, x/y/z are checked, mux failure stops the write, and a null bus mutex reports IO instead of retrying forever. Existing mutex waiting behavior remains. Device-specific byte order, result/completion reporting, and response-path wait policy remain open.

**Historical issue excerpt — compare with current status above**

Offset writes do not reject an unset register or stop after mux failure; the sentinel test repeats y instead of checking z.

```cpp
if (_offsets.x != SensorUtils::UNUSED_OFFSETS &&
    _offsets.y != SensorUtils::UNUSED_OFFSETS &&
    _offsets.y != SensorUtils::UNUSED_OFFSETS) {
    // Existing locked section:
    confirmStatus(_sensorCommBusI2C.selectMuxChannel(getName()));
    // The write still follows even if mux selection failed.
}
```

**Historical proposal — adapt before applying**

`ARC_Sensor.h` → replace `setSensorOffsets()`. **Attempted:** gate on register
configuration, stop on mux failure, and swap optical device bytes. This removes the
old repeated-y sentinel check; raw `0xFFFF` is not rejected by this version.

```cpp
void setSensorOffsets() {
    if (_offsetMemReg == SensorUtils::UNUSED_OFFSET_REG || !_busRegistered) { return; }
    LockGuard lock(_sensorCommBusI2C.getMutex());
    if (!lock.isMutexLocked()) { confirmStatus(ESP_ERR_INVALID_STATE); return; }
    const esp_err_t muxStatus = _sensorCommBusI2C.selectMuxChannel(getName());
    if (muxStatus != ESP_OK) { confirmStatus(muxStatus); return; }
    uint8_t data[6]{};
    SensorUtils::convertOffsetsToArr(data, _offsets);
    if (_offsetsLittleEndian) {
        for (size_t i = 0; i < 6; i += 2) { std::swap(data[i], data[i + 1]); }
    }
    confirmStatus(_sensorCommBusI2C.writeArrayRegister(getName(), _offsetMemReg, data));
}
```

Device endianness needs hardware/spec confirmation; RS485 wire order is separate.

## 22. Unsafe task deletion and stale sensor registrations

**Current status and next change**

**Runtime shutdown deferred by explicit lifetime decision.** Benjamin confirmed that sensors live for the entire firmware run. Keep sensors and their buses alive, register/configure before starting workers, and start RS485 reception after all sensors are ready. No runtime replacement/removal is required. Do not add the worker-exit handshake for this scope. The stop-request code and I2C removal helper already added may remain unused; no source was removed by this decision. Startup failures must still leave no running task accessing invalid state, report failure, and undo registrations where appropriate (item 15). Historical shutdown snippets below are out of current scope.

**Historical issue excerpt — compare with current status above**

Deleting the task immediately can strand an owned mutex, while the registries retain the Sensor/device.

```cpp
~Sensor() {
    if (_taskObject != NULL) {
        vTaskDelete(_taskObject);
        _taskObject = NULL;
    }
}
// No corresponding unregister/remove operation.
```

**Historical proposal — adapt before applying**

`I2CBus.h`. **Attempted:** add removal so setup failures and destruction can clean up.

```cpp
esp_err_t removeSensorFromBus(std::string_view name) {
    const auto it = _registryMap.find(std::string(name));
    if (it == _registryMap.end()) { return ESP_ERR_INVALID_ARG; }
    const esp_err_t status = i2c_master_bus_rm_device(it->second.sensorHandle);
    if (status == ESP_OK) { _registryMap.erase(it); }
    return status;
}
```

`ARC_Sensor.h`. **Attempted:** cooperate with the task instead of deleting it while
it might hold a mutex. Apply the following fields/helpers together.

```cpp
std::atomic<bool> _setupRan{false}, _stopRequested{false};
bool _busRegistered = false;
SemaphoreHandle_t _taskStopped = nullptr;
// Constructor: create and check it alongside the reading mutex.
// _taskStopped = xSemaphoreCreateBinary();

void rollbackBus() { // Caller holds bus lock.
    if (_busRegistered && _sensorCommBusI2C.removeSensorFromBus(getName()) == ESP_OK) {
        _busRegistered = false;
    }
}
void cleanupBus() {
    LockGuard lock(_sensorCommBusI2C.getMutex());
    if (lock.isMutexLocked()) { rollbackBus(); }
}
void fullStopFAIL() {
    if (_taskObject) {
        _stopRequested = true;
        xSemaphoreTake(_taskStopped, portMAX_DELAY);
        _taskObject = nullptr;
    }
}
```

**Attempted task-loop/destructor edits:** retain existing loop callbacks/delay inside
the loop. Callbacks must terminate; do not destroy a Sensor from its own task.

```cpp
// taskLoop():
while (!_stopRequested) {
    if (!_setupRan) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    // update(); existing loop callbacks; existing delay.
}
xSemaphoreGive(_taskStopped);
vTaskDelete(nullptr);

// Destructor body:
RS485ARSENAL::removeSensor(this);
fullStopFAIL();
cleanupBus();
if (_readingMutex) { vSemaphoreDelete(_readingMutex); }
if (_taskStopped) { vSemaphoreDelete(_taskStopped); }
```

## 23. UART initialization overwrites earlier errors

**Current status and next change**

**Open.** Each UART initialization step still overwrites `_status`; a later success can mask an earlier failure. Stop at the first failure, track driver ownership for cleanup, and prevent receiver startup after failed configuration. Verify errors from parameter configuration, pins, installation, mode selection, and task creation independently.

**Historical issue excerpt — compare with current status above**

Each UART step overwrites _status, hiding an earlier failure.

```cpp
_status = uart_param_config(RS485_COMM_UART, &configUart);
_status = uart_set_pin(RS485_COMM_UART,
    UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_PIN_NO_CHANGE);
_status = uart_driver_install(RS485_COMM_UART, 2048, 0, 0, nullptr, 0);
_status = uart_set_mode(RS485_COMM_UART, UART_MODE_RS485_HALF_DUPLEX);
```

**Historical proposal — adapt before applying**

`RS485Comm.h`, constructor. **Attempted:** return after each failed call; preserve
the existing `configUart` construction. Track driver ownership and forbid copying.

```cpp
_status = uart_param_config(RS485_COMM_UART, &configUart);
if (_status != ESP_OK) { return; }
_status = uart_set_pin(RS485_COMM_UART, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_PIN_NO_CHANGE);
if (_status != ESP_OK) { return; }
_status = uart_driver_install(RS485_COMM_UART, 2048, 0, 0, nullptr, 0);
if (_status != ESP_OK) { return; }
_driverInstalled = true;
_status = uart_set_mode(RS485_COMM_UART, UART_MODE_RS485_HALF_DUPLEX);

// Members/declarations:
bool _driverInstalled = false;
RS485Comm(const RS485Comm&) = delete;
RS485Comm& operator=(const RS485Comm&) = delete;
```

The attempted destructor deleted the receiver task, owned driver, and registry mutex.
Keep the communicator alive for the application lifetime; safe runtime shutdown is
not established by that destructor and would need a cooperative stop before cleanup.

## 24. Registry accepts duplicate/null/oversized IDs

**Current status and next change**

**Partial.** Null pointers and ID zero are rejected; removal checks pointer identity. IDs above 15 are accepted, duplicate emplace failure reports success, and mutation after receiver startup has no safe policy. Reject unsupported IDs and duplicates, then either freeze registration while running or synchronize registry access/lifetime. Verify duplicate IDs, ID 16, empty start policy, and late registration/removal.

**Historical issue excerpt — compare with current status above**

Null pointers can be dereferenced, duplicate insertion still returns success, and IDs can exceed four bits.

```cpp
esp_err_t addSensortoRegistry(RS485InterfaceSensor* sensor) {
    if (sensor->getID() == 0) { return ESP_ERR_INVALID_ARG; }
    _registryMap.emplace(sensor->getID(), sensor);
    return ESP_OK;
}
```

**Historical proposal — adapt before applying**

`RS485Comm.h`. **Attempted:** add a registry mutex, validate IDs, and reject additions
after receiver startup. Allocate/check `_registryMutex` in the constructor.

```cpp
SemaphoreHandle_t _registryMutex = nullptr; // Member.
// Constructor: _registryMutex = xSemaphoreCreateMutex();
// if (!_registryMutex) { _status = ESP_ERR_NO_MEM; return; }

esp_err_t addSensortoRegistry(RS485InterfaceSensor* sensor) {
    if (!sensor || sensor->getID() == 0 || sensor->getID() > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    LockGuard lock(_registryMutex);
    if (!lock.isMutexLocked()) { return ESP_ERR_INVALID_STATE; }
    if (_status != ESP_OK) { return _status; }
    if (_taskObject) { return ESP_ERR_INVALID_STATE; }
    return _registryMap.emplace(sensor->getID(), sensor).second
        ? ESP_OK : ESP_ERR_INVALID_ARG;
}
void removeSensorFromRegistry(RS485InterfaceSensor* sensor) {
    LockGuard lock(_registryMutex);
    if (!lock.isMutexLocked() || !sensor) { return; }
    const auto it = _registryMap.find(sensor->getID());
    if (it != _registryMap.end() && it->second == sensor) { _registryMap.erase(it); }
}
```

The attempt also locked `start()`, `extractData()`, and `compileSentPacket()`.
Add this guard to `start()` after acquiring the lock, retaining its existing checks:

```cpp
if (_registryMap.empty()) { return ESP_ERR_INVALID_STATE; }
```

## 25. Offset extraction uses single bytes and can read outside the payload

**Current status and next change**

**Open; V5 contract required.** Offset extraction reads three single bytes with a `3 * slot` stride rather than three 16-bit values with a six-byte stride. It does not validate that those bytes belong to the payload, and one global `_isFirstRead` flag marks initialization complete even if some sensors were absent or a write failed. Confirm whether V5 permits ID-only requests and which packets carry offsets, validate every slot before reading, decode the agreed byte order, and track completion according to that contract.

**Historical issue excerpt — compare with current status above**

Offsets consume one byte per component and iterate over all IDs regardless of received payload length.

```cpp
// Inside a loop over all 16 ID slots:
offsets.x = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + 3 * i]);
offsets.y = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + 3 * i + 1]);
offsets.z = static_cast<uint16_t>(_packet[INDEX_SENSOR_DATA + 3 * i + 2]);
// Initialization is consumed even by an ID-only frame:
_isFirstRead = false;
```

**Historical proposal — adapt before applying**

`RS485Utils.h`. **Attempted:** add a big-endian 16-bit decoder.

```cpp
template <size_t S>
uint16_t convertU16InArr(const uint8_t (&packet)[S], size_t index) {
    return (static_cast<uint16_t>(packet[index]) << 8) |
           static_cast<uint16_t>(packet[index + 1]);
}
```

`RS485Comm.h` → replace the `_isFirstRead` block in `extractData()`.
**Attempted:** six-byte slots, bounded by received payload length.

```cpp
if (_isFirstRead) {
    for (size_t i = 0; i < (_packetLength - PACKET_LENGTH_MIN) / 6u; ++i) {
        if (!_registryMap.contains(_sensorIDS[i])) { continue; }
        const size_t p = INDEX_SENSOR_DATA + 6 * i;
        Vec3<uint16_t> offsets{
            RS485Utils::convertU16InArr(_packet, p),
            RS485Utils::convertU16InArr(_packet, p + 2),
            RS485Utils::convertU16InArr(_packet, p + 4)
        };
        _registryMap.at(_sensorIDS[i])->setOffsets(offsets);
    }
    if (_packetLength > PACKET_LENGTH_MIN) { _isFirstRead = false; }
}
```

**Confirm V5 first:** this assumes three big-endian 16-bit offsets per slot and all
initial offsets in the first data-bearing frame. The V5 request contract was not
verified. Failed offset writes still need an agreed retry/reporting policy.

## 26. Always sending 128 bytes

**Current status and next change**

**Open.** `sendData()` transmits `sizeof(sendingPacket)` (128) regardless of encoded length. Return the serialized size from compilation and pass it to `uart_write_bytes`; compare the write result against that same size. Under the existing response layout, total size is body length + 15: one slot is 37 bytes and a response through slot 16 is 127 bytes. Verify sparse slots and missing IDs against V5 expectations.

**Historical issue excerpt — compare with current status above**

Every reply transmits the entire 128-byte buffer instead of its encoded frame size.

```cpp
int bytesWritten = uart_write_bytes(
    UART_NUM_1, sendingPacket, sizeof(sendingPacket));
if (bytesWritten != static_cast<int>(sizeof(sendingPacket))) {
    _status = ESP_ERR_INVALID_STATE;
}
```

**Historical proposal — adapt before applying**

`RS485Comm.h` → `sendData()`. **Attempted:** send the encoded frame size.

```cpp
const size_t sendingLength =
    RS485Utils::convertU16InArr(sendingPacket, INDEX_PACKET_LENGTH) + 15;
const int bytesWritten = uart_write_bytes(RS485_COMM_UART, sendingPacket, sendingLength);
if (bytesWritten != static_cast<int>(sendingLength)) { _status = ESP_ERR_INVALID_STATE; }
```

Also attempted: use `uint16_t packetLengthInvariant` and a `uint64_t timeNow`
inside `compileSentPacket()`, with explicit casts into wire fields. For the existing
response layout: one slot = 37 bytes; a response reaching slot 16 = 127 bytes.

## 27. Parser length checks and incomplete-frame recovery

**Current status and next change**

**Partial.** The checksum endpoint is `_dataLastIndex + 8`, the final escape endpoint is `+12`, and the parser resets after an invalid final escape. Remaining work includes payload layout validation, bounded writes, truncated-frame recovery, and helper index/unsigned-shift cleanup. The constants still say length-field size 4, timestamp-section sum 4, and maximum body 113. The earlier proposal of body sizes `16 + 6*n` (max 112) requires V5 confirmation before enforcing it. Test malformed checksums/markers, partial UART reads, idle recovery, consecutive packets, and sparse slots.

**Historical issue excerpt — compare with current status above**

Length checks allow unaligned payloads; there is no idle reset for an incomplete frame.

```cpp
#define PACKET_LENGTH_BYTECOUNT 4
#define SENSOR_TIME_SECTION_BYTECOUNT_SUM 4
#define PACKET_LENGTH_MAX 113

bool invalidPacketLength() {
    return _packetLength > PACKET_LENGTH_MAX || _packetLength < PACKET_LENGTH_MIN;
}
// getBytesLoop() has no reset when uart_read_bytes() times out.
```

**Historical proposal — adapt before applying**

`RS485Comm.h`. **Attempted:** correct constants and validate six-byte slot lengths.
Apply only if item 25's V5 request assumption is confirmed.

```cpp
#define PACKET_LENGTH_BYTECOUNT 2
#define SENSOR_TIME_SECTION_BYTECOUNT_SUM 8
#define PACKET_LENGTH_MAX 112

bool invalidPacketLength() {
    return _packetLength < PACKET_LENGTH_MIN || _packetLength > PACKET_LENGTH_MAX ||
           (_packetLength - PACKET_LENGTH_MIN) % 6 != 0;
}
bool validDataLayout() const {
    if (_packetLength == PACKET_LENGTH_MIN) { return true; }
    const size_t slots = (_packetLength - PACKET_LENGTH_MIN) / 6;
    for (size_t i = slots; i < 16; ++i) {
        const uint8_t packed = _packet[INDEX_SENSOR_IDS + i / 2];
        if ((i % 2 == 0 ? packed >> 4 : packed & 0xF) != 0) { return false; }
    }
    return true;
}
```

**Attempted:** require `validDataLayout()` along with the first escape check;
add the bounds guard at `parseByte()` entry. Expose these transport/test methods:

```cpp
// parseByte() entry:
if (_currentPacketIndex >= sizeof(_packet)) { resetParsing(); }

// Public methods:
void resetReceiveState() { resetParsing(); _validPacket = false; }
bool processReceivedByte(uint8_t byte) {
    parseByte(byte);
    if (!_validPacket) { return false; }
    _validPacket = false;
    sendData();
    return true;
}
```

In `getBytesLoop()`, the attempt fed bytes through `processReceivedByte()` and called
`resetReceiveState()` on an idle read timeout. It also cleared `_validPacket` when
accepting a new header. Preserve normal frames split across UART reads; confirm
the existing 100 ms idle timeout is appropriate for V5.

`RS485Utils.h`: the attempted unsigned decode and index-type changes were:

```cpp
// convertU32InArr() loop:
converted |= static_cast<uint32_t>(packet[startingIndex + i]) << (8 * (3 - i));
// Use size_t startingIndex in convertU32InArr/writeU16ToArr/writeU32ToArr.
```

## 28. No two-bus application startup

**Current status and next change**

**Open.** `app_main()` constructs no buses/sensors, never starts RS485, and spins without yielding. Supply confirmed wiring and startup configuration, keep object lifetimes valid, check bus/sensor setup, then start reception only after registration succeeds. Return from app_main or block/delay appropriately. `N` is mux count per controller; two direct controllers use two `I2CBus<0>` objects. No production pin assignment or two-bus hardware validation is established.

**Historical issue excerpt — compare with current status above**

app_main creates no buses or sensors and never starts the receiver.

```cpp
extern "C" void app_main(void) {
    for (;;) {
    }
}
```

**Historical proposal — adapt before applying**

The attempted `examples/TwoBusPipeline.h` used the following constructor and start
sequence; that file is now removed. Supply actual pins and keep instances alive.

```cpp
struct TwoBusPins {
    gpio_num_t bus0Scl, bus0Sda, bus1Scl, bus1Sda;
};
class TwoBusPipeline {
public:
    explicit TwoBusPipeline(const TwoBusPins& pins)
        : bus0(pins.bus0Scl, pins.bus0Sda, I2C_NUM_0),
          bus1(pins.bus1Scl, pins.bus1Sda, I2C_NUM_1),
          optical0("optical0", bus0, OTOS_I2C_ADDRESS, 1, OPTICAL),
          optical1("optical1", bus1, OTOS_I2C_ADDRESS, 2, OPTICAL) {}
    esp_err_t start() {
        if (bus0.getStatus() != ESP_OK) { return bus0.getStatus(); }
        if (bus1.getStatus() != ESP_OK) { return bus1.getStatus(); }
        if (optical0.setup() != SensorUtils::SENSOR_OK) { return ESP_FAIL; }
        if (optical1.setup() != SensorUtils::SENSOR_OK) { return ESP_FAIL; }
        return RS485ARSENAL::startRS485ARSENAL();
    }
    I2CBus<0> bus0, bus1;
    Sensor<0> optical0, optical1;
};
```

**Suggested integration, not previously activated:** after defining the four pin
constants from your wiring, use this in `src/main.cpp`. The previous attempt left
hardware startup inactive and replaced the empty busy loop with a return.

```cpp
extern "C" void app_main() {
    static TwoBusPipeline pipeline({BUS0_SCL, BUS0_SDA, BUS1_SCL, BUS1_SDA});
    ESP_ERROR_CHECK(pipeline.start());
}
```

`N` is mux count, not controller count: two direct buses are two `I2CBus<0>` objects.
The optical addresses/IDs shown are the attempted example, not confirmed hardware.

## 29. SDK flash size differs from the board profile

**Current status and next change**

**Open.** The firmware build still reports 8 MB in the board profile versus 2 MB in SDK configuration. Confirm the actual module flash capacity, update the owning defaults/menuconfig selection and active generated config coherently, then rebuild and check the partition/image limits. A successful link alone does not resolve the warning.

**Historical issue excerpt — compare with current status above**

The existing SDK selection is 2 MB while the selected board profile expects 8 MB; verify the physical board before changing it.

```ini
# sdkconfig.seeed_xiao_esp32s3:
CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y
# CONFIG_ESPTOOLPY_FLASHSIZE_8MB is not set
CONFIG_ESPTOOLPY_FLASHSIZE="2MB"
```

**Historical proposal — adapt before applying**

**Suggested only; no previous patch.** The observed SDK setting was 2 MB while
the board profile expected 8 MB. If the actual board is 8 MB, select 8 MB in
menuconfig, or set the default and regenerate/update the existing SDK config:

```ini
# sdkconfig.defaults — only after verifying physical flash size:
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="8MB"
```

Defaults alone do not override an existing sdkconfig selection. This warning did
not block compilation; it is a deployment configuration check.

## Check your implementation

The 17 pipeline tests expect helper APIs shown above. You can adapt those tests
to your design. They currently fail to compile against the reverted production
source; the original 22 checks pass. Rebuild before running cached test binaries.

```powershell
cmake -S test -B .pio/native-tests
cmake --build .pio/native-tests --config Release
ctest --test-dir .pio/native-tests -C Release --output-on-failure
pio run -e seeed_xiao_esp32s3
```

The snippets are not a fresh tested patch. Finish with both real buses active,
a V5 request/response capture, and task-stack measurements. No firmware was flashed.

## Verification while automated tests are removed

At Benjamin's request, the generated desktop test suite, mocks, and its build
artifacts were removed on October 26, 2026. Do not recreate tests or add APIs just
to satisfy those removed tests unless requested. Earlier test results and test
snippets above are historical evidence only; they are not current work items.

Check each manual source change by inspection, then use an appropriate firmware
build or hardware observation when needed. Firmware builds do not establish
physical mux routing, OTOS operation, or V5 interoperability.









