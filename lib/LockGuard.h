#pragma once

//FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class LockGuard {
    //Class structure that automatically locks the mutex that
    //is passed in by reference

    public:
        //CONSTRUCTOR: Take the mutex if it exists and if it able to be taken
        explicit LockGuard(SemaphoreHandle_t& mutex): _mutexReference(mutex) {
            if (this->_mutexReference != nullptr && xSemaphoreTake(this->_mutexReference, portMAX_DELAY) != pdFALSE) {
                // This specific instance locked the mutex, and so this sensor has the actual access to the mutex
                this->_isLockedByLockGuard = true;
            }
        }

        //DESTRUCTOR: unlocks the mutex
        // This will automatically be called when the LockGuard goes out of scope
        // of its declaration
        // KEY DETAIL: The mutex will only release itself if this specific iteration
        // of the LockGuard took the mutex
        ~LockGuard() noexcept {
            if (this->_isLockedByLockGuard) {
                xSemaphoreGive(this->_mutexReference);
            }
        }

        //COPY PREVENTION
        //Prevent the LockGuard from being copied to ensure the mutex is secure
        //behind this class instance
        LockGuard(const LockGuard&) = delete;
        LockGuard& operator=(const LockGuard&) = delete;

        bool isMutexLocked() {
            return this->_isLockedByLockGuard;
        }
    private:
        SemaphoreHandle_t& _mutexReference;
        bool _isLockedByLockGuard = false;
};
