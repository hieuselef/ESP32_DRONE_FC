#pragma once
#include <Arduino.h>
#include "types.h"

extern SemaphoreHandle_t mtx_rc;
extern SemaphoreHandle_t mtx_sensor;
extern SemaphoreHandle_t mtx_control;
extern SemaphoreHandle_t mtx_battery;
extern SharedRC shared_rc;
extern SharedMTF shared_mtf;
extern SharedControl shared_control;
extern SharedBattery shared_battery;

extern SemaphoreHandle_t mtx_nav;
extern SharedNav shared_nav;

// V6.3H: silence verbose main-loop logging during guided acquisition.
// Only diagnostic console output is affected; controller tasks are unchanged.
extern volatile uint32_t guided_quiet_until_ms;
