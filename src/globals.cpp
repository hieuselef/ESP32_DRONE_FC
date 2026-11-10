#include "globals.h"
SemaphoreHandle_t mtx_rc = nullptr;
SemaphoreHandle_t mtx_sensor = nullptr;
SemaphoreHandle_t mtx_control = nullptr;
SemaphoreHandle_t mtx_battery = nullptr;
SharedRC shared_rc;
SharedMTF shared_mtf;
SharedControl shared_control;

SharedBattery shared_battery;

SemaphoreHandle_t mtx_nav = nullptr;
SharedNav shared_nav;

volatile uint32_t guided_quiet_until_ms = 0;
