#pragma once

#include <esp_log.h> 
#include <driver/gpio.h>

#include <Arduino.h>

#include <rgh/ucp/core.hpp>
using namespace rgh;
using namespace rgh::freertos_literals;

using namespace std;

enum TaskPriority_ {
    TaskPriority_Default   = 3,
    TaskPriority_High      = 4,
    TaskPriority_Immediate = 5
};

constexpr const char* const   TAG   = "oca-cubles";

/* =-. USB serial .-= */
constexpr uint32_t   USB_SERIAL_BAUD_RATE   = 115200;
constexpr uint32_t   USB_SERIAL_FAST_MS     = 100;
constexpr uint32_t   USB_SERIAL_SLOW_MS     = 3000;
constexpr uint32_t   USB_SERIAL_TIMEOUT_MS  = 30000;

/* =-. GPIO .-= */
constexpr gpio_num_t   GPIO_I_D1_4   = GPIO_NUM_NC; // <- K2.1
constexpr gpio_num_t   GPIO_Q_D1_1   = GPIO_NUM_19; // -> K1.1
constexpr gpio_num_t   GPIO_Q_D1_2   = GPIO_NUM_18; // -> K1.2

/* =-. Hidropump .-= */
constexpr uint32_t   HIDROPUMP_MAX_ENGAGE_PERIOD_MS   = 120 * 60 * 1000; // 2 hours

/* =-. Daemons .-= */
constexpr uint32_t   DAEMON_CLUSTER_ITERATE_INTERVAL_MS   = 30000;

constexpr int64_t    REMOTE_CLASS_I_INTERVAL_MS   = 60000;

constexpr uint32_t   REMOTE_WIFI_CONNECT_TRIES      = 5;
constexpr uint32_t   REMOTE_WIFI_CONNECT_DELAY_MS   = 1000;

constexpr uint32_t   REMOTE_TB_MAIN_TASK_DELAY_MS      = 150;
constexpr uint32_t   REMOTE_TB_MAX_SHARED_ATTRIBUTES   = 10;
constexpr uint32_t   REMOTE_TB_SATTR_REQ_TIMEOUT_MS    = 15000;

/* =-. Storage .-= */
constexpr const char* const   STORAGE_SECTOR      = "oca-cubles";
constexpr const char* const   STORAGE_WIFI_SSID   = "wifi-ssid";
constexpr const char* const   STORAGE_WIFI_PWRD   = "wifi-pwrd";
constexpr const char* const   STORAGE_TB_SERVER   = "tb-server";
constexpr const char* const   STORAGE_TB_PORT     = "tb-port";
constexpr const char* const   STORAGE_TB_TOKEN    = "tb-token";

void critical_handler( void ) {
	esp_restart();
}



 