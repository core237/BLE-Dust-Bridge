#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// M5Stack AtomS3 Lite (ESP32-S3) built-in front button & WS2812 RGB LED
#define BTN_ATOM_GPIO 41
#define RGB_LED_GPIO  35

#ifndef DEFAULT_AUTH_EN
#define DEFAULT_AUTH_EN 1
#endif
#ifndef DEFAULT_AUTH_USER
#define DEFAULT_AUTH_USER "admin"
#endif
#ifndef DEFAULT_AUTH_PASS
#define DEFAULT_AUTH_PASS "admin"
#endif

static const size_t MAX_USED_NONCES = 8;
static const size_t MAX_AUTH_SESSIONS = 4;
static const unsigned long AUTH_SESSION_TTL_MS = 86400000UL;

static const uint8_t FMP_ROUTE_APP_TO_MAIN = 0x41;
static const uint8_t FMP_ROUTE_APP_TO_BLE  = 0x21;

static const size_t MAX_MON_MAC_CACHE = 16;
static const size_t MAX_BLE_LIST_ENTRIES = 48;
static const size_t SCAN_QUEUE_CAP = 64;

static const unsigned long ADV_TOGGLE_TAP_RELEASE_MS  = 500UL;
static const unsigned long ADV_TAP_RELEASE_MS         = 95UL;
static const unsigned long ADV_HOLD_RELEASE_MS        = 190UL;
static const unsigned long ADV_MIN_TAP_GAP_MS         = 85UL;
static const unsigned long BTN_MOMENTARY_TAP_MAX_MS   = 550UL;
static const unsigned long BTN_GATT_TAP_WINDOW_MS     = 500UL;

static const int MAX_GATT_BTN_SLOTS = 2;
static const int MAX_GATT_PENDING = 4;

extern const char* hostname;
extern NimBLEUUID DUST_SVC_UUID;
extern NimBLEUUID CHAR_WRITE_UUID;
extern NimBLEUUID CHAR_NOTIFY_UUID;
