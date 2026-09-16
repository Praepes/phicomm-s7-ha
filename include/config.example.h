#pragma once

// Optional factory defaults. Copy this file to include/config.h if you want
// first boot to already know your network. You can also leave config.h absent
// and configure everything from the built-in web page.

#define S7_WIFI_SSID "your-wifi"
#define S7_WIFI_PASSWORD "your-password"

#define S7_MQTT_HOST "192.168.1.2"
#define S7_MQTT_PORT 1883
#define S7_MQTT_USER ""
#define S7_MQTT_PASSWORD ""
#define S7_OTA_PASSWORD ""

#define S7_DEVICE_ID "phicomm_s7"
#define S7_DEVICE_NAME "Phicomm S7"

// User profile for first-pass body composition estimation.
#define S7_PROFILE_GENDER_MALE 1
#define S7_PROFILE_AGE 30
#define S7_PROFILE_HEIGHT_M 1.75f

// Weight calibration:
// weight_kg = max(0, (raw - tare) * scale)
// Start with scale=1.0 and use MQTT commands to calibrate after checking raw movement.
#define S7_WEIGHT_TARE_DEFAULT 0
#define S7_WEIGHT_SCALE_DEFAULT 1.0f

// Experimental weight read setup. Adjust if live raw values do not move with load.
#define S7_WEIGHT_REG8 0x33
#define S7_WEIGHT_REG7 0x00
#define S7_WEIGHT_USE_REG7 0

// Stock firmware points to GPIO13 active-high for the heat-test output.
// It remains disabled by default; set 13 after PCB verification.
#define S7_HEATER_PIN -1
#define S7_HEATER_ACTIVE_HIGH 1
#define S7_HEATER_MAX_SECONDS 30
