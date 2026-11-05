// Copy this file to wifi_secrets.h and fill in your network. wifi_secrets.h is never committed.
#pragma once

// Your 2.4 GHz Wi-Fi network. If the connection fails, the robot creates its own network
// (AP_SSID / AP_PASS) at 192.168.4.1.
#define WIFI_SSID "network_name"
#define WIFI_PASS "network_password"
#define AP_SSID   "NavCore"
#define AP_PASS   "robot_password"

// Password asked by the app for firmware updates over Wi-Fi (AP_PASS is used if missing).
#define OTA_PASS  "update_password"
